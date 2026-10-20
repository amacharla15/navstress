"""LLM clients: any OpenAI-compatible chat-completions endpoint, or a deterministic mock.

The HTTP client uses only the standard library so the agent needs no vendor SDK.
Tests and CI use MockLLM exclusively; nothing in the test suite reaches a provider.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

Message = dict[str, str]

# Google's documented OpenAI-compatibility base URL for the Gemini API.
DEFAULT_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/"
CONTEXT_PREFIX = "CONTEXT_JSON: "


class LLMClient(Protocol):
    def complete(self, messages: list[Message]) -> str: ...


class LLMError(RuntimeError):
    pass


@dataclass
class OpenAICompatibleClient:
    base_url: str
    api_key: str
    model: str
    timeout_s: float = 120.0
    temperature: float = 0.2
    # response_format={"type": "json_object"} is part of the OpenAI API; endpoints that
    # reject it can turn it off with NAVSTRESS_LLM_JSON_MODE=0.
    json_mode: bool = True

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> OpenAICompatibleClient:
        env = os.environ if env is None else env
        missing = [k for k in ("NAVSTRESS_LLM_API_KEY", "NAVSTRESS_LLM_MODEL") if not env.get(k)]
        if missing:
            raise LLMError(f"set {', '.join(missing)} (see .env.example) or use --mock")
        return cls(
            base_url=env.get("NAVSTRESS_LLM_BASE_URL", DEFAULT_BASE_URL),
            api_key=env["NAVSTRESS_LLM_API_KEY"],
            model=env["NAVSTRESS_LLM_MODEL"],
            json_mode=env.get("NAVSTRESS_LLM_JSON_MODE", "1") != "0",
        )

    def _url(self, path: str) -> str:
        return self.base_url.rstrip("/") + "/" + path

    def _request(self, path: str, body: dict | None) -> dict:
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(
            self._url(path),
            data=data,
            method="GET" if body is None else "POST",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode(errors="replace")[:500]
            raise LLMError(f"HTTP {exc.code} from {self._url(path)}: {detail}") from exc
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise LLMError(f"request to {self._url(path)} failed: {exc}") from exc

    def complete(self, messages: list[Message]) -> str:
        body: dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
        }
        if self.json_mode:
            body["response_format"] = {"type": "json_object"}
        out = self._request("chat/completions", body)
        try:
            return out["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as exc:
            raise LLMError(f"unexpected response shape: {str(out)[:300]}") from exc

    def list_models(self) -> list[str]:
        out = self._request("models", None)
        return sorted(m.get("id", "") for m in out.get("data", []))


_FENCE = re.compile(r"```(?:json)?\s*(.*?)```", re.DOTALL)


def extract_json(text: str) -> Any:
    """Parse the JSON object in a reply, tolerating code fences and surrounding prose."""
    m = _FENCE.search(text)
    candidate = m.group(1) if m else text
    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        pass
    start, end = candidate.find("{"), candidate.rfind("}")
    if start == -1 or end <= start:
        raise ValueError("no JSON object found in the response")
    return json.loads(candidate[start : end + 1])


def read_context(prompt: str) -> dict:
    for line in prompt.splitlines():
        if line.startswith(CONTEXT_PREFIX):
            return json.loads(line[len(CONTEXT_PREFIX) :])
    raise ValueError("prompt has no CONTEXT_JSON line")


@dataclass
class MockLLM:
    """Deterministic stand-in for an LLM. It does not reason; it follows the prompt.

    Without a focus family it returns four scenario families placed between the start
    and the first goals. With a focus family it sweeps one parameter of that family's
    worst case. `scripted` replies, when given, are returned first, in order, which is
    how tests exercise malformed or invalid responses.
    """

    scripted: list[str] = field(default_factory=list)
    sweep_points: int = 6
    calls: list[list[Message]] = field(default_factory=list)

    def complete(self, messages: list[Message]) -> str:
        self.calls.append([dict(m) for m in messages])
        if self.scripted:
            return self.scripted.pop(0)
        ctx = read_context(messages[-1]["content"])
        reply = self._sweep(ctx) if ctx.get("focus_family") else self._explore(ctx)
        return json.dumps(reply)

    @staticmethod
    def _midpoint(ctx: dict, goal_id: str) -> tuple[float, float]:
        gx, gy, _ = ctx["goals"][goal_id]
        sx, sy, _ = ctx["default_start"]
        return (round((sx + gx) / 2, 3), round((sy + gy) / 2, 3))

    def _explore(self, ctx: dict) -> dict:
        goal_ids = sorted(ctx["goals"])
        per_family = max(1, ctx["max_scenarios"] // 4)

        def scenarios(family: str, build) -> dict:
            out = []
            for i in range(per_family):
                gid = goal_ids[i % len(goal_ids)]
                s = {
                    "scenario_id": f"{family}/{i:02d}",
                    "family": family,
                    "goal_id": gid,
                    "seed": 1000 + i,
                    **build(*self._midpoint(ctx, gid), ctx["goals"][gid]),
                }
                out.append(s)
            return out

        def box(name, x, y, sx, sy, h):
            return {
                "name": name,
                "asset": "box",
                "pose": {"x": x, "y": y, "yaw": 0.0},
                "size_x": sx,
                "size_y": sy,
                "height": h,
            }

        families = [
            (
                "pallet_half_block",
                "A pallet stack covering half the corridor forces a tight detour.",
                lambda x, y, g: {"obstacles": [box("pallet", x, y + 0.4, 1.2, 1.0, 0.6)]},
            ),
            (
                "lidar_degradation",
                "Range noise and dropouts corrupt the costmap and localization.",
                lambda x, y, g: {"sensor_noise": {"gaussian_std_m": 0.08, "dropout_prob": 0.3}},
            ),
            (
                "crossing_obstacle",
                "An obstacle crossing the path is seen late and forces stops or contact.",
                lambda x, y, g: {
                    "obstacles": [
                        {
                            "name": "mover",
                            "asset": "cylinder",
                            "pose": {"x": x, "y": y - 2.0, "yaw": 0.0},
                            "size_x": 0.5,
                            "size_y": 0.5,
                            "height": 1.0,
                            "motion": {"end_x": x, "end_y": y + 2.0, "speed_mps": 0.6},
                        }
                    ]
                },
            ),
            (
                "low_obstacle",
                "An obstacle below the sensors' marking height is invisible to the planner.",
                lambda x, y, g: {"obstacles": [box("low_box", x, y, 0.6, 0.6, 0.08)]},
            ),
        ]
        return {
            "families": [
                {"family": f, "hypothesis": h, "scenarios": scenarios(f, b)}
                for f, h, b in families
            ],
            "sweep": None,
        }

    def _sweep(self, ctx: dict) -> dict:
        focus = ctx["focus_family"]
        template = dict(ctx["focus_examples"][0])
        template.pop("config_hash", None)
        n = max(2, min(self.sweep_points, ctx["max_scenarios"]))
        if template.get("obstacles"):
            parameter, lo, hi = "obstacles[0].height", 0.04, 0.6
        else:
            parameter, lo, hi = "sensor_noise.gaussian_std_m", 0.0, 0.3
        values = [round(lo + (hi - lo) * i / (n - 1), 4) for i in range(n)]
        scenarios = []
        for i, v in enumerate(values):
            s = json.loads(json.dumps(template))
            s["scenario_id"] = f"{focus}/sweep_{i:02d}"
            s["family"] = focus
            if parameter.startswith("obstacles"):
                s["obstacles"][0]["height"] = v
            else:
                s.setdefault("sensor_noise", {})["gaussian_std_m"] = v
            scenarios.append(s)
        return {
            "families": [
                {
                    "family": focus,
                    "hypothesis": f"{focus}: success depends on {parameter}; find the flip.",
                    "scenarios": scenarios,
                }
            ],
            "sweep": {"family": focus, "parameter": parameter, "values": values},
        }
