"""Prompt text. The schema and goal list come from the validator's own tables."""

from __future__ import annotations

import json
from collections.abc import Mapping

from navstress_agent.llm import CONTEXT_PREFIX, Message
from navstress_eval.summary import FamilySummary, summary_table

SYSTEM_PROMPT = """\
You design test scenarios for a mobile robot (NVIDIA Nova Carter) that navigates a \
warehouse with the ROS 2 Nav2 stack in simulation. Your job is to find conditions \
under which navigation fails, then narrow down where failure starts.

You never control the robot. You only propose scenario configurations. A separate \
system validates them against a schema, runs them, and reports the results to you.

Reply with one JSON object and nothing else:
{"families": [{"family": "<lower_snake_case>", "hypothesis": "<one sentence: what \
should fail and why>", "scenarios": [<scenario>, ...]}],
 "sweep": {"family": "<name>", "parameter": "<path, e.g. obstacles[0].height>", \
"values": [<numbers>]} or null}

The scenario schema (fields, numeric ranges, allowlists), the goal list and the \
robot's default start pose are in the CONTEXT_JSON line of each request. Numbers \
outside a range are clamped. Unknown fields, assets or override names are rejected. \
Choose goals by goal_id. Positions are map-frame metres; yaw is radians."""


def round_messages(
    round_no: int,
    max_rounds: int,
    max_scenarios: int,
    schema: dict,
    goals: Mapping[str, tuple[float, float, float]],
    default_start: tuple[float, float, float],
    history: Mapping[str, FamilySummary],
    focus_family: str | None,
    focus_examples: list[dict],
) -> list[Message]:
    parts = [f"Round {round_no} of {max_rounds}. Propose at most {max_scenarios} scenarios."]
    if focus_family is None:
        parts.append(
            "Propose 3 to 5 distinct scenario families you expect to stress navigation, for "
            "example obstacles partly blocking the path, degraded lidar, moving obstacles, "
            "or obstacles of unusual height. Give each family a hypothesis and several "
            "scenarios (different seeds or small variations). Set sweep to null."
        )
    else:
        parts.append(
            f"The weakest family so far is '{focus_family}'. Every scenario must have family "
            f"'{focus_family}'. Start from its worst case below, keep everything fixed except "
            "ONE parameter, and sweep that parameter across evenly spaced values to find "
            "where success collapses. Fill in sweep with that parameter's path and values."
        )
    if history:
        parts.append("Results so far:\n" + summary_table(history))
    if focus_examples:
        parts.append(f"Worst cases of '{focus_family}':\n" + json.dumps(focus_examples))
    context = {
        "round": round_no,
        "max_scenarios": max_scenarios,
        "focus_family": focus_family,
        "focus_examples": focus_examples,
        "default_start": list(default_start),
        "goals": {k: list(v) for k, v in sorted(goals.items())},
        "schema": schema,
    }
    parts.append(CONTEXT_PREFIX + json.dumps(context, sort_keys=True))
    return [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": "\n\n".join(parts)},
    ]


def retry_message(problem: str) -> Message:
    return {
        "role": "user",
        "content": f"Your previous reply could not be used: {problem}\n"
        "Reply again with one JSON object in the required format.",
    }
