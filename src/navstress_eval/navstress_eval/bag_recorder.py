"""One rosbag2 recording per episode, via the `ros2 bag record` CLI.

A subprocess keeps recording off the runner's executor threads and its GIL, and the
CLI is the documented, stable interface. MCAP is the storage so Foxglove can open
the bags directly.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import yaml


def qos_overrides(best_effort: list[str], latched: list[str]) -> dict:
    """rosbag2 QoS override profiles (format from the rosbag2 README).

    Without them the recorder may subscribe before it has seen a publisher and fall
    back to RELIABLE, which never matches the best-effort sensor publishers Isaac Sim
    uses: those topics are then silently missing from the bag. Latched topics
    (/tf_static, the active scenario) need TRANSIENT_LOCAL to get the one message that
    was published before recording started.
    """
    out = {}
    for t in (t for t in best_effort if t):
        out[t] = {"history": "keep_last", "depth": 100, "reliability": "best_effort",
                  "durability": "volatile"}  # fmt: skip
    for t in (t for t in latched if t):
        out[t] = {"history": "keep_last", "depth": 10, "reliability": "reliable",
                  "durability": "transient_local"}  # fmt: skip
    return out


class BagRecorder:
    def __init__(
        self,
        topics: list[str],
        use_sim_time: bool,
        start_delay_s: float = 1.0,
        best_effort_topics: list[str] | None = None,
        latched_topics: list[str] | None = None,
    ):
        self.topics = [t for t in topics if t]
        self.use_sim_time = use_sim_time
        self.start_delay_s = start_delay_s
        self.overrides = qos_overrides(best_effort_topics or [], latched_topics or [])
        self.proc: subprocess.Popen | None = None
        self.path: Path | None = None
        self._log = None

    def command(self, out: Path, qos_file: Path | None = None) -> list[str]:
        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(out), "--topics", *self.topics]
        cmd += ["--include-unpublished-topics", "--disable-keyboard-controls"]
        if qos_file is not None:
            cmd += ["--qos-profile-overrides-path", str(qos_file)]
        if self.use_sim_time:
            cmd.append("--use-sim-time")
        return cmd

    def start(self, out: Path) -> None:
        if self.proc is not None:
            raise RuntimeError("recorder already running")
        if out.exists():
            raise FileExistsError(f"bag {out} already exists")
        out.parent.mkdir(parents=True, exist_ok=True)
        self.path = out
        qos_file = None
        if self.overrides:
            qos_file = out.parent / f"{out.name}.qos.yaml"
            qos_file.write_text(yaml.safe_dump(self.overrides, sort_keys=True))
        self._log = open(out.parent / f"{out.name}.record.log", "w")  # noqa: SIM115
        # New session so a Ctrl-C to the runner doesn't kill the recorder mid-write.
        self.proc = subprocess.Popen(
            self.command(out, qos_file),
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        # Give the recorder time to discover publishers before the episode starts.
        time.sleep(self.start_delay_s)
        if self.proc.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited early; see {self._log.name}")

    def stop(self, timeout_s: float = 15.0) -> Path | None:
        if self.proc is None:
            return None
        if self.proc.poll() is None:
            # SIGINT lets rosbag2 close the MCAP file properly (writes the summary).
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.proc = None
        if self._log:
            self._log.close()
        path, self.path = self.path, None
        return path
