"""EpisodeResult message <-> results row dict."""

from __future__ import annotations

from navstress_eval.results_io import RESULT_FIELDS
from navstress_interfaces.msg import EpisodeResult


def result_to_row(msg: EpisodeResult) -> dict:
    return {f: getattr(msg, f) for f in RESULT_FIELDS}


def row_to_result(row: dict) -> EpisodeResult:
    msg = EpisodeResult()
    for f in RESULT_FIELDS:
        setattr(msg, f, row[f])
    return msg
