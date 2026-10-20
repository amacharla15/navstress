"""Static charts (PNG), one light and one dark rendering of each.

Colors are the validated reference palette (light and dark steps chosen separately,
checked with the dataviz palette validator): slot 1 blue for a single series or
"before", slot 2 orange for "after". Text always uses ink tokens, never series color.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: CI and the pod have no display

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import PercentFormatter  # noqa: E402


@dataclass(frozen=True)
class Theme:
    name: str
    surface: str
    ink: str
    ink_secondary: str
    muted: str
    grid: str
    baseline: str
    series: tuple[str, str]


LIGHT = Theme(
    "light", "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7",
    ("#2a78d6", "#eb6834"),
)  # fmt: skip
DARK = Theme(
    "dark", "#1a1a19", "#f0efec", "#c3c2b7", "#898781", "#2c2c2a", "#383835",
    ("#3987e5", "#d95926"),
)  # fmt: skip
THEMES = (LIGHT, DARK)

_DPI = 144
# Bundled with matplotlib, so charts render the same on CI, the pod and laptops.
_FONT = ["DejaVu Sans"]


def _axes(theme: Theme, width_in: float, height_in: float):
    plt.rcParams["font.family"] = _FONT
    fig, ax = plt.subplots(figsize=(width_in, height_in), dpi=_DPI)
    fig.patch.set_facecolor(theme.surface)
    ax.set_facecolor(theme.surface)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(theme.baseline)
        ax.spines[side].set_linewidth(1)
    ax.tick_params(colors=theme.muted, labelcolor=theme.ink_secondary, labelsize=9)
    ax.grid(color=theme.grid, linewidth=0.7, linestyle="-")
    ax.set_axisbelow(True)
    return fig, ax


def _title(ax, theme: Theme, title: str, subtitle: str = "") -> None:
    ax.set_title(title, loc="left", color=theme.ink, fontsize=11, fontweight="semibold", pad=18)
    if subtitle:
        ax.text(0, 1.02, subtitle, transform=ax.transAxes, color=theme.ink_secondary, fontsize=8.5)


def _save(fig, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, facecolor=fig.get_facecolor())
    plt.close(fig)


def _percent_axis(ax, theme: Theme, room_for_labels: float, start: float = 0.0) -> None:
    # Ticks and baseline stop at 100%; the extra x-range is only room for end labels.
    ax.set_xlim(start, 1.0 + room_for_labels)
    ax.set_xticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.xaxis.set_major_formatter(PercentFormatter(1.0, decimals=0))
    ax.spines["bottom"].set_bounds(0, 1)
    ax.grid(axis="y", visible=False)


def _paths(out_dir: Path, stem: str) -> dict[str, Path]:
    return {t.name: out_dir / (f"{stem}.png" if t.name == "light" else f"{stem}-dark.png")
            for t in THEMES}  # fmt: skip


def success_by_family(families: list[dict], out_dir: Path, stem: str, subtitle: str) -> dict:
    """Horizontal bars, one per family, sorted worst first. A single series: no legend."""
    fams = sorted(families, key=lambda f: (f["success_rate"], f["family"]))
    paths = _paths(out_dir, stem)
    for theme in THEMES:
        fig, ax = _axes(theme, 7.0, 0.45 * len(fams) + 1.4)
        ys = range(len(fams))
        rates = [f["success_rate"] for f in fams]
        # 0.35 of a 0.45 in row at 144 dpi keeps bars under the 24 px mark spec
        ax.barh(ys, rates, height=0.35, color=theme.series[0])
        ax.set_yticks(list(ys), [f["family"] for f in fams])
        _percent_axis(ax, theme, 0.2)
        for y, f in zip(ys, fams, strict=True):
            ax.text(
                f["success_rate"] + 0.015,
                y,
                f"{f['success_rate']:.0%}  ({f['successes']}/{f['episodes']})",
                va="center",
                color=theme.ink_secondary,
                fontsize=8.5,
            )
        _title(ax, theme, "Success rate by scenario family", subtitle)
        _save(fig, paths[theme.name])
    return paths


def sweep(curve: list[dict], parameter: str, boundary: dict | None, out_dir: Path, stem: str,
          subtitle: str) -> dict:  # fmt: skip
    """Success rate against the swept value; the bracketing pair is shaded, not a fitted line."""
    paths = _paths(out_dir, stem)
    xs = [p["value"] for p in curve]
    ys = [p["success_rate"] for p in curve]
    for theme in THEMES:
        fig, ax = _axes(theme, 7.0, 3.4)
        if boundary and boundary.get("found"):
            lo, hi = boundary["between"]
            ax.axvspan(lo, hi, color=theme.series[0], alpha=0.10, linewidth=0)
            ax.text(
                (lo + hi) / 2, 1.06, "crossing", ha="center", color=theme.ink_secondary, fontsize=8
            )
        ax.plot(xs, ys, color=theme.series[0], linewidth=2, solid_capstyle="round",
                solid_joinstyle="round", marker="o", markersize=7,
                markeredgecolor=theme.surface, markeredgewidth=2)  # fmt: skip
        ax.set_ylim(-0.05, 1.12)
        ax.yaxis.set_major_formatter(PercentFormatter(1.0, decimals=0))
        ax.set_xlabel(parameter, color=theme.ink_secondary, fontsize=9)
        _title(ax, theme, "Boundary sweep: success rate vs swept value", subtitle)
        _save(fig, paths[theme.name])
    return paths


def before_after(families: list[dict], out_dir: Path, stem: str, subtitle: str) -> dict:
    """Dumbbell per family: before (slot 1) and after (slot 2), with a legend."""
    fams = [f for f in families if f["before_success_rate"] is not None]
    fams.sort(key=lambda f: f["family"], reverse=True)
    paths = _paths(out_dir, stem)
    for theme in THEMES:
        fig, ax = _axes(theme, 7.0, 0.5 * len(fams) + 1.6)
        for y, f in enumerate(fams):
            b, a = f["before_success_rate"], f["after_success_rate"]
            ax.plot([b, a], [y, y], color=theme.baseline, linewidth=2, zorder=1)
            # "before" is a ring and "after" a dot, so equal values still show both marks
            # and identity never rests on color alone.
            ax.scatter(
                [b],
                [y],
                s=150,
                facecolors="none",
                edgecolors=theme.series[0],
                linewidths=2,
                zorder=2,
                label="before" if y == 0 else None,
            )
            ax.scatter(
                [a],
                [y],
                s=50,
                color=theme.series[1],
                edgecolors=theme.surface,
                linewidths=1.5,
                zorder=3,
                label="after" if y == 0 else None,
            )
            ax.text(
                max(a, b) + 0.04,
                y,
                f"{b:.0%} → {a:.0%}",
                va="center",
                color=theme.ink_secondary,
                fontsize=8.5,
            )
        ax.set_yticks(range(len(fams)), [f["family"] for f in fams])
        ax.set_ylim(-0.6, len(fams) - 0.4)
        _percent_axis(ax, theme, 0.25, start=-0.03)  # room for the 0% ring
        leg = ax.legend(
            loc="lower right",
            bbox_to_anchor=(1.0, 1.0),
            ncol=2,
            frameon=False,
            fontsize=8.5,
            borderaxespad=0.2,
        )
        for t in leg.get_texts():
            t.set_color(theme.ink_secondary)
        _title(ax, theme, "Success rate before and after the fix (same scenarios)", subtitle)
        _save(fig, paths[theme.name])
    return paths
