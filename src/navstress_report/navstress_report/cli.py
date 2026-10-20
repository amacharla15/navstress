"""navstress-report: results/<run_id>/ directories -> docs/RESULTS.md section + site/."""

from __future__ import annotations

import argparse
from pathlib import Path

from navstress_report.compare import compare
from navstress_report.data import load_agent, load_run
from navstress_report.render import ReportBuilder, to_html, to_markdown, write_markdown


def build(
    runs: list[Path],
    agents: list[Path],
    before: Path | None,
    after: Path | None,
    out_md: Path,
    assets_dir: Path,
    site_dir: Path | None,
    title: str,
    videos: list[tuple[Path, str]] | None = None,
) -> None:
    loaded = [load_run(r) for r in runs]
    agent_runs = [load_agent(a) for a in agents]
    pair = (load_run(before), load_run(after)) if before and after else None
    every_run = loaded + [b for a in agent_runs for b in a.batches.values()]
    if pair:
        every_run += list(pair)

    rb = ReportBuilder(out_md, assets_dir)
    rb.header(title, every_run)
    for run in loaded:
        rb.run_section(run)
    for agent in agent_runs:
        rb.agent_section(agent)
    if pair:
        rb.comparison_section(compare(*pair))
    rb.videos(videos or [])

    write_markdown(out_md, to_markdown(rb.blocks, out_md))
    if site_dir is not None:
        site_dir.mkdir(parents=True, exist_ok=True)
        (site_dir / "index.html").write_text(to_html(rb.blocks, site_dir, title))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", action="append", default=[], type=Path, help="results/<run_id>")
    ap.add_argument("--agent", action="append", default=[], type=Path, help="agent run dir")
    ap.add_argument("--before", type=Path, help="run dir before the fix")
    ap.add_argument("--after", type=Path, help="run dir after the fix (same scenarios)")
    ap.add_argument("--out-md", type=Path, default=Path("docs/RESULTS.md"))
    ap.add_argument("--assets-dir", type=Path, help="chart PNGs (default: <out-md dir>/img)")
    ap.add_argument("--site", type=Path, help="also write a static site here (e.g. site/)")
    ap.add_argument("--title", default="NavStress results")
    ap.add_argument("--video", nargs=2, action="append", default=[], metavar=("MP4", "CAPTION"))
    args = ap.parse_args(argv)
    if bool(args.before) != bool(args.after):
        ap.error("--before and --after go together")
    if not (args.run or args.agent or args.before):
        ap.error("nothing to report: pass --run, --agent or --before/--after")
    build(
        args.run,
        args.agent,
        args.before,
        args.after,
        args.out_md,
        args.assets_dir or args.out_md.parent / "img",
        args.site,
        args.title,
        [(Path(p), c) for p, c in args.video],
    )
    print(f"wrote {args.out_md}" + (f" and {args.site}/index.html" if args.site else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
