"""Build a local, offline review page from recorded pose-sweep episodes."""

import argparse
import html
import json
from pathlib import Path

from PIL import Image, ImageDraw


def build_review(root):
    report = json.loads((root / "run_report.json").read_text())
    cards, records, images = [], [], []
    for index, case in enumerate(report["cases"]):
        folder = root / f"episode_{index:03d}"
        path = folder / "episode_report.json"
        if not path.is_file():
            continue
        episode = json.loads(path.read_text())
        failures = [k for group in ("checks", "sync_checks")
                    for k, passed in episode.get(group, {}).items() if not passed]
        if not episode.get("api_success"):
            failures.append("api_success")
        lift = episode.get("hold_lift_range_m")
        slip = episode.get("max_relative_slip_m")
        record = {"episode": folder.name, "variation": case, "passed": episode["passed"],
                  "failed_checks": failures, "hold_lift_range_m": lift, "max_relative_slip_m": slip,
                  "max_render_clock_error_s": episode["max_render_clock_error_s"],
                  "hold_finger_contact_fraction": episode.get("hold_finger_contact_fraction")}
        records.append(record)
        name = html.escape(case["name"])
        badge = "PASS" if episode["passed"] else "FAIL"
        detail = (f"Lift {lift[0] * 1000:.2f}–{lift[1] * 1000:.2f} mm; slip {slip * 1000:.3f} mm"
                  if lift is not None else html.escape(episode.get("failure_reason", "Incomplete episode")))
        cards.append(f'''<article><h2>{index:02d} · {name} <b class="{badge}">{badge}</b></h2>
<p>XY {case['offset_xy_m']} m · yaw {case['yaw_deg']}°<br>{detail}</p>
<video controls preload="none" poster="{folder.name}/four_cameras_preview.png"
src="{folder.name}/four_cameras_preview.mp4"></video>
<p>{html.escape(', '.join(failures)) or 'All recorded grasp and synchronization checks passed.'}</p>
<a href="{folder.name}/episode_report.json">Episode report</a></article>''')
        preview = folder / "four_cameras_preview.png"
        if preview.is_file():
            canvas = Image.new("RGB", (480, 480), "white")
            with Image.open(preview) as source:
                canvas.paste(source.resize((480, 450)), (0, 30))
            ImageDraw.Draw(canvas).text((8, 8), f"{index:02d} {case['name']}  {badge}", fill="black")
            images.append(canvas)
    if images:
        sheet = Image.new("RGB", (960, 480 * ((len(images) + 1) // 2)), "#dddddd")
        for index, image in enumerate(images):
            sheet.paste(image, ((index % 2) * 480, (index // 2) * 480))
        sheet.save(root / "sweep_overview.jpg", quality=92)
    passed = sum(record["passed"] for record in records)
    summary = {"status": report["status"], "passed_count": passed, "recorded_count": len(records),
               "planned_count": len(report["cases"]), "episodes": records}
    (root / "sweep_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>Dexmate pose sweep</title><style>
body{{font:16px system-ui;background:#f3f5f8;color:#202734;max-width:1500px;margin:30px auto;padding:0 20px}}
main{{display:grid;grid-template-columns:repeat(auto-fit,minmax(450px,1fr));gap:24px}}
article{{background:white;padding:18px;border-radius:12px}}h2{{font-size:19px}}video{{width:100%}}
b{{float:right}}.PASS{{color:#167345}}.FAIL{{color:#b42318}}p{{line-height:1.6}}
</style><h1>Dexmate：位置与朝向变化</h1>
<p>已记录 {len(records)} / {len(report['cases'])} 轮，{passed} 轮通过。
每轮上方四相机 RGB，下方同一时刻的四片触觉；视频 30 fps，数值数据 120 Hz。
抓取使用已知目标位姿和脚本轨迹。通过范围仅限本次案例。</p>
<p><a href="sweep_summary.json">汇总数据</a> · <a href="sweep_overview.jpg">全部保持阶段缩略图</a></p>
<main>{''.join(cards)}</main></html>'''
    (root / "review.html").write_text(page)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    print(json.dumps(build_review(args.root), indent=2))
