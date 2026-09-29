"""SteamVR Tracking Enhancement, report tool: diagnoses lighthouse tracking from SteamVR's own logs and config plus the
driver's telemetry, and shows the driver's live status.

  python svrenhance.py report [--html report.html]   one-off diagnosis (default)
  python svrenhance.py live                          live driver status, refreshed every second

Standard library only. Reads files; never writes anywhere except the --html path you give it.
"""
import argparse
import csv
import html
import json
import os
import re
import statistics
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime, timedelta
from pathlib import Path

def steamvr_dirs():
    """SteamVR's log and config folders, from openvrpaths.vrpath; the default Steam folder otherwise."""
    steam = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Steam"
    logs, config = steam / "logs", steam / "config"
    try:
        paths = json.loads((Path(os.environ["LOCALAPPDATA"]) / "openvr" / "openvrpaths.vrpath").read_text(encoding="utf-8"))
        if paths.get("log"):
            logs = Path(paths["log"][0])
        if paths.get("config"):
            config = Path(paths["config"][0])
    except (KeyError, OSError, ValueError, IndexError, TypeError):
        pass
    return logs, config


LOGS, CONFIG = steamvr_dirs()
LHDB = CONFIG / "lighthouse" / "lighthousedb.json"
TELEMETRY = Path(os.environ.get("LOCALAPPDATA", ".")) / "SVREnhance"

LINE = re.compile(r"^\w{3} (\w{3} \d{1,2} \d{4} \d{2}:\d{2}:\d{2}\.\d{3}) \[\w+\] - (.*)$")
DROPPED = re.compile(r"lighthouse: (LHR-[0-9A-F]+) (\w): Dropped (.*) during the previous tracking session")
COUNTS = re.compile(r"(\d+) ([a-z\- ]+?)(?:,|$)")
DEV_MSG = re.compile(r"lighthouse: (LHR-[0-9A-F]+) (\w): (.*)$")
RX_CONN = re.compile(r"lighthouse: (LHR-[0-9A-F]+): (Connected to|Disconnected from) receiver (\S+)")
STATION = re.compile(r"finished adding tracked device with serial number '(LHB-[0-9A-F]+)'")


def parse_logs():
    """Collects per-device lighthouse diagnostics from vrserver.previous.txt then vrserver.txt."""
    dev = defaultdict(lambda: {"kind": "?", "sessions": [], "imu_off_scale": 0, "sync_asserts": 0,
                               "imu_panics": 0, "connects": [], "disconnects": []})
    stations, files, span = set(), [], [None, None]
    for name in ("vrserver.previous.txt", "vrserver.txt"):
        path = LOGS / name
        if not path.exists():
            continue
        files.append(str(path))
        with open(path, encoding="utf-8", errors="replace") as f:
            for raw in f:
                m = LINE.match(raw.rstrip("\n"))
                if not m:
                    continue
                try:
                    ts = datetime.strptime(m.group(1), "%b %d %Y %H:%M:%S.%f")
                except ValueError:
                    continue
                span[0] = span[0] or ts
                span[1] = ts
                msg = m.group(2)
                if (s := STATION.search(msg)):
                    stations.add(s.group(1))
                    continue
                if (d := DROPPED.search(msg)):
                    counts = {k.strip(): int(n) for n, k in COUNTS.findall(d.group(3))}
                    dev[d.group(1)]["kind"] = d.group(2)
                    dev[d.group(1)]["sessions"].append((ts, counts))
                    continue
                if (c := RX_CONN.search(msg)):
                    key = "connects" if c.group(2) == "Connected to" else "disconnects"
                    dev[c.group(1)][key].append((ts, c.group(3)))
                    continue
                if (g := DEV_MSG.search(msg)):
                    serial, kind, text = g.groups()
                    dev[serial]["kind"] = kind
                    if "IMU went off scale" in text:
                        dev[serial]["imu_off_scale"] += 1
                    elif "Explicit right sync does not match pending frame" in text:
                        dev[serial]["sync_asserts"] += 1
                    elif "early IMU related panics" in text:
                        dev[serial]["imu_panics"] += 1
    return dev, sorted(stations), files, span


def parse_lighthousedb():
    if not LHDB.exists():
        return []
    db = json.loads(LHDB.read_text(encoding="utf-8"))
    out = []
    for b in db.get("base_stations", []):
        cfg = b.get("config", {})
        states = b.get("dynamic_states") or [{}]
        st = states[-1].get("dynamic_state", {})
        out.append({
            "serial": "LHB-%08X" % (cfg.get("serialNumber", 0) & 0xFFFFFFFF),
            "model": cfg.get("modelId"),
            "channel": st.get("sobChannel"),
            "mode": st.get("basestation_mode"),
            "faults": st.get("faults"),
            "firmware": st.get("firmware_version"),
            "resets": st.get("reset_count"),
        })
    return out


def read_status():
    p = TELEMETRY / "status.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def read_csv(name):
    p = TELEMETRY / name
    if not p.exists():
        return []
    with open(p, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def analyze():
    findings, advice = [], []
    dev, seen_stations, files, span = parse_logs()
    stations = parse_lighthousedb()
    status = read_status()
    events = read_csv("events.csv")
    coverage = read_csv("coverage.csv")

    # ---- reflections, as counted by SteamVR's own lighthouse driver ----
    refl_rows = []
    for serial, d in sorted(dev.items()):
        if not d["sessions"]:
            continue
        tot = Counter()
        for _, c in d["sessions"]:
            tot.update(c)
        worst = max(d["sessions"], key=lambda s: s[1].get("rejected updates", 0) + s[1].get("back-facing hits", 0))
        refl_rows.append((serial, d["kind"], len(d["sessions"]), tot, worst))
    heavy = [r for r in refl_rows if r[3].get("rejected updates", 0) > 1000 or r[3].get("back-facing hits", 0) > 50000]
    if heavy:
        findings.append(("high", "Reflections",
                         "SteamVR's lighthouse driver is discarding large numbers of reflected hits: " +
                         "; ".join(f"{s} ({k}): {t.get('back-facing hits', 0):,} back-facing hits, "
                                   f"{t.get('rejected updates', 0):,} rejected pose updates over {n} sessions"
                                   for s, k, n, t, _ in heavy) +
                         ". Back-facing hits are laser sweeps arriving at sensors that face away from every "
                         "station, i.e. light bounced off something. Rejected updates are whole optical solves "
                         "SteamVR threw out."))
        advice.append("Cover or angle away mirrors, windows, glossy screens/TVs, glass tables and shiny floors "
                      "in the stations' view. The events map (--html) shows where glitches cluster.")

    # ---- sync ----
    sync = {s: d["sync_asserts"] for s, d in dev.items() if d["sync_asserts"]}
    if sync:
        findings.append(("medium", "Sync decoding",
                         "Lighthouse sync decode assertions ('Explicit right sync does not match pending frame'): " +
                         ", ".join(f"{s} x{n}" for s, n in sorted(sync.items(), key=lambda x: -x[1])) +
                         ". The device received sync data it could not match to a sweep; interference and "
                         "reflections are the usual causes."))

    # ---- base stations ----
    if stations:
        chans = Counter(s["channel"] for s in stations if s["channel"] is not None)
        dup = [c for c, n in chans.items() if n > 1]
        if dup:
            findings.append(("high", "Base station channels",
                             "Stations share a channel (sobChannel " + ", ".join(map(str, dup)) +
                             "): they will interfere. Give every station a unique channel."))
        else:
            findings.append(("ok", "Base station channels",
                             "All %d stations are on distinct channels (sobChannel %s)." %
                             (len(stations), ", ".join(str(s["channel"]) for s in stations))))
        resets = [s["resets"] for s in stations if isinstance(s["resets"], int)]
        if resets:
            med = statistics.median(resets)
            for s in stations:
                if isinstance(s["resets"], int) and s["resets"] >= 5 and s["resets"] > 3 * max(med, 1):
                    findings.append(("high", "Base station stability",
                                     f"{s['serial']} reset counter is {s['resets']} while the median is {med:g}: "
                                     "this station has restarted far more often than the others. Check its "
                                     "power adapter and cable, and that its mount is rigid."))
        for s in stations:
            if s["faults"]:
                findings.append(("high", "Base station fault", f"{s['serial']} reports faults={s['faults']}."))
        fws = {s["firmware"] for s in stations}
        if len(fws) > 1:
            findings.append(("low", "Base station firmware", "Stations run different firmware: " +
                             ", ".join(f"{s['serial']}={s['firmware']}" for s in stations)))

    # ---- connection stability ----
    for serial, d in sorted(dev.items()):
        n_disc = len(d["disconnects"])
        if n_disc >= 3 and span[0] and span[1]:
            hours = max((span[1] - span[0]).total_seconds() / 3600, 1e-6)
            conns = sorted(d["connects"])
            lens = []
            for t_on, _ in conns:
                off = [t for t, _ in d["disconnects"] if t > t_on]
                if off:
                    lens.append((min(off) - t_on).total_seconds() / 60)
            rx = ", ".join(sorted({r for _, r in d["disconnects"]}))
            findings.append(("medium", "Wireless link",
                             f"{serial} dropped its receiver link {n_disc} times in {hours:.1f} h of logs "
                             f"(receiver {rx})" + (f", median session {statistics.median(lens):.1f} min" if lens else "") +
                             ". Some of these are you switching it off; frequent short sessions are not."))
    if any(len(d["disconnects"]) >= 3 for d in dev.values()):
        advice.append("Put the controller/tracker dongles on a USB extension with line of sight to the play "
                      "area, away from USB 3 ports and Wi-Fi routers, which radiate in the same 2.4 GHz band.")
    imu = {s: d["imu_off_scale"] for s, d in dev.items() if d["imu_off_scale"]}
    if imu:
        findings.append(("low", "IMU saturation",
                         "IMU went off scale: " + ", ".join(f"{s} x{n}" for s, n in imu.items()) +
                         ". This happens on very fast swings; SteamVR recovers optically, which is exactly the "
                         "kind of correction the driver eases in instead of snapping."))

    # ---- driver telemetry ----
    if status:
        hook = status.get("hook", {})
        if not hook.get("installed"):
            findings.append(("high", "Driver", "Driver is loaded but its pose hook is not installed."))
        elif hook.get("effective") is False:
            findings.append(("high", "Driver",
                             "The pose hook is installed but no poses pass through it: filtering is NOT active."))
        elif not hook.get("calls"):
            findings.append(("info", "Driver",
                             f"Hook installed, no poses yet (status from {status.get('time')}, uptime {status.get('uptime_s', 0):.0f} s)."))
        else:
            findings.append(("ok", "Driver", f"Pose hook active ({hook.get('calls', 0):,} poses processed)."))
        for w in status.get("warnings", []):
            findings.append(("high", "Live warning", w))
        for d in status.get("devices", []):
            if d.get("jitter_mm", 0) > 1.0:
                findings.append(("medium", "Jitter", f"{d['serial']} ({d['class']}) jitters {d['jitter_mm']:.2f} mm "
                                 "while still. Poor station coverage of that spot or reflections are likely."))
    else:
        findings.append(("info", "Driver",
                         f"No live telemetry at {TELEMETRY} (driver not installed, or SteamVR not running)."))

    # Hotspots over the last 12 h, ranked by glitches per presence sample so that places where the
    # hands merely spend a lot of time do not dominate.
    hotspots = []
    since = (datetime.now() - timedelta(hours=12)).strftime("%Y-%m-%dT%H:%M:%S")
    events = [e for e in events if e.get("time", "") >= since]
    coverage = [c for c in coverage if c.get("time", "") >= since]
    if events:
        kinds = Counter(e["event"] for e in events)
        findings.append(("info", "Driver events (last 12 h)", ", ".join(f"{k}: {n}" for k, n in kinds.most_common())))
        cell = 0.5

        def key(row):
            return (round(float(row["x"]) / cell), round(float(row["z"]) / cell))

        g, presence = Counter(), Counter()
        for e in events:
            if e["event"] in ("glitch_rejected", "relocation_eased"):
                try:
                    g[key(e)] += 1
                except (KeyError, ValueError):
                    pass
        for c in coverage:
            try:
                presence[key(c)] += 1
            except (KeyError, ValueError):
                pass
        scored = sorted(((k, n, presence.get(k, 0), n / max(presence.get(k, 0), 1)) for k, n in g.items() if n >= 5),
                        key=lambda s: -s[3])
        hotspots = [((k[0] * cell, k[1] * cell), n) for k, n, _, _ in scored[:5]]
        if scored:
            findings.append(("medium", "Glitch hotspots",
                             "Highest glitch rate per time present (x, z, raw tracking space): " +
                             ", ".join(f"({k[0] * cell:+.1f}, {k[1] * cell:+.1f}) m: {n} glitches / {p} presence samples"
                                       for k, n, p, _ in scored[:5]) +
                             ". Look for reflective surfaces in line of sight from those spots."))

    return {"findings": findings, "advice": advice, "refl": refl_rows, "stations": stations,
            "logged_stations": seen_stations, "files": files, "span": span, "status": status,
            "events": events, "coverage": coverage, "hotspots": hotspots}


def print_report(r):
    title = "SteamVR Tracking Enhancement - tracking report"
    print(title)
    print("=" * len(title))
    if r["span"][0]:
        print(f"Logs: {', '.join(r['files'])}\n      {r['span'][0]} .. {r['span'][1]}")
    print()
    if r["stations"]:
        print("Base stations (lighthousedb.json)")
        for s in r["stations"]:
            print(f"  {s['serial']}  channel(sobChannel)={s['channel']}  firmware={s['firmware']}  "
                  f"resets={s['resets']}  faults={s['faults']}")
        print()
    if r["refl"]:
        print("Reflection rejects per device (from SteamVR's lighthouse driver)")
        for serial, kind, n, tot, worst in r["refl"]:
            print(f"  {serial} ({kind})  sessions={n}  back-facing={tot.get('back-facing hits', 0):,}  "
                  f"rejected-updates={tot.get('rejected updates', 0):,}  non-clustered={tot.get('non-clustered hits', 0):,}")
        print()
    order = {"high": 0, "medium": 1, "low": 2, "info": 3, "ok": 4}
    print("Findings")
    for sev, title, text in sorted(r["findings"], key=lambda f: order.get(f[0], 9)):
        print(f"  [{sev.upper():6}] {title}: {text}")
    if r["advice"]:
        print("\nWhat to do")
        for a in r["advice"]:
            print(f"  - {a}")


def write_html(r, path):
    pts = []
    for c in r["coverage"]:
        try:
            pts.append((float(c["x"]), float(c["z"])))
        except (KeyError, ValueError):
            pass
    evs = []
    colors = {"glitch_rejected": "#d6453d", "relocation_eased": "#e8912d", "dropout_bridge_start": "#3b7dd8",
              "dropout_too_long": "#7b3fd0", "update_gap": "#8a8a8a"}
    for e in r["events"]:
        if e.get("event") in colors:
            try:
                evs.append((float(e["x"]), float(e["z"]), e["event"]))
            except (KeyError, ValueError):
                pass
    stations = []
    if r["status"]:
        stations = [(s["pos"][0], s["pos"][2], s["serial"]) for s in r["status"].get("stations", []) if s.get("pos")]
    allx = [p[0] for p in pts] + [e[0] for e in evs] + [s[0] for s in stations] or [-2, 2]
    allz = [p[1] for p in pts] + [e[1] for e in evs] + [s[1] for s in stations] or [-2, 2]
    x0, x1, z0, z1 = min(allx) - 0.3, max(allx) + 0.3, min(allz) - 0.3, max(allz) + 0.3
    scale = 560 / max(x1 - x0, z1 - z0, 0.5)

    def sx(x): return 20 + (x - x0) * scale

    def sz(z): return 20 + (z - z0) * scale

    svg = [f'<svg viewBox="0 0 600 600" role="img" aria-label="Top-down map of the play space">',
           '<rect x="0" y="0" width="600" height="600" fill="var(--panel)"/>']
    svg += [f'<circle cx="{sx(x):.1f}" cy="{sz(z):.1f}" r="1.6" fill="var(--muted)" opacity="0.35"/>' for x, z in pts]
    svg += [f'<circle cx="{sx(x):.1f}" cy="{sz(z):.1f}" r="4" fill="{colors[k]}" opacity="0.8"><title>{k}</title></circle>'
            for x, z, k in evs]
    svg += [f'<path d="M{sx(x):.1f},{sz(z) - 8:.1f} l7,13 h-14z" fill="var(--fg)"><title>{html.escape(n)}</title></path>'
            for x, z, n in stations]
    svg.append("</svg>")

    order = {"high": 0, "medium": 1, "low": 2, "info": 3, "ok": 4}
    rows = "".join(f'<li class="{s}"><b>{html.escape(t)}</b> {html.escape(x)}</li>'
                   for s, t, x in sorted(r["findings"], key=lambda f: order.get(f[0], 9)))
    adv = "".join(f"<li>{html.escape(a)}</li>" for a in r["advice"])
    legend = " ".join(f'<span><i style="background:{c}"></i>{k.replace("_", " ")}</span>' for k, c in colors.items())
    doc = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Tracking Report</title>
<style>
:root{{--bg:#f7f7f5;--panel:#fff;--fg:#1d1d1b;--muted:#6b6b66;--line:#dcdcd6}}
@media (prefers-color-scheme:dark){{:root{{--bg:#161615;--panel:#1f1f1d;--fg:#ececea;--muted:#9a9a94;--line:#33332f}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0;padding:24px 16px}}
main{{max-width:900px;margin:0 auto}} svg{{width:100%;max-width:600px;border:1px solid var(--line);border-radius:8px}}
li{{margin:6px 0}} li.high b{{color:#d6453d}} li.medium b{{color:#e8912d}} li.ok b{{color:#2f9a58}}
.legend span{{margin-right:14px;font-size:13px;color:var(--muted)}} .legend i{{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px}}
</style></head><body><main>
<h1>Tracking report</h1><p style="color:var(--muted)">SteamVR Tracking Enhancement &middot; generated {datetime.now():%Y-%m-%d %H:%M}</p>
<h2>Findings</h2><ul>{rows}</ul>{f"<h2>What to do</h2><ul>{adv}</ul>" if adv else ""}
<h2>Play-space map (top-down, x/z)</h2><p class="legend">{legend} <span>grey = where devices have been, triangles = base stations</span></p>
{"".join(svg)}
</main></body></html>"""
    Path(path).write_text(doc, encoding="utf-8")
    print(f"\nWrote {path}")


def live():
    try:
        while True:
            s = read_status()
            os.system("cls" if os.name == "nt" else "clear")
            if not s:
                print(f"Waiting for {TELEMETRY / 'status.json'} (is SteamVR running with the driver installed?)")
            else:
                h = s.get("hook", {})
                print(f"SteamVR Tracking Enhancement {s.get('version')}  {s.get('time')}  hook installed={h.get('installed')} "
                      f"effective={h.get('effective')} poses={h.get('calls', 0):,}")
                print(f"\n{'serial':14} {'class':10} {'state':22} {'Hz':>6} {'glitch':>6} {'reloc':>5} {'bridge':>6} "
                      f"{'>max':>4} {'gaps':>4} {'disc':>4} {'jitter':>7}")
                for d in s.get("devices", []):
                    print(f"{d['serial'][:14]:14} {d['class'][:10]:10} {d['state'][:22]:22} {d['rate_hz']:6.0f} "
                          f"{d['glitches_rejected']:6} {d['relocations_eased']:5} {d['dropouts_bridged']:6} "
                          f"{d['dropouts_too_long']:4} {d['gaps_50ms']:4} {d['disconnects']:4} {d['jitter_mm']:6.2f}mm")
                print(f"\n{'station':14} {'channel':8} {'valid':6} {'drift':>9} {'resolves':>8} {'moved':>5} {'lost':>4}")
                for st in s.get("stations", []):
                    print(f"{st['serial'][:14]:14} {st['mode_label'][:8]:8} {str(st['valid']):6} "
                          f"{st['drift_mm']:7.2f}mm {st['pose_changes']:8} {st['move_events']:5} {st['lost_events']:4}")
                for w in s.get("warnings", []):
                    print(f"\n! {w}")
            print("\nCtrl+C to quit")
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", default="report", choices=["report", "live"])
    ap.add_argument("--html", help="also write an HTML report with a play-space map to this path")
    a = ap.parse_args()
    if a.command == "live":
        live()
        return 0
    r = analyze()
    print_report(r)
    if a.html:
        write_html(r, a.html)
    return 0


if __name__ == "__main__":
    sys.exit(main())
