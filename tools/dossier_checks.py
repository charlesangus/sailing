"""Consistency checks for a boat reference dossier (default data/boats/j80)."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TOLERANCE = 0.03


def load(boat, name):
    with open(boat / name, encoding="utf-8") as f:
        return json.load(f)


def v(entry):
    return entry["value"]


def main_measured_area(p, e, mqw, mhw, mtw, muw, mhb):
    """ORC Rating Systems 2019, rule 109.1: mainsail measured area."""
    return p / 8 * (e + 2 * mqw + 2 * mhw + 1.5 * mtw + muw + 0.5 * mhb)


def headsail_measured_area(hlu, hlp, hhw, hhb, hqw=None, htw=None, huw=None):
    """ORC Rating Systems 2019, rules 111.1 and 111.2 (unmeasured widths)."""
    if hqw is None:
        hqw = 0.75 * hlp + 0.25 * hhb
    if htw is None:
        htw = 0.25 * hlp + 0.75 * hhb
    if huw is None:
        huw = 0.125 * hlp + 0.875 * hhb
    return 0.1125 * hlu * (1.445 * hlp + 2 * hqw + 2 * hhw + 1.5 * htw + huw + 0.5 * hhb)


class Report:
    def __init__(self):
        self.failures = 0

    def compare(self, label, computed, reference, tolerance=TOLERANCE, gating=True):
        rel = (computed - reference) / reference
        ok = abs(rel) <= tolerance
        status = "PASS" if ok else ("FAIL" if gating else "INFO")
        print(f"[{status}] {label}: computed {computed:.3f} vs reference {reference:.3f} ({rel:+.2%}, "
              f"tolerance {tolerance:.0%})")
        if gating and not ok:
            self.failures += 1


def check_sail_areas(rep, boat):
    particulars = load(boat, "particulars.json")
    rig = load(boat, "rig.json")
    sails = load(boat, "sails.json")
    main, jib = sails["main"], sails["jib"]
    p, e = v(rig["p"]), v(rig["e"])

    print("Sail areas (ORC Rating Systems 2019 measured-area formulas)")
    cert_main = v(particulars["main_area"])
    rep.compare("main, certificate girths vs certificate area",
                main_measured_area(p, e, v(main["mqw"]), v(main["mhw"]), v(main["mtw"]), v(main["muw"]), v(main["mhb"])),
                cert_main)
    rep.compare("main, class-maximum girths (MUW from certificate) vs certificate area",
                main_measured_area(p, e, v(main["class_max_mqw"]), v(main["class_max_mhw"]), v(main["class_max_mtw"]),
                                   v(main["muw"]), v(main["class_max_mhb"])),
                cert_main)

    cert_jib = v(particulars["jib_area"])
    rep.compare("jib, certificate widths vs certificate area",
                headsail_measured_area(v(jib["hlu"]), v(jib["hlp"]), v(jib["hhw"]), v(jib["hhb"])), cert_jib)
    rep.compare("jib, class-maximum widths vs certificate area",
                headsail_measured_area(v(jib["hlu"]), v(jib["hlp"]), v(jib["class_max_hhw"]), v(jib["hhb"])), cert_jib)

    tack, head, clew = v(jib["tack"]), v(jib["head"]), v(jib["clew"])
    ax, az = head[0] - tack[0], head[2] - tack[2]
    bx, bz = clew[0] - tack[0], clew[2] - tack[2]
    rep.compare("jib, triangle from provisional-frame tack/head/clew vs certificate area (excludes roach)",
                abs(ax * bz - az * bx) / 2, cert_jib)

    i_m, j_m = v(rig["i_jib"]), v(rig["j"])
    sa100 = v(sails["jboats_sail_area_100"])
    rep.compare(f"J/Boats '100% SA' {sa100:.2f} m^2 against P*E/2 + I*J/2 from certificate rig (informational)",
                p * e / 2 + i_m * j_m / 2, sa100, gating=False)


def check_mass(rep, boat):
    particulars = load(boat, "particulars.json")
    mass = load(boat, "mass.json")
    displacement = v(particulars["displacement"])

    print("Mass (certificate displacement is measurement trim: no crew, no sails)")
    included = {k: e for k, e in mass["items"].items() if e["in_displacement"]}
    for name, entry in included.items():
        print(f"    {name:30s} {v(entry):8.1f} kg  ({entry['confidence']})")
    total = sum(v(e) for e in included.values())
    # hull_deck_fittings is the builder's weight minus the other items, so the
    # sum is close to the builder's weight by construction.
    rep.compare("sum of in-displacement mass items vs certificate displacement (crew not subtracted; "
                "informational, hull_deck_fittings is a residual from the builder's weight)",
                total, displacement, gating=False)
    sourced = sum(v(e) for e in included.values() if e["confidence"] != "estimated")
    print(f"    sourced (non-estimated) items alone: {sourced:.1f} kg")

    bw_min, bw_max = v(mass["builders_weight_min"]), v(mass["builders_weight_max"])
    in_range = bw_min <= displacement <= bw_max
    print(f"[{'PASS' if in_range else 'FAIL'}] certificate displacement {displacement:.0f} kg within class builder's "
          f"weight range {bw_min:.0f}-{bw_max:.0f} kg")
    if not in_range:
        rep.failures += 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boat-dir", type=Path, default=ROOT / "data" / "boats" / "j80",
                    help="dossier directory (default data/boats/j80)")
    args = ap.parse_args(argv)
    rep = Report()
    check_sail_areas(rep, args.boat_dir)
    print()
    check_mass(rep, args.boat_dir)
    print()
    print("all checks passed" if rep.failures == 0 else f"{rep.failures} check(s) failed")
    return 1 if rep.failures else 0


if __name__ == "__main__":
    sys.exit(main())
