"""
Fit the SCARA forward kinematics from logged joint/pose pairs. Offline only: reads files, touches no hardware.

Data (all pairs of SDK joints [j1..j4] and SDK pose [x,y,z,rx,ry,rz] read within a few ms of each other):
  runs/trace_microscope_to_polar_01.json          884 samples, drag recording of route B          -> FIT
  runs/discarded_trace_polar_to_hotel_01.json    3184 samples, a long discarded drag of route A    -> FIT
  runs/trace_polar_to_hotel_01.json               941 samples, drag recording of route A           -> HELD OUT
  layout.json trajectories (joints + poses, 68 waypoints; smoothed averages, so slightly off FK)   -> HELD OUT
  layout.json fixtures.microscope joints_at_teach / pose (taught 2026-10-19 20:42)                 -> HELD OUT
The run logs' AJ records carry no joints or poses (only dt and dev), so they cannot be used for the FK.

Model (linear least squares after the yaw identity is established):
  x = x0 + A1 c(j2) - B1 s(j2) + A2 c(j2+j3) - B2 s(j2+j3) + At c(rz) - Bt s(rz)
  y = y0 + A1 s(j2) + B1 c(j2) + A2 s(j2+j3) + B2 c(j2+j3) + At s(rz) + Bt c(rz)
  with l_k = hypot(A_k, B_k), a_k = atan2(B_k, A_k);  z = z_scale*j1 + z_offset;  rz = j2 + j3 + j4 + yaw_offset.
Samples recorded while the arm was moving fast carry a read-lag error; one robust pass drops |residual| > 1 mm.
Writes fk_model.json and prints the residual table (copied into REPORT.md).
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)


def load_sets():
    lay = json.load(open(os.path.join(ROOT, "layout.json"), encoding="utf-8"))
    sets = {}
    for n in ["trace_microscope_to_polar_01", "discarded_trace_polar_to_hotel_01", "trace_polar_to_hotel_01"]:
        d = json.load(open(os.path.join(ROOT, "runs", n + ".json"), encoding="utf-8"))
        sets[n] = (np.asarray(d["joints"], float)[:, :4], np.asarray(d["poses"], float))
    for n, tr in lay["trajectories"].items():
        sets["layout." + n] = (np.asarray(tr["joints"], float)[:, :4], np.asarray(tr["poses"], float))
    mic = lay["fixtures"]["microscope"]
    sets["layout.microscope.joints_at_teach"] = (np.asarray([mic["joints_at_teach"]], float), np.asarray([mic["pose"]], float))
    return sets


def design(J, sign=1.0):
    t1 = np.deg2rad(sign * J[:, 1]); t12 = np.deg2rad(sign * (J[:, 1] + J[:, 2])); ty = np.deg2rad(sign * J[:, 1:4].sum(1))
    n = len(J)
    X = np.zeros((2 * n, 8)); X[:n, 0] = 1; X[n:, 1] = 1
    for k, t in enumerate((t1, t12, ty)):
        X[:n, 2 + 2 * k] = np.cos(t); X[:n, 3 + 2 * k] = -np.sin(t)
        X[n:, 2 + 2 * k] = np.sin(t); X[n:, 3 + 2 * k] = np.cos(t)
    return X


def xy_resid(c, J, P):
    X = design(J); r = X @ c - np.r_[P[:, 0], P[:, 1]]; n = len(J)
    return np.hypot(r[:n], r[n:])


def main():
    sets = load_sets()
    fit_keys = ["trace_microscope_to_polar_01", "discarded_trace_polar_to_hotel_01"]
    J = np.vstack([sets[k][0] for k in fit_keys]); P = np.vstack([sets[k][1] for k in fit_keys])

    # --- which joint is the column, which sum is the yaw (sanity, printed) ---
    allJ = np.vstack([s[0] for s in sets.values()]); allP = np.vstack([s[1] for s in sets.values()])
    corr = [abs(np.corrcoef(allJ[:, k], allP[:, 2])[0, 1]) for k in range(4)]
    print("corr(j_k, z):", np.round(corr, 4), "-> column is j%d" % (int(np.argmax(corr)) + 1))
    yaw_err = (allP[:, 5] - allJ[:, 1:4].sum(1) + 180) % 360 - 180
    print("rz - (j2+j3+j4): mean %.4f  sd %.4f  |max| %.3f deg (n=%d)" % (yaw_err.mean(), yaw_err.std(), np.abs(yaw_err).max(), len(allJ)))
    for sign in (1.0, -1.0):
        X = design(J, sign); c, *_ = np.linalg.lstsq(X, np.r_[P[:, 0], P[:, 1]], rcond=None)
        e = np.hypot(*(X @ c - np.r_[P[:, 0], P[:, 1]]).reshape(2, -1))
        print("angle sign %+.0f: xy rms %.3f mm" % (sign, np.sqrt((e ** 2).mean())))

    # --- robust xy fit ---
    keep = np.ones(len(J), bool)
    for it in range(3):
        X = design(J[keep]); y = np.r_[P[keep, 0], P[keep, 1]]
        c, *_ = np.linalg.lstsq(X, y, rcond=None)
        e = xy_resid(c, J, P)
        new = e < 1.0
        print("pass %d: n=%d rms %.3f max %.3f mm; dropping %d lag outliers" % (it, keep.sum(), np.sqrt((e[keep] ** 2).mean()), e[keep].max(), (keep & ~new).sum()))
        if (new == keep).all():
            break
        keep = new
    x0, y0 = c[0], c[1]
    links = [(float(np.hypot(c[2 + 2 * k], c[3 + 2 * k])), float(np.degrees(np.arctan2(c[3 + 2 * k], c[2 + 2 * k])))) for k in range(3)]
    # z and yaw
    A = np.c_[J[keep, 0], np.ones(keep.sum())]
    zc, *_ = np.linalg.lstsq(A, P[keep, 2], rcond=None)
    yaw_off = float(np.median(((P[keep, 5] - J[keep, 1:4].sum(1)) + 180) % 360 - 180))
    model = {
        "x0": float(x0), "y0": float(y0),
        "l1": links[0][0], "a1_deg": links[0][1],
        "l2": links[1][0], "a2_deg": links[1][1],
        "lt": links[2][0] if links[2][0] > 0.05 else 0.0, "at_deg": links[2][1] if links[2][0] > 0.05 else 0.0,
        "z_scale": float(zc[0]), "z_offset": float(zc[1]),
        "yaw_offset_deg": yaw_off,
    }
    print("\nfitted model:")
    for k, v in model.items():
        print("  %-15s %12.4f" % (k, v))
    print("  (tool offset lt fitted as %.3f mm -> treated as 0)" % links[2][0])

    import fk
    fk.MODEL.update(model)
    rows = []
    for name, (Jt, Pt) in sets.items():
        pred = np.asarray([fk.fk(j) for j in Jt])
        exy = np.hypot(pred[:, 0] - Pt[:, 0], pred[:, 1] - Pt[:, 1])
        ez = np.abs(pred[:, 2] - Pt[:, 2])
        eyaw = np.abs((pred[:, 5] - Pt[:, 5] + 180) % 360 - 180)
        role = "FIT" if name in fit_keys else "held-out"
        rows.append(dict(set=name, role=role, n=int(len(Jt)), xy_rms=float(np.sqrt((exy ** 2).mean())), xy_p95=float(np.percentile(exy, 95)),
                         xy_max=float(exy.max()), z_rms=float(np.sqrt((ez ** 2).mean())), z_max=float(ez.max()),
                         yaw_rms=float(np.sqrt((eyaw ** 2).mean())), yaw_max=float(eyaw.max())))
    print("\n%-42s %-8s %6s %8s %8s %8s %8s %8s %8s %8s" % ("set", "role", "n", "xy_rms", "xy_p95", "xy_max", "z_rms", "z_max", "yaw_rms", "yaw_max"))
    for r in rows:
        print("%-42s %-8s %6d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f" % (r["set"], r["role"], r["n"], r["xy_rms"], r["xy_p95"], r["xy_max"], r["z_rms"], r["z_max"], r["yaw_rms"], r["yaw_max"]))

    # IK round trip on every sample (taught branch), and the elbow branch census
    bad = 0; worst = 0.0
    for Jt, Pt in sets.values():
        for j in Jt:
            p = fk.fk(j)
            q = fk.ik(p, elbow=fk.elbow_sign(j), j4_ref=j[3], j2_ref=j[1], j3_ref=j[2])
            d = max(abs(a - b) for a, b in zip(q, j))
            worst = max(worst, d); bad += d > 0.01
    print("\nIK round trip: worst joint error %.5f (units), %d samples off by > 0.01" % (worst, bad))
    print("elbow branch of taught data: j3 min %.1f max %.1f over traces/routes; microscope seat j3 = %.1f (mirror branch)" %
          (allJ[:-1, 2].min(), allJ[:-1, 2].max(), sets["layout.microscope.joints_at_teach"][0][0, 2]))
    mic = sets["layout.microscope.joints_at_teach"][1][0]
    r = float(np.hypot(mic[0] - model["x0"], mic[1] - model["y0"]))
    print("microscope seat radius from the shoulder axis: %.2f mm of %.2f reach" % (r, model["l1"] + model["l2"]))

    model["_fit"] = {
        "fit_sets": fit_keys, "n_fit": int(keep.sum()), "outliers_dropped": int((~keep).sum()),
        "residuals": rows,
        "joint_order": ["j1 column mm", "j2 shoulder deg", "j3 elbow deg", "j4 wrist deg"],
        "note": "x = x0 + l1 cos(j2+a1) + l2 cos(j2+j3+a2); y likewise with sin; z = z_scale*j1 + z_offset; rz = j2+j3+j4+yaw_offset. "
                "Fitted 2026-10-19 from drag traces; validated on the held-out route-A trace, the layout waypoints and the teach pose.",
    }
    with open(os.path.join(HERE, "fk_model.json"), "w", encoding="utf-8") as f:
        json.dump(model, f, indent=2)
    print("\nwrote", os.path.join(HERE, "fk_model.json"))


if __name__ == "__main__":
    main()
