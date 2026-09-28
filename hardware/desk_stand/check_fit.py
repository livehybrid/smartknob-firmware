"""
Fit, printability and stability checks for the desk stand, plus a preview render.

Needs the SeedLabs enclosure meshes (cad/ folder of
https://github.com/SeedLabs-it/smartknob-hardware) for the fit check:

    python check_fit.py --cad /path/to/smartknob-hardware/cad [--tilt 35] [--compact]
    python check_fit.py --cad ... --table        # compare several variants

Requires: cadquery, trimesh, manifold3d, shapely, matplotlib, numpy
"""

import argparse
import math
import os
import tempfile

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

import cadquery as cq  # noqa: E402
import desk_stand as ds  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEVICE_MASS_G = 130.0     # estimate for the assembled Dev Kit
PLA_EFFECTIVE = 0.35      # g/cm3: PLA at ~20 % infill with 3 walls


def stand_mesh(p):
    stand, g = ds.build(p)
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        path = f.name
    cq.exporters.export(stand, path, tolerance=0.02, angularTolerance=0.1)
    m = trimesh.load(path, force="mesh")
    os.unlink(path)
    return m, g


def device_transform(g):
    M = trimesh.transformations.rotation_matrix(math.radians(-g.p.tilt), [1, 0, 0])
    M[2, 3] = g.z0
    return M


def load_device(cad_dir, M):
    parts = {}
    for name, colour in (("front_panel_v81.3mf", (0.93, 0.93, 0.95)),
                         ("back_plate_v49.stl", (0.55, 0.65, 0.9)),
                         ("knob_shell_v57.stl", (0.22, 0.22, 0.25))):
        m = trimesh.load(os.path.join(cad_dir, name), force="mesh")
        m.apply_transform(M)
        parts[name] = (m, colour)
    # Stand-in for the 39.5 mm watch glass over the display (not in the CAD set)
    glass = trimesh.creation.cylinder(radius=20.5, height=1.2, sections=96)
    glass.apply_translation([0, 0, 27.6])
    glass.apply_transform(M)
    parts["display_glass"] = (glass, (0.08, 0.1, 0.16))
    return parts


def section_plot(stand, parts, plug, g, path):
    """Centre-line cross-section (X = 0) showing how the cable is routed."""
    fig, ax = plt.subplots(figsize=(9, 6))
    for mesh, colour, label in ([(stand, (0.95, 0.55, 0.2), "stand")]
                                + [(m, c, k) for k, (m, c) in parts.items()]
                                + [(plug, (0.15, 0.6, 0.3), "USB-C plug")]):
        sec = mesh.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
        if sec is None:
            continue
        for loop in sec.discrete:
            ax.fill(loop[:, 1], loop[:, 2], color=colour, alpha=0.9, lw=0.5,
                    edgecolor="k")
        ax.plot([], [], color=colour, lw=6, label=label)
    if not g.p.compact:
        # indicative cable path: along the plug, down the shaft, back along the groove
        y_end = g.to_stand(ds.DEV_W / 2 + g.p.plug_len, ds.USB_Z, lifted=True)
        xs = [y_end[0], (g.shaft_y0 + g.shaft_y1) / 2, (g.shaft_y0 + g.shaft_y1) / 2 - 4,
              g.y_heel - 8]
        zs = [y_end[1], y_end[1] - 8, g.p.groove_h / 2, g.p.groove_h / 2]
        ax.plot(xs, zs, color="k", lw=3, ls="--", label="cable")
    ax.set_aspect("equal")
    ax.set_xlabel("depth Y (mm, + towards you)")
    ax.set_ylabel("height Z (mm)")
    ax.set_title(f"{ds.output_name(g.p)}: section through the centre line")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=100)
    plt.close(fig)


def plug_mesh(g, M):
    plug = trimesh.creation.box(extents=[12.4, g.p.plug_len, 7.0])
    plug.apply_translation([0, ds.DEV_W / 2 + g.p.plug_len / 2 + 0.05, ds.USB_Z])
    plug.apply_transform(M)
    return plug


def stability(stand, g, M):
    """Press force along the knob axis (N) that tips the stand, and slide forces."""
    ms = stand.volume / 1000 * PLA_EFFECTIVE
    W = (ms + DEVICE_MASS_G) * 9.81e-3
    com_dev = (M @ np.array([0, 0, 4.0, 1]))[:3]
    com = (stand.center_mass * ms + com_dev * DEVICE_MASS_G) / (ms + DEVICE_MASS_G)
    press = (M @ np.array([0, 0, 28.5, 1]))[:3]        # centre of the knob top
    axis = M[:3, :3] @ np.array([0, 0, -1.0])           # pushing into the knob
    y_line = g.y_heel + g.p.feet_inset                  # rear feet (conservative)
    restoring = W * (com[1] - y_line)
    arm = (press[1] - y_line) * (-axis[2]) - press[2] * (-axis[1])
    tip = math.inf if arm >= 0 else restoring / -arm
    fy, fz = abs(axis[1]), abs(axis[2])
    slide = {mu: (mu * W / (fy - mu * fz) if fy > mu * fz else math.inf) for mu in (0.35, 0.8)}
    return ms, com, tip, slide


def printability(stand):
    n, a, zc = stand.face_normals, stand.area_faces, stand.triangles_center[:, 2]
    steep = (n[:, 2] < -math.cos(math.radians(45))) & (zc > 0.3)
    flat = (n[:, 2] < -0.99) & (zc > 0.3)
    return a[steep].sum(), a[flat].sum()


def fmt(x):
    return "never" if x == math.inf else f"{x:.1f} N"


def render(meshes, path, title):
    """All triangles in one collection so matplotlib depth-sorts them together."""
    light = np.array([0.35, 0.55, 0.9])
    light /= np.linalg.norm(light)
    tris, cols = [], []
    for m, colour in meshes:
        i = np.clip(m.face_normals @ light, 0, 1) * 0.6 + 0.4
        tris.append(m.triangles)
        cols.append(np.clip(np.outer(i, colour), 0, 1))
    tris, cols = np.vstack(tris), np.vstack(cols)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    ctr, span = (lo + hi) / 2, (hi - lo).max() / 2 * 0.95
    views = [(16, 62, "front three-quarter"), (0, 0, "side (USB plug in green)"),
             (32, 118, "front left"), (22, -128, "rear: cable exit"),
             (-58, 62, "underside: groove and feet"), (78, 90, "top")]
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(title, fontsize=12)
    for k, (e, az, t) in enumerate(views, 1):
        ax = fig.add_subplot(2, 3, k, projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolors="none",
                                             linewidths=0))
        ax.set_xlim(ctr[0] - span, ctr[0] + span)
        ax.set_ylim(ctr[1] - span, ctr[1] + span)
        ax.set_zlim(0, 2 * span)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=e, azim=az)
        ax.set_axis_off()
        ax.set_title(t, fontsize=10)
    plt.tight_layout()
    plt.savefig(path, dpi=90)


def check(p, cad_dir, verbose=True, preview=None):
    stand, g = stand_mesh(p)
    M = device_transform(g)
    ext = stand.bounds[1] - stand.bounds[0]
    res = {"name": ds.output_name(p), "size": ext, "vol": stand.volume / 1000,
           "watertight": stand.is_watertight}
    parts = load_device(cad_dir, M) if cad_dir else {}
    plug = plug_mesh(g, M)
    res["interference"] = {k: trimesh.boolean.intersection([stand, m], engine="manifold").volume
                           for k, (m, _) in parts.items()}
    res["interference"]["usb_plug"] = trimesh.boolean.intersection(
        [stand, plug], engine="manifold").volume
    res["steep"], res["bridges"] = printability(stand)
    res["mass"], com, res["tip"], res["slide"] = stability(stand, g, M)
    top = max((m.bounds[1][2] for m, _ in parts.values()), default=stand.bounds[1][2])
    res["height_with_knob"] = top
    if verbose:
        print(f"\n== {res['name']}")
        print(f"  size {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm, solid {res['vol']:.0f} cm3, "
              f"watertight={res['watertight']}, ~{res['mass']:.0f} g printed")
        print(f"  overall height with knob {top:.0f} mm")
        print("  interference (mm3): " + ", ".join(f"{k} {v:.3f}"
                                                  for k, v in res["interference"].items()))
        print(f"  downward faces >45 deg: {res['steep']:.0f} mm2, of which flat bridges "
              f"{res['bridges']:.0f} mm2")
        print(f"  press force to tip backwards: {fmt(res['tip'])}; to slide: bare PLA "
              f"{fmt(res['slide'][0.35])}, rubber feet {fmt(res['slide'][0.8])}")
    if preview:
        meshes = [(stand, (0.95, 0.55, 0.2))] + list(parts.values()) + [(plug, (0.15, 0.6, 0.3))]
        render(meshes, preview, f"{res['name']}: about {res['mass']:.0f} g printed, "
                                f"press force to tip {fmt(res['tip'])}")
        print(f"  wrote {preview}")
        section = preview.replace(".png", "_section.png")
        section_plot(stand, parts, plug, g, section)
        print(f"  wrote {section}")
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cad", help="path to smartknob-hardware/cad")
    ap.add_argument("--tilt", type=float, default=35.0)
    ap.add_argument("--compact", action="store_true")
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--preview", default=os.path.join(HERE, "preview.png"))
    a = ap.parse_args()
    if a.table:
        print(f"{'variant':38s} {'W x D x H mm':>18s} {'g':>5s} {'tip':>8s} {'H+knob':>7s}")
        for compact in (False, True):
            for tilt in (25, 30, 35, 40, 45):
                r = check(ds.Params(tilt=tilt, compact=compact), a.cad, verbose=False)
                s = r["size"]
                print(f"{r['name']:38s} {s[0]:5.0f} x{s[1]:5.0f} x{s[2]:4.0f} "
                      f"{r['mass']:5.0f} {fmt(r['tip']):>8s} {r['height_with_knob']:6.0f}")
        return
    check(ds.Params(tilt=a.tilt, compact=a.compact), a.cad, preview=a.preview)


if __name__ == "__main__":
    main()
