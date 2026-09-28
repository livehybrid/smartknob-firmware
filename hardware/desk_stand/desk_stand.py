"""
Parametric desk stand for the SeedLabs SmartKnob Dev Kit.

Device dimensions come from the SeedLabs CAD files
(https://github.com/SeedLabs-it/smartknob-hardware: cad/front_panel_v81.3mf,
cad/back_plate_v49.stl, cad/knob_shell_v57.stl and electronics/view_base).
Those parts share one assembly origin: z = 0 is the top face of the PCB,
+z points out of the knob face and +y points towards the USB-C edge.

The knob leans back at --tilt degrees and rests on its back plate, the surface
it would sit on if screwed to a wall. The stand provides:
  * a tray locating the 85 mm square front-panel skirt on three sides,
  * a bottom rail with an open slot for the USB-C plug,
  * hidden mode (default): a vertical shaft and under-floor groove take the
    cable out of the back; compact mode: the plug exits the front and the cable
    tucks into the under-floor groove,
  * an open top edge, so the Stemma QT sockets stay usable and the
    backward-facing VEML7700 ambient-light sensor still sees room light,
  * a low rear heel and rubber-foot recesses so a firm press cannot tip it,
  * optional M3 pilot holes matching the back plate's countersunk corner holes.

Requires CadQuery 2.x (pip install cadquery). All dimensions in millimetres.

    python desk_stand.py                 # hidden cable, 35 degrees
    python desk_stand.py --tilt 30
    python desk_stand.py --compact       # plug exits the front, smaller print
"""

import argparse
import math
import os
from dataclasses import dataclass, fields

import cadquery as cq

# --------------------------------------------------------------------------
# Device geometry (measured from the SeedLabs CAD, see module docstring)
# --------------------------------------------------------------------------
DEV_W = 85.0          # front panel: 85 x 85 mm rounded square...
DEV_R = 5.0           # ...with 5 mm corner radii
FACE_Z = 5.9          # top of the front face
BACK_Z = -6.5         # rear face of the back plate = contact plane
USB_Z = 0.0           # centre of the 8.5 x 2.8 mm USB-C window (+y edge)
CORNER_HOLE = 32.1    # back plate M3 countersunk holes at (+/-32.1, +/-32.1)
BACK_TOP_Y = -36.1    # top edge of the 72.2 mm back plate


@dataclass
class Params:
    tilt: float = 35.0          # knob face angle from the desk (0 flat, 90 upright)
    compact: bool = False       # plug exits the front instead of a hidden shaft
    clear: float = 0.4          # clearance per side between skirt and tray
    wall: float = 3.5           # tray side-wall thickness
    wall_top_z: float = 1.0     # side walls grip the skirt up to this height
    wall_end_y: float = -12.0   # side walls stop here, leaving the top free
    chin_top_z: float = 4.0     # bottom-rail height (face is at 5.9)
    plug_w: float = 14.0        # USB-C plug slot width (overmould <= 12.4 mm)
    plug_floor_z: float = -4.5  # slot floor, ~1 mm below a 7 mm overmould
    plug_len: float = 26.0      # rigid plug length before the cable can bend
    shaft_len: float = 11.0     # hidden mode: shaft size along the stand depth
    shaft_min_h: float = 10.0   # hidden mode: shaft height above the pocket roof
    pocket_x: float = 17.0      # hidden mode: pocket under the shaft...
    pocket_y: float = 18.0
    pocket_z: float = 8.0
    compact_lip: float = 9.0    # compact mode: rail length along the slope
    compact_cable_z: float = 9.0  # compact mode: plug end height above the desk
    groove_w: float = 7.0       # under-floor cable groove
    groove_h: float = 5.5
    heel_len: float = 16.0      # rear heel beyond the cradle's top edge
    heel_h: float = 14.0        # heel height at the rear face...
    heel_end_h: float = 6.0     # ...sloping down to this at the back
    plan_r: float = 6.0         # footprint corner radius
    base_chamfer: float = 0.6   # bottom edge chamfer (hides elephant's foot)
    top_chamfer: float = 3.0    # chamfer on the cradle's top rear edge
    feet_d: float = 10.5        # rubber-foot recesses
    feet_depth: float = 0.8
    feet_inset: float = 8.0
    m3_holes: bool = True       # pilot holes for M3 thread-forming screws
    m3_pilot_d: float = 2.6
    m3_pilot_depth: float = 10.0


class Geometry:
    """Derived dimensions and the device-to-stand transform."""

    def __init__(self, p: Params):
        self.p = p
        t = math.radians(p.tilt)
        self.S, self.C = math.sin(t), math.cos(t)
        self.half_in = DEV_W / 2 + p.clear          # tray inner half-width
        self.half_out = self.half_in + p.wall       # stand half-width
        self.lip_y = DEV_W / 2 + p.clear            # lip face (device y)
        chin_len = p.compact_lip if p.compact else p.plug_len + p.shaft_len + 7.0
        self.chin_end_y = self.lip_y + chin_len
        self.y_rear = self.to_stand(BACK_TOP_Y, BACK_Z)[0]
        self.y_front = self.to_stand(self.chin_end_y, p.chin_top_z)[0]
        self.y_heel = self.y_rear - p.heel_len

        plug_end_y = DEV_W / 2 + p.plug_len
        if p.compact:
            # plug end (bottom of a 7 mm overmould) sits compact_cable_z above the desk
            self.z0 = p.compact_cable_z - self.to_stand(plug_end_y, USB_Z - 3.5)[1]
            self.shaft_y0 = self.shaft_y1 = None
        else:
            self.shaft_y0 = self.to_stand(plug_end_y, p.plug_floor_z)[0] - 2.0
            self.shaft_y1 = self.shaft_y0 + p.shaft_len
            floor_at_shaft = self.to_stand(self.device_y(self.shaft_y1, p.plug_floor_z),
                                           p.plug_floor_z)[1]
            self.z0 = p.pocket_z + p.shaft_min_h - floor_at_shaft

    def to_stand(self, y, z, lifted=False):
        """Device (y, z) -> stand (Y, Z). X is unchanged."""
        return (y * self.C + z * self.S,
                -y * self.S + z * self.C + (self.z0 if lifted else 0.0))

    def device_y(self, Y, z):
        """Device y where the plane of constant device z meets stand depth Y."""
        return (Y - z * self.S) / self.C

    def place(self, shape):
        """Rotate a solid modelled in device coordinates into the stand frame."""
        return (shape.rotate((0, 0, 0), (1, 0, 0), -self.p.tilt)
                .translate((0, 0, self.z0)))


def _yz_prism(points, half_x):
    return cq.Workplane("YZ").polyline(points).close().extrude(half_x, both=True)


def build(p: Params = Params()):
    g = Geometry(p)
    BIG = 400.0

    # 1. Base: everything below the contact plane, cut off at the rear face.
    base = g.place(cq.Workplane("XY")
                   .box(2 * g.half_out, BIG, BIG, centered=(True, True, False))
                   .translate((0, 0, BACK_Z - BIG)))
    rear_cut = (cq.Workplane("XY")
                .box(2 * g.half_out + 2, BIG, BIG, centered=(True, False, False))
                .translate((0, g.y_rear, -1)))
    base = base.intersect(rear_cut)

    # 2. Tray and bottom rail: everything above the contact plane.
    profile = [
        (p.wall_end_y, BACK_Z),
        (p.wall_end_y, p.wall_top_z),
        (g.lip_y - 16.0, p.wall_top_z),
        (g.lip_y - 6.0, p.chin_top_z),
        (g.chin_end_y + 30.0, p.chin_top_z),
        (g.chin_end_y + 30.0, BACK_Z),
    ]
    tray = _yz_prism(profile, g.half_out)
    device_pocket = (cq.Workplane("XY").workplane(offset=BACK_Z)
                     .placeSketch(cq.Sketch().rect(2 * g.half_in, 2 * g.half_in)
                                  .vertices().fillet(DEV_R + p.clear))
                     .extrude(60.0))
    plug_slot = (cq.Workplane("XY")
                 .box(p.plug_w, 120.0, 40.0, centered=(True, False, False))
                 .translate((0, g.lip_y - 8.0, p.plug_floor_z)))
    tray = g.place(tray.cut(device_pocket).cut(plug_slot))

    # 3. Rear heel: low wedge behind the cradle for stability.
    heel = _yz_prism([(g.y_heel, 0), (g.y_rear + 1, 0), (g.y_rear + 1, p.heel_h),
                      (g.y_heel, p.heel_end_h)], g.half_out)

    # 4. Footprint envelope with rounded corners and a bottom chamfer.
    envelope = (cq.Workplane("XY")
                .center(0, (g.y_front + g.y_heel) / 2)
                .rect(2 * g.half_out, g.y_front - g.y_heel)
                .extrude(BIG)
                .edges("|Z").fillet(p.plan_r)
                .faces("<Z").edges().chamfer(p.base_chamfer))

    stand = base.union(tray).union(heel).intersect(envelope)

    # 5. Cuts.
    cuts = []
    # Chamfer the cradle's top rear edge (more light for the ambient-light sensor).
    zt = g.to_stand(BACK_TOP_Y, BACK_Z, lifted=True)[1]
    tc = p.top_chamfer
    cuts.append(_yz_prism([(g.y_rear - 1, zt + 1), (g.y_rear + tc * g.C + 1, zt + 1 + tc * g.S),
                           (g.y_rear - 1, zt - tc - 1)], g.half_out + 1))

    groove_front = g.y_front + 2 if p.compact else g.shaft_y0
    if not p.compact:
        # Vertical shaft from the slot floor to the pocket, with a softened
        # front edge where the cable bends over it.
        cuts.append(cq.Workplane("XY")
                    .box(p.plug_w, p.shaft_len, BIG, centered=(True, False, False))
                    .translate((0, g.shaft_y0, p.pocket_z - 1)))
        lz = g.to_stand(g.device_y(g.shaft_y1, p.plug_floor_z), p.plug_floor_z, lifted=True)[1]
        cuts.append(_yz_prism([(g.shaft_y1 - 0.5, lz + 0.5),
                               (g.shaft_y1 + 4.0, lz + 0.5 + 4.0 * math.tan(math.radians(p.tilt))),
                               (g.shaft_y1 - 0.5, lz - 4.5)], p.plug_w / 2))
        # Pocket under the shaft, open to the desk, for threading the plug up.
        cuts.append(cq.Workplane("XY")
                    .box(p.pocket_x, p.pocket_y, p.pocket_z + 1, centered=(True, False, False))
                    .translate((0, g.shaft_y0 - (p.pocket_y - p.shaft_len) / 2, -1)))
    else:
        # Front entry: chamfered notch guiding the cable into the groove.
        cuts.append(cq.Workplane("XZ").workplane(offset=-(g.y_front + 1))
                    .rect(p.groove_w + 8, 2 * (p.groove_h + 4))
                    .workplane(offset=8).rect(p.groove_w, 2 * p.groove_h).loft())

    # Under-floor groove to the back of the heel, flared at the exit.
    cuts.append(cq.Workplane("XY")
                .box(p.groove_w, groove_front - g.y_heel + 2, p.groove_h + 1,
                     centered=(True, False, False))
                .translate((0, g.y_heel - 2, -1)))
    cuts.append(cq.Workplane("XZ").workplane(offset=-(g.y_heel - 1))
                .rect(p.groove_w + 6, 2 * (p.groove_h + 3))
                .workplane(offset=-7).rect(p.groove_w, 2 * p.groove_h).loft())

    # Rubber-foot recesses near the four corners of the footprint.
    fx = g.half_out - p.feet_inset
    feet = [(sx * fx, y) for sx in (-1, 1)
            for y in (g.y_heel + p.feet_inset, g.y_front - p.feet_inset)]
    cuts.append(cq.Workplane("XY").pushPoints(feet).circle(p.feet_d / 2)
                .extrude(p.feet_depth).translate((0, 0, -0.01)))

    # Optional M3 pilot holes, normal to the contact plane.
    if p.m3_holes:
        holes = (cq.Workplane("XY").workplane(offset=BACK_Z + 0.01)
                 .pushPoints([(sx * CORNER_HOLE, sy * CORNER_HOLE)
                              for sx in (-1, 1) for sy in (-1, 1)])
                 .circle(p.m3_pilot_d / 2).extrude(-p.m3_pilot_depth))
        cuts.append(g.place(holes))

    for c in cuts:
        stand = stand.cut(c)
    return stand.clean(), g


def output_name(p: Params):
    return f"smartknob_desk_stand_{'compact_' if p.compact else ''}{p.tilt:g}deg"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    for f in fields(Params):
        if f.type is bool or isinstance(f.default, bool):
            ap.add_argument("--" + f.name.replace("_", "-"), action="store_true",
                            default=f.default, help=f"(default {f.default})")
        else:
            ap.add_argument("--" + f.name.replace("_", "-"), type=float, default=f.default,
                            help=f"(default {f.default})")
    ap.add_argument("--no-m3-holes", action="store_true")
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    p = Params(**{f.name: getattr(a, f.name) for f in fields(Params)})
    if a.no_m3_holes:
        p.m3_holes = False

    stand, g = build(p)
    solid = stand.val()
    bb = solid.BoundingBox()
    name = output_name(p)
    print(f"{name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, "
          f"{solid.Volume() / 1000:.0f} cm3 solid, valid={solid.isValid()}")
    cq.exporters.export(stand, os.path.join(a.out, name + ".stl"),
                        tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(stand, os.path.join(a.out, name + ".step"))


if __name__ == "__main__":
    main()
