# 12 — Chair measurement worksheet (do this before printing the mount)

The CAD uses a **provisional** Drive Silver Sport 2. Only the published catalogue values are real: seat width 16/18/20", seat depth 16", seat-to-floor 17.5–19.5", back height 16", 8" casters. Everything below tagged **[P]** is a placeholder.

Measure your chair with a tape and calipers. Enter each value in `cad/wra/params.yaml`, then run:

```bash
cd cad && python build.py --parts --verify     # re-generates the mount parts, re-checks every clearance and refreshes
                                               # arm_geometry.json (the driver's chair -> arm transform and user keep-out boxes)
```

Then copy the new `arm_geometry.json` to the Jetson (docs/11 §G). The driver only uses your chair geometry after that, and test T4.0 (docs/09) checks it.

Measure with the chair on a level floor, tyres at normal pressure, seat cushion fitted, footrests in the driving position. **Origin:** floor, directly under the centre of the rear axle. **x** forward, **y** to the user's left, **z** up.

| # | what to measure | how | param | provisional | yours |
|---|---|---|---|---|---|
| T1 | outside diameter of the vertical **front frame post** where the bracket will clamp | calipers, at 2 heights | `mount.tube_od`, `chair.frame_tube_od` | 25.4 mm | |
| T2 | free, straight length of that post (top of the caster housing … underside of the seat rail) | tape | `chair.front_post_z` [low, high] | 220 … 470 mm | |
| T3 | post axis distance in front of the rear-axle centre | tape on the floor, plumb line from the post | `chair.front_post_x` | 445 mm | |
| T4 | post axis outboard of the seat edge | ruler | `chair.post_from_seat_edge` | 15 mm | |
| T5 | seat width (rail to rail) | tape | `chair.seat_width` | 457 mm | |
| T6 | height of the dock top you want (≈ the highest point of T2 minus 10 mm). Higher is better for knobs (docs/10) | — | `chair.mount_z` | **460** (default) | |
| T7 | lower side-frame rail: height and the free length behind the post | tape | `chair.lower_rail_z`, `chair.strut_clamp_x` | 200 mm, x 330 mm | |
| T8 | swing-away footrest hanger: top attachment height | tape | `chair.hanger_top_z` | 440 mm | |
| T9 | wheel lock (brake) lever position x and the space it sweeps | tape, lever on and off | `chair.wheel_lock_x` | 300 mm | |
| T10 | rear wheel: diameter and how far its outer face is outboard of the frame | tape | `chair.rear_wheel_dia`, `chair.rear_wheel_camber_y` | 610 mm, 40 mm | |
| T11 | hand-rim outer face from the chair centreline (this sets the added width) | tape | (computed) | — | |
| T12 | armrest height (if fitted) | tape | `chair.armrest_height` | 735 mm | |
| T13 | rear cane OD and free length below the seat (for the e-box clamps) | calipers, tape | `chair.ebox_center_z` | Ø25.4, z 360 | |
| T14 | door: hinge side, knob height, knob diameter, and whether it opens toward you | tape | task parameters (docs/10) | Ø56, 36" | |
| T15 | user reach: where the user can press the pendant E-stop comfortably | with the user | pendant cable length | — | |
| T16 | clear width of each doorway you use. With the arm parked it adds ≈ 127–141 mm beyond the hand rim (`output/reports/chair_check.json`) | tape, door fully open | compare with T11 × 2 + the added width | 813 mm (32") | |
| T17 | latch torque of each knob: hang a luggage scale on a lever clamped to the knob and turn until it unlatches; torque = force × lever length | luggage scale, 100 mm lever | must stay below 2.65 N·m (docs/10) | 0.3–1.5 N·m [E] | |

**Also photograph** the post, the lower rail and the rear cane with a ruler in view, and keep the photos with this sheet. Mark the clamp positions with tape before drilling or fitting anything. Nothing in this design drills the chair frame.

If your tube is not round (oval or square sections), the clamps `02/03` must be redesigned. Tell me the section and I will regenerate them.
