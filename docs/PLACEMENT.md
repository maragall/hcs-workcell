# Physical placement: microscope relative to the rotating stage and the hotel

Numbers are from the Shanghai cell, robot base frame, mm. They are the working envelope that the taught
routes and the safety zones assume. A new site should reproduce these relationships within about 20 mm, then
re-teach the three seats and two routes with the commands in `REPLICATION.md`.

## Measured positions (TCP at the seat, plate seated)

| Fixture | x | y | z | gripper yaw | grip |
|---|---|---|---|---|---|
| microscope seat | 123 | 400 | 268 | +90° | short edges |
| rotating stage seat | 317 | 305 | 70 | −92° | short edges at 0°, long edges at −90° |
| hotel A1 seat | 293 | 137.5 | 74 | −1° | long edges |
| hotel A9 seat | 293 | 137.5 | 374 | −1° | long edges |

Derived relationships the routes depend on:
- Stage to microscope: Δx −194, Δy +95, Δz +197. The microscope seat is 197 mm higher than the stage seat and
  the gripper turns 180° between them. The turn is done over the base side of the cell (around x 160, y 80 at
  z 326), never beside the microscope.
- Stage to hotel: the stage sits beside the hotel's A column, 170 mm further in +y and at row-1 height. Route A
  is a 100 mm swing at z 103.
- The hotel is approached along +x from a point 100 mm in front of the slot at the slot height plus 25 mm.

## Rules for a new placement

1. **Stage next to the hotel, same height band as row 1.** Keeps route A short and lets the long-edge plate
   go straight into a slot with no yaw change after the stage.
2. **Microscope away from the hotel in y, with free air above its seat.** The arm arrives from straight above:
   the hover is 67 mm over the seat and only vertical moves happen inside the microscope zone (y above the
   seat minus 150). Nothing may stand in the cylinder above the holder up to 70 mm higher than the plate, and
   the column needs room to turn the claw 180° somewhere outside that zone at travel height.
3. **Reach.** All seats lie between radius 240 and 420 mm from the base. Keep new positions inside that band;
   the inner limit of the SCARA refuses closer targets.
4. **Height order.** Microscope seat highest, hotel rows spanning around it, stage lowest worked well: the
   plate descends during the carry and never has to climb inside the rack.
5. **Nothing between the stage and the microscope** along the route corridor at z 300 to 335.
6. **Rack orientation.** Slots open toward −x; the rack's front face must be at least 65 mm (half a plate)
   plus margin away from any climb column the routes use.

## Camera consequences

The down camera is 90 mm behind the fingertips. From a hover it looks at whatever is 90 mm behind the seat,
which at the microscope is the instrument body. If a seated check from the arm's own cameras is wanted, leave
a clear view 90 mm behind the microscope holder at hover height, or plan a fixed camera on the holder.

## After relocating the microscope

Re-teach in this order, 20 minutes: `record-pose microscope`, `record-pose microscope.hover --local-only`,
re-drag route B (`trace microscope_to_polar`, `normalise`), then one supervised cycle. The stage and the hotel
are untouched.
