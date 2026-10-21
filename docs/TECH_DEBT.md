# Tech debt

Known gaps that are accepted for now. Each entry says what breaks, why it is deferred, and what the fix looks like.
Newest first.

## Brightfield illumination from the top blocks route B (2026-10-20)

- **What breaks:** when the microscope is set up for brightfield with the transmitted-light illuminator over the
  stage, the illuminator arm stands in the corridor that route B (microscope <-> rotating stage) uses at z 335
  near (181 to 186, 321 to 353). The plate hit it on 2026-10-20 22:25 (wrist collision, docs/INCIDENTS.md), after
  41 clean cycles that day with the illuminator out of the way.
- **Why nothing caught it:** route B is a fixed joint replay recorded with the illuminator away; the arm's two
  cameras look away from the plate at that pose; the only sensor that fired was the controller's wrist collision
  detection, after contact.
- **Interim rule:** before any run, the illuminator is swung clear of the stage and stays there. The user mode
  "brightfield" is not supported by the cell yet.
- **Fix when brightfield becomes a user mode:** a second microscope approach for that configuration: either a
  route B variant that stays below or beside the illuminator (re-record with the illuminator in place, verify the
  clearance with the plate held), or a pre-flight check that the illuminator is parked, from a camera that can see
  it (the front camera from the hotel via point sees the microscope; a reference pair "illuminator parked / over
  stage" is cheap to collect). The layout should carry the microscope's illumination state so the cycle picks the
  route, and refuses to carry a plate when the state is unknown.
