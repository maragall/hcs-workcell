# Data model: what each kind of file means

Proposal, 2026-10-26. The file split is not implemented: today everything is in `layout.json` (the setup writes its results under `setup`, `vision.calibration`, `gripper`, `fixtures`) and `state_microscope.json` (`contents`). The initialization below is implemented as `python cli.py setup`.
Patterns it follows: Opentrons (shipped definitions, offsets stored separately), PyLabRobot (definitions, layout and
state serialized separately), ROS (nominal description, calibration kept out of it).

Each kind answers one question, has one writer, and a lifetime.

| kind | question it answers | who writes it | lifetime |
|---|---|---|---|
| `definitions/` | What is the product? True of every cell of this model. | Nobody at run time. Comes from CAD / the vendor, changes with a product revision. | Fixed. Read-only. |
| `calibration/` | What did this cell find out about itself? | The setup flow (`python cli.py setup`), nobody else. | Until re-measured. Each entry carries time and the check that passed. |
| `live/` | What is true right now? | The loop, after every step. | Changes every step. A claim on restart, checked against sensors. |

`profiles/` (speeds) and `references/` (learned photos) are versioned on their own: neither describes the product,
the customer or this cell's geometry.

## Rules

1. A file is written only by its kind's writer. Nothing outside `cellmodel/` opens a file under `data/`.
2. A measurement is either a **delta over a nominal value** in `definitions/` (grips, plate offset) or an **absolute
   fact** with no nominal (where the microscope is, hotel size found, camera results). Both live in `calibration/`.
3. `effective value = definition + measurement`, with a manual re-teach (`calibration/overrides.toml`) winning. Resolved
   in one place, `cellmodel/resolve.py`.
4. A failed or out-of-limit measurement leaves its file unchanged and stops the arm, with one line in the run log.
5. `live/` is first written by the initialization (the machine's first task); after that only by the loop.
6. Occupancy and the module angle are not trusted from disk: occupancy is re-looked, the angle is re-read from the axis.

## Files

```
data/
├─ definitions/   arm  rotation_module  hotel  plate  fingers  keepout
├─ config.toml    plate in use, Squid folder: the few things the customer declares
├─ calibration/   cameras  hotel  grips  microscope  overrides
├─ profiles/      speeds.toml
├─ references/    learned photos
└─ live/          system_state.json  run_log.jsonl
```

## Initialization, by file (actors: user, polar, nova, squid, computer)

`hotel`: nova images the hotel and the computer finds its size (occupancy goes into `live/system_state.json`, the size into `calibration/hotel`). The rotation module and hotel poses are definitions, not measured per cell. `grips`: polar closes on the
plate, short then long edges. `microscope`: squid homes, polar descends and nova finds the stage. `cameras`: the computer
solves intrinsic and extrinsic from nova's frames and uploads the files to nova. `live/system_state.json`: polar looks at microscope, module and gripper.
