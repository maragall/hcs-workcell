"""
Teach daemon: holds the one SDK connection open and records poses on demand, so a teach session is
"write a word, read a line" instead of a fresh connect per pose.

    python teach_daemon.py            # runs until the command "quit"
    commands (one word in runs/teach_cmd.txt):
        A1 | A9 | B1 | C1 | C9 ...    hotel slot seat   -> fixtures.hotel.taught.<slot> (6-list)
        microscope | polar.pose_at_0 | polar.pose_at_90 | hotel.rack_front ...   -> fixtures.<path>.pose
        drag | undrag | status | quit
Every record stores pose, joints, jaw, stage axis reading and both camera frames (reference_images/teach/<name>/).
Results are appended to runs/teach_log.txt.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices.vision_box import VisionClient  # noqa: E402

CMD = os.path.join(HERE, "runs", "teach_cmd.txt")
LOG = os.path.join(HERE, "runs", "teach_log.txt")


def out(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def frames(vision, layout, name):
    d = os.path.join(R.REFS, "teach", name.replace(".", "_"))
    os.makedirs(d, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for label, cam in (("down", layout["vision"]["down_camera"]), ("front", layout["vision"]["front_camera"])):
        try:
            img = vision.frame(cam, 1280, 720, 85, None, layout["vision"]["led"])
            with open(os.path.join(d, f"{stamp}_{label}.jpg"), "wb") as f:
                f.write(img)
        except Exception as e:
            out(f"  frame {cam}: {e}")
    return d


def record(robot, vision, name):
    layout = R.load_layout()
    pose, joints, jaw, e2 = robot.pose(), robot.joints(), robot.jaw(), robot.ext_axis(1)
    rec = {"pose": [round(v, 4) for v in pose], "joints_at_teach": [round(v, 3) for v in joints[:4]],
           "jaw_at_teach": round(jaw, 3), "stage_axis_at_teach": round(e2, 3),
           "taught_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    h = layout["fixtures"]["hotel"]
    if len(name) in (2, 3) and name[0] in h["columns"] and name[1:].isdigit():
        h["taught"][name] = rec["pose"]
        h.setdefault("taught_detail", {})[name] = rec
        label = f"hotel.taught.{name}"
    else:
        node = layout["fixtures"]
        for key in name.split("."):
            node = node.setdefault(key, {})
        node.update(rec)
        node.pop("_todo", None)
        label = f"fixtures.{name}"
    rec["teach_frames"] = frames(vision, layout, name)
    R.save_layout(layout)
    out(f"{label} = {[round(v, 1) for v in pose[:3]]} yaw {pose[5]:.2f}  jaw {jaw:.2f}  stage {e2:.2f}  frames ok")


def main():
    layout = R.load_layout()
    robot, st = R.connect_robot_readonly(layout, lambda *a, **k: None)
    vision = VisionClient(layout["vision"]["host"], layout["vision"]["port"])
    out(f"daemon up: {st}")
    while True:
        if not os.path.exists(CMD):
            time.sleep(0.2)
            continue
        with open(CMD, encoding="utf-8") as f:
            name = f.read().strip()
        os.remove(CMD)
        try:
            if name == "quit":
                out("quit")
                break
            if name == "drag":
                robot._r.EnableDragModeAsync(True); time.sleep(1.5); out(f"drag on: {robot.state()}")
            elif name == "undrag":
                robot._r.EnableDragModeAsync(False); time.sleep(1.5)
                if not robot.state().enabled:
                    robot.enable()
                out(f"drag off: {robot.state()}")
            elif name == "status":
                out(f"{robot.state()} pose {[round(v, 1) for v in robot.pose()]} jaw {robot.jaw():.2f} stage {robot.ext_axis(1):.2f}")
            elif name:
                record(robot, vision, name)
        except Exception as e:
            out(f"ERROR {name}: {e!r}")
    robot.disconnect()


if __name__ == "__main__":
    main()
