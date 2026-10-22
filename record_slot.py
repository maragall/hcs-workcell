"""Record the current TCP pose as a hotel slot seat (fixtures.hotel.taught.<slot>), with joints, jaw, stage
reading and both camera frames. Usage: python record_slot.py A1"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cli as R  # noqa: E402
from devices.vision_box import VisionClient  # noqa: E402

slot = sys.argv[1].upper()
layout = R.load_layout()
h = layout["fixtures"]["hotel"]
if slot[0] not in h["columns"] or not slot[1:].isdigit():
    sys.exit(f"unknown slot {slot}")
robot, st = R.connect_robot_readonly(layout, lambda *a, **k: None)
pose, joints, jaw, e2 = robot.pose(), robot.joints(), robot.jaw(), robot.ext_axis(1)
robot.disconnect()
rec = {"pose": [round(v, 4) for v in pose], "joints_at_teach": [round(v, 3) for v in joints[:4]],
       "jaw_at_teach": round(jaw, 3), "stage_axis_at_teach": round(e2, 3), "taught_at": time.strftime("%Y-%m-%d %H:%M:%S")}
d = os.path.join(R.REFS, "teach", f"hotel_{slot}")
os.makedirs(d, exist_ok=True)
v = VisionClient(layout["vision"]["host"], layout["vision"]["port"])
stamp = time.strftime("%Y%m%d_%H%M%S")
for label, cam in (("down", layout["vision"]["down_camera"]), ("front", layout["vision"]["front_camera"])):
    try:
        with open(os.path.join(d, f"{stamp}_{label}.jpg"), "wb") as f:
            f.write(v.frame(cam, 1280, 720, 85, None, layout["vision"]["led"]))
    except Exception as e:
        print(f"frame {cam}: {e}")
rec["teach_frames"] = d
old = h["taught"].get(slot)
h["taught"][slot] = rec["pose"]
h.setdefault("taught_detail", {})[slot] = rec
R.save_layout(layout)
print(f"hotel.taught.{slot} = {[round(x, 1) for x in pose[:3]]} yaw {pose[5]:.2f}  jaw {jaw:.2f}"
      + (f"  (was {[round(x, 1) for x in old[:3]]})" if old else ""))
