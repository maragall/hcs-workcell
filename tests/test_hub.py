import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import GATES  # noqa: E402
from test_sim import Rig  # noqa: E402

from perception import hub  # noqa: E402


def rows(tmp_path, name):
    with open(tmp_path / f"{name}.jsonl", encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def test_serve_returns_none_when_model_not_cached(tmp_path):
    assert hub.serve("seating", "v3", models_dir=str(tmp_path)) is None


def test_serve_returns_path_of_pinned_model(tmp_path):
    d = tmp_path / "seating"
    d.mkdir()
    (d / "v3.onnx").write_bytes(b"")
    assert hub.serve("seating", "v3", models_dir=str(tmp_path)) == str(d / "v3.onnx")


def test_serve_never_returns_a_model_for_fixed_checks(tmp_path):
    d = tmp_path / "align"
    d.mkdir()
    (d / "v1.onnx").write_bytes(b"")
    assert hub.serve("align", "v1", models_dir=str(tmp_path)) is None


def test_record_and_outcome_rows_share_the_step(tmp_path):
    hub.record("grip", "runs/x/0001.jpg", "ok", "pick:microscope", outbox_dir=str(tmp_path))
    hub.record("grip", "runs/x/0002.jpg", "not_ok", "pick:polar", outbox_dir=str(tmp_path))
    hub.outcome("pick:polar", False, outbox_dir=str(tmp_path))
    grip, outcomes = rows(tmp_path, "grip"), rows(tmp_path, "outcomes")
    assert [r["frame"] for r in grip] == ["runs/x/0001.jpg", "runs/x/0002.jpg"]
    assert grip[1]["verdict"] == "not_ok" and grip[1]["step"] == "pick:polar"
    assert outcomes == [{"step": "pick:polar", "succeeded": False, "time": outcomes[0]["time"]}]
    assert outcomes[0]["time"].endswith("+00:00")


def test_the_cell_records_gate_verdicts_and_step_outcomes_when_opted_in():
    r = Rig(refs=GATES + [("stage", "to_90")])
    outbox = os.path.join(r.tmp, "outbox")
    r.cyc.hub_cfg = {"record": True, "outbox": outbox}
    r.cyc.store.on_done = lambda step: hub.outcome(step, True, outbox)
    r.cyc.L["vision"]["stage_check"]["auto_refs"] = 3
    P0 = r.L["fixtures"]["polar"]["pose_at_0"]["pose"]
    r.robot.set_pose_for_test(P0)
    try:
        r.cyc._polar_rotate(90, P0)                               # trained stage gate: one verdict row, one outcome row
        stage = [json.loads(line) for line in open(os.path.join(outbox, "stage.jsonl"), encoding="utf-8")]
        outcomes = [json.loads(line) for line in open(os.path.join(outbox, "outcomes.jsonl"), encoding="utf-8")]
        assert stage[0]["verdict"] == "ok" and stage[0]["step"] == "polar:90" and os.path.exists(stage[0]["frame"])
        assert outcomes[-1] == {"step": "polar:90", "succeeded": True, "time": outcomes[-1]["time"]}
    finally:
        r.close()
        shutil.rmtree(outbox, ignore_errors=True)
