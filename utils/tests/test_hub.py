import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)

from rig import GATES  # noqa: E402
from test_sim import Rig  # noqa: E402

from vision import hub  # noqa: E402


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


