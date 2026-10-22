import json

from perception import hub


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


def test_record_appends_one_row_per_call(tmp_path):
    hub.record("grip", "runs/x/0001.jpg", "ok", True, outbox_dir=str(tmp_path))
    hub.record("grip", "runs/x/0002.jpg", "bad", False, outbox_dir=str(tmp_path))
    rows = [json.loads(l) for l in open(tmp_path / "grip.jsonl", encoding="utf-8")]
    assert [r["frame"] for r in rows] == ["runs/x/0001.jpg", "runs/x/0002.jpg"]
    assert rows[1]["verdict"] == "bad" and rows[1]["step_succeeded"] is False
    assert rows[0]["time"].endswith("+00:00")
