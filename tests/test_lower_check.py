"""The lowering gate's reference store: the frame is kept as the camera sent it, older PNG references still count."""
from __future__ import annotations

import io
import os
import sys

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from rig import BAD, SAFE, jpeg  # noqa: E402

from perception.lower_check import LowerCheck  # noqa: E402


def test_reference_is_the_camera_frame_itself(tmp_path):
    lc = LowerCheck(str(tmp_path), min_refs=1)
    path = lc.add_reference("polar@0", "empty", jpeg(SAFE))
    with open(path, "rb") as f:
        assert f.read() == jpeg(SAFE)
    assert lc.verdict("polar@0", "empty", jpeg(SAFE)).safe
    assert not lc.verdict("polar@0", "empty", jpeg(BAD)).safe


def test_numbering_and_verdict_span_png_and_jpg_references(tmp_path):
    d = tmp_path / "polar@0" / "empty"
    d.mkdir(parents=True)
    Image.open(io.BytesIO(jpeg(SAFE))).convert("RGB").save(d / "ref_001.png")      # as an older cell stored them
    lc = LowerCheck(str(tmp_path), min_refs=3)
    assert os.path.basename(lc.add_reference("polar@0", "empty", jpeg(SAFE))) == "ref_002.jpg"
    assert os.path.basename(lc.add_reference("polar@0", "empty", jpeg(SAFE))) == "ref_003.jpg"
    assert lc.count("polar@0", "empty") == 3
    v = lc.verdict("polar@0", "empty", jpeg(SAFE))
    assert v.n_refs == 3 and v.safe
