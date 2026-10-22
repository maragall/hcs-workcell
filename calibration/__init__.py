"""
Camera calibration for the Nova vision box, replacing the vendor's Windows tool.

    marker       the 40 mm ArUco marker on the practice plate: its four corners in an image
    plan         the one capture plan around wherever the arm is
    capture      drive the arm through the plan, keep a picture and the arm's pose at every stop
    solve        one joint least squares: camera model (CamParam.dat), camera-in-tool (CamInTool.dat), marker-in-base
    synth        the marker rendered through known files, for validation and a fake box
    upload       SFTP both files to the box, reboot it
    verify       the box's own marker reading from two poses
    teach_free   the vendor manual section 5 teach/trigger/relation flow
    validate     `python -m calibration.validate`: render through the vendor's files, solve back, compare

Only numpy, cv2, scipy, pillow, paramiko. Nothing here imports a device module except capture,
verify and teach_free, which take the device objects as arguments.
"""
