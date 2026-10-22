"""
Camera calibration for the Nova vision box, replacing the vendor's Windows tool.

    plate        find the 7x7 dot plate in an image: image points and plate points (metres)
    intrinsics   HALCON area_scan_polynomial fit -> CamParam.dat
    handeye      eye-in-hand camera-in-tool pose -> CamInTool.dat
    capture      drive the arm through a plan, save image + pose like the vendor's folders
    upload       SFTP both files to the box, reboot it
    verify       the box's own pose estimate against the arm's displacement
    teach_free   the vendor manual section 5 teach/trigger/relation flow
    validate     `python -m calibration.validate`: the whole pipeline on vendor_data/

Only numpy, cv2, scipy, pillow, paramiko. Nothing here imports a device module except capture,
verify and teach_free, which take the device objects as arguments.
"""
