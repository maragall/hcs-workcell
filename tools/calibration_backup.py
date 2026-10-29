"""Copy a camera's active calibration files off the box (backup), or put a saved copy back (restore).

    python tools/calibration_backup.py backup [camera]            -> runs/calibration_backup_<time>/<camera>/
    python tools/calibration_backup.py restore FOLDER [camera] [--reboot]

Both check every file by SHA-256 on the box and here. Restore puts the files back under their own names and leaves the box
running unless --reboot is given (the box reads them at its next start)."""
from __future__ import annotations

import hashlib
import os
import posixpath
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cli
from calibration import upload as U

FILES = ("CamParam.dat", "CamInTool.dat", "calibration_info.yaml")


def _sha_local(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _sha_remote(client, path):
    _, out, _ = client.exec_command(f"sha256sum '{path}'")
    return out.read().decode().split()[0]


def main(argv):
    L = cli.load_layout()
    creds, host = U.credentials(L), L["vision"]["host"]
    mode = argv[0]
    camera = L["vision"]["down_camera"]
    client = U.connect(host, creds)
    sftp = client.open_sftp()
    try:
        if mode == "backup":
            cam = argv[1] if len(argv) > 1 else camera
            dest = os.path.join(cli.RUNS, f"calibration_backup_{time.strftime('%Y%m%d_%H%M%S')}", cam)
            os.makedirs(dest, exist_ok=True)
            for name in FILES:
                remote = posixpath.join(U.remote_dir(cam), name)
                if not U._exists(sftp, remote):
                    print(f"  {name}: not on the box")
                    continue
                sftp.get(remote, os.path.join(dest, name))
                ok = _sha_local(os.path.join(dest, name)) == _sha_remote(client, remote)
                print(f"  {name}: saved, checksum {'matches' if ok else 'DIFFERS'}")
                if not ok:
                    raise SystemExit("backup failed")
            print("backup in", os.path.dirname(dest))
        elif mode == "restore":
            folder = argv[1]
            cam = next((a for a in argv[2:] if not a.startswith("--")), camera)
            src = os.path.join(folder, cam)
            for name in FILES:
                path = os.path.join(src, name)
                if not os.path.isfile(path):
                    continue
                remote = posixpath.join(U.remote_dir(cam), name)
                sftp.put(path, remote)
                ok = _sha_local(path) == _sha_remote(client, remote)
                print(f"  {name}: put back, checksum {'matches' if ok else 'DIFFERS'}")
                if not ok:
                    raise SystemExit("restore failed")
            if "--reboot" in argv:
                U.reboot(host, creds)
                print("  the box is restarting")
    finally:
        sftp.close()
        client.close()


if __name__ == "__main__":
    main(sys.argv[1:])
