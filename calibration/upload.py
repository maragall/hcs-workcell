"""
Put CamParam.dat and CamInTool.dat on the box and reboot it, as the vendor's tool does.

The box keeps a camera's calibration in /home/pi/Nova/data/calibration/<camera>/active/. The
old files are renamed next to themselves with a timestamp before the new ones go up. SSH
credentials come from layout.json `vision.ssh`: {"user", "password", "key_path"}; a missing or
null entry refuses, nothing is hard-coded. paramiko is imported on use only.
"""
from __future__ import annotations

import json
import os
import posixpath
import time
from typing import Callable, Dict, List, Optional

REMOTE_ROOT = "/home/pi/Nova/data/calibration"
ACTIVE = "active"
FILES = ("CamParam.dat", "CamInTool.dat")


class NoCredentials(Exception):
    pass


def credentials(layout: dict) -> Dict[str, Optional[str]]:
    """The ssh block of layout.json, checked: user plus a password or a key path."""
    ssh = (layout.get("vision") or {}).get("ssh") or {}
    user, password, key = ssh.get("user"), ssh.get("password"), ssh.get("key_path")
    local = {}
    try:                                                            # nova_ssh.local.json beside layout.json: kept out of git, read here
        with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "nova_ssh.local.json"), encoding="utf-8") as f:
            local = json.load(f)
    except (OSError, ValueError):
        pass
    user, password, key = user or local.get("user"), password or local.get("password"), key or local.get("key_path")
    user = user or os.environ.get("NOVA_SSH_USER")                 # the login can live in the environment, out of every file
    password = password or os.environ.get("NOVA_SSH_PASSWORD")
    key = key or os.environ.get("NOVA_SSH_KEY")
    if not user or not (password or key):
        raise NoCredentials("the box login is missing: layout.json vision.ssh, or the environment NOVA_SSH_USER with "
                            "NOVA_SSH_PASSWORD or NOVA_SSH_KEY (null refuses), or the file nova_ssh.local.json beside layout.json: {\"user\": ..., \"password\": ...}")
    return {"user": user, "password": password, "key_path": key}


def connect(host: str, creds: Dict[str, Optional[str]]):
    """A connected paramiko SSHClient."""
    import paramiko
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=creds["user"], password=creds.get("password"),
                   key_filename=creds.get("key_path"), timeout=10)
    return client


def remote_dir(camera: str) -> str:
    return posixpath.join(REMOTE_ROOT, camera, ACTIVE)


def push(box_host: str, camera: str, camparam_path: str, camintool_path: str, creds: Dict[str, Optional[str]],
         connect_fn: Callable = connect, now: Optional[float] = None) -> List[str]:
    """Upload both files; returns the backup names made for the old ones."""
    for p in (camparam_path, camintool_path):
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    target = remote_dir(camera)
    client = connect_fn(box_host, creds)
    backups = []
    try:
        sftp = client.open_sftp()
        try:
            _mkdirs(sftp, target)
            for local, name in ((camparam_path, FILES[0]), (camintool_path, FILES[1])):
                remote = posixpath.join(target, name)
                if _exists(sftp, remote):
                    backup = f"{remote}.{stamp}.bak"
                    sftp.rename(remote, backup)
                    backups.append(backup)
                sftp.put(local, remote)
        finally:
            sftp.close()
    finally:
        client.close()
    return backups


def reboot(box_host: str, creds: Dict[str, Optional[str]], connect_fn: Callable = connect) -> None:
    client = connect_fn(box_host, creds)
    try:
        _, out, err = client.exec_command("sudo reboot")
        out.channel.recv_exit_status()
    finally:
        client.close()


def _exists(sftp, path: str) -> bool:
    try:
        sftp.stat(path)
        return True
    except IOError:
        return False


def _mkdirs(sftp, path: str) -> None:
    parts = path.strip("/").split("/")
    cur = ""
    for p in parts:
        cur += "/" + p
        if not _exists(sftp, cur):
            sftp.mkdir(cur)
