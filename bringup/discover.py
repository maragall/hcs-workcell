"""
Find the arm and the vision box on the network, so there is nothing to type. The known addresses in the
positions file are tried first, then every /24 this PC sits on. A host answers as the vision box when one of VISION_PORTS
replies to get_version; as the arm when its controller ports (SDK 52525, BRCAPI 56788) accept a connection.
Squid is found when its software is importable on this PC.
"""
from __future__ import annotations

import importlib.util
import os
import ipaddress
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

from devices import nova

VISION_PORTS, ARM_PORTS = (9000, 9001, 8888), (52525, 56788)
PROBE_TIMEOUT_S = 0.4


def local_subnets() -> List[ipaddress.IPv4Network]:
    """The /24 of every IPv4 interface, from ifconfig/ipconfig (the stdlib has no interface list)."""
    cmd = ["ipconfig"] if sys.platform == "win32" else ["ifconfig"]
    try:
        text = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=5).stdout   # any console code page
    except Exception:
        text = ""
    nets = []
    for word in text.replace(":", " ").split():
        try:
            ip = ipaddress.IPv4Address(word)
        except ValueError:
            continue
        if ip.is_private and not ip.is_loopback:
            net = ipaddress.IPv4Network(f"{ip}/24", strict=False)
            if net not in nets:
                nets.append(net)
    return nets


def port_open(host: str, port: int, timeout: float = PROBE_TIMEOUT_S) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def scan(port: int, first: Optional[str] = None) -> Optional[str]:
    """The first host with `port` open: the known address, then the local subnets in parallel."""
    if first and port_open(first, port, 1.0):
        return first
    hosts = [str(h) for net in local_subnets() for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=64) as pool:
        for host, ok in zip(hosts, pool.map(lambda h: port_open(h, port), hosts)):
            if ok:
                return host
    return None


def ask_version(host: str, port: int, timeout: float, tries: int = 1) -> str:
    """The box's get_version reply, or raises the last error. A port counts as the vision box only if it answers."""
    last: Exception = OSError("no attempt")
    for _ in range(tries):
        try:
            return nova.VisionClient(host, port, timeout=timeout).version()
        except Exception as e:
            last = e
    raise last


def why(e: Exception) -> str:
    """One word the user can act on: refused (nothing listens), timed out (listens or filtered, no reply), or the error."""
    if isinstance(e, ConnectionRefusedError):
        return "refused"
    if isinstance(e, (TimeoutError, socket.timeout)) or "timeout" in str(e).lower() or "timed out" in str(e).lower():
        return "timed out"
    return str(e)


def vision_box(layout: dict) -> Dict[str, str]:
    """{"host", "port", "version", "cameras"} or {"error"}. The saved host and port first, patiently (the box can take
    seconds to accept; 3 tries of 5 s), then the other candidate ports on that host, then every host of the local
    subnets with any candidate port open. A port is the vision box only if it answers get_version, so a stray open
    port is never taken for it."""
    v = layout["vision"]
    ports = (v["port"],) + tuple(p for p in VISION_PORTS if p != v["port"])
    notes: List[str] = []

    def found(host: str, port: int, version: str) -> Dict[str, str]:
        try:
            cams = nova.VisionClient(host, port, timeout=5).status().cameras
        except Exception as e:
            return {"host": host, "port": port, "version": version, "error": f"{host}:{port} answers get_version but not get_status: {why(e)}"}
        return {"host": host, "port": port, "version": version, "cameras": cams}

    try:
        return found(v["host"], v["port"], ask_version(v["host"], v["port"], 5.0, tries=3))
    except Exception as e:
        notes.append(f"{v['host']}:{v['port']} {why(e)}")
    for port in ports[1:]:
        if port_open(v["host"], port, 1.0):
            try:
                return found(v["host"], port, ask_version(v["host"], port, 3.0))
            except Exception as e:
                notes.append(f"{v['host']}:{port} open, {why(e)} on get_version")
    hosts = [str(h) for net in local_subnets() for h in net.hosts() if str(h) != v["host"]]
    with ThreadPoolExecutor(max_workers=64) as pool:
        hits = [(h, p) for h, p, ok in pool.map(lambda hp: (*hp, port_open(*hp)), [(h, p) for h in hosts for p in ports]) if ok]
    for host, port in hits:
        try:
            return found(host, port, ask_version(host, port, 3.0))
        except Exception:
            continue
    return {"error": "no vision box answers: " + "; ".join(notes or ["nothing tried"]) + f"; no other host on {len(hosts)} addresses"}


def arm(layout: dict, connected: bool = False) -> Dict[str, str]:
    """{"ip", "port"} of the first host with a controller port open, or {"error"}. The port returned is always the
    SDK port from the positions file (robot.port): BRCAPI answering only says where the controller is. While the
    SDK is connected (`connected`) only BRCAPI is probed: the controller accepts one SDK client and a probe on
    52525 must not touch the live session."""
    r = layout["robot"]
    ports = (ARM_PORTS[1],) if connected else (r["port"],) + tuple(p for p in ARM_PORTS if p != r["port"])
    for port in ports:
        ip = scan(port, r["ip"])
        if ip:
            return {"ip": ip, "port": r["port"]}
    return {"error": "no controller answers on ports 52525 or 56788"}


SQUID_FALLBACKS = ("C:/Users/ASUS/Desktop/Squid",)      # where this cell's PC keeps it, tried after the saved folder


def squid_root(folder: str) -> Optional[str]:
    """The folder that holds Squid's `control` package: `folder` itself or its `software` subfolder, or None."""
    for base in (folder, os.path.join(folder, "software")):
        if base and os.path.isfile(os.path.join(base, "control", "microscope.py")):
            return os.path.normpath(base)
    return None


def squid(L: Optional[dict] = None) -> Dict[str, str]:
    """Squid's software, if this PC has it: the folder saved in the positions file, then the fallbacks, then an
    installed `control` package. Driving it needs the objects the Squid GUI builds (devices/squid.py)."""
    saved = ((L or {}).get("squid_software") or {}).get("folder")
    for folder in [f for f in (saved, *SQUID_FALLBACKS) if f]:
        root = squid_root(folder)
        if root:
            return {"module": "control.microscope", "folder": root}
    try:
        if importlib.util.find_spec("control.microscope") is not None:
            return {"module": "control.microscope", "folder": "installed"}
    except Exception:
        pass
    return {"error": "can't find the Squid software folder", "browse": True}
