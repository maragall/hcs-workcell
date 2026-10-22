"""
Find the arm and the vision box on the network, so there is nothing to type. The known addresses in the
positions file are tried first, then every /24 this PC sits on. A host answers as the vision box when TCP 9000
replies to get_version; as the arm when its controller ports (SDK 52525, BRCAPI 56788) accept a connection.
Squid is found when its software is importable on this PC.
"""
from __future__ import annotations

import importlib.util
import ipaddress
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

from devices import nova

VISION_PORT, ARM_PORTS = 9000, (52525, 56788)
PROBE_TIMEOUT_S = 0.4


def local_subnets() -> List[ipaddress.IPv4Network]:
    """The /24 of every IPv4 interface, from ifconfig/ipconfig (the stdlib has no interface list)."""
    cmd = ["ipconfig"] if sys.platform == "win32" else ["ifconfig"]
    try:
        text = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
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


def vision_box(layout: dict) -> Dict[str, str]:
    """{"host", "version", "cameras"} or {"error"}."""
    v = layout["vision"]
    host = scan(v["port"], v["host"])
    if not host:
        return {"error": "no vision box answers on port 9000"}
    client = nova.VisionClient(host, v["port"], timeout=5)
    try:
        return {"host": host, "version": client.version(), "cameras": client.status().cameras}
    except Exception as e:
        return {"host": host, "error": f"{host} answers but not as a vision box: {e}"}


def arm(layout: dict) -> Dict[str, str]:
    """{"ip", "port"} of the first host with a controller port open, or {"error"}."""
    r = layout["robot"]
    for port in (r["port"],) + tuple(p for p in ARM_PORTS if p != r["port"]):
        ip = scan(port, r["ip"])
        if ip:
            return {"ip": ip, "port": port}
    return {"error": "no controller answers on ports 52525 or 56788"}


def squid() -> Dict[str, str]:
    """Squid's software, if this PC has it. Driving it needs the objects the Squid GUI builds (devices/squid.py)."""
    try:
        found = importlib.util.find_spec("control.microscope") is not None
    except Exception:
        found = False
    return {"module": "control.microscope"} if found else {"error": "Squid software not installed on this PC"}
