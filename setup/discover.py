"""
Find the arm on the network, so there is nothing to type, and ask every Squid of the positions file whether it answers.
The arm's known address is tried first, then every /24 this PC sits on; a host is the arm when its controller ports
(SDK 52525, BRCAPI 56788) accept a connection. A Squid answers on its control server (utils/devices/squid_link.py).
"""
from __future__ import annotations

import ipaddress
import socket
import time
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

ARM_PORTS = (52525, 56788)
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


def port_open(host: str, port: int, timeout: float = PROBE_TIMEOUT_S, retry_s: float = 0.0) -> bool:
    """The controller's BRCAPI port takes one client at a time: it is probed under this PC's lock on it (utils/devices/portlock.py, shared with the
    window's pose poller and the force clamp), and with retry_s a refusal is tried again for that long (a client on another PC)."""
    from utils.devices.portlock import PortLock
    lock = PortLock(host, port) if port == ARM_PORTS[1] else None
    if lock and not lock.acquire(max(retry_s, 1.0)):
        return False
    try:
        deadline = time.time() + retry_s
        while True:
            try:
                with socket.create_connection((host, port), timeout=timeout):
                    return True
            except ConnectionRefusedError:
                if time.time() >= deadline:
                    return False
                time.sleep(0.05)
            except OSError:
                return False
    finally:
        if lock:
            lock.release()


def scan(port: int, first: Optional[str] = None) -> Optional[str]:
    """The first host with `port` open: the known address, then the local subnets in parallel."""
    if first and port_open(first, port, 1.0, retry_s=2.0):
        return first
    hosts = [str(h) for net in local_subnets() for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=64) as pool:
        for host, ok in zip(hosts, pool.map(lambda h: port_open(h, port), hosts)):
            if ok:
                return host
    return None


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
    here = ", ".join(str(n) for n in local_subnets()) or "no network adapter with an address"
    return {"error": f"no controller answers on ports 52525 or 56788 at {r['ip']} or anywhere on this PC's networks ({here})",
            "fix": f"power the arm and wait for a steady status light; network cable from this PC to port 1 of the controller; the arm is at {r['ip']} "
                   f"unless it was changed, so this PC's adapter on that cable needs an address in the same range ({'.'.join(str(r['ip']).split('.')[:3])}.x, mask 255.255.255.0)"}


def squid(layout: dict) -> Dict[str, dict]:
    """Every Squid of the positions file (`squid`), asked on its control server: name -> {"at", "acquiring", "python_exec"}, or
    {"at", "error"} with what to do. Read-only: nothing on the microscope moves."""
    from utils.devices.squid_link import SquidError, SquidLink
    out: Dict[str, dict] = {}
    for name, config in ((layout or {}).get("squid") or {}).items():
        link = SquidLink(name, config or {})
        at = f"{link.host}:{link.port}"
        try:
            out[name] = dict(link.ping(), at=at)
        except SquidError as e:
            out[name] = {"at": at, "error": str(e)}
    return out
