"""The vision box's service (BRVision on the box, started at boot by cron from /home/pi/Nova/Nova.sh), over ssh with nova_ssh.local.json.

  python tools/box_service.py status      open files of the service, its uptime, the last log lines
  python tools/box_service.py restart     kill it and start it again (about 20 s); the cameras show "no signal" until it is back

When to restart: every connection to port 9000 times out and the box's log repeats "Too many open files" (the service leaks one file
descriptor per client connection and dies at 1024; 2026-11-03 after three hours of a connection per frame). Our clients now keep one
connection each, which should make this rare.
"""
from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    import paramiko
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    c = json.load(open(os.path.join(HERE, "nova_ssh.local.json"), encoding="utf-8"))
    host = c.get("host") or json.load(open(os.path.join(HERE, "layout.json"), encoding="utf-8"))["vision"]["host"]
    s = paramiko.SSHClient()
    s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    s.connect(host, username=c.get("user", "pi"), password=c.get("password"), timeout=8)

    def run(sh):
        i, o, e = s.exec_command(sh, timeout=60)
        return (o.read().decode("utf-8", "replace") + e.read().decode("utf-8", "replace")).strip()

    if cmd == "status":
        pid = run("pgrep -f '^./BRVision' | tail -1")
        print("service pid", pid or "(not running)")
        if pid:
            print("open files", run(f"sudo ls /proc/{pid}/fd | wc -l"), "of", run("ulimit -n"), "| started", run(f"ps -o lstart= -p {pid}"))
        print(run("tail -n 4 $(ls -t /home/pi/Nova/logs/*.log | head -1) | cut -c1-160"))
    elif cmd == "restart":
        print("stopping", run("sudo pkill -f './BRVision'; sleep 3; pgrep -f '^./BRVision' | wc -l"), "left")
        run("cd /home/pi/Nova && (setsid nohup sudo ./BRVision > /dev/null 2>&1 &)")
        sys.path.insert(0, HERE)
        from devices.vision_box import VisionClient
        v = VisionClient(host, 9000, timeout=8)
        for _ in range(12):
            time.sleep(5)
            try:
                st = v.status()
                print("back:", st.state, st.cameras, "cameras")
                break
            except Exception:
                print("waiting ...")
        else:
            print("the service did not answer within a minute: reboot the box (ssh, sudo reboot) or power-cycle it")
    else:
        sys.exit(__doc__)
    s.close()


if __name__ == "__main__":
    main()
