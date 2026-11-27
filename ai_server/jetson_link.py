"""
Reach the Jetson over the internet, through its own Cloudflare tunnel (remote link).

On AUS wireless the PC and the Jetson cannot reach each other directly. The Jetson keeps an ssh
tunnel open (console/jetson/remote_link.sh in stage4) and tells the PC's front door where it is
every 30 s; the front door keeps that in front_door_jetson.json. This reads it.

    python jetson_link.py           where the Jetson's tunnel is, and how old that news is
    python jetson_link.py proxy     for ssh: ProxyCommand of `Host jetson-remote` in ~/.ssh/config

~/.ssh/config:
    Host jetson-remote
        User apnvi
        IdentityFile ~/.ssh/id_ed25519
        HostKeyAlias 192.168.137.30          # the Jetson's known key: same machine as `jetson`
        ProxyCommand python C:/APNVI/.../ai_server/jetson_link.py proxy   # forward slashes:
                                             # Git Bash's ssh misreads backslashes
        ServerAliveInterval 15
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from front_door import read_jetson  # noqa: E402 (path line above)

CLOUDFLARED = r'C:\APNVI\tools\cloudflared\cloudflared.exe'
FRESH_S = 120.0             # the Jetson reports every 30 s: older than this, it is gone


def where(read=read_jetson):
    """The Jetson's tunnel address if its news is fresh, else ''."""
    kept = read()
    if kept.get('ssh') and kept.get('age_s', FRESH_S + 1) <= FRESH_S:
        return kept['ssh']
    return ''


def main():
    kept = read_jetson()
    if len(sys.argv) > 1 and sys.argv[1] == 'proxy':
        address = where()
        if not address:
            sys.stderr.write('jetson_link: the Jetson has not reported a tunnel lately (%s)\n'
                             % (kept or 'never'))
            return 1
        return subprocess.call([CLOUDFLARED, 'access', 'ssh', '--hostname', address])
    if not kept:
        print('The Jetson has never reported its tunnel to the front door.')
        return 1
    print('The Jetson\'s ssh tunnel: %s (reported %.0f s ago, %s)'
          % (kept['ssh'], kept['age_s'], 'fresh' if where() else 'too old: the Jetson is gone'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
