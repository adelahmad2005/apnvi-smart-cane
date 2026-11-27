#!/bin/bash
# Remote link: make it everyday on the Jetson. Run once, with admin:
#     sudo bash install_remote_link.sh
# 1. ssh accepts keys only (passwords refused), because ssh is reachable over the internet now.
#    Skipped if the apnvi user has no authorised key, so nobody gets locked out.
# 2. The reporter (remote_link.sh) and the console helper start at boot and restart themselves.
# 3. Switches from copies started by hand to the services, detached from this session: an ssh
#    session through the tunnel drops for up to a minute while the reporter restarts.
# Needs ~/apnvi_tools/cloudflared and ~/apnvi_remote.conf (see remote_link.sh).
set -e
[ "$(id -u)" = 0 ] || { echo "Run it with sudo: sudo bash $0"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR=/home/apnvi

if [ -s "$HOME_DIR/.ssh/authorized_keys" ]; then
  printf 'PasswordAuthentication no\nKbdInteractiveAuthentication no\n' \
    > /etc/ssh/sshd_config.d/50-apnvi-keys-only.conf
  sshd -t
  systemctl reload ssh
  echo "ssh: keys only"
else
  echo "ssh: NOT changed (no $HOME_DIR/.ssh/authorized_keys: passwords would be the only way in)"
fi

install -o apnvi -g apnvi -m 755 "$HERE/remote_link.sh" "$HOME_DIR/apnvi_tools/remote_link.sh.new"
install -m 644 "$HERE/apnvi-remote-link.service" /etc/systemd/system/apnvi-remote-link.service
install -m 644 "$HERE/apnvi-console.service" /etc/systemd/system/apnvi-console.service
systemctl daemon-reload
systemctl enable apnvi-remote-link.service apnvi-console.service

# The patterns cannot match this command's own text ([.] and [c] break the match).
systemd-run --quiet --unit=apnvi-remote-switch-$$ --on-active=3s /bin/bash -c '
  mv /home/apnvi/apnvi_tools/remote_link.sh.new /home/apnvi/apnvi_tools/remote_link.sh
  pkill -f "^bash /home/apnvi/apnvi_tools/remote_link[.]sh$" || true
  pkill -f "^/home/apnvi/apnvi_tools/cloudflared tunnel" || true
  # The helper (a ROS node) ignores a plain stop (18 Nov): Ctrl+C first, then a hard stop.
  pkill -INT -f "[c]onsole/jetson/helper[.]py" || true
  sleep 5
  pkill -9 -f "[c]onsole/jetson/helper[.]py" || true
  sleep 1
  systemctl restart apnvi-remote-link.service
  systemctl restart apnvi-console.service'
echo "Services enabled. Switching over in 3 s: the reporter gets a new tunnel address and tells"
echo "the PC within a minute. Check: systemctl status apnvi-remote-link apnvi-console"
