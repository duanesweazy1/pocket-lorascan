#!/usr/bin/env bash
set -euo pipefail
install -D -m 0644 lorascan.py "$STAGE$APP_INSTALL_DIR/lorascan.py"
# Lets the gpio group switch the Cap's 5 V rail (systemd-tmpfiles applies it at boot)
install -D -m 0644 packaging/pocket-lorascan.tmpfiles "$STAGE/usr/lib/tmpfiles.d/pocket-lorascan.conf"
tmp=$(mktemp)
cat >"$tmp" <<'LAUNCHER'
#!/bin/sh
exec python3 /usr/share/APPLaunch/apps/pocket-lorascan/lorascan.py "$@"
LAUNCHER
install -D -m 0755 "$tmp" "$STAGE$INSTALL_PREFIX/bin/pocket-lorascan"
rm -f "$tmp"
