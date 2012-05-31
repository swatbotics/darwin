#!/bin/bash
# Install the boot scripts in this directory to the appropriate locations
# in the DARwIn's filesystem.

SCRIPT_DIR=$(dirname $0)
BOOT_SCRIPTS="darwin-startup.sh darwin-shutdown.sh darwin-reboot.sh"
RC_LOCAL_LINE="/etc/init.d/darwin-startup.sh"

if ! which play >/dev/null; then
  echo "The 'play' command must be installed; try running"
  echo "  sudo apt-get install sox libsox-fmt-all"
  exit 1
fi

echo "Adding boot scripts in /etc/init.d."
(cd "$SCRIPT_DIR"; sudo cp $BOOT_SCRIPTS /etc/init.d/)
(cd /etc/init.d; sudo chmod 755 $BOOT_SCRIPTS)
echo "Adding boot script symlinks in /etc/rc0.d and /etc/rc6.d."
sudo ln -fs ../init.d/darwin-shutdown.sh /etc/rc0.d/K99darwin-shutdown.sh
sudo ln -fs ../init.d/darwin-reboot.sh /etc/rc6.d/K99darwin-reboot.sh

if ! grep -q -F "$RC_LOCAL_LINE" /etc/rc.local; then
  sudo cp /etc/rc.local /etc/rc.local.bak
  echo "Modifying /etc/rc.local."
  echo -e "\$\n?^exit 0\$?\ni\n$RC_LOCAL_LINE\n.\nwq" \
    | sudo ed -s /etc/rc.local >/dev/null
fi
