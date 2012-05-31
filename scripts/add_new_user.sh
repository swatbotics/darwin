#!/bin/bash
if (( $# < 1 )); then
  echo "Usage: $0 USERNAME"
  exit 1
fi

echo "Creating new user '$1'..."
sudo adduser $1 --ingroup darwin
echo "Adding user to appropriate groups..."
DARWIN_GROUPS=$(/usr/bin/groups darwin | cut -d ' ' -f 4-)
for group in $DARWIN_GROUPS; do
  echo addgroup $1 $group
  sudo addgroup $1 $group
done
