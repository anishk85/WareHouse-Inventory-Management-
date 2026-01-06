#!/usr/bin/env bash

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🧹 Cleaning and recreating /tmp/.docker.xauth"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

XAUTH=/tmp/.docker.xauth

# If it's a directory, remove it
if [ -d "$XAUTH" ]; then
    echo "👉 $XAUTH is a directory! Removing it..."
    sudo rm -rf "$XAUTH"
fi

# If it's a file, remove it
if [ -f "$XAUTH" ]; then
    echo "👉 Removing old file $XAUTH ..."
    sudo rm -f "$XAUTH"
fi

echo "👉 Creating new Xauthority file..."
sudo touch "$XAUTH"
sudo chown $USER:$USER "$XAUTH"
chmod 777 "$XAUTH"

echo "👉 Adding Xauthority entries..."
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -

echo "✅ Done. Final file:"
ls -l "$XAUTH"

