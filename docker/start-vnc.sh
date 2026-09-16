#!/bin/bash
# Start VNC services for GUI access

RESOLUTION="${VNC_RESOLUTION:-1920x1080}"
DISPLAY_NUM="${DISPLAY:-:99}"
DISPLAY_NUM="${DISPLAY_NUM#:}"  # Remove leading colon

echo "Starting VNC services..."
echo "Resolution: $RESOLUTION"
echo "Display: :$DISPLAY_NUM"

# Start Xvfb (virtual framebuffer)
Xvfb :$DISPLAY_NUM -screen 0 ${RESOLUTION}x24 &
sleep 1

# Start x11vnc
x11vnc -display :$DISPLAY_NUM -forever -shared -rfbport 5900 -nopw &
sleep 1

# Start window manager (fluxbox) - suppress warnings
fluxbox 2>/dev/null &
sleep 1

# Start a terminal for convenience
xterm -geometry 100x30+50+50 &

# Start noVNC websockify
websockify --web /usr/share/novnc 6080 localhost:5900 &

echo "VNC services started!"
echo "  - noVNC web access: http://localhost:6080"
echo "  - VNC client access: localhost:5900"
