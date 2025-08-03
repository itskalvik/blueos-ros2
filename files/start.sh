#!/bin/bash

echo "Starting ROS2 BlueOS container..."

# Create necessary directories for nginx
mkdir -p /var/log/nginx
mkdir -p /var/run
mkdir -p /usr/share/ttyd

# Ensure nginx configuration is in place
if [ ! -f /etc/nginx/nginx.conf ]; then
    echo "ERROR: nginx.conf not found!"
    exit 1
fi

# Ensure the HTML file is in place
if [ ! -f /usr/share/ttyd/index.html ]; then
    echo "ERROR: index.html not found in /usr/share/ttyd/"
    exit 1
fi

# Start nginx if not already running
echo "Starting nginx on port 4717..."
# Kill any existing nginx processes
pkill nginx 2>/dev/null || true
sleep 1

# Start nginx with explicit error handling
nginx -g "daemon off;" &
NGINX_PID=$!
sleep 3

if kill -0 $NGINX_PID 2>/dev/null; then
    echo "nginx started successfully with PID: $NGINX_PID"
else
    echo "WARNING: nginx failed to start automatically"
    echo "Attempting manual nginx start..."
    nginx &
    sleep 2
    if pgrep nginx > /dev/null; then
        echo "nginx started manually"
    else
        echo "ERROR: nginx failed to start. Check nginx configuration."
        nginx -t
        exit 1
    fi
fi

# Start ttyd web terminal with simple configuration
echo "Starting ttyd web terminal on port 4718..."

# Test ttyd installation first
echo "Testing ttyd installation..."
if ! command -v ttyd &> /dev/null; then
    echo "ERROR: ttyd is not installed!"
    exit 1
fi
echo "✓ ttyd is installed: $(which ttyd)"
echo "✓ ttyd version: $(ttyd --version)"

# Kill any existing ttyd processes
pkill ttyd 2>/dev/null || true
sleep 1

# Fix .bashrc to remove the problematic ROS2 workspace setup line
sed -i '/source \/home\/ros2_ws\/install\/setup.sh/d' /root/.bashrc

# Create a wrapper script for ttyd with proper environment
cat > /tmp/ttyd-shell.sh << 'EOF'
#!/bin/bash
# Source ROS2 environment
source /opt/ros/humble/setup.sh
# Ensure proper terminal handling
export TERM=xterm-256color
export COLUMNS=80
export LINES=24
# Start an interactive bash shell with proper terminal handling
exec /bin/bash -i
EOF
chmod +x /tmp/ttyd-shell.sh

# Start ttyd with login shell for better interactive terminal handling
echo "Starting ttyd with login shell for enhanced terminal experience..."
ttyd -p 4718 -d 1 --writable -t fontSize=14 -t fontFamily="Monaco, Menlo, Ubuntu Mono, monospace" -t cursorBlink=true -t cursorStyle=block -t enableTrzsz=true -t enableZmodem=true -t enableSixel=true -t enableReconnect=true -t enableResize=true /bin/bash -l &
TTYD_PID=$!

# Wait a moment for ttyd to start
sleep 2

# Check if ttyd is running
if kill -0 $TTYD_PID 2>/dev/null; then
    echo "ttyd started successfully with PID: $TTYD_PID"
    
    # Wait a moment and check if ttyd is listening on port 4718
    sleep 3
    if ss -tuln | grep -q ":4718 "; then
        echo "✓ ttyd is listening on port 4718"
    else
        echo "WARNING: ttyd is not listening on port 4718"
    fi
    
    # Test ttyd with curl
    echo "Testing ttyd HTTP endpoint..."
    if curl -s http://localhost:4718 > /dev/null 2>&1; then
        echo "✓ ttyd HTTP endpoint is responding"
    else
        echo "WARNING: ttyd HTTP endpoint is not responding"
    fi
else
    echo "ERROR: ttyd failed to start!"
    exit 1
fi

echo "Container startup complete!"
echo "XTerm Web Terminal available at: http://localhost:4717"
echo "Direct ttyd terminal available at: http://localhost:4718"

# Keep container running
sleep infinity