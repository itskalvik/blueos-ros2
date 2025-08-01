#!/bin/bash

# Simple script to start nginx
echo "Starting nginx..."

# Create necessary directories
mkdir -p /var/log/nginx
mkdir -p /var/run

# Start nginx
nginx

echo "nginx started" 