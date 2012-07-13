#!/bin/bash

# Usage: ./status_client.sh [HOSTNAME] [PORT]
# Simple script using netcat to test status server.

# Defaults.
HOSTNAME=127.0.0.1
PORT=9000
INTERVAL=0.05

# Set vars from command line args if present.
if (($# > 0)); then
    HOSTNAME="$1"
    shift 1
    if (($# > 0)); then
	PORT="$1"
	shift 1
    fi
fi

# Use netcat to send empty messages to the server and show responses.
while true; do
    # Send an empty message as request datagram to the server.
    echo
    # Wait (to avoid flooding server).
    sleep "$INTERVAL"
    # Echo a newline to stderr so that responses from server appear to
    # start on new lines - kind of hacky but oh well.  Wish netcat could
    # delimit UDP datagrams with newlines (or a custom string) for you.
    echo 1>&2
done | nc -u "$HOSTNAME" "$PORT"
