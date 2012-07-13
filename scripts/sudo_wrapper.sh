#!/bin/bash

if (( $# >= 1 )); then
    # Pop off the first argument as the command to run.
    COMMAND="$1"
    shift 1
    # Make sure we can find the command first.
    if which "$COMMAND" >/dev/null; then
	# Run sudo, using which to locate the command's real path,
	# and pass -E so that environment variables are preserved.
	sudo -E "$(which "$COMMAND")" "$@"
    else
	echo "Could not find command - is it a builtin?"
	exit 1
    fi
else
    # Nothing to run, so print usage.
    echo "Usage: sudow <command> [args ...]"
    echo "This command wraps sudo, using 'which' to find the full paths "
    echo "to commands for the current user, without relying on root's PATH."
    exit 1
fi
