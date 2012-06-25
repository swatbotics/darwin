# Script to set up useful shell environment for working on darwin code.
# Based largely on the /bin/activate script for virtualenv.

# This file must be used with "source scripts/activate" *from bash*. It
# can't be run directly or it won't be able to set environment variables.

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
DARWIN_ROOT=$(dirname "$SCRIPT_DIR")
export DARWIN_ROOT

deactivate () {
    # Restore the original path.
    if [ -n "$_OLD_VIRTUAL_PATH" ] ; then
        PATH="$_OLD_VIRTUAL_PATH"
        export PATH
        unset _OLD_VIRTUAL_PATH
    fi

    # This should detect bash and zsh, which have a hash command that must
    # be called to get it to forget past commands.  Without forgetting
    # past commands the $PATH changes we made may not be respected
    if [ -n "$BASH" -o -n "$ZSH_VERSION" ] ; then
        hash -r
    fi

    # Restore the original prompt.
    if [ -n "$_OLD_VIRTUAL_PS1" ] ; then
        PS1="$_OLD_VIRTUAL_PS1"
        export PS1
        unset _OLD_VIRTUAL_PS1
    fi

    # Get rid of 'deactivate' function itself.
    if [ ! "$1" = "nondestructive" ] ; then
    # Self destruct!
        unset -f deactivate
    fi
}


############################################
##### Actual 'activate' part of script #####

# Unset irrelevant variables.
deactivate nondestructive

# Add useful directories to the path.
_OLD_VIRTUAL_PATH="$PATH"
SCRIPT_DIR_PATH="$SCRIPT_DIR"
BUILD_BIN_PATH="$DARWIN_ROOT/build/bin"
BUILD_DEBUG_BIN_PATH=$"$DARWIN_ROOT/build-debug/bin"
PATH="$SCRIPT_DIR_PATH:$BUILD_DEBUG_BIN_PATH:$BUILD_BIN_PATH:$PATH"
export PATH

# Change the prompt to show we're in a virtualenv.
if [ -z "$VIRTUAL_ENV_DISABLE_PROMPT" ] ; then
    _OLD_VIRTUAL_PS1="$PS1"
    PS1="(darwin-code) $PS1"
    export PS1
fi

# This should detect bash and zsh, which have a hash command that must
# be called to get it to forget past commands.  Without forgetting
# past commands the $PATH changes we made may not be respected
if [ -n "$BASH" -o -n "$ZSH_VERSION" ] ; then
    hash -r
fi
