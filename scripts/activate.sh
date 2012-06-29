# Script to set up useful shell environment for working on darwin code.
# Based largely on the /bin/activate script for virtualenv.

# This file must be used with "source scripts/activate" *from bash*. It
# can't be run directly or it won't be able to set environment variables.

######################
### INITIALIZATION ###

SCRIPT_SOURCED_NAME="$0"
# Hack for Bash compatibility since its $0 is useless when using 'source'.
if [ -n "$BASH" ] && [ -z "$ZSH_VERSION" ]; then
    SCRIPT_SOURCED_NAME="${BASH_SOURCE[0]}"
fi
SCRIPT_DIR=$(cd "$( dirname "$SCRIPT_SOURCED_NAME" )" && pwd)
_ACTIVATE_SCRIPT_PATH="$SCRIPT_DIR/$(basename "$SCRIPT_SOURCED_NAME")"
DARWIN_ROOT=$(dirname "$SCRIPT_DIR")
export DARWIN_ROOT

############################
### ACTIVATION FUNCTIONS ###

reactivate () {
    deactivate nondestructive
    source "$_ACTIVATE_SCRIPT_PATH"
}

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

    # Reset bash completions to pre-script state.
    reset_completions

    # Get rid of 'deactivate' function itself.
    if [ ! "$1" = "nondestructive" ] ; then
    # Self destruct!
	unset -f reactivate
        unset -f deactivate
	unset -f list_binaries
	unset -f load_completions
	unset -f reset_completions
	unset -f reload_completions
	unset _ACTIVATE_SCRIPT_PATH
	unset DARWIN_ROOT
    fi
}

#######################
### OTHER FUNCTIONS ###

list_binaries() {
    BINARY_DIR="$DARWIN_ROOT/build/bin"
    if [ -d "$BINARY_DIR" ]; then
	(cd "$BINARY_DIR" && ls -1)
    fi
}

reload_completions () {
    if [ -n "$_OLD_TAB_COMPLETIONS" ]; then
	reset_completions
    fi
    _COMPLETION_BINARIES=$(list_binaries)
    _OLD_TAB_COMPLETIONS=$(complete -p $_COMPLETION_BINARIES 2>/dev/null)
    complete -o bashdefault -o default -o nospace \
	-C '$DARWIN_ROOT/gflags/bin/gflags_completions.sh \
        --tab_completion_columns $COLUMNS' \
	$_COMPLETION_BINARIES
}

reset_completions () {
    if [ -n "$_COMPLETION_BINARIES" ]; then
	complete -r $_COMPLETION_BINARIES
    fi
    unset _COMPLETION_BINARIES
    if [ -n "$_OLD_TAB_COMPLETIONS" ]; then
	eval "$_OLD_TAB_COMPLETIONS"
    fi
    unset _OLD_TAB_COMPLETIONS
}


############################################
##### Actual 'activate' part of script #####

# Unset irrelevant variables.
deactivate nondestructive

# Add useful directories to the path.
_OLD_VIRTUAL_PATH="$PATH"
PATH="$DARWIN_ROOT/build/bin:$PATH"
PATH="$DARWIN_ROOT/build-debug/bin:$PATH"
PATH="$SCRIPT_DIR:$PATH"
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

# Set up special bash completion for gflags binaries.
if [ -z "$ZSH_VERSION" ]; then
    reload_completions
fi

# Unset no-longer-needed environment variables so zsh doesn't try to use them.
unset SCRIPT_SOURCED_NAME
unset SCRIPT_DIR
