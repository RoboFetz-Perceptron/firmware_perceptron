#!/usr/bin/env bash

__git_branch() {
    local branch
    branch=$(git symbolic-ref --short HEAD 2>/dev/null) \
        || branch=$(git rev-parse --short HEAD 2>/dev/null) \
        || return
    printf ' \001\033[01;33m\002(%s)\001\033[00m\002' "$branch"
}

PS1='\[\033[01;32m\]\u \[\033[01;34m\]\w$(__git_branch)\[\033[00m\] $ '

alias ib='idf.py build'
alias ifl='idf.py flash'
alias im='idf.py monitor'
alias ifm='idf.py flash monitor'
alias ic='idf.py fullclean'
