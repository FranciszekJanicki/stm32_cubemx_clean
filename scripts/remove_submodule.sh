#!/bin/bash
set -e

DIR=$1

if git config -f .gitmodules --get-regexp path | grep -q "^submodule..*.path $DIR"; then
    echo "Removing submodule at $DIR"
    git submodule deinit -f "$DIR"
    git rm --cached "$DIR"
    git config -f .gitmodules --remove-section submodule."$DIR" || true
    git config --remove-section submodule."$DIR" || true
    git add .gitmodules
    rm -rf .git/modules/"$DIR" "$DIR"
else
    echo "No submodule found at $DIR"
fi
