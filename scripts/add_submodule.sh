#!/bin/bash
set -e

SSH_URL=$1
DIR=$2

if [ ! -d "$DIR" ]; then
    echo "Adding submodule at $DIR"
    git submodule add "$SSH_URL" "$DIR"
else
    echo "Submodule already exists at $DIR"
fi
