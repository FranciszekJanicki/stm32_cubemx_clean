#!/bin/bash
set -e

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <component_name>"
  exit 1
fi

COMP_NAME=$1
COMP_PATH="app/components/$COMP_NAME"

if [ ! -d "$COMP_PATH" ]; then
  echo "Component $COMP_NAME does not exist at $COMP_PATH"
  exit 0
fi

echo "Removing component $COMP_NAME"
git submodule deinit -f "$COMP_PATH"
git rm -f "$COMP_PATH"
rm -rf ".git/modules/$COMP_PATH"
echo "Done."
