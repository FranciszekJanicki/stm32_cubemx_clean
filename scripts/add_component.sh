#!/bin/bash
set -e

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <git_repo_url> <component_name>"
  exit 1
fi

REPO_URL=$1
COMP_NAME=$2
COMP_PATH="app/components/$COMP_NAME"

if [ -d "$COMP_PATH" ]; then
  echo "Component $COMP_NAME already exists at $COMP_PATH"
  exit 0
fi

echo "Adding component $COMP_NAME"
git submodule add -f "$REPO_URL" "$COMP_PATH"
echo "Done."
