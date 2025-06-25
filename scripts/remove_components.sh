#!/bin/bash
set -e

COMPONENTS_FILE="components.txt"

if [ ! -f "$COMPONENTS_FILE" ]; then
  echo "File $COMPONENTS_FILE not found"
  exit 1
fi

while read -r REPO NAME; do
  [[ -z "$NAME" || "$NAME" =~ ^# ]] && continue
  ./scripts/remove_component.sh "$NAME"
done < "$COMPONENTS_FILE"
