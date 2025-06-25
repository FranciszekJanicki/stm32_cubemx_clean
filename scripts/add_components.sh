#!/bin/bash
set -e

COMPONENTS_FILE="components/components.txt"

if [ ! -f "$COMPONENTS_FILE" ]; then
  echo "File $COMPONENTS_FILE not found"
  exit 1
fi

while read -r REPO NAME; do
  # skip empty lines or lines starting with #
  [[ -z "$REPO" || "$REPO" =~ ^# ]] && continue
  ./scripts/add_component.sh "$REPO" "$NAME"
done < "$COMPONENTS_FILE"
