#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>
# Fetch MOTChallenge annotations for TRACE's real-data replay.
#
# Only the label archives are downloaded (~30 MB total). TRACE consumes
# detections rather than pixels, so the multi-gigabyte image sets are not
# needed.
set -euo pipefail

DEST="${1:-./data/mot}"
mkdir -p "$DEST"

for archive in MOT17Labels MOT20Labels; do
    if [ -d "$DEST/$archive" ] || [ -f "$DEST/$archive.zip" ]; then
        echo "$archive: already present"
        continue
    fi
    echo "fetching $archive..."
    curl -fsSL -o "$DEST/$archive.zip" "https://motchallenge.net/data/$archive.zip"
    unzip -q -o "$DEST/$archive.zip" -d "$DEST"
done

echo
echo "done. Replay with:"
echo "  ./build/src/apps/trace_mot $DEST/train"
