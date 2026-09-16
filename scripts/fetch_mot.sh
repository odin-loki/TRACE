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

# These annotations are the MOTChallenge authors' work, not this project's, and
# motchallenge.net's terms - non-commercial research use, with citation - bind
# you directly. See THIRD-PARTY.md.

for archive in MOT17Labels MOT20Labels; do
    # A VALIDATED archive is what counts as "already fetched". The marker used
    # to be the mere existence of the .zip, so a transfer cut off halfway left
    # a truncated file behind and every later run saw it, said "already
    # present", and skipped it - one bad download poisoned the cache
    # permanently, and the only cure was knowing to delete a file the script
    # never mentioned. Checking the extracted directory instead does not work
    # either: MOT17Labels.zip unpacks to train/ and test/, not to a directory
    # of its own, so that marker never matches and the archive is fetched every
    # single run. Testing the zip is the one check that is right for both.
    if [ -f "$DEST/$archive.zip" ]; then
        if unzip -tq "$DEST/$archive.zip" >/dev/null 2>&1; then
            echo "$archive: already present"
            continue
        fi
        echo "$archive: cached archive is corrupt, refetching" >&2
        rm -f "$DEST/$archive.zip"
    fi

    echo "fetching $archive..."
    tmp="$DEST/.$archive.zip.part"
    rm -f "$tmp"
    # Nothing is left behind by a failed attempt, however it fails.
    trap 'rm -f "$tmp"' EXIT
    if ! curl -fsSL --retry 3 --retry-delay 2 -o "$tmp" \
         "https://motchallenge.net/data/$archive.zip"; then
        echo "$archive: download failed - nothing kept, run again to retry" >&2
        exit 1
    fi
    # Verify before committing to it: a 200 response carrying an error page is
    # still a failed download, and unzip is the cheapest test of that.
    if ! unzip -tq "$tmp" >/dev/null 2>&1; then
        echo "$archive: downloaded file is not a valid archive - nothing kept" >&2
        exit 1
    fi
    unzip -q -o "$tmp" -d "$DEST"
    mv "$tmp" "$DEST/$archive.zip"
    trap - EXIT
done

echo
echo "done. Replay with:"
echo "  ./build/src/apps/trace_mot $DEST/train"
