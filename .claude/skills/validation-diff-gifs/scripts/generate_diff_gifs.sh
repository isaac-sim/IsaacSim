#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Generate per-camera difference GIFs between captured and golden images.
# Usage: ./generate_diff_gifs.sh <captured_dir> <golden_dir> [amplify] [fps]
#   captured_dir  Root of a capture run (contains Robots/Robot_*/...)
#   golden_dir    Root of golden data  (contains Robots/Robot_*/...)
#   amplify       Multiply difference pixel values by this factor (default: 10)
#   fps           Frames per second for the output GIF (default: 5)
#
# Requires: ImageMagick (composite, convert)
#
# For each rgb/ directory found under captured_dir, the script:
#   1. Matches captured PNGs to golden PNGs by filename (timestamp-based)
#   2. Computes pixel-wise absolute difference via `composite -compose difference`
#   3. Amplifies the result for visibility
#   4. Combines all difference frames into a looping GIF
#   5. Writes diff_animation.gif in the same rgb/ directory

set -euo pipefail

CAPTURED_DIR="${1:?Usage: $0 <captured_dir> <golden_dir> [amplify] [fps]}"
GOLDEN_DIR="${2:?Usage: $0 <captured_dir> <golden_dir> [amplify] [fps]}"
AMPLIFY="${3:-10}"
FPS="${4:-5}"
DELAY=$((100 / FPS))

if ! command -v composite &>/dev/null || ! command -v convert &>/dev/null; then
    echo "ERROR: ImageMagick (composite, convert) is required but not found." >&2
    exit 1
fi

found=0
for rgb_dir in $(find "$CAPTURED_DIR" -type d -name rgb | sort); do
    rel_path="${rgb_dir#$CAPTURED_DIR/}"
    golden_rgb="$GOLDEN_DIR/$rel_path"

    if [ ! -d "$golden_rgb" ]; then
        echo "SKIP $rel_path (no matching golden directory)"
        continue
    fi

    camera_name=$(echo "$rel_path" | sed 's|/rgb$||')
    echo "Processing: $camera_name"

    diff_frames=""
    matched=0
    skipped=0
    for cap_png in $(ls "$rgb_dir"/rgb_*.png 2>/dev/null | sort -t_ -k2 -g); do
        basename=$(basename "$cap_png")
        golden_png="$golden_rgb/$basename"
        if [ ! -f "$golden_png" ]; then
            skipped=$((skipped + 1))
            continue
        fi
        diff_tmp="$rgb_dir/.diff_tmp_${basename}"
        composite "$cap_png" "$golden_png" -compose difference "$diff_tmp"
        convert "$diff_tmp" -evaluate multiply "$AMPLIFY" -clamp "$diff_tmp"
        diff_frames="$diff_frames $diff_tmp"
        matched=$((matched + 1))
    done

    if [ -n "$diff_frames" ]; then
        convert -delay "$DELAY" -loop 0 $diff_frames "$rgb_dir/diff_animation.gif"
        rm -f $diff_frames
        echo "  Created: $rgb_dir/diff_animation.gif ($matched frames, ${AMPLIFY}x amplified, ${FPS}fps)"
        found=$((found + 1))
    else
        echo "  No matching frames found"
    fi
    [ "$skipped" -gt 0 ] && echo "  ($skipped captured frames had no golden match)"
done

if [ "$found" -eq 0 ]; then
    echo "WARNING: No difference GIFs generated. Check that directory structures match." >&2
    exit 1
fi

echo ""
echo "Done. Generated $found diff_animation.gif files."
