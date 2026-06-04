#!/bin/bash
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

# Validate SDG output directory
# Usage: validate_sdg_output.sh <output_dir> [expected_frames]

OUTPUT_DIR="${1:?Usage: validate_sdg_output.sh <output_dir> [expected_frames]}"
EXPECTED="${2:-0}"

echo "=== SDG Output Validation ==="
echo "Directory: $OUTPUT_DIR"

if [ ! -d "$OUTPUT_DIR" ]; then
    echo "FAIL: Output directory does not exist"
    exit 1
fi

# Count RGB images
RGB_COUNT=$(find "$OUTPUT_DIR" -name "*.png" -o -name "*.jpg" | wc -l)
echo "RGB images: $RGB_COUNT"

# Count annotation files
ANNO_COUNT=$(find "$OUTPUT_DIR" -name "*.json" -o -name "*.npy" | wc -l)
echo "Annotation files: $ANNO_COUNT"

# Check for empty images (< 1KB likely means black frame)
EMPTY=$(find "$OUTPUT_DIR" -name "*.png" -size -1k | wc -l)
if [ "$EMPTY" -gt 0 ]; then
    echo "WARN: $EMPTY potentially empty/black images found (< 1KB)"
fi

# Check expected frame count
if [ "$EXPECTED" -gt 0 ] && [ "$RGB_COUNT" -lt "$EXPECTED" ]; then
    echo "FAIL: Expected $EXPECTED frames, got $RGB_COUNT"
    exit 1
fi

echo "=== Validation PASSED ==="
