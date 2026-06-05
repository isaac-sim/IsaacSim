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

# Isaac Sim Script Validator
# Usage: validate_sim.sh --level <1|2|3> <script.py> [--output-dir <dir>] [--timeout <seconds>]

set -euo pipefail

LEVEL=1
SCRIPT=""
OUTPUT_DIR=""
TIMEOUT=120
ERRORS=0
WARNINGS=0

usage() {
    echo "Usage: $0 --level <1|2|3> <script.py> [--output-dir <dir>] [--timeout <seconds>]"
    exit 1
}

log_pass() { echo "  ✅ $1"; }
log_fail() { echo "  ❌ $1"; ERRORS=$((ERRORS + 1)); }
log_warn() { echo "  ⚠️  $1"; WARNINGS=$((WARNINGS + 1)); }

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
        --level) LEVEL="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        -*) usage ;;
        *) SCRIPT="$1"; shift ;;
    esac
done

[[ -z "$SCRIPT" ]] && usage
[[ ! -f "$SCRIPT" ]] && echo "Script not found: $SCRIPT" && exit 1

echo "🔍 Validating: $SCRIPT (Level $LEVEL)"
echo ""

# --- Level 1: Syntax ---
echo "📋 Level 1: Syntax Checks"

if python3 -m py_compile "$SCRIPT" 2>/dev/null; then
    log_pass "Python syntax valid"
else
    log_fail "Python syntax errors"
fi

if grep -q "from isaacsim import SimulationApp" "$SCRIPT"; then
    log_pass "SimulationApp import found"
else
    log_fail "Missing 'from isaacsim import SimulationApp'"
fi

if grep -q "SimulationApp(" "$SCRIPT"; then
    log_pass "SimulationApp instantiated"
else
    log_fail "SimulationApp never instantiated"
fi

if grep -q "simulation_app.close()" "$SCRIPT"; then
    log_pass "simulation_app.close() called"
else
    log_warn "Missing simulation_app.close() — may leak resources"
fi

[[ "$LEVEL" -lt 2 ]] && { echo ""; echo "Level 1 complete. Errors: $ERRORS, Warnings: $WARNINGS"; exit $ERRORS; }

# --- Level 2: Structure ---
echo ""
echo "📋 Level 2: Structure Checks"

if grep -q '"headless":\s*True\|"headless": True' "$SCRIPT"; then
    log_pass "Headless mode enabled"
else
    log_warn "Headless mode not detected — this VM has no display"
fi

if grep -q "create_new_stage\|open_stage" "$SCRIPT"; then
    log_pass "Stage creation/loading found"
else
    log_warn "No stage creation detected"
fi

# Check SimulationApp is created before other isaacsim imports
SIMAPP_LINE=$(grep -n "SimulationApp(" "$SCRIPT" | head -1 | cut -d: -f1)
FIRST_ISAACSIM=$(grep -n "from isaacsim\.\|import isaacsim\." "$SCRIPT" | grep -v "SimulationApp" | head -1 | cut -d: -f1)
if [[ -n "$SIMAPP_LINE" && -n "$FIRST_ISAACSIM" ]]; then
    if [[ "$SIMAPP_LINE" -lt "$FIRST_ISAACSIM" ]]; then
        log_pass "SimulationApp created before other isaacsim imports"
    else
        log_fail "SimulationApp must be created BEFORE other isaacsim imports (line $SIMAPP_LINE vs $FIRST_ISAACSIM)"
    fi
fi

[[ "$LEVEL" -lt 3 ]] && { echo ""; echo "Level 2 complete. Errors: $ERRORS, Warnings: $WARNINGS"; exit $ERRORS; }

# --- Level 3: Runtime ---
echo ""
echo "📋 Level 3: Runtime Checks"

echo "  ⏳ Running script (timeout: ${TIMEOUT}s)..."
RUNTIME_LOG=$(mktemp)
if timeout "$TIMEOUT" python3 "$SCRIPT" > "$RUNTIME_LOG" 2>&1; then
    log_pass "Script executed successfully"
else
    EXIT_CODE=$?
    if [[ $EXIT_CODE -eq 124 ]]; then
        log_warn "Script timed out after ${TIMEOUT}s (may be expected for long simulations)"
    else
        log_fail "Script failed with exit code $EXIT_CODE"
        echo "  📄 Last 20 lines of output:"
        tail -20 "$RUNTIME_LOG" | sed 's/^/     /'
    fi
fi

if [[ -n "$OUTPUT_DIR" ]]; then
    if [[ -d "$OUTPUT_DIR" ]]; then
        FILE_COUNT=$(find "$OUTPUT_DIR" -type f | wc -l)
        if [[ "$FILE_COUNT" -gt 0 ]]; then
            log_pass "Output directory has $FILE_COUNT files"
            # Check for empty files
            EMPTY_COUNT=$(find "$OUTPUT_DIR" -type f -empty | wc -l)
            if [[ "$EMPTY_COUNT" -gt 0 ]]; then
                log_warn "$EMPTY_COUNT empty files in output directory"
            fi
        else
            log_fail "Output directory is empty"
        fi
    else
        log_fail "Output directory does not exist: $OUTPUT_DIR"
    fi
fi

rm -f "$RUNTIME_LOG"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Validation complete. Errors: $ERRORS, Warnings: $WARNINGS"
[[ $ERRORS -gt 0 ]] && echo "❌ FAILED" && exit 1
[[ $WARNINGS -gt 0 ]] && echo "⚠️  PASSED with warnings" && exit 0
echo "✅ PASSED" && exit 0
