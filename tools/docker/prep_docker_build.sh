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

# Import section marker functions if we are in CI
if [[ "${CI}" == "true" && -f ./tools/ci/gitlab/section_marker.sh ]]; then
    source ./tools/ci/gitlab/section_marker.sh
else
    section_start() { :; }
    section_end()   { :; }
fi


# Parse command line arguments
SKIP_DEDUPE=false
RUN_BUILD=false
CONTAINER_PLATFORM=linux-x86_64

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Prepares Docker build by generating rsync script and copying necessary files.

OPTIONS:
    --build          Build Isaac Sim
    --x86_64         Build x86_64 container (default)
    --aarch64        Build aarch64 container
    --skip-dedupe    Skip the deduplication process
    --help, -h       Show this help message

EOF
}

build_function() {
    echo "Starting build sequence..."

    echo "Running build.sh -r"
    if ! ./build.sh -r; then
        echo "Error: build.sh -r failed" >&2
        return 1
    fi

    echo "Build sequence completed successfully!"
}

# Parse command line options
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-dedupe)
            SKIP_DEDUPE=true
            shift
            ;;
        --build)
            RUN_BUILD=true
            shift
            ;;
        --x86_64)
            CONTAINER_PLATFORM=linux-x86_64
            shift
            ;;
        --aarch64)
            CONTAINER_PLATFORM=linux-aarch64
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Use --help for usage information" >&2
            exit 1
            ;;
    esac
done


# Run build sequence if --build was specified
if [[ "$RUN_BUILD" == "true" ]]; then
    echo ""
    # Remove stale Docker context before rebuild
    if [[ -d "_container_temp" ]]; then
        echo "Removing existing _container_temp for fresh Docker context..."
        rm -rf _container_temp
    fi
    build_function
    if [[ $? -ne 0 ]]; then
        echo "Build sequence failed, exiting with error code 1" >&2
        exit 1
    fi
fi

# Check that _build/linux-x86_64 or _build/linux-aarch64 exists
if [[ ! -d "_build/${CONTAINER_PLATFORM}/release" ]]; then
    echo "Error: _build/${CONTAINER_PLATFORM}/release does not exist" >&2
    echo "Please rerun the script with --build" >&2
    exit 1
fi


# Goes a bit faster if you have used PM_PATH_TO_SANDBOX="_"
section_start "installing_python_requirements" "Installing Python requirements"
if ! tools/packman/python.sh -m pip install -r tools/docker/requirements.txt; then
    echo "Failed to install Python requirements" >&2
    exit 1
fi
section_end "installing_python_requirements"


section_start "generating_rsync_script" "Generating rsync script"
if ! tools/packman/python.sh tools/docker/generate_rsync_script.py --platform ${CONTAINER_PLATFORM} --target isaac-sim-docker --output-folder _container_temp; then
    echo "Failed to generate rsync script" >&2
    exit 1
fi
section_end "generating_rsync_script"


section_start "running_rsync_script" "Running rsync script"
./generated_rsync_package.sh
section_end "running_rsync_script"



echo "Removing empty folders"
# Remove empty folders from container temp
find _container_temp -type d -empty -delete


echo "Copying data from tools/docker/data"

cp -r tools/docker/data/* _container_temp


find_chained_symlinks(){
    echo "Searching for chained symlinks"
    count=$((0))
    find $1 -type l | while read -r symlink; do
        target="$(dirname "$symlink")/$(readlink "$symlink")"
        if [ -L "$target" ]; then
            target_of_target="$(dirname "$target")/$(readlink "$target")"
            echo "Correcting chained link $(basename "$symlink") -> $(basename "$target") -> $(basename "$target_of_target")"
            ln -sfr "$target_of_target" "$symlink"
            count=$((count + 1))
        fi
    done
    echo "Replaced $count chained symlinks"
}




dedupe_folder(){
    echo "Starting a dedupe of $1"
    hash=""
    true_path=""
    echo "Searching for duplicates (ignoring paths with spaces)"
    echo "Initial find command can take a while, started at $(date)"
    # Use ! -regex to exclude paths containing spaces
    data=$(find $1 -type f ! -regex '.* .*' ! -empty -exec sh -c 'echo $(md5sum "$1" | cut -f1 -d " ") $(du -h "$1")' _ {} \; | sort | uniq -w32 -dD)
    echo "Initial find command resolved.  Deduplicating files now at $(date)"
    if [[ -n "$data" ]]; then
        count=$((0))
        dupe_count=$((0))
        while IFS= read -r LINE; do
            new_hash=$(echo "$LINE" | cut -d " " -f1)
            test_path=$(echo "$LINE" | cut -d " " -f3-)
            # new file check
            if [[ ${new_hash} != ${hash} ]]; then
                count=$((count + 1))
                hash=${new_hash}
                true_path="${test_path}"
            else
                dupe_count=$((dupe_count + 1))
                rm "${test_path}"
                ln -sr "${true_path}" "${test_path}"
            fi
        done < <(printf '%s\n' "$data")
        echo "Removed ${dupe_count} duplicates of ${count} files"
        echo "Note: Files with spaces in their paths were skipped"
        echo "Deduplication complete at $(date)"
    else
        echo "No duplicated files found at $(date)"
    fi
    find_chained_symlinks $1
}


# Run deduplication unless --skip-dedupe was specified
if [[ "$SKIP_DEDUPE" != "true" ]]; then
    echo "Running deduplication (use --skip-dedupe to skip this step)"
    section_start "deduplicating_files" "Deduplicating files"
    dedupe_folder _container_temp
    section_end "deduplicating_files"
else
    echo "Skipping deduplication as requested"
fi

# Clean up our venv
rm -rf .container_venv
