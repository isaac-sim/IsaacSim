#!/bin/bash


# Parse command line arguments
SKIP_DEDUPE=false
RUN_BUILD=false
DOCKER_BUILD=false
CONTAINER_PLATFORM=linux-x86_64
BUILDER_IMAGE="isaac-sim-builder:latest"
PACKMAN_CACHE_DIR="$(pwd)/_packman_cache"

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Prepares Docker build by generating rsync script and copying necessary files.

OPTIONS:
    --build          Build Isaac Sim natively (requires GCC 11)
    --docker-build   Build Isaac Sim inside a container (recommended for Ubuntu 24.04+)
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

docker_build_function() {
    echo "Building Isaac Sim inside container..."

    # Build the builder image from the Dockerfile (has GCC 11)
    if ! docker build -t "$BUILDER_IMAGE" -f tools/docker/Dockerfile tools/docker/; then
        echo "Error: Failed to build builder image" >&2
        return 1
    fi

    # Create the packman cache directory
    # This is mounted inside the container so symlinks created during build
    # point to paths that exist on both host and container
    mkdir -p "$PACKMAN_CACHE_DIR"

    # Run build inside container with source mounted
    # Mount packman cache to the SAME path used inside container so symlinks work on host
    # Run with host user's UID/GID to ensure build artifacts have correct ownership
    if ! docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
        -e TERM=xterm-256color \
        -v "$(pwd):/workspace" \
        -v "$PACKMAN_CACHE_DIR:$PACKMAN_CACHE_DIR" \
        -e PM_PACKAGES_ROOT="$PACKMAN_CACHE_DIR" \
        -w /workspace "$BUILDER_IMAGE" \
        -c "touch .eula_accepted && ./build.sh -r"; then
        echo "Error: Containerized build failed" >&2
        return 1
    fi

    echo "Containerized build completed successfully!"
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
        --docker-build)
            DOCKER_BUILD=true
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
    build_function
    if [[ $? -ne 0 ]]; then
        echo "Build sequence failed, exiting with error code 1" >&2
        exit 1
    fi
fi

# Run containerized build if --docker-build was specified
if [[ "$DOCKER_BUILD" == "true" ]]; then
    echo ""
    docker_build_function
    if [[ $? -ne 0 ]]; then
        echo "Containerized build failed, exiting with error code 1" >&2
        exit 1
    fi
fi

# Check that _build/linux-x86_64 or _build/linux-aarch64 exists
if [[ ! -d "_build/${CONTAINER_PLATFORM}/release" ]]; then
    echo "Error: _build/${CONTAINER_PLATFORM}/release does not exist" >&2
    echo "Please rerun the script with --build or --docker-build" >&2
    exit 1
fi


# Prep steps: generate rsync, copy files
# Use container if --docker-build was specified (no host dependencies)
# Otherwise use native Python (original behavior)
if [[ "$DOCKER_BUILD" == "true" ]]; then
    PACKMAN_CACHE_DIR="$(pwd)/_packman_cache"
    
    if ! docker build -q -t "$BUILDER_IMAGE" -f tools/docker/Dockerfile tools/docker/ >/dev/null; then
        echo "Error: Failed to build prep image" >&2
        exit 1
    fi

    if ! docker run --rm --user "$(id -u):$(id -g)" --entrypoint bash \
        -v "$(pwd):/workspace" \
        -v "$PACKMAN_CACHE_DIR:$PACKMAN_CACHE_DIR" \
        -e PM_PACKAGES_ROOT="$PACKMAN_CACHE_DIR" \
        -w /workspace \
        "$BUILDER_IMAGE" \
        -c "
            pip install -q --break-system-packages -r tools/docker/requirements.txt && \
            python3 tools/docker/generate_rsync_script.py --platform ${CONTAINER_PLATFORM} --target isaac-sim-docker --output-folder _container_temp && \
            ./generated_rsync_package.sh && \
            find _container_temp -type d -empty -delete && \
            cp -r tools/docker/data/* _container_temp
        "; then
        echo "Error: Prep failed" >&2
        exit 1
    fi
else
    echo "Preparing Docker build context..."
    
    if ! python3 -m pip install -r tools/docker/requirements.txt; then
        echo "Failed to install Python requirements" >&2
        exit 1
    fi

    if ! python3 tools/docker/generate_rsync_script.py --platform ${CONTAINER_PLATFORM} --target isaac-sim-docker --output-folder _container_temp; then
        echo "Failed to generate rsync script" >&2
        exit 1
    fi

    ./generated_rsync_package.sh

    echo "Removing empty folders"
    find _container_temp -type d -empty -delete

    echo "Copying data from tools/docker/data"
    cp -r tools/docker/data/* _container_temp
fi


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
    dedupe_folder _container_temp
else
    echo "Skipping deduplication as requested"
fi

# Clean up our venv
rm -rf .container_venv
