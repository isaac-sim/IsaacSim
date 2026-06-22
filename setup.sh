#!/usr/bin/env bash

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

set -e
set -u

# Tested on:
#  - Ubuntu 22.04 / 24.04

install_hostdeps_ubuntu() {
    # Historically we take care of installing these deps for devs.
    echo "Warning: about to run potentially destructive apt-get commands."
    echo "         waiting 5 seconds..."
    sleep 5
    sudo apt-get update
    # Removed python2.7 as it is EOL. Added build-essential and git which are commonly needed.
    sudo apt-get install -y curl git build-essential
}

do_usermod_and_end() {
    # $USER can be unset, $(whoami) works more reliably.
    sudo usermod -aG docker $(whoami)
    echo "You need to log out and back in for your environment to pick up 'docker' group membership."
    sleep 3
    echo "Attempting to force group membership reload for this shell. You may be prompted for your account password."
    set -x
    exec su --login $(whoami)
}

install_docker_ubuntu() {
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg
    
    sudo install -m 0755 -d /etc/apt/keyrings
    # Check if the key already exists, if so remove it to ensure we get a fresh one
    if [ -f /etc/apt/keyrings/docker.gpg ]; then
        sudo rm -f /etc/apt/keyrings/docker.gpg
    fi
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    # Add the repository to Apt sources:
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

    do_usermod_and_end
}

main() {
    sudo -V >& /dev/null && HAVE_SUDO=1 || HAVE_SUDO=0
    if [[ "$HAVE_SUDO" == "0" ]]; then
        echo "Install 'sudo' before running this script."
        exit 1
    fi

    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        if [[ "x$NAME" == "xUbuntu" ]]; then
            install_hostdeps_ubuntu
            if ! command -v docker &> /dev/null; then
                install_docker_ubuntu
            else
                echo "Docker is already installed."
            fi
        else
            echo "Only Ubuntu based OSs are supported."
            exit 1
        fi
    else
        echo "Unable to determine distribution. Can't read /etc/os-release" | tee /dev/stderr
        exit 1
    fi
}

main
