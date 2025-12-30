#!/bin/bash

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

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Display help function
function display_help {
  echo "Usage: $0 [workspace] [colcon-args...]"
  echo ""
  echo "Build a specific ROS2 workspace using colcon."
  echo ""
  echo "Arguments:"
  echo "  workspace    Specify which workspace to build: 'jazzy' (required)"
  echo "  colcon-args  Additional arguments to pass to colcon build"
  echo ""
  echo "Examples:"
  echo "  $0 jazzy                     # Build jazzy_ws"
  echo "  $0 jazzy --packages-select pkg1 pkg2  # Build specific packages in jazzy_ws"
  echo ""
}

# Check if workspace is provided
if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  display_help
  exit 0
fi

WORKSPACE=$1
shift  # Remove first argument, rest are colcon args

# Validate workspace
if [ "$WORKSPACE" != "jazzy" ]; then
  echo "Error: Invalid workspace '$WORKSPACE'"
  echo "Valid workspace is: jazzy"
  exit 1
fi

WORKSPACE_DIR="${SCRIPT_DIR}/${WORKSPACE}_ws"

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo "Error: Workspace directory not found: $WORKSPACE_DIR"
  exit 1
fi

# Check if src directory exists
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Error: Source directory not found: $WORKSPACE_DIR/src"
  exit 1
fi

echo "Building $WORKSPACE workspace..."
echo "Workspace directory: $WORKSPACE_DIR"
echo ""

# Change to workspace directory and build
cd "$WORKSPACE_DIR"
colcon build "$@"

echo ""
echo "Build complete for $WORKSPACE workspace!"
echo "To use this workspace, source: source $WORKSPACE_DIR/install/setup.bash"

