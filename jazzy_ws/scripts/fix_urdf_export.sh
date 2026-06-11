#!/usr/bin/env bash
# Post-process a fresh LinkForge export of arm_w_mm.urdf for use in this workspace.
# Run from jazzy_ws/ after every re-export, then: colcon build --packages-select es165_moveit
set -euo pipefail
URDF="$(dirname "$0")/../src/es165_moveit/urdf/arm_w_mm.urdf"

# 1. Mesh paths -> package URIs (meshes install to share/es165_moveit/urdf/meshes/).
#    Handles both raw exports (meshes/...) and partially-fixed paths missing the urdf/ segment.
sed -i -e 's|filename="meshes/|filename="package://es165_moveit/urdf/meshes/|g' \
       -e 's|filename="package://es165_moveit/meshes/|filename="package://es165_moveit/urdf/meshes/|g' "$URDF"

# 2. Strip the inertial block from the root "world" link (KDL rejects root-link inertia).
python3 - "$URDF" <<'EOF'
import re, sys
p = sys.argv[1]
s = open(p).read()
s, n = re.subn(
    r'<link name="world">\s*<inertial>.*?</inertial>\s*</link>',
    '<!-- No inertial on the root link: KDL does not support root-link inertia -->\n  <link name="world" />',
    s, flags=re.DOTALL)
open(p, "w").write(s)
print(f"world-link inertia blocks removed: {n}")
EOF

echo "mesh refs fixed: $(grep -c 'package://es165_moveit/urdf/meshes/' "$URDF")"
