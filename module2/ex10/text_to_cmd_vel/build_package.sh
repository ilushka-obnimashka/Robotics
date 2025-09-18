ROS2_WS_PATH="$HOME/ros2_ws"
ROS2_DISTRO="jazzy"
PKG_NAME="text_to_cmd_vel"
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/$ROS2_DISTRO/setup.bash

if [ -d "$ROS2_WS_PATH" ]; then
    echo "Workspace $ROS2_WS_PATH exists"
else
    echo "Workspace $ROS2_WS_PATH does not exist"
    mkdir -p $ROS2_WS_PATH/src
fi

cd $ROS2_WS_PATH

if [ -d "src/$PKG_NAME" ]; then
    echo "$PKG_NAME already exists in workspace"
    rm -rf "src/$PKG_NAME"
fi

echo "Copying package to workspace..."
cp -r "$CURRENT_DIR" "src/$PKG_NAME"

echo "Building package $PKG_NAME..."
colcon build --symlink-install --packages-select $PKG_NAME

source install/setup.bash

echo "Checking installation..."
if ros2 pkg list | grep -q "^$PKG_NAME$"; then
    echo "Package $PKG_NAME successfully installed!"
else
    echo "Package not found in system."
    exit 1
fi
