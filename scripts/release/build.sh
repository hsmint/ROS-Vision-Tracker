#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 2 || ! $1 =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]]; then
  echo "Usage: bash scripts/release/build.sh VERSION OUTPUT_DIRECTORY" >&2
  exit 2
fi

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
version=$1
mkdir -p -- "$2"
output=$(cd -- "$2" && pwd)
ros_setup=/opt/ros/lyrical/setup.bash
if [[ ! -f $ros_setup ]]; then
  echo "ROS Lyrical is required: $ros_setup is missing" >&2
  exit 1
fi
source "$ros_setup"
set -u

work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
packages=(xmen xmen_bringup xmen_control xmen_description xmen_tracker)
mkdir -p "$work/src" "$work/archive/dependencies"
for package in "${packages[@]}"; do
  cp -R "$root/$package" "$work/src/"
  mkdir -p "$work/archive/dependencies/$package"
  cp "$root/$package/package.xml" "$work/archive/dependencies/$package/"
done

cd "$work"
# Copy installations rather than creating links back into the build workspace.
colcon build --base-paths "$work/src" --merge-install \
  --install-base "$work/archive/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
cp "$root/scripts/release/check-runtime.py" "$work/archive/"
cp "$root/README.md" "$work/archive/"
printf '%s\n' "$version" > "$work/archive/VERSION"

name="xmen-$version-ubuntu-26.04-ros-lyrical-arm64.tar.gz"
tar -czf "$work/$name" -C "$work/archive" .
mkdir "$work/relocated"
tar -xzf "$work/$name" -C "$work/relocated"
# Remove the original paths so accidental references cannot pass the smoke test.
rm -rf "$work/archive" "$work/src" "$work/build" "$work/log"
(
  set +u
  source "$work/relocated/install/setup.bash"
  set -u
  cd "$work/relocated"
  python3 check-runtime.py
  for package in "${packages[@]}"; do
    ros2 pkg prefix "$package"
  done
)
cp "$work/$name" "$output/$name"
cd "$output"
sha256sum "$name" > "$name.sha256"
echo "Release archive: $output/$name"
