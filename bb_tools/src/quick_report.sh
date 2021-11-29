#!/usr/bin/env bash

#############################################
#
# Script to help user create new error reports
#
# author:   Cyrus Chan
# modified: 2021-11-29 1640 HKT
#
#############################################

set -o errexit
set -o pipefail
set -o nounset
# set -o xtrace

__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__base="$(basename ${__file} .sh)"
__root="$(cd "$(dirname "${__dir}")" && pwd)" # <-- change this as it depends on your app

usage() {
    echo -e "Usage: ${__base}.sh [-h]"
    echo -e "-h\t show this Help message"
    # echo -e "WARNING: use these shortcuts with care"
}

while getopts ":h" opt; do
  case ${opt} in
    [h?] ) usage && exit;;
  esac
done

# Setup logs and make sure ROS is up
#[ -z ${ROS_DISTRO+x} ] && >&2 echo -e "Is ROS installed on the machine?" && exit 1

# Find device name from yaml or use the default "LIONEL"
set +o errexit; set +o pipefail
DEVICE_NAME=$(cat ~/catkin_ws/src/boothbot/common/scripts/common/device_name.yaml 2>/dev/null | grep -oE "\b[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[0-9]{1,}\b") 
DEVICE_NAME=${DEVICE_NAME:-LIONEL}
set -o errexit; set -o pipefail

# Setting up the report file
TEMP_REPORT=$(mktemp /tmp/${__base}.XXXXXXXXX.log)
BOOTHBOT_DIR="${HOME}/catkin_ws/src/boothbot"
( echo "Date: $(date -R)"; echo "Pwd: $(pwd)";
echo "Device: ${DEVICE_NAME}";)>>$TEMP_REPORT

# pushd $BOOTHBOT_DIR
pushd ~/aug-robotics/boothbot >/dev/null 2>&1
git rev-parse --is-inside-work-tree
( echo "---"; 
echo "main repo: $(git rev-parse --show-toplevel)";
echo "remote: $(git remote -v | head -n1 | sed 's/(fetch)//g')";
echo "branch: $(git branch | sed -n '/\* /s///p')";
echo "commit: $(git rev-parse HEAD)"; ) >>$TEMP_REPORT

# Get git info from the root level
for file in *; do
    if [ -d "$file" ]; then
        pushd $file >/dev/null 2>&1
        if [ "$(git rev-parse --show-superproject-working-tree)" != "" ]; then
          ( echo "---"; 
          echo "main repo: $(git rev-parse --show-toplevel)";
          echo "remote: $(git remote -v | head -n1 | sed 's/(fetch)//g')";
          echo "branch: $(git branch | sed -n '/\* /s///p')";
          echo "commit: $(git rev-parse HEAD)"; ) >>$TEMP_REPORT
        fi
        popd >/dev/null 2>&1
    fi
done

( echo "---"; echo "Tester's remarks:";
echo "# Issue desc:";
echo "# Error logs:";
echo ""; 
echo "# Please use 'Save as' in the editor to move this file out of /tmp";
echo ""; )>>$TEMP_REPORT

echo "# - Top-level git status dump -" >>$TEMP_REPORT
git status | sed "s/^/# /g" >>$TEMP_REPORT

popd

# Open a new editor window, then tar everything
$EDITOR "$TEMP_REPORT"
RND_NAME="logs_$(echo $TEMP_REPORT | cut -d'.' -f2)"
tar --ignore-failed-read -czhf $RND_NAME.tar.gz ~/.ros/log/latest $TEMP_REPORT
if [ $? -eq 0 ]; then
  echo "Logs archived: see ./$RND_NAME.tar.gz"
else
  echo "Failed to pack log files, report is still availible at $TEMP_REPORT"
fi