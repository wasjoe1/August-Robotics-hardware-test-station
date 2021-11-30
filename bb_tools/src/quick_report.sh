#!/usr/bin/env bash

#############################################
#
# Script to help user create new error reports
#
# author:   Cyrus Chan
# modified: 2021-11-30 1413 HKT
#
#############################################

set -o errexit
set -o pipefail
set -o nounset

__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__base="$(basename ${__file} .sh)"

if [ -z ${HOME+x} ]; then
    HOME="/home/$USER"
fi

# Setup usage options
usage() {
    echo -e "Usage: $(basename ${__file}).sh [-h] [-v]"
    echo -e "-h\t show this Help message"
    echo -e "-v\t Verbose output, equiv. to 'set -x'"
    # echo -e "WARNING: use these shortcuts with care"
}

while getopts ":hv" opt; do
    case ${opt} in
        v    ) set -o xtrace;;
        [h?] ) usage && exit;;
    esac
done

# Setup signal traps
catch_err() {
    echo "Error occured on $(caller), offending command:"
    echo "'${BASH_COMMAND}'"
    echo "Aborted."
    exit 1
}
trap catch_err ERR

catch_int() {
    echo -e "User interrupt recieved, terminating"
    exit 1
}
trap catch_int SIGHUP SIGINT SIGTERM

# Setup logs and make sure ROS is up
#[ -z ${ROS_DISTRO+x} ] && >&2 echo -e "Is ROS installed on the machine?" && exit 1

# Find device name from yaml or use the default "LIONEL"
set +o errexit; set +o pipefail; trap - ERR;
DEVICE_NAME=$(cat ~/catkin_ws/src/boothbot/common/scripts/common/device_name.yaml 2>/dev/null | grep -oE "\b[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[0-9]{1,}\b")
DEVICE_NAME=${DEVICE_NAME:-LIONEL}
set -o errexit; set -o pipefail; trap catch_err ERR;

# Setting up the report file
TEMP_REPORT=$(mktemp /tmp/${__base}.XXXXXXXXX.log)
BOOTHBOT_DIR="$HOME/catkin_ws/src/boothbot"
( echo "Date: $(date -R)"; echo "Pwd: $(pwd)";
echo "Device: ${DEVICE_NAME}"; )>>$TEMP_REPORT

BOOTHBOT_DIR="$HOME/aug-robotics/boothbot"
pushd $BOOTHBOT_DIR >/dev/null 2>&1
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || >&2 echo "Not a git repository: $(pwd)"
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

# Template fields for jotting remarks
( echo "---"; echo "Tester's remarks:";
echo "# Issue desc:";
echo "# Error logs:";
echo "";
echo "# Remember to save the modifications (if any),";
echo "# after you exit the editor, this file will also be copied to the tar file";
echo ""; )>>$TEMP_REPORT

echo "# - Top-level git status dump -" >>$TEMP_REPORT
git status | sed "s/^/# /g" >>$TEMP_REPORT

popd >/dev/null 2>&1

# Set a temp default if $EDITOR is no set in env
if [ -z ${EDITOR+x} ]; then
    >&2 echo "No default editor found. Can be specified using EDITOR='<command>' $(basename ${__file})"
    if hash nvim; then
        >&2 echo "Found editor 'nvim', invoking..."
        EDITOR="nvim"
    elif hash vi; then
        >&2 echo "Found editor 'vi', invoking..."
        EDITOR="vi"
    else
        >&2 echo "Can't find a valid word editor, skipping."
    fi
fi

# Open a new editor window, then tar everything
[ -n ${EDITOR+x} ] && $EDITOR "$TEMP_REPORT"
RND_NAME="logs_$(echo $TEMP_REPORT | cut -d'.' -f2)"
tar --ignore-failed-read -czhf $RND_NAME.tar.gz ~/.ros/log/latest $TEMP_REPORT
if [ $? -eq 0 ]; then
    echo "Logs archived: see ./$RND_NAME.tar.gz"
else
    echo "Failed to pack log files, report is still availible at $TEMP_REPORT"
fi