#!/usr/bin/env bash

#############################################
#
# Script to scrub and rename rosbags
#
# author:   Cyrus Chan
# modified: 2021-11-30 1630 HKT
#
#############################################

set -o errexit
set -o pipefail
set -o nounset
# set -o xtrace

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

# Gethering candidates for cleanup
ROS_TEMP_DIR="$HOME/.ros"
# CANDIDATE_COUNT="$(find $ROS_TEMP_DIR -type f -name '*.bag.active' -print0 | wc -l)"
# echo "$CANDIDATE_COUNT '.active' bags found"

shopt -s globstar
for file in $ROS_TEMP_DIR/**/*.active; do

    [ -f ${file:-} ] || continue

    set +o errexit; set +o pipefail; trap - ERR;

    # remove_suffix_default_yes()
    read -r -p "Strip the suffix form ${file:-}? [Y/n] " response
    case "$response" in
        [yY][eE][sS]|[yY]|'')
            file_rename="${file%%.active}"
            mv $file $file_rename
            ;;
        *)
            ;;
    esac

    # reindex_default_no()
    file=${file_rename:-$file}
    rosbag info $file >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        read -r -p "This rosbag $(basename ${file:-}) is corrupted, reindex now? [y/N]" response
        case "$response" in
            [yY][eE][sS]|[yY])
                orig_file="$(echo $file | sed 's/.bag/.orgi.bag/g')"
                rosbag reindex $file && rm $orig_file
                ;;
            *)
                ;;
        esac
    fi

    set -o errexit; set -o pipefail; trap catch_err ERR;

done

