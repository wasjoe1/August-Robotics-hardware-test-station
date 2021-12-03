#!/usr/bin/env bash

# This script is using `rsync` to transfer local log folder to
# remote machine. On remote machine, each sent log files will
# be in a summary folder with their hostname.
# INSTALLTION: `rsync` (apt install -y rsync)

# TODO make sure the below content is fixed when using on product
LOCALHOSTNAME=$(hostname)
SOURCE="$HOME/.ros/log/"  # the end / is important
TARGET="/home/guiding/Desktop/${LOCALHOSTNAME}"
REMOTEUSER="guiding"


####################################
# GET remote host IP or domainname #
####################################
REMOTEHOST=""
DELETEOLD=false  # if True, will delete log folder more than 7 days

show_help () {
    echo "[ERROR] Usage: ./remote_trans.sh -h <REMOTEHOST> [-r]"
}

while getopts "h:r" opt; do
    case "$opt" in
        h)  REMOTEHOST=${OPTARG}
            ;;
        r)  DELETEOLD=true
            ;;
    esac
done

if [[ "$REMOTEHOST" == "" ]]; then
    show_help
    exit 1
fi

###############
# RUN `rsync` #
###############

# -a archive mode
# --no-links Skip all link file/folder, espacially for `latest` folder of ROS.
#            It has to put after `-a`.
# --info=progress2 Better way to show process info.
# --timeout=60 If not data transfer in 60 secs, will raise error.
# --rsync-path="mkdir -p <remote_folder> && rsync"
#     Will create target folder on remote machine if it doesn't exsit.
#     https://stackoverflow.com/questions/1636889/rsync-how-can-i-configure-it-to-create-target-directory-on-server
# -e "ssh -p 3022" This useful if remote machine's ssh port is not 22.
# SOURCE Folder which will be sync.
# TARGET Saving folder on remote machine.
rsync -a --no-links \
    --info=progress2 \
    # -e "ssh -p 3022" \
    --timeout=60 \
    --rsync-path="mkdir -p ${TARGET} && rsync" \
    "${SOURCE}" \
    "${REMOTEUSER}"@"${REMOTEHOST}":"${TARGET}"\
/


if [[ "$DELETEOLD" == true ]]; then
    # Only check 1st depth folders or files which overduded 7 days.
    find ${SOURCE} -mindepth 1 -maxdepth 1 -mtime +7 -delete
    echo "DELETED all overduded 7 days log files"
fi
