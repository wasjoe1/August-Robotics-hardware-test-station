#!/usr/bin/env bash

# This file includes funtions that are used in camera exposure tunning scripts.
# Variables used here must be defined in invoking scripts.

function show_notice {
    echo "------------------------------------------------------------------------------------------"
    echo "| Press 'n' to release motor, and pointing camera sets to the reference beacon, then make|"
    echo "| sure that the reference beacon is shown on the screen, then press 'r' to fix the camera|"
    echo "| After putting the beacon inside the screen, press 't' to start auto tuning. When tuning|"
    echo "| process is over, press ‘e’ to quit.                                                    |"
    echo "------------------------------------------------------------------------------------------"
    echo "Now will wait for 10 seconds for launching script"
    sleep 10
    echo "Running..."
}

function show_menu {
    clear
    echo "Target color: ${BEACON_COLOR}. Change color by pressing: '1' for 'BOG', '2' for 'BOR', '3' for 'GOB', '4' for 'GOR', '5' for 'ROB', '6' for 'ROG', '7' for 'BLUE', '8' for 'GREEN', '9' for 'RED'"
    echo "Press 'n' to release motor, 'r' to fix the camera, 't' to start auto tuning, 'e' to exit:"
    echo ""
    echo "当前目标颜色: ${BEACON_COLOR}. 按'1'选择'BOG', '2'选择'BOR', '3'选择'GOB', '4'选择'GOR', '5'选择'ROB', '6'选择'ROG', '7'选择'BLUE' '8'选择'GREEN', '9'选择'RED'"
    echo "取消使能 'n',  使能'r', 开始自动调整‘t’，退出 'e':"
}

function show_loop {
    while true
        do
            read -rsn1 input
            if [ "$input" = "n" ]; then
                eval `tmux send-keys -t "${EXPOSURE_WINDOW}":exposure_tuning_pub "${NEWCMD}" C-m`
            elif [ "$input" = "r" ]; then
                eval `tmux send-keys -t "${EXPOSURE_WINDOW}":exposure_tuning_pub "${RUNCMD}" C-m`
            elif [ "$input" = "t" ]; then
                tmux kill-session -t show_image
                eval `tmux send-keys -t "${EXPOSURE_WINDOW}":exposure_tuning_node "${EXPOCMD}" "${BEACON_COLOR}" C-m`
                echo "Please wait 10s for launching..."
            elif [ "$input" = "1" ]; then
                BEACON_COLOR='BOG'
                show_menu
            elif [ "$input" = "2" ]; then
                BEACON_COLOR='BOR'
                show_menu
            elif [ "$input" = "3" ]; then
                BEACON_COLOR='GOB'
                show_menu
            elif [ "$input" = "4" ]; then
                BEACON_COLOR='GOR'
                show_menu
            elif [ "$input" = "5" ]; then
                BEACON_COLOR='ROB'
                show_menu
            elif [ "$input" = "6" ]; then
                BEACON_COLOR='ROG'
                show_menu
            elif [ "$input" = "7" ]; then
                BEACON_COLOR='BLUE'
                show_menu
            elif [ "$input" = "8" ]; then
                BEACON_COLOR='GREEN'
                show_menu
            elif [ "$input" = "9" ]; then
                BEACON_COLOR='RED'
                show_menu
            elif [ "$input" = "e" ]; then
                break
            fi
        done
}

function run_tuning {
    tmux new-session -d -s show_image -n image
    tmux new-session -d -s "${EXPOSURE_WINDOW}" -n exposure_tuning
    tmux new-window -t "${EXPOSURE_WINDOW}" -n exposure_tuning_pub
    tmux new-window -t "${EXPOSURE_WINDOW}" -n exposure_tuning_node

    v4l2-ctl -d "${CAMERA_PORT}" -c exposure_absolute="${EXPOSURE_ABSOLUTE}"

    eval `tmux send-keys -t "${EXPOSURE_WINDOW}":exposure_tuning "${CMD}" C-m`
    eval `tmux send-keys -t show_image:image "$SHOWCMD" C-m`

    show_notice
    show_menu
    show_loop

    tmux kill-session -t "${EXPOSURE_WINDOW}"
    tmux kill-session -t show_image
}