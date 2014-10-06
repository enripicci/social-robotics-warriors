#!/bin/bash
#
# Sync the repository package with the ROS package.
#

# These lines should remain untouched
PKG_NAME="obj_rec"
ROS_WS_PKG_PATH="src/$PKG_NAME"

# Set this macro to the path to your ROS workspace
ROS_WS_PATH=~/rosws

# Set this macro to a number different from 0 and you will be asked for confirmation
#ASK_DEL_CONFIRMATION=1

if [ -z $ROS_WS_PATH ]; then
    echo 'Set up your ROS_WSPATH before running this script!'
    exit 1
fi

ROS_PKG_PATH="$ROS_WS_PATH/$ROS_WS_PKG_PATH"
#echo "$ROS_PKG_PATH"

if [ -d "$ROS_PKG_PATH" ]; then
    if [ -n "$ASK_DEL_CONFIRMATION" ] && [ $ASK_DEL_CONFIRMATION -ne 0 ]; then
        echo "Overwrite the old directory \"$ROS_PKG_PATH\" [Y/n]?"
        read ANS

        # make all lowercase
        ANS="$(echo $ANS | tr '[:upper:]' '[:lower:]')"

        if [ -n "$ANS" ] && [ "$ANS" != "y" ] && [ "$ANS" != "yes" ]; then
            echo "Abort"
            exit 0
        fi
    fi
    echo "Delete old directory \"$ROS_PKG_PATH\" ..."
    rm -rf "$ROS_PKG_PATH"
fi

# copy the new directory
echo 'Copy the new package...'
cp -r "$PKG_NAME" "$ROS_PKG_PATH"

