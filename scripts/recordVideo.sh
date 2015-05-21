#!/bin/bash
function showHelp(){
    echo 
    echo "This script record video from a webcam"
    echo "you can specify the name of the file, location, resolution and the camera device id"
    echo 
}

if [ "$1" = "-h" ]; then
    showHelp
else 
    echo "Let's begin!"
    dateStamp=$(date +"%Y-%m-%d_%H:%M:%S")
    filename="/home/kuri/ownCloud/Videos/AutoRecorded/$dateStamp.mp4"
    echo $filename
    avconv -f video4linux2 -r 30 -i /dev/video0 -f alsa -i plughw:C615,0 -s 1920x1080 -strict experimental -acodec aac -threads 4 -vcodec mpeg4 -y $filename    
    #roslaunch $@
fi

