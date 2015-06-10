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
    #avconv -f video4linux2 -r 30 -i /dev/video0 -f alsa -i plughw:C615,0 -s 1920x1080 -q:v 1 -ar 44100 -ab 128k -strict experimental -acodec aac -threads 4 -vcodec mpeg4 -y $filename    
    cvlc v4l2:// :v4l-vdev="/dev/video0"  :v:v4l2-standard= :input-slave="alsa://hw:2,0" :v4l-norm=3 :v4l-frequency=-1 :v4l-caching=300 :v4l-fps=-1.000000 :v4l-samplerate=44100 :v4l-channel=0 :v4l-tuner=-1 :v4l-audio=-1 :v4l-stereo :v4l-width=1920 :v4l-height=1080 :v4l-brightness=-1 :v4l-colour=-1 :v4l-hue=-1 :v4l-contrast=-1 :no-v4l-mjpeg :v4l-decimation=1 :v4l-quality=100 --sout="#transcode{vcodec=mp1v,vb=1024,scale=1,acodec=mp3,ab=128,channels=2}:duplicate{dst=std{access=file,mux=mpeg1,dst=$filename}}"    
    #roslaunch $@
fi

