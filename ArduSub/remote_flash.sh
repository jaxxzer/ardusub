#!/bin/bash
DATE=$(date +%y-%m-%d.%H:%M:%S)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
COMMIT=$(git rev-parse --short HEAD)

FILENAME=$DATE.$BRANCH.$COMMIT.px4
echo $FILENAME

scp ~/git/ArduSub/ArduSub/ArduSub-v2.px4 ubuntu@10.0.0.20:~/$FILENAME
ssh ubuntu@10.0.0.20 \
"python px_uploader.py --port /dev/ttyACM0 $FILENAME; \
mv ~/ArduSub/current/*.px4 ~/ArduSub/previous/; \
mv $FILENAME ~/ArduSub/current/
"
