#!/bin/bash

TYPE=$1
AMP=${2:-50%}

if [[ "$1" = "-h" || "$1" = "--help" ]];then
#if [ "$1" = "-h" ] || [ "$1" = "--help" ];then
  echo -e This script will resize image to 640x480 or 480x640.'\n'
  echo Usage:
  echo -e '\t'source resize.sh \[format\]
  echo Example:
  echo -e '\t'source resize.sh jpg
  echo -e '\t'source resize.sh png
else
  for file in *.$TYPE
  do
    echo $file
    width=`identify -format "%[fx:w]" $file`
    height=`identify -format "%[fx:h]" $file`
    if [ "${width}" -gt "${height}" ];then
      convert ${file} -resize 640x480 ${file}
    else
      convert ${file} -resize 480x640 ${file}
    fi
    echo -en "\e[1A"
  done
fi
