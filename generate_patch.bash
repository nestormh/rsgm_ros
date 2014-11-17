#!/bin/bash

rSGM_folder="src/rSGM"

function onExit {
  rm -rf $unzip_folder
  rm $rSGM_folder/src_orig

  exit
}

unzip_folder="/tmp/rsgm_ros_$(date +%Y_%m_%d_%H_%M_%S_%N)"
mkdir -p $unzip_folder

unzip src/rSGMv1.0.zip -d $unzip_folder


if [ $? -ne 0 ]; then
  echo "Error while uncompressing src/rSGMv1.0.zip."
  onExit
fi

ln -sf $unzip_folder/rSGM/src $rSGM_folder/src_orig

diff -ruN $rSGM_folder/src_orig $rSGM_folder/src > rSGM.patch

onExit