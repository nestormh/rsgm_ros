#!/bin/bash

rSGM_folder="src/rSGM"
currentFolder=$(pwd)

rm -rf $rSGM_folder
unzip src/rSGMv1.0.zip -d src
cd $rSGM_folder/src
patch -p3 < ../../../rSGM.patch 
cd $currentFolder