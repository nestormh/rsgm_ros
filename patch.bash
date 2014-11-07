 #! /bin/bash
 
STEREO_COMMON=src/rSGM/src/StereoCommon.h
STEREO_BM_HELPER=src/rSGM/src/StereoBMHelper.cpp
STEREO_MAIN=src/rSGM/src/rSGMCmd.cpp
MY_IMAGE_H=src/rSGM/src/MyImage.h

sed -i 's/UINT16_MAX/UINT16_MAX_/g' $STEREO_COMMON
sed -i 's/_ASSERT/assert/g' $STEREO_BM_HELPER
sed -i 's/int main(/int console(/g' $STEREO_MAIN
sed -i 's/std::cout/\/\/Hidden comment/g' $STEREO_MAIN
sed -i 's/#include "MyImage.hpp"/\/\/#include "MyImage.hpp"/g' $MY_IMAGE_H
