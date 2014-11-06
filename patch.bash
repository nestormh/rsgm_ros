 #! /bin/bash
 
STEREO_COMMON=src/rSGM/src/StereoCommon.h
STEREO_BM_HELPER=src/rSGM/src/StereoBMHelper.cpp

sed -i 's/UINT16_MAX/UINT16_MAX_/g' $STEREO_COMMON
sed -i 's/_ASSERT/assert/g' $STEREO_BM_HELPER
