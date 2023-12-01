#!/bin/sh
echo "build help"
echo "./kernel_platform/oplus/build/oplus_build.sh waipio consolidate thin all true 2>&1 |tee build.log"
echo ""
source ./kernel_platform/oplus/build/oplus_setup.sh $1 $2 $3 $4 $5
build_start_time
echo "platform=$variants_platform type=$variants_type LTO=$LTO  target=$target_type RECOMPILE_KERNEL=$RECOMPILE_KERNEL REPACK_IMG=$REPACK_IMG"
./kernel_platform/build/android/prepare_vendor.sh $variants_platform $variants_type
build_end_time

