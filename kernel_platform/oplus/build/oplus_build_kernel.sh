#!/bin/bash

function init_environment() {
    source kernel_platform/oplus/build/oplus_setup.sh $1 $2
    export EXTRA_KBUILD_ARGS="--skip abl"
    export TOPDIR=$(readlink -f ${PWD})
    export ANDROID_BUILD_TOP=${TOPDIR}
    export CHIPSET_COMPANY=QCOM
    export OPLUS_VND_BUILD_PLATFORM=SM8550
    export ANDROID_PRODUCT_OUT=${TOPDIR}/out/target/product/$variants_platform
    export TARGET_BOARD_PLATFORM=$variants_platform
}

function build_cmd() {
    build_start_time
    mkdir -p LOGDIR
    source vendor/oplus/kernel/prebuilt/vendorsetup.sh
    ${OPLUS_EXTRA_KBUILD_ARGS} ./kernel_platform/build/android/prepare_vendor.sh $variants_platform $variants_type 2>&1 |tee LOGDIR/build_$(date +"%Y_%m_%d_%H_%M_%S").log
    build_end_time
}

init_environment $1 $2
build_cmd
