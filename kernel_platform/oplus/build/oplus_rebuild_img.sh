#!/bin/bash

function init_image_environment() {
    source kernel_platform/oplus/build/oplus_setup.sh $1 $2
    #set -x
    TOPDIR=${PWD}
    ACKDIR=${TOPDIR}/kernel_platform
    TOOLS=${ACKDIR}/oplus/tools
    ORIGIN_IMAGE=${ACKDIR}/oplus/prebuild/origin_img
    BOOT_TMP_IMAGE=${ACKDIR}/oplus/prebuild/boot_tmp
    IMAGE_OUT=${ACKDIR}/oplus/prebuild/out
    VENDOR_BOOT_TMP_IMAGE=${ACKDIR}/oplus/prebuild/vendor_boot_tmp
    VENDOR_DLKM_TMP_IMAGE=${ACKDIR}/oplus/prebuild/vendor_dlkm_tmp
    SYSTEM_DLKM_TMP_IMAGE=${ACKDIR}/oplus/prebuild/system_dlkm_tmp
    DT_TMP_IMAGE=${ACKDIR}/oplus/prebuild/dt_tmp
    DT_DIR=${ACKDIR}/out/msm-kernel-${variants_platform}-${variants_type}/dist
    PYTHON_TOOL="${ACKDIR}/prebuilts/build-tools/path/linux-x86/python3"
    MKBOOTIMG_PATH=${ACKDIR}/"tools/mkbootimg/mkbootimg.py"
    UNPACK_BOOTIMG_TOOL="${ACKDIR}/tools/mkbootimg/unpack_bootimg.py"
    LZ4="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/lz4"
    CPIO="${ACKDIR}/prebuilts/build-tools/path/linux-x86/cpio"
    SIMG2IMG="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/simg2img"
    IMG2SIMG="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/img2simg"
    EROFS="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/mkfs.erofs"
    BUILD_IMAGE="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/build_image"
    MKDTIMG="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/mkdtimg"
    MKDTOIMG="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/mkdtboimg.py"
    MKBOOTFS="${ACKDIR}/prebuilts/kernel-build-tools/linux-x86/bin/mkbootfs"
    #IMAGE_SERVER="http://gpw13.myoas.com/artifactory/phone-snapshot-local/PSW/SM8650_14/Waffle/22825/Daily/PublicMarket/BringUp/domestic/userdebug/14.0.0.1_2023062806090158_userdebug/"
    mkdir -p ${IMAGE_OUT}
}
function download_prebuild_image() {
    if [[ ! -e "${ORIGIN_IMAGE}/vendor_boot.img" ]]; then
        mkdir -p ${ORIGIN_IMAGE}

        if [ -z ${IMAGE_SERVER} ]; then
            echo ""
            echo ""
            echo "you need input base version like this:"
            echo "http://gpw13.myoas.com/artifactory/phone-snapshot-local/PSW/SM8650_14/Waffle/22825/Daily/PublicMarket/BringUp/domestic/userdebug/14.0.0.1_2023062806090158_userdebug/"
            echo ""
            echo "or you exit it and then exoprt like this:"
            echo "export IMAGE_SERVER=http://gpw13.myoas.com/artifactory/phone-snapshot-local/PSW/SM8650_14/Waffle/22825/Daily/PublicMarket/BringUp/domestic/userdebug/14.0.0.1_2023062806090158_userdebug/"
            read IMAGE_SERVER
        fi

        if ! wget -qS ${IMAGE_SERVER}/compile.ini; then
            echo "server can't connect,please set IMAGE_SERVER and try again"
            return
        fi

        wget ${IMAGE_SERVER}/compile.ini  -O ${ORIGIN_IMAGE}/compile.ini
        OFP_DRI=`cat ${ORIGIN_IMAGE}/compile.ini | grep "ofp_folder =" | awk '{print $3 }'`
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/boot.img -O ${ORIGIN_IMAGE}/boot.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vendor_boot.img -O ${ORIGIN_IMAGE}/vendor_boot.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/system_dlkm.img -O ${ORIGIN_IMAGE}/system_dlkm.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/vendor_dlkm.img -O ${ORIGIN_IMAGE}/vendor_dlkm.img
        wget ${IMAGE_SERVER}/${OFP_DRI}/META/system_dlkm_image_info.txt -O ${ORIGIN_IMAGE}/system_dlkm_image_info.txt
        wget ${IMAGE_SERVER}/${OFP_DRI}/IMAGES/dtbo.img -O ${ORIGIN_IMAGE}/dtbo.img
    fi
}

function get_image_info() {

    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/boot.img >  ${ORIGIN_IMAGE}/local_boot_image_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vendor_boot.img >  ${ORIGIN_IMAGE}/local_vendor_boot_image_info.txt
    ${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/system_dlkm.img >  ${ORIGIN_IMAGE}/local_system_dlkm_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vendor_dlkm.img >  ${ORIGIN_IMAGE}/local_vendor_dlkm_image_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/dtbo.img >  ${ORIGIN_IMAGE}/local_dtbo_image_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta.img >  ${ORIGIN_IMAGE}/local_vbmeta_image_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta_system.img >  ${ORIGIN_IMAGE}/local_vbmeta_system_image_info.txt
    #${AVBTOOL} info_image --image  ${ORIGIN_IMAGE}/vbmeta_vendor.img >  ${ORIGIN_IMAGE}/local_vbmeta_vendor_image_info.txt
}

function sign_system_dlkm_image() {

    algorithm=$(awk -F '[= ]' '$1=="avb_system_dlkm_algorithm" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    partition_size=$(awk -F '[= ]' '$1=="system_dlkm_size" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    footer_args=$(awk -F '[= ]' '$1=="avb_add_hashtree_footer_args" {$1="";print}' ${ORIGIN_IMAGE}/system_dlkm_image_info.txt)
    partition_name=system_dlkm
    salt=`uuidgen | sed 's/-//g'`

    if [ -z "$algorithm" ]; then
     algorithm="SHA256_RSA4096"
    fi

    if [ -z "$partition_size" ]; then
     partition_size=`cat ${ORIGIN_IMAGE}/local_system_dlkm_info.txt | grep "Image size:" | awk '{print $3 }'`
    fi

    ${AVBTOOL} add_hashtree_footer \
        --partition_name ${partition_name} \
        --partition_size ${partition_size}\
        --do_not_generate_fec \
        --image ${IMAGE_OUT}/system_dlkm.img  \
        --hash_algorithm sha256 \
        --salt ${salt}  \
        ${footer_args}
}

rebuild_boot_image() {
    echo "rebuild boot.img"
    rm -rf ${BOOT_TMP_IMAGE}/*
    boot_mkargs=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img ${ORIGIN_IMAGE}/boot.img --out ${BOOT_TMP_IMAGE} --format=mkbootimg)
    cp ${TOPDIR}/kernel_platform/out/msm-${pre_path}-${variants_platform}-${variants_type}/dist/Image ${BOOT_TMP_IMAGE}/kernel
    #cp /work/oplus_80318998/work/8650master/source/vnd/device/qcom/pineapple-kernel/Image ${BOOT_TMP_IMAGE}/kernel
    bash -c "${PYTHON_TOOL} ${MKBOOTIMG_PATH} ${boot_mkargs} -o ${IMAGE_OUT}/boot.img"
}

rebuild_vendor_boot_image() {
    echo "rebuild vendor_boot.img"
    rm -rf ${VENDOR_BOOT_TMP_IMAGE}/*
    boot_mkargs=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img ${ORIGIN_IMAGE}/vendor_boot.img --out ${VENDOR_BOOT_TMP_IMAGE}/origin --format=mkbootimg)
    #rebuild_dtb_image
    index="00"
    for index in  $index
    do
        echo " index  $index "
        mv ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        ${LZ4} -d ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4
        mkdir -p ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}
        pushd  ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}
        ${CPIO} -idu < ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        popd
        rm ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index}/vendor_ramdisk${index}

        vendor_boot_modules_update
        ${MKBOOTFS} ${VENDOR_BOOT_TMP_IMAGE}/ramdisk${index} > ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        ${LZ4} -l -12 --favor-decSpeed ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
        mv ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}.lz4 ${VENDOR_BOOT_TMP_IMAGE}/origin/vendor_ramdisk${index}
        rm ${VENDOR_BOOT_TMP_IMAGE}/vendor_ramdisk${index}
    done
    bash -c "${PYTHON_TOOL} ${MKBOOTIMG_PATH} ${boot_mkargs} --vendor_boot ${IMAGE_OUT}/vendor_boot.img"
    #sign_vendor_boot_image
}

rebuild_system_dlkm_image() {
    echo "rebuild system_dlkm.img"
    mkdir -p ${SYSTEM_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/system_dlkm.img  ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img
    ${TOOLS}/7z_new ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_out
    ${BUILD_IMAGE} ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_out ${TOOLS}/system_dlkm_image_info.txt ${IMAGE_OUT}/system_dlkm.img /dev/null
	sign_system_dlkm_image
}

rebuild_vendor_dlkm_image() {
    echo "rebuild vendor_dlkm.img"
    mkdir -p ${VENDOR_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img
    ${TOOLS}/7z_new ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_out
    ${BUILD_IMAGE} ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_out ${TOOLS}/vendor_dlkm_image_info.txt ${IMAGE_OUT}/vendor_dlkm.img /dev/null
}

rebuild_vendor_dlkm_erofs_image() {
    echo "rebuild vendor_dlkm.img"
    rm -rf ${VENDOR_DLKM_TMP_IMAGE}/*
    mkdir -p ${VENDOR_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img
    ${TOOLS}/erofs_unpack.sh ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm.img  ${VENDOR_DLKM_TMP_IMAGE}/mnt ${VENDOR_DLKM_TMP_IMAGE}/out
    touch ${VENDOR_DLKM_TMP_IMAGE}/out/lib/modules/readme.txt
    ${EROFS} ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_repack.img  ${VENDOR_DLKM_TMP_IMAGE}/out

    ${IMG2SIMG} ${VENDOR_DLKM_TMP_IMAGE}/vendor_dlkm_repack.img  ${IMAGE_OUT}/vendor_dlkm.img
}
rebuild_system_dlkm_erofs_image() {
    echo "rebuild system_dlkm.img"
    rm -rf ${SYSTEM_DLKM_TMP_IMAGE}/*
    mkdir -p ${SYSTEM_DLKM_TMP_IMAGE}
    ${SIMG2IMG} ${ORIGIN_IMAGE}/system_dlkm.img  ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img
    ${TOOLS}/erofs_unpack.sh ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm.img  ${SYSTEM_DLKM_TMP_IMAGE}/mnt ${SYSTEM_DLKM_TMP_IMAGE}/out
    #rm -rf ${SYSTEM_DLKM_TMP_IMAGE}/out/lib/modules/6.1.23-android14-4-g5cf7530f7c55-qki-consolidate/kernel/*
    #cp -r device/qcom/pineapple-kernel/system_dlkm/lib/modules/6.1.23-android14-4-00873-g5cf7530f7c55-qki-consolidate/kernel/* \
    #${SYSTEM_DLKM_TMP_IMAGE}/out/lib/modules/6.1.23-android14-4-g5cf7530f7c55-qki-consolidate/kernel/
    touch ${SYSTEM_DLKM_TMP_IMAGE}/out/lib/modules/readme.txt
    ${EROFS} ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_repack.img  ${SYSTEM_DLKM_TMP_IMAGE}/out

    ${IMG2SIMG} ${SYSTEM_DLKM_TMP_IMAGE}/system_dlkm_repack.img  ${IMAGE_OUT}/system_dlkm.img
}

rebuild_dtb_image() {
    echo "rebuild dtb.img"
    mkdir -p ${DT_TMP_IMAGE}
    cp ${DT_DIR}/*.dtb ${DT_TMP_IMAGE}
    ${MKDTIMG} create  ${DT_TMP_IMAGE}/dtb.img  ${DT_TMP_IMAGE}/*.dtb
}

rebuild_dtbo_image() {
    echo "rebuild dtbo.img"
    mkdir -p ${DT_TMP_IMAGE}
    cp ${DT_DIR}/*.dtbo ${DT_TMP_IMAGE}
    ${MKDTOIMG} create ${IMAGE_OUT}/dtbo.img  --page_size=4096 ${DT_TMP_IMAGE}/*.dtbo
}
init_image_environment $1 $2
download_prebuild_image
get_image_info
rebuild_boot_image
rebuild_system_dlkm_image
#rebuild_system_dlkm_erofs_image
rebuild_vendor_dlkm_image
#rebuild_vendor_dlkm_erofs_image
#rebuild_dtb_image
rebuild_dtbo_image
rebuild_vendor_boot_image
