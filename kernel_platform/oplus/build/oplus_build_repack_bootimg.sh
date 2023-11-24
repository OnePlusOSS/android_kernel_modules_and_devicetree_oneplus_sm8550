#!/bin/bash

function usage() {
    cat << USAGE

Description:
    Repacks the boot image for kernel versions v5.10 or higher.
    Step 1. Unpack the boot image A and boot image B
    Step 2. Repacks the boot image with the "kernel" of image A and the "ramdisk" of image B

OPTIONS:
    -k, --kernel_bootimg
        Will use the "kernel" part in it to repack the boot image

    -r, --ramdisk_bootimg
        Will use the "ramdisk" part in it to repack the boot image

    -o, --out
        Output dir to save the repacked boot image

    -h, --help
        Display this help message

Usage:
    $0 -k boot_A.img -r boot_B.img -o output
        Repacks the boot image with the "kernel" of boot_A.img and the "ramdisk" of boot_B.img

USAGE
}

function get_opt() {
    local args="$@"
    local opts="h"
    local long_opts="help,"

    opts+="k:r:o:"
    long_opts+="kernel_bootimg:,ramdisk_bootimg:,out:"

    getopt_cmd=$(getopt -o "$opts" --long "$long_opts" -n $(basename $0) -- "$@") || {
       usage
       exit 1
    }

    eval set -- ${getopt_cmd}

    while true; do
        case "$1" in
        -k | --kernel_bootimg)
            KERNEL_BOOTIMG="$2"
            shift
            ;;
        -r | --ramdisk_bootimg)
            RAMDISK_BOOTIMG="$2"
            shift
            ;;
        -o | --out)
            OUTPUT_DIR="$2"
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        esac
        shift
    done

    if [ -z "${KERNEL_BOOTIMG}" -o -z "${RAMDISK_BOOTIMG}" ]; then
        echo "Missing parameter --kernel_bootimg or --kernel_bootimg."
        usage
        exit 1
    fi

    if [ ! -e "${KERNEL_BOOTIMG}" -o ! -e "${RAMDISK_BOOTIMG}" ]; then
        echo "${KERNEL_BOOTIMG} or ${RAMDISK_BOOTIMG} does not exist."
        usage
        exit 1
    fi

    if [ -z ${OUTPUT_DIR} ]; then
        OUTPUT_DIR="out"
    fi
}

function init_env() {
    ROOT_DIR=$(readlink -f $(dirname $0)/../..)
    PYTHON_TOOL="${ROOT_DIR}/prebuilts/build-tools/path/linux-x86/python3"
    AVB_TOOL="${ROOT_DIR}/prebuilts/kernel-build-tools/linux-x86/bin/avbtool"
    MKBOOTIMG_TOOL="${ROOT_DIR}/tools/mkbootimg/mkbootimg.py"
    UNPACK_BOOTIMG_TOOL="${ROOT_DIR}/tools/mkbootimg/unpack_bootimg.py"
    KERNEL_BOOTIMG_OUT_DIR="tmp_kernel_bootimg_out"
    RAMDISK_BOOTIMG_OUT_DIR="tmp_ramdisk_bootimg_out"
    AVB_BOOT_PARTITION_SIZE="201326592"
    AVB_BOOT_KEY="${ROOT_DIR}/tools/mkbootimg/tests/data/testkey_rsa2048.pem"
    AVB_BOOT_ALGORITHM="SHA256_RSA2048"
    DEST_BOOTIMG="${OUTPUT_DIR}/repacked-boot.img"

    [ -d ${OUTPUT_DIR} ] && rm -rf ${DEST_BOOTIMG} || mkdir -p ${OUTPUT_DIR}
    [ -f ${AVB_TOOL} ] || {
        echo "Failed to find tools, ${TOOL_DIR} does not exist."
        exit 1
    }
}

function main() {
    get_opt "$@"
    init_env

    # unpack kernel bootimg
    echo "========== unpacking kernel bootimg =========="
    ${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img="${KERNEL_BOOTIMG}" --out="${KERNEL_BOOTIMG_OUT_DIR}" --format=mkbootimg
    echo "=============================================="
    echo ""

    # unpack ramdisk bootimg and get packing args
    echo "========= unpacking ramdisk bootimg =========="
    MKBOOTIMG_ARGS=$(${PYTHON_TOOL} ${UNPACK_BOOTIMG_TOOL} --boot_img="${RAMDISK_BOOTIMG}" --out="${RAMDISK_BOOTIMG_OUT_DIR}" --format=mkbootimg)
    echo "MKBOOTIMG_ARGS=${MKBOOTIMG_ARGS}"
    echo "=============================================="
    echo ""

    # repack boot img
    cp -f ${KERNEL_BOOTIMG_OUT_DIR}/kernel ${RAMDISK_BOOTIMG_OUT_DIR}
    echo "=========== repacking new bootimg ============"
    ${PYTHON_TOOL} ${MKBOOTIMG_TOOL} ${MKBOOTIMG_ARGS} -o ${DEST_BOOTIMG}
    rm -rf ${KERNEL_BOOTIMG_OUT_DIR} ${RAMDISK_BOOTIMG_OUT_DIR}
    echo "=============================================="
    echo ""

    # add avb hash footer
    echo "=========== Adding avb hash footer ==========="
    ${AVB_TOOL} add_hash_footer --partition_name boot \
        --partition_size ${AVB_BOOT_PARTITION_SIZE} \
        --algorithm ${AVB_BOOT_ALGORITHM} \
        --key ${AVB_BOOT_KEY} \
        --image ${DEST_BOOTIMG}
    echo "=============================================="
    echo ""

    echo "Repacked boot image : ${DEST_BOOTIMG}"
}

main "$@"
