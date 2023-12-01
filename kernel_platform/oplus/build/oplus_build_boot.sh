#!/bin/bash
set -e
set -x
export ROOT_DIR=$(readlink -f $(dirname $0)/../..)
cd ${ROOT_DIR}
source "${ROOT_DIR}/build/_setup_env.sh"

export MAKE_ARGS="$* ${KBUILD_OPTIONS}"
export MAKEFLAGS="-j$(nproc) ${MAKEFLAGS}"
export MODULES_STAGING_DIR=$(readlink -m ${COMMON_OUT_DIR}/staging)
export MODULES_PRIVATE_DIR=$(readlink -m ${COMMON_OUT_DIR}/private)
export UNSTRIPPED_DIR=${DIST_DIR}/unstripped
export MODULE_UAPI_HEADERS_DIR=$(readlink -m ${COMMON_OUT_DIR}/module_uapi_headers)
ANDROID_KERNEL_OUT=${ANDROID_BUILD_TOP}/device/qcom/${TARGET_PRODUCT}-kernel
cd ${ROOT_DIR}

export CLANG_TRIPLE CROSS_COMPILE CROSS_COMPILE_COMPAT CROSS_COMPILE_ARM32 ARCH SUBARCH MAKE_GOALS

if [ -z "${UNPACK_BOOTIMG_TOOL}" ]; then
UNPACK_BOOTIMG_TOOL="${ROOT_DIR}/tools/mkbootimg/unpack_bootimg.py"
fi
export UNPACK_BOOTIMG_BIN=${ROOT_DIR}/oplus/prebuild/img
export UNPACK_BOOTIMG_RAMDISK=${ROOT_DIR}/oplus/prebuild/ramdisk
export UNPACK_BOOTIMG_OUT=${ROOT_DIR}/oplus/prebuild/out
export UNPACK_BOOTIMG_MERGE=${ROOT_DIR}/oplus/prebuild/merge
export GKI_RAMDISK_PREBUILT_BINARY=${ROOT_DIR}/oplus/prebuild/ramdisk/boot_ramdisk/ramdisk

if [ -e "${ANDROID_PRODUCT_OUT}/boot.img" ]; then
    echo "${ANDROID_PRODUCT_OUT}/boot.img ${ROOT_DIR}/oplus/prebuild/img/"
    cp ${ANDROID_PRODUCT_OUT}/boot.img ${ROOT_DIR}/oplus/prebuild/img/
fi

if [ -e "${ANDROID_PRODUCT_OUT}/vednor_dlkm.img" ]; then
    echo "${ANDROID_PRODUCT_OUT}/vednor_dlkm.img ${ROOT_DIR}/oplus/prebuild/img/"
    cp ${ANDROID_PRODUCT_OUT}/vednor_dlkm.img ${ROOT_DIR}/oplus/prebuild/img/
fi

if [ -e "${ANDROID_PRODUCT_OUT}/vendor_boot.img" ]; then
    echo "${ANDROID_PRODUCT_OUT}/vendor_boot.img ${ROOT_DIR}/oplus/prebuild/img/"
    cp ${ANDROID_PRODUCT_OUT}/vendor_boot.img ${ROOT_DIR}/oplus/prebuild/img/
fi


################################################################################
if [ ! -e "${UNPACK_BOOTIMG_BIN}/vendor_boot.img" -o ! -e "${UNPACK_BOOTIMG_BIN}/boot.img" ]; then
  echo
  echo "  boot.img vendor_boot.img not exist, exit for repack img"
  echo "  you can copy boot.img vendor_boot.img to ${UNPACK_BOOTIMG_BIN}/"
  echo "  or you can do a full build vendor and confirm "
  echo "  ${ANDROID_PRODUCT_OUT}/vendor_boot.img "
  echo "  ${ANDROID_PRODUCT_OUT}/boot.img exist"
  echo "  then try again"
  exit 1
fi
set -x

rm -rf ${UNPACK_BOOTIMG_RAMDISK} ${UNPACK_BOOTIMG_OUT} ${UNPACK_BOOTIMG_MERGE}

"$UNPACK_BOOTIMG_TOOL" --boot_img="${UNPACK_BOOTIMG_BIN}/boot.img" \
    --out="${UNPACK_BOOTIMG_RAMDISK}/boot_ramdisk/"

"$UNPACK_BOOTIMG_TOOL" --boot_img="${UNPACK_BOOTIMG_BIN}/vendor_boot.img" \
    --out="${UNPACK_BOOTIMG_RAMDISK}/vendor_ramdisk/"

pushd ${UNPACK_BOOTIMG_RAMDISK}/vendor_ramdisk/
mv vendor_ramdisk vendor_ramdisk.gz
gunzip vendor_ramdisk.gz
file vendor_ramdisk
popd
mkdir -p ${UNPACK_BOOTIMG_OUT}/vendor_ramdisk/
pushd ${UNPACK_BOOTIMG_OUT}/vendor_ramdisk/
cpio -ivdu < ${UNPACK_BOOTIMG_RAMDISK}/vendor_ramdisk/vendor_ramdisk
pwd
ls -l
popd
mkdir -p ${UNPACK_BOOTIMG_MERGE}/vendor_ramdisk/
pushd  ${UNPACK_BOOTIMG_MERGE}/vendor_ramdisk/
cp -rf ${UNPACK_BOOTIMG_OUT}/vendor_ramdisk/*  ${UNPACK_BOOTIMG_MERGE}/vendor_ramdisk/
ls -l
#cp ${ANDROID_KERNEL_OUT}/*.ko lib/modules/
find .|cpio -ov -H newc | gzip > vendor_ramdisk

#mv ${UNPACK_BOOTIMG_MERGE}/vendor_ramdisk/vendor_ramdisk ${UNPACK_BOOTIMG_MERGE}/
ls -l
popd

if [ ! -z "${BUILD_BOOT_IMG}" ] ; then
  MKBOOTIMG_ARGS=()
  if [ -n  "${BASE_ADDRESS}" ]; then
    MKBOOTIMG_ARGS+=("--base" "0")
  fi
  if [ -n  "${PAGE_SIZE}" ]; then
    MKBOOTIMG_ARGS+=("--pagesize" "${PAGE_SIZE}")
  fi
  if [ -n "${KERNEL_VENDOR_CMDLINE}" -a "${BOOT_IMAGE_HEADER_VERSION}" -lt "3" ]; then
    KERNEL_CMDLINE+=" ${KERNEL_VENDOR_CMDLINE}"
  fi
  if [ -n "${KERNEL_CMDLINE}" ]; then
    MKBOOTIMG_ARGS+=("--cmdline" "${KERNEL_CMDLINE}")
  fi
  if [ -n "${TAGS_OFFSET}" ]; then
    MKBOOTIMG_ARGS+=("--tags_offset" "${TAGS_OFFSET}")
  fi
  if [ -n "${RAMDISK_OFFSET}" ]; then
    MKBOOTIMG_ARGS+=("--ramdisk_offset" "${RAMDISK_OFFSET}")
  fi

  MKBOOTIMG_ARGS+=("--dtb" "${ANDROID_KERNEL_OUT}/dtbs/dtb.img")

  MKBOOTIMG_RAMDISKS=()


  if [ -z "${MKBOOTIMG_PATH}" ]; then
    MKBOOTIMG_PATH="tools/mkbootimg/mkbootimg.py"
  fi
  if [ ! -f "$MKBOOTIMG_PATH" ]; then
    echo "mkbootimg.py script not found. MKBOOTIMG_PATH = $MKBOOTIMG_PATH"
    exit 1
  fi

  if [ ! -f "${DIST_DIR}/$KERNEL_BINARY" ]; then
    echo "kernel binary(KERNEL_BINARY = $KERNEL_BINARY) not present in ${DIST_DIR}"
    exit 1
  fi

  if [ "${BOOT_IMAGE_HEADER_VERSION}" -eq "3" ]; then
    if [ -f "${GKI_RAMDISK_PREBUILT_BINARY}" ]; then
      MKBOOTIMG_ARGS+=("--ramdisk" "${GKI_RAMDISK_PREBUILT_BINARY}")
    fi

    if [ -z "${SKIP_VENDOR_BOOT}" ]; then
      MKBOOTIMG_ARGS+=("--vendor_boot" "${DIST_DIR}/vendor_boot.img" \
        "--vendor_ramdisk" "${UNPACK_BOOTIMG_MERGE}/vendor_ramdisk/vendor_ramdisk")
      if [ -n "${KERNEL_VENDOR_CMDLINE}" ]; then
        MKBOOTIMG_ARGS+=("--vendor_cmdline" "${KERNEL_VENDOR_CMDLINE}")
      fi
    fi
  else
    MKBOOTIMG_ARGS+=("--ramdisk" "${DIST_DIR}/ramdisk.${RAMDISK_EXT}")
  fi

  "$MKBOOTIMG_PATH" --kernel "${DIST_DIR}/${KERNEL_BINARY}" \
    --header_version "${BOOT_IMAGE_HEADER_VERSION}" \
    "${MKBOOTIMG_ARGS[@]}" -o "${DIST_DIR}/boot.img"
  if [ -f "${DIST_DIR}/boot.img" ]; then
    echo "boot image created at ${DIST_DIR}/boot.img"

    if [ -n "${AVB_SIGN_BOOT_IMG}" ]; then
      if [ -n "${AVB_BOOT_PARTITION_SIZE}" ] \
          && [ -n "${AVB_BOOT_KEY}" ] \
          && [ -n "${AVB_BOOT_ALGORITHM}" ]; then
        echo "Signing the boot.img..."
        avbtool add_hash_footer --partition_name boot \
            --partition_size ${AVB_BOOT_PARTITION_SIZE} \
            --image ${DIST_DIR}/boot.img \
            --algorithm ${AVB_BOOT_ALGORITHM} \
            --key ${AVB_BOOT_KEY}
      else
        echo "Missing the AVB_* flags. Failed to sign the boot image" 1>&2
        exit 1
      fi
    fi
  fi

  [ -z "${SKIP_VENDOR_BOOT}" ] \
    && [ "${BOOT_IMAGE_HEADER_VERSION}" -eq "3" ] \
    && [ -f "${DIST_DIR}/vendor_boot.img" ] \
    && echo "vendor boot image created at ${DIST_DIR}/vendor_boot.img"
fi

set +x

