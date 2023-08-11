#!/bin/bash
set -x
KERNEL_PARAM=${1:-waipio}
BUILD_VARIANT=user
SOC_VARIANT=QCOM
SCRIPT_REAL_PATH=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SRC_TOP_REAL_PATH=$(realpath $SCRIPT_REAL_PATH/../../../../)

BZL_TOOL="$SRC_TOP_REAL_PATH/prebuilts/build-tools/path/linux-x86/bazel"
BAZEL_OUTPUT_REAL_PATH="$SRC_TOP_REAL_PATH/out/bazel_output"
WORKSPACE_REAL_PATH="$SRC_TOP_REAL_PATH/out/bazel_workspace/kernel"
KERNEL_PLATFORM_REAL_PATH="$SRC_TOP_REAL_PATH/kernel_platform"
OLD_ENVIRONMENT_REAL_PATH="${WORKSPACE_REAL_PATH}/environment"
GKI_ABI_FILE_REAL_PATH="${KERNEL_PLATFORM_REAL_PATH}/oplus/config/abi_gki_aarch64_oplus_internal"
QCOM_GKI_ABI_FILE_REAL_PATH="${KERNEL_PLATFORM_REAL_PATH}/msm-kernel/android/abi_gki_aarch64_qcom"

## relative to vnd/kernel_platform
KERNEL_OUT_DIR="out"
GKI_PREBUILTS_DIR="prebuilts/gki-kernel-binaries"
##

## this values exported to bazel should not use real path
## relative to path vnd/out/bazel_output/execroot/__main__
KERNEL_PLATFORM_RELATIVE_PATH="../../../../kernel_platform"
if [ "${KERNEL_PARAM}" == "waipio" ]; then
  GKI_KERNEL_BINARIES_RELATIVE_PATH="${KERNEL_PLATFORM_RELATIVE_PATH}/${KERNEL_OUT_DIR}/msm-waipio-waipio-gki/gki_kernel/dist"
else
  GKI_KERNEL_BINARIES_RELATIVE_PATH="${KERNEL_PLATFORM_RELATIVE_PATH}/${KERNEL_OUT_DIR}/msm-kernel-${KERNEL_PARAM}-gki/gki_kernel/dist"
fi
OLD_ENVIRONMENT_RELATIVE_PATH="../../../bazel_workspace/kernel/environment"
##

## build commands
BUILD_KERNEL_COMMAND="kernel_platform/build/android/prepare_vendor.sh ${KERNEL_PARAM} gki"
BUILD_KERNEL_WITH_PREBUILT_GKB_COMMAND="GKI_PREBUILTS_DIR=${GKI_PREBUILTS_DIR} kernel_platform/build/android/prepare_vendor.sh ${KERNEL_PARAM} gki"
##

## prepare environment
rm -rf ${BAZEL_OUTPUT_REAL_PATH}
rm -rf ${WORKSPACE_REAL_PATH}
mkdir -p $(dirname ${WORKSPACE_REAL_PATH})
cp -rf ${SCRIPT_REAL_PATH}/ ${WORKSPACE_REAL_PATH}
# need to consider abi_gki_aarch64_oplus_internal, which not in kernel/common but affect building result
# copy to workspace to be part of revision
cp -rf ${GKI_ABI_FILE_REAL_PATH} ${WORKSPACE_REAL_PATH}
cp -rf ${QCOM_GKI_ABI_FILE_REAL_PATH} ${WORKSPACE_REAL_PATH}
export -p > ${OLD_ENVIRONMENT_REAL_PATH}
##

## functions
function copy_logs() {
  if [[ -d "$SRC_TOP_REAL_PATH/LOGDIR" ]]; then
    if [[ -e "${WORKSPACE_REAL_PATH}/bazel-bin/kernel_log.txt" ]]; then
      cp "${WORKSPACE_REAL_PATH}/bazel-bin/kernel_log.txt" "$SRC_TOP_REAL_PATH/LOGDIR"
    fi
    if [[ -e "${WORKSPACE_REAL_PATH}/exe_log.json" ]]; then
      cp "${WORKSPACE_REAL_PATH}/exe_log.json" "$SRC_TOP_REAL_PATH/LOGDIR"
    fi
  fi
}

function build_kernel() {
  echo "========================================================"
  echo " Build kernel"
  eval ${BUILD_KERNEL_COMMAND}
}

function build_kernel_with_prebuilts_gki_kerenl_binaries() {
  echo "========================================================"
  echo " Build kernel with prebuilts gki kernel binares"
  eval ${BUILD_KERNEL_WITH_PREBUILT_GKB_COMMAND}
}
##

## check bazel tools
if [ ! -f "${BZL_TOOL}" ]; then
  echo "can not find bazel tools, build kernel directly"
  build_kernel
  exit
fi
##

## for local build, build kernel directly if there is any modification in kernel/common repo
## we made some linkfile in kernel/common, just ignore them
EXCLUDE_PATTERN_FILE="${WORKSPACE_REAL_PATH}/pattern"
EXCLUDE_MK_FILE="drivers/staging/greybus/tools/Android.mk"
if [ "${KERNEL_PARAM}" == "waipio" ]; then
  EXCLUDE_PATTERN="android/abi_gki_aarch64_oplus_internal\nbuild.config.msm.lahaina\nbuild.config.msm.waipio\nbuild.config.msm.waipio.tuivm"
else
  EXCLUDE_PATTERN="android/abi_gki_aarch64_oplus_internal"
fi
echo -e ${EXCLUDE_PATTERN} > ${EXCLUDE_PATTERN_FILE}
CHANGES=$(cd ${KERNEL_PLATFORM_REAL_PATH}/common && git ls-files --others --modified --exclude-from=${EXCLUDE_PATTERN_FILE} | grep -v ${EXCLUDE_MK_FILE})
if [ -n "${CHANGES}" ]; then
  echo "Local changes haven't submitted yet, build kernel directly"
  build_kernel
  exit
fi
##

## generate revision via git write-tree based on kernel/common repo
REVISION=${SOC_VARIANT}-${BUILD_VARIANT}-$(cd ${KERNEL_PLATFORM_REAL_PATH}/common && git write-tree)
if [[ ! $REVISION =~ ${SOC_VARIANT}-${BUILD_VARIANT}-[0-9a-f]+ ]]; then
  echo "can not get git write-tree value"
  echo "build kernel directly"
  build_kernel
  exit
fi
echo "REVISION=${REVISION}"
##

## bazel special settings
BAZEL_RC_CONTENT="build --remote_cache=grpc://bazel-remote-cache.myoas.com:9093"
# "$OPLUS_SCM_PRIVATE_COMPILE" != "true", if it's local environment(not ccm/cheetah), turn off uploading
if [ "${OPLUS_SCM_PRIVATE_COMPILE}" != "true" ]; then
  BAZEL_RC_CONTENT+="\nbuild --remote_upload_local_results=false"
fi
my_id=$( id -u)
if [[ "0" == "$my_id" ]]; then
  TEST_TMPDIR=/work/.bazel_cache
else
  TEST_TMPDIR=$HOME/.bazel_cache
fi
export TEST_TMPDIR=$TEST_TMPDIR
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
##

## check if there is a cache on the bazel server
## if so, fetch the cache to local environment
## if not, build kernel and then upload gki kernel binares(note:local environment doesn't upload)
cd ${WORKSPACE_REAL_PATH}
echo -e "$BAZEL_RC_CONTENT" > .bazelrc
server_status=$(${BZL_TOOL} info > /dev/null 2>&1 ;echo $?)
if [[ "0" == "${server_status}" ]]; then \
  echo -e "$BAZEL_RC_CONTENT" > .bazelrc
else \
  echo "" > .bazelrc
  echo "cannot connect to bazel remote"
fi
if ! ${BZL_TOOL} --output_base="${BAZEL_OUTPUT_REAL_PATH}" \
            build :all \
          --strategy=Genrule=local --spawn_strategy=local \
          --action_env=REVISION="${REVISION}" \
          --action_env=ENVIRONMENT="${OLD_ENVIRONMENT_RELATIVE_PATH}" \
          --action_env=BUILD_KERNEL_COMMAND="${BUILD_KERNEL_COMMAND}" \
          --action_env=KERNEL_PLATFORM_DIR="$KERNEL_PLATFORM_RELATIVE_PATH" \
          --action_env=GKI_KERNEL_BINARIES_DIR="$GKI_KERNEL_BINARIES_RELATIVE_PATH" \
          --execution_log_json_file=exe_log.json ; then
  cd -
  echo "failed to build kernel"
  rm ${OLD_ENVIRONMENT_REAL_PATH}
  tail ${WORKSPACE_REAL_PATH}/bazel-bin/kernel_log.txt
  copy_logs
  exit 1
fi
rm ${OLD_ENVIRONMENT_REAL_PATH}
cd -

copy_logs

# check if remote cache hit
if grep -q '"remoteCacheHit": true' ${WORKSPACE_REAL_PATH}/exe_log.json ; then
  echo "remoteCacheHit is true"
  # unzip prebuilt binaries to gki-kernel-binaries
  rm -rf  ${KERNEL_PLATFORM_REAL_PATH}/${GKI_PREBUILTS_DIR}/*
  mkdir -p ${KERNEL_PLATFORM_REAL_PATH}/${GKI_PREBUILTS_DIR}
  unzip -d ${KERNEL_PLATFORM_REAL_PATH}/${GKI_PREBUILTS_DIR} ${WORKSPACE_REAL_PATH}/bazel-bin/out.zip
  build_kernel_with_prebuilts_gki_kerenl_binaries
  rm -rf  ${KERNEL_PLATFORM_REAL_PATH}/${GKI_PREBUILTS_DIR}
else
  echo "remoteCacheHit is false"
fi
##
