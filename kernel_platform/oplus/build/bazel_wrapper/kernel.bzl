def init():
  native.genrule(
    name="kernel",
    srcs=["abi_gki_aarch64_oplus_internal", "abi_gki_aarch64_qcom"],
    outs=["out.zip"],
    cmd="""
      set -x
      output_dir=$$(realpath $$(dirname $(@)))
      mkdir -p $$output_dir
      source $$ENVIRONMENT
      pushd $$KERNEL_PLATFORM_DIR/../
      eval $$BUILD_KERNEL_COMMAND > $$output_dir/kernel_log.txt
      result=$$?
      popd
      echo $$REVISION > $$output_dir/bazel_cached_git_write_tree
      # zip -j, include files without directory
      zip -rqyj $(@) $$output_dir/bazel_cached_git_write_tree \
        $$GKI_KERNEL_BINARIES_DIR/Image \
        $$GKI_KERNEL_BINARIES_DIR/Image.lz4 \
        $$GKI_KERNEL_BINARIES_DIR/modules.builtin  \
        $$GKI_KERNEL_BINARIES_DIR/modules.builtin.modinfo  \
        $$GKI_KERNEL_BINARIES_DIR/System.map  \
        $$GKI_KERNEL_BINARIES_DIR/vmlinux  \
        $$GKI_KERNEL_BINARIES_DIR/vmlinux.symvers \
        $$GKI_KERNEL_BINARIES_DIR/system_dlkm.img
    """
  )
