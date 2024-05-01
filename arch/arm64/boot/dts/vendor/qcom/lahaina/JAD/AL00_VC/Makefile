ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_LAHAINA) += \
    JAD-AL00-overlay.dtbo

$(obj)/JAD-AL00-overlay.dtbo: $(src)/overlay.dts FORCE
	$(call if_changed,dtc,dtb,-@)
	$(call if_changed,dtbo_verify)
	rm -rf $(obj)/overlay.dtbo
endif

always		:= $(dtbo-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtbo
