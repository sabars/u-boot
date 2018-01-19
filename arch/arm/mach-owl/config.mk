ifdef CONFIG_OWL_NAND
	ifeq ($(CONFIG_S900),y)
		PLATFORM_LIBS += $(srctree)/arch/arm/mach-owl/s900/nandlib/libnand_s900.lib
	else 
		ifeq ($(CONFIG_S700),y)
			PLATFORM_LIBS += $(srctree)/arch/arm/mach-owl/s700/nandlib/libnand_s700.lib
		else
			ifeq  ($(CONFIG_OWL_MLC_NAND),y)
				PLATFORM_LIBS += $(srctree)/arch/arm/mach-owl/ats3605/nandlib/libnand_ats3605_mlc.lib
			else
				PLATFORM_LIBS += $(srctree)/arch/arm/mach-owl/ats3605/nandlib/libnand_ats3605.lib
			endif
		endif
	endif
endif
