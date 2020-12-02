#!/bin/sh

PROJ_ROOT=$PWD

if [ "$#" != "1" ]; then
    echo "usage: $0 configs/config.chip"
fi
if [ -e configs ]; then


    echo PROJ_ROOT = $PROJ_ROOT > configs/current.configs
    echo CONFIG_NAME = config_module_list.mk >> configs/current.configs
    echo SOURCE_MK = ../sdk/sdk.mk >> configs/current.configs
	echo "KERNEL_MEMADR = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM phyaddr)" >> configs/current.configs
	echo "KERNEL_MEMLEN = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM size)" >> configs/current.configs
	echo "KERNEL_MEMADR2 = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM2 phyaddr)" >> configs/current.configs
	echo "KERNEL_MEMLEN2 = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM2 size)" >> configs/current.configs
	echo "KERNEL_MEMADR3 = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM3 phyaddr)" >> configs/current.configs
	echo "KERNEL_MEMLEN3 = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) E_LX_MEM3 size)" >> configs/current.configs
	echo "LOGO_ADDR = \$(shell $PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/\$(CHIP)/mmap/\$(MMAP) \$(CHIP) \$(BOOTLOGO_ADDR) miuaddr)" >> configs/current.configs
    cat $1 >> configs/current.configs
    echo "ARCH=arm" >> configs/current.configs
    echo "CROSS_COMPILE=$(sed -n "/TOOLCHAIN_REL/p"  configs/current.configs | awk '{print $3}')" >> configs/current.configs

    c=$(sed -n "/^CHIP/p"  configs/current.configs | awk '{print $3}')

    if [ "$c" = "i5" ]; then
        echo "CHIP_ALIAS = pretzel" >> configs/current.configs
    elif [ "$c" = "i6" ]; then
        echo "CHIP_ALIAS = macaron" >> configs/current.configs
    elif [ "$c" = "i2m" ]; then
        echo "CHIP_ALIAS = taiyaki" >> configs/current.configs
    elif [ "$c" = "p2" ]; then
        echo "CHIP_ALIAS = takoyaki" >> configs/current.configs
    elif [ "$c" = "i6e" ]; then
        echo "CHIP_ALIAS = pudding" >> configs/current.configs
    elif [ "$c" = "i6b0" ]; then
        echo "CHIP_ALIAS = ispahan" >> configs/current.configs
    fi
else
    echo "can't found configs directory!"
fi

CHIP=`cat $1 | awk '/CHIP/ {print substr($3,$1)}'`
MMAP=`cat $1 | awk '/MMAP/ {print substr($3,$1)}'`
kernel_reserved_env="mmap_reserved="
FBADDR=`$PROJ_ROOT/image/makefiletools/bin/mmapparser $PROJ_ROOT/board/$CHIP/mmap/$MMAP $CHIP E_MMAP_ID_FB phyaddr`
assemble_kernel_reserved_env()
{
	n=$#
	n=$(expr $n \/ 5 \- 1)
	for i in $(seq 0 $n)  
	do
		j=$(expr $i \* 5)
		j=$(expr $j \+ 1)
		name=$(eval "echo \${$j}")
		name=$(tr [A-Z] [a-z] <<< $name)
		j=$(expr $j \+ 1)
		miu=$(eval "echo \${$j}")
		j=$(expr $j \+ 1)
		sz=$(eval "echo \${$j}")
		j=$(expr $j \+ 1)
		start=$(eval "echo \${$j}")
		j=$(expr $j \+ 1)
		end=$(eval "echo \${$j}")
		kernel_reserved_env+="$name,miu=$miu,sz=$sz,max_start_off=$start,max_end_off=$end "
		if [ "$name" == "bootlogo" ]; then
			sed -i "s/LOGO_ADDR = .*/LOGO_ADDR = $start/g" configs/current.configs
		fi
		if [ "$FBADDR" != "" ]; then
			sed -i "s/LOGO_ADDR = .*/LOGO_ADDR = $FBADDR/g" configs/current.configs
		fi
	done
}
data=`$PROJ_ROOT/image/makefiletools/bin/reserved $PROJ_ROOT/board/$CHIP/mmap/$MMAP $CHIP `
assemble_kernel_reserved_env $data
if [ $kernel_reserved_env != "mmap_reserved=" ]; then
	sed -i "s/KERNEL_BOOT_ENV.*/& \$(KERNEL_RESERVED_ENV)/g" configs/current.configs
	echo "KERNEL_RESERVED_ENV = $kernel_reserved_env" >> configs/current.configs
fi

cat configs/current.configs
