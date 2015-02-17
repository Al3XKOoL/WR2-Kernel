#!/bin/bash

PRJ=wiko

# make boot.img
 echo "		==> [WORKING] build boot.img in process..."
 echo ""
 echo ""
 echo ""
 echo ""
 mediatek/build/tools/images/acp -uv out/target/product/${PRJ}/kernel_${PRJ}.bin out/target/product/${PRJ}/kernel
 mediatek/build/tools/images/mkbootfs mediatek/host/root | mediatek/build/tools/images/minigzip > out/target/product/${PRJ}/ramdisk.img
 mediatek/build/tools/mkimage out/target/product/${PRJ}/ramdisk.img ROOTFS > out/target/product/${PRJ}/ramdisk_android.img
 mv out/target/product/${PRJ}/ramdisk.img mediatek/kernel/trace32/${PRJ}_ramdisk.img
 mv out/target/product/${PRJ}/ramdisk_android.img out/target/product/${PRJ}/ramdisk.img
 mediatek/build/tools/images/mkbootimg  --kernel out/target/product/${PRJ}/kernel --ramdisk out/target/product/${PRJ}/ramdisk.img --board 1336460062 --output out/target/product/${PRJ}/boot.img
 echo "		==> [SUCCESS]   boot.img done!"


# Copy all to build_results folder
 echo ""
 echo ""
 echo ""
 echo ""
 echo "		==> [WORKING] moving files!"
 rm -R ./build_results/
 mkdir -p build_results
 cp out/target/product/${PRJ}/boot.img build_results
 cp out/target/product/${PRJ}/kernel build_results
 mv build_results/kernel build_results/zImage
 echo ""
 echo ""
 echo ""
 echo "		==> [SUCCESS] find files into build_results folder!"


# Make flashable zip
 cp out/target/product/${PRJ}/boot.img mediatek/host/zip
 cd mediatek/host/zip
 zip -r -0 WR-Kernel.zip ./*
 mv WR-Kernel.zip ../../../build_results
 rm ../../../mediatek/host/zip/boot.img
 echo "		==> [OK]   Find all filen into build_results folder!"

