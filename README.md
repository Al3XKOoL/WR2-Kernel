Wiko Rainbow Custom Kernel [3.4.106] 
========================================

kernel:

cd tinno82_S5501

run command: 

./export.sh

========================================

Now build:

./mk -o=TARGET_BUILD_VARIANT=user n k

Or verbose

./mk -t -o=TARGET_BUILD_VARIANT=user n k

========================================

Then, to create the boot.img:
./pack_bootimage.sh
