#!/system/bin/sh

/system/xbin/busybox echo 1 > /sys/block/zram0/reset
/system/xbin/busybox echo 100114841 > /sys/block/zram0/disksize
/system/xbin/busybox mkswap /dev/block/zram0 &> /dev/null
/system/xbin/busybox swapon /dev/block/zram0 &> /dev/null

/system/xbin/busybox echo 0 > /proc/sys/vm/page-cluster
sysctl -w vm.swappiness=15
sysctl -w
vm.vfs_cache_pressure=100
