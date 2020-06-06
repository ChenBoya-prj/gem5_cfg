export M5_PATH="/home/chenboya/wrk/fs_home/"
export GEM5_PATH="/home/chenboya/wrk/gem5/"

$GEM5_PATH/build/ARM/gem5.opt ./configs/arm_fs.py --cpu-type atomic --disk $M5_PATH/disks/linaro-minimal-aarch64.img --kernel $M5_PATH/binaries/vmlinux.vexpress_gem5_v1_64 --big-cpus 1 --caches --dtb $M5_PATH/binaries/armv8_gem5_v1_1cpu.dtb

