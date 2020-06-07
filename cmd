export M5_PATH="/home/chenboya/GEM5/fs_home/"
export GEM5_PATH="/home/chenboya/GEM5/gem5/"
sudo sysctl -w kernel.perf_event_paranoid=1

$GEM5_PATH/build/ARM/gem5.opt -d big_u4 ./configs/arm_fs.py --cpu-type atomic --disk $M5_PATH/disks/linaro-minimal-aarch64.img --big-cpus 1

