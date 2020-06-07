# fs mode arm cpu

from __future__ import print_function
from __future__ import absolute_import

import argparse
import os
import m5
import m5.util
from m5.objects import *

m5.util.addToPath("../")
m5.util.addToPath(os.environ['GEM5_PATH']+'/configs/')

from common import FSConfig
from common import SysPaths
from common import ObjectList
from common import Options
import arm_sys
from arm_sys import AtomicCluster, KvmCluster

default_disk = 'linaro-minimal-aarch64.img'
default_kernel = 'vmlinux.vexpress_gem5_v1_64'
default_rcs = 'bootscript.rcS'
default_mem_size= "8GB"

def _to_ticks(value):
    """Helper function to convert a latency from string format to Ticks"""

    return m5.ticks.fromSeconds(m5.util.convert.anyToLatency(value))

def _using_pdes(root):
    """Determine if the simulator is using multiple parallel event queues"""

    for obj in root.descendants():
        if not m5.proxy.isproxy(obj.eventq_index) and \
               obj.eventq_index != root.eventq_index:
            return True

    return False


def createSystem(caches, kernel, bootscript, machine_type="VExpress_GEM5_V1",
        disks=[], mem_size=default_mem_size, bootloader=None):

    platform = ObjectList.platform_list.get(machine_type)
    m5.util.inform("Simulated platform: %s", platform.__name__)

    sys = arm_sys.simpleSystem(ArmSystem, caches, mem_size, platform(),
                            workload=ArmFsLinux(object_file=SysPaths.binary(kernel)),
                            readfile=bootscript)
    sys.mem_ctrls = [ SimpleMemory(range = r, port=sys.membus.master)
                        for r in sys.mem_ranges ]
    sys.connect()

    if disks:
        def cow_disk(image_file):
            image = CowDiskImage()
            image.child.image_file = SysPaths.disk(image_file)
            return image

        sys.disk_images = [ cow_disk(f) for f in disks ]
        sys.pci_vio_block = [ PciVirtIO(vio=VirtIOBlock(image=img))
                              for img in sys.disk_images ]
        for dev in sys.pci_vio_block:
            sys.attach_pci(dev)

    sys.realview.setupBootLoader(sys, SysPaths.binary, bootloader)

    return sys

cpu_types = {
    "atomic" : (AtomicCluster, AtomicCluster, AtomicCluster),
}
# Only add the KVM CPU if it has been compiled into gem5
if arm_sys.have_kvm:
    cpu_types["kvm"] = (KvmCluster, KvmCluster, KvmCluster)

def addOptions(parser):
    parser.add_argument("--restore-from", type=str, default=None,
                        help="Restore from checkpoint")
    parser.add_argument("--dtb", type=str, default=None,
                        help="DTB file to load")
    parser.add_argument("--kernel", type=str, default=default_kernel,
                        help="Linux kernel")
    parser.add_argument("--root", type=str, default="/dev/vda1",
                        help="Specify the kernel CLI root= argument")
    parser.add_argument("--machine-type", type=str,
                        choices=ObjectList.platform_list.get_names(),
                        default="VExpress_GEM5",
                        help="Hardware platform class")
    parser.add_argument("--disk", action="append", type=str, default=[],
                        help="Disks to instantiate")
    parser.add_argument("--bootscript", type=str, default=default_rcs,
                        help="Linux bootscript")
    parser.add_argument("--cpu-type", type=str, choices=list(cpu_types.keys()),
                        default="timing",
                        help="CPU simulation mode. Default: %(default)s")
    parser.add_argument("--kernel-init", type=str, default="/sbin/init",
                        help="Override init")
    parser.add_argument("--big-cpus", type=int, default=1,
                        help="Number of big CPUs to instantiate")
    parser.add_argument("--mid-cpus", type=int, default=0,
                        help="Number of middle CPUs to instantiate")
    parser.add_argument("--little-cpus", type=int, default=0,
                        help="Number of little CPUs to instantiate")
    parser.add_argument("--caches", action="store_true", default=False,
                        help="Instantiate caches")
    parser.add_argument("--last-cache-level", type=int, default=2,
                        help="Last level of caches (e.g. 3 for L3)")
    parser.add_argument("--big-cpu-clock", type=str, default="3GHz",
                        help="Big CPU clock frequency")
    parser.add_argument("--mid-cpu-clock", type=str, default="2GHz",
                        help="Middle CPU clock frequency")
    parser.add_argument("--little-cpu-clock", type=str, default="2GHz",
                        help="Little CPU clock frequency")
    parser.add_argument("--sim-quantum", type=str, default="1ms",
                        help="Simulation quantum for parallel simulation. " \
                        "Default: %(default)s")
    parser.add_argument("--mem-size", type=str, default=default_mem_size,
                        help="System memory size")
    parser.add_argument("--kernel-cmd", type=str, default=None,
                        help="Custom Linux kernel command")
    parser.add_argument("--bootloader", action="append",
                        help="executable file that runs before the --kernel")
    parser.add_argument("-P", "--param", action="append", default=[],
        help="Set a SimObject parameter relative to the root node. "
             "An extended Python multi range slicing syntax can be used "
             "for arrays. For example: "
             "'system.cpu[0,1,3:8:2].max_insts_all_threads = 42' "
             "sets max_insts_all_threads for cpus 0, 1, 3, 5 and 7 "
             "Direct parameters of the root object are not accessible, "
             "only parameters of its children.")
    parser.add_argument("--vio-9p", action="store_true",
                        help=Options.vio_9p_help)
    return parser

def build(options):
    m5.ticks.fixGlobalFrequency()

    kernel_cmd = [
            "earlyprintk=pl011,0x1c090000",
            "console=ttyAMA0",
            "lpj=19988480",
            "norandmaps",
            "loglevel=8",
            "mem=%s" % options.mem_size,
            "root=%s" % options.root,
            "rw",
            "init=%s" % options.kernel_init,
            "vmalloc=768MB",
            ]
    root = Root(full_system=True)
    disks = [default_disk] if len(options.disk)==0 else options.disk
    system = createSystem(  options.caches,
                            options.kernel,
                            options.bootscript,
                            options.machine_type,
                            disks=disks,
                            mem_size=options.mem_size,
                            bootloader=options.bootloader)

    root.system = system
    if options.kernel_cmd:
        system.workload.command_line = options.kernel_cmd
    else:
        system.workload.command_line = " ".join(kernel_cmd)

    if options.big_cpus+options.little_cpus+options.mid_cpus ==0:
        m5.util.panic("Empty CPU clusters")

    big_model, mid_model, little_model = cpu_types[options.cpu_type]
    all_cpus = []

    if options.big_cpus > 0:
        system.bigCluster = big_model(system, options.big_cpus,
                                    options.big_cpu_clock)
        system.mem_mode = system.bigCluster.memoryMode()
        all_cpus += system.bigCluster.cpus

    if options.mid_cpus > 0:
        system.midCluster = mid_model(system, options.mid_cpus,
                                    options.mid_cpu_clock)
        ssytem.mem_mode = system.midCluster.memoryMode()
        all_cpus += system.midCluster.cpus

    if options.little_cpus > 0:
        system.littleCluster = little_model(system, options.little_cpus,
                                    options.little_cpu_clock)
        ssytem.mem_mode = system.littleCluster.memoryMode()
        all_cpus += system.littleCluster.cpus


    system.addCaches(options.caches)
    if not options.caches:
        if options.big_cpus > 0 and system.bigCluster.requireCaches(): 
            m5.util.panic("Big CPU model requires caches")
        if options.mid_cpus > 0 and system.bigCluster.requireCaches(): 
            m5.util.panic("Mid CPU model requires caches")
        if options.little_cpus > 0 and system.littleCluster.requireCaches():
            m5.util.panic("Little CPU model requires caches")

    if issubclass(big_model, KvmCluster):
        _build_kvm(system, all_cpus)

    # Linux device tree
    if options.dtb is not None:
        system.workload.dtb_filename = SysPaths.binary(options.dtb)
    else:
        system.workload.dtb_filename = \
            os.path.join(m5.options.outdir, 'system.dtb')
        system.generateDtb(system.workload.dtb_filename)

    if options.vio_9p:
        FSConfig.attach_9p(system.realview, system.iobus)

    return root

def _build_kvm(system, cpus):
    system.kvm_vm = KvmVM()

    # Assign KVM CPUs to their own event queues / threads. This
    # has to be done after creating caches and other child objects
    # since these mustn't inherit the CPU event queue.
    if len(cpus) > 1:
        device_eq = 0
        first_cpu_eq = 1
        for idx, cpu in enumerate(cpus):
            # Child objects usually inherit the parent's event
            # queue. Override that and use the same event queue for
            # all devices.
            for obj in cpu.descendants():
                obj.eventq_index = device_eq
            cpu.eventq_index = first_cpu_eq + idx


def instantiate(options, checkpoint_dir=None):
    # Setup the simulation quantum if we are running in PDES-mode
    # (e.g., when using KVM)
    root = Root.getInstance()
    if root and _using_pdes(root):
        m5.util.inform("Running in PDES mode with a %s simulation quantum.",
                       options.sim_quantum)
        root.sim_quantum = _to_ticks(options.sim_quantum)

    # Get and load from the chkpt or simpoint checkpoint
    if options.restore_from:
        if checkpoint_dir and not os.path.isabs(options.restore_from):
            cpt = os.path.join(checkpoint_dir, options.restore_from)
        else:
            cpt = options.restore_from

        m5.util.inform("Restoring from checkpoint %s", cpt)
        m5.instantiate(cpt)
    else:
        m5.instantiate()

def run(checkpoint_dir=m5.options.outdir):
    # start simulation (and drop checkpoints when requested)
    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(checkpoint_dir, "cpt.%d" % m5.curTick())
            m5.checkpoint(cpt_dir)
            print("Checkpoint done.")
        else:
            print(exit_msg, " @ ", m5.curTick())
            break

    sys.exit(event.getCode())

def main():
    parser = argparse.ArgumentParser(
        description="Generic ARM big.LITTLE configuration")
    addOptions(parser)
    options = parser.parse_args()
    root = build(options)
    root.apply_config(options.param)
    instantiate(options)
    run()


if __name__ == "__m5_main__":
    main()
