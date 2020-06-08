
from __future__ import print_function
from __future__ import absolute_import

import m5
from m5.objects import *
m5.util.addToPath('../')
m5.util.addToPath(os.environ['GEM5_PATH']+'/configs/')
from common.Caches import *
from common import ObjectList

have_kvm = "ArmV8KvmCPU" in ObjectList.cpu_list.get_names()

class L1I(Cache):
    tag_latency = 1
    data_latency = 1
    response_latency = 2
    mshrs = 20
    tgts_per_mshr = 16
    size = '64kB'
    assoc = 4
    is_read_only = True
    writeback_clean = True
    replacement_policy = TreePLRURP()


class L1D(Cache):
    tag_latency = 1
    data_latency = 1
    response_latency = 2
    mshrs = 20
    tgts_per_mshr = 16
    size = '64kB'
    assoc = 4
    write_buffers = 16
    sequential_access = True
    writeback_clean = True
    replacement_policy = TreePLRURP()



class WalkCache(PageTableWalkerCache):
    tag_latency = 1
    data_latency = 1
    response_latency = 2
    mshrs = 48
    tgts_per_mshr = 48
    size = '6kB'
    assoc = 48
    write_buffers = 16
    is_read_only = True
    writeback_clean = True

class L2(Cache):
    tag_latency = 9
    data_latency = 9
    response_latency = 6
    mshrs = 46
    tgts_per_mshr = 32
    size = '512kB'
    assoc = 8
    write_buffers = 8
    clusivity='mostly_excl'
    tags = BaseSetAssoc()
    replacement_policy = LRURP()


class L3(Cache):
    tag_latency = 31
    data_latency = 31
    response_latency = 20
    mshrs = 94
    tgts_per_mshr = 16
    size = '4MB'
    assoc = 16
    write_buffers = 16
    clusivity='mostly_excl'
    replacement_policy = LRURP()

class SysCache(Cache):
    tag_latency = 60
    data_latency = 60
    response_latency = 40
    mshrs = 64
    tgts_per_mshr = 32
    size = '8MB'
    assoc = 16
    write_buffers = 32
    clusivity='mostly_excl'
    replacement_policy = LRURP()

class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio

class CpuCluster(SubSystem):
    def __init__(self, system,  num_cpus, cpu_clock, cpu_voltage,
                 cpu_type, l1i_type, l1d_type, wcache_type, l2_type):
        super(CpuCluster, self).__init__()
        self._cpu_type = cpu_type
        self._l1i_type = l1i_type
        self._l1d_type = l1d_type
        self._wcache_type = wcache_type
        self._l2_type = l2_type

        assert num_cpus > 0

        self.voltage_domain = VoltageDomain(voltage=cpu_voltage)
        self.clk_domain = SrcClockDomain(clock=cpu_clock,
                                         voltage_domain=self.voltage_domain)

        self.cpus = [ self._cpu_type(cpu_id=system.numCpus() + idx,
                                     clk_domain=self.clk_domain)
                      for idx in range(num_cpus) ]

        for cpu in self.cpus:
            cpu.createThreads()
            cpu.createInterruptController()
            cpu.socket_id = system.numCpuClusters()
        system.addCpuCluster(self, num_cpus)

    def requireCaches(self):
        return self._cpu_type.require_caches()

    def memoryMode(self):
        return self._cpu_type.memory_mode()

    # new cpu cluster, every ARM core has its own L2 cache
    def addL1L2(self):
        for cpu in self.cpus:
            l1i = None if self._l1i_type is None else self._l1i_type()
            l1d = None if self._l1d_type is None else self._l1d_type()
            iwc = None if self._wcache_type is None else self._wcache_type()
            dwc = None if self._wcache_type is None else self._wcache_type()
            l2  = None if self._l2_type is None else self._l2_type()
            l2b = L2XBar(width = 64, clk_domain = self.clk_domain)
            cpu.addTwoLevelCacheHierarchy(l1i, l1d, l2, iwc, dwc, l2b)

    def addPMUs(self, ints, events=[]):
        assert len(ints) == len(self.cpus)
        for cpu, pint in zip(self.cpus, ints):
            int_cls = ArmPPI if pint < 32 else ArmSPI
            for isa in cpu.isa:
                isa.pmu = ArmPMU(interrupt=int_cls(num=pint))
                isa.pmu.addArchEvents(cpu=cpu, itb=cpu.itb, dtb=cpu.dtb,
                                      icache=getattr(cpu, 'icache', None),
                                      dcache=getattr(cpu, 'dcache', None),
                                      l2cache=getattr(cpu, 'l2cache', None))
                for ev in events:
                    isa.pmu.addEvent(ev)

    def connectMemSide(self, bus):
        for cpu in self.cpus:
            cpu.connectAllPorts(bus)

class AtomicCluster(CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock, cpu_voltage="1.0V"):
        cpu_config = [ ObjectList.cpu_list.get("AtomicSimpleCPU"), L1I,
                       L1D, WalkCache, L2]
        super(AtomicCluster, self).__init__(system, num_cpus, cpu_clock,
                                            cpu_voltage, *cpu_config)
    #def addL1L2(self):
    #    pass

class KvmCluster(CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock, cpu_voltage="1.0V"):
        cpu_config = [ ObjectList.cpu_list.get("ArmV8KvmCPU"), None, None,
            None, None ]
        super(KvmCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)
    def addL1L2(self):
        pass

def simpleSystem(BaseSystem, caches, mem_size, platform=None, **kwargs):

    class SimpleSystem(BaseSystem):

        cache_line_size = 64

        def __init__(self, caches, mem_size, platform=None, **kwargs):
            super(SimpleSystem, self).__init__(**kwargs)

            self.voltage_domain = VoltageDomain(voltage="1.0V")
            self.clk_domain = SrcClockDomain(
                clock="1GHz",
                voltage_domain=Parent.voltage_domain)

            if platform is None:
                self.realview = VExpress_GEM5_V1()
            else:
                self.realview = platform

            if hasattr(self.realview.gic, 'cpu_addr'):
                self.gic_cpu_addr = self.realview.gic.cpu_addr
            self.flags_addr = self.realview.realview_io.pio_addr + 0x30

            self.membus = MemBus()

            self.intrctrl = IntrControl()
            self.terminal = Terminal()
            self.vncserver = VncServer()

            self.iobus = IOXBar()

            self.iobridge = Bridge(delay='50ns')

            mem_range = self.realview._mem_regions[0]
            assert long(mem_range.size()) >= long(Addr(mem_size))
            self.mem_ranges = [
                AddrRange(start=mem_range.start, size=mem_size) ]

            self._caches = caches
            self.dmabridge = Bridge(delay='50ns', 
                                    ranges=[self.mem_ranges[0]])

            self._clusters = []
            self._num_cpus = 0

        def attach_pci(self, dev):
            self.realview.attachPciDevice(dev, self.iobus)

        def connect(self):
            self.iobridge.master = self.iobus.slave
            self.iobridge.slave = self.membus.master
            self.dmabridge.master = self.membus.slave
            self.dmabridge.slave = self.iobus.master

            if hasattr(self.realview.gic, 'cpu_addr'):
                self.gic_cpu_addr = self.realview.gic.cpu_addr
            self.realview.attachOnChipIO(self.membus, self.iobridge)
            self.realview.attachIO(self.iobus)
            self.system_port = self.membus.slave

        def numCpuClusters(self):
            return len(self._clusters)

        def addCpuCluster(self, cpu_cluster, num_cpus):
            assert cpu_cluster not in self._clusters
            assert num_cpus > 0
            self._clusters.append(cpu_cluster)
            self._num_cpus += num_cpus

        def numCpus(self):
            return self._num_cpus

        def addCaches(self, need_caches):
            if not need_caches:
                for cluster in self._clusters:
                    cluster.connectMemSide(self.membus)
                return

            cluster_mem_bus = self.membus

            for cluster in self._clusters:
                cluster.addL1L2()

            max_clock_cluster = max(self._clusters,
                    key = lambda c: c.clk_domain.clock[0])
            self.l3 = L3(clk_domain=max_clock_cluster.clk_domain)
            self.toL3Bus = L2XBar(width=64)
            self.toL3Bus.master = self.l3.cpu_side
            cluster_mem_bus = self.toL3Bus

            self.l4 = SysCache(clk_domain=max_clock_cluster.clk_domain)
            self.toL4Bus = L2XBar(width=64)
            self.toL4Bus.master = self.l4.cpu_side
            self.l4.mem_side = self.membus.slave
            self.l3.mem_side = self.toL4Bus.slave

            for cluster in self._clusters:
                cluster.connectMemSide(cluster_mem_bus)

    return SimpleSystem(caches, mem_size, platform, **kwargs)
