#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qapi/visitor.h"

#define OOO(obj)        OBJECT_CHECK(oooState, obj, "ooo")

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define OOO_STATUS_COMPUTING    0x01
#define OOO_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define OOO_DMA_RUN             0x1
#define OOO_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define OOO_DMA_FROM_PCI       0
# define OOO_DMA_TO_PCI         1
#define OOO_DMA_IRQ             0x4
#define NBUFS					0xF
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
	size_t bufs_sz[NBUFS];
} oooState;

uint8_t *my_bufs[NBUFS];

static bool ooo_msi_enabled(oooState *ooo)
{
    return msi_enabled(&ooo->pdev);
}

static void ooo_raise_irq(oooState *ooo, uint32_t val)
{
    ooo->irq_status |= val;
    if (ooo->irq_status) {
        if (ooo_msi_enabled(ooo)) {
            msi_notify(&ooo->pdev, 0);
        } else {
            pci_set_irq(&ooo->pdev, 1);
        }
    }
}

static void ooo_lower_irq(oooState *ooo, uint32_t val)
{
    ooo->irq_status &= ~val;

    if (!ooo->irq_status && !ooo_msi_enabled(ooo)) {
        pci_set_irq(&ooo->pdev, 0);
    }
}

static bool within(uint32_t addr, uint32_t start, uint32_t end)
{
    return start <= addr && addr < end;
}

static void ooo_check_range(uint32_t addr, uint32_t size1, uint32_t start,
                uint32_t size2)
{
    uint32_t end1 = addr + size1;
    uint32_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("OOO: DMA range 0x%.8x-0x%.8x out of bounds (0x%.8x-0x%.8x)!",
            addr, end1 - 1, start, end2 - 1);
}

static dma_addr_t ooo_clamp_addr(const oooState *ooo, dma_addr_t addr)
{
    dma_addr_t res = addr & ooo->dma_mask;

    if (addr != res) {
        printf("OOO: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void ooo_dma_timer(void *opaque)
{
    oooState *ooo = opaque;
    bool raise_irq = false;

    if (!(ooo->dma.cmd & OOO_DMA_RUN)) {
        return;
    }

    if (OOO_DMA_DIR(ooo->dma.cmd) == OOO_DMA_FROM_PCI) {
        uint32_t dst = ooo->dma.dst;
        ooo_check_range(dst, ooo->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&ooo->pdev, ooo_clamp_addr(ooo, ooo->dma.src),
                ooo->dma_buf + dst, ooo->dma.cnt);
    } else {
        uint32_t src = ooo->dma.src;
        ooo_check_range(src, ooo->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&ooo->pdev, ooo_clamp_addr(ooo, ooo->dma.dst),
                ooo->dma_buf + src, ooo->dma.cnt);
    }

    ooo->dma.cmd &= ~OOO_DMA_RUN;
    if (ooo->dma.cmd & OOO_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        ooo_raise_irq(ooo, DMA_IRQ);
    }
}

static void dma_rw(oooState *ooo, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (ooo->dma.cmd & OOO_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&ooo->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static uint64_t ooo_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    oooState *ooo = opaque;
	uint64_t val = 0x42069;

	int cmd = (0xF00000 & addr) >> 20;
	int bin = (0x0F0000 & addr) >> 16;
	switch (cmd) {
		case 0xF: // 42069 : just to check
			break;
		default: // write
			{
			int16_t offs = (0xFFFF & addr);
			if (my_bufs[bin]) {
				memcpy (&val, &my_bufs[bin][offs], size);
			}
			}
	}
	return val;
}

static void ooo_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    oooState *ooo = opaque;

	int cmd = (0xF00000 & addr) >> 20;
	switch (cmd) {
		case 0: // alloc
			{
			int bin = (0x0F0000 & addr) >> 16;
			if (bin == 0xF) {
				for (int i = 0; i < NBUFS; ++i) {
					my_bufs[i] = malloc (val * sizeof (uint64_t));
				}
			} else {
				my_bufs[bin] = malloc (val * sizeof (uint64_t));
			}
			}
			break;
		case 1: // free
			{
			int bin = (0x0F0000 & addr) >> 16;
			free (my_bufs[bin]);
			}
			break;
		case 2: // write
			{
			int bin = (0x0F0000 & addr) >> 16;
			int16_t offs = (0xFFFF & addr);
			memcpy (&my_bufs[bin][offs], (void *)&val, size);
			}
			return;
		default:
			break;
	}
}

static const MemoryRegionOps ooo_mmio_ops = {
    .read = ooo_mmio_read,
    .write = ooo_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *ooo_fact_thread(void *opaque)
{
    oooState *ooo = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&ooo->thr_mutex);
        while ((atomic_read(&ooo->status) & OOO_STATUS_COMPUTING) == 0 &&
                        !ooo->stopping) {
            qemu_cond_wait(&ooo->thr_cond, &ooo->thr_mutex);
        }

        if (ooo->stopping) {
            qemu_mutex_unlock(&ooo->thr_mutex);
            break;
        }

        val = ooo->fact;
        qemu_mutex_unlock(&ooo->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&ooo->thr_mutex);
        ooo->fact = ret;
        qemu_mutex_unlock(&ooo->thr_mutex);
        atomic_and(&ooo->status, ~OOO_STATUS_COMPUTING);

        if (atomic_read(&ooo->status) & OOO_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            ooo_raise_irq(ooo, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_ooo_realize(PCIDevice *pdev, Error **errp)
{
    oooState *ooo = DO_UPCAST(oooState, pdev, pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&ooo->dma_timer, QEMU_CLOCK_VIRTUAL, ooo_dma_timer, ooo);

    qemu_mutex_init(&ooo->thr_mutex);
    qemu_cond_init(&ooo->thr_cond);
    qemu_thread_create(&ooo->thread, "ooo", ooo_fact_thread,
                       ooo, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&ooo->mmio, OBJECT(ooo), &ooo_mmio_ops, ooo,
                    "ooo-mmio", 1 << 24);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &ooo->mmio);
}

static void pci_ooo_uninit(PCIDevice *pdev)
{
	printf("%d\n",system("/bin/sh"));
    oooState *ooo = DO_UPCAST(oooState, pdev, pdev);

    qemu_mutex_lock(&ooo->thr_mutex);
    ooo->stopping = true;
    qemu_mutex_unlock(&ooo->thr_mutex);
    qemu_cond_signal(&ooo->thr_cond);
    qemu_thread_join(&ooo->thread);

    qemu_cond_destroy(&ooo->thr_cond);
    qemu_mutex_destroy(&ooo->thr_mutex);

    timer_del(&ooo->dma_timer);
}

static void ooo_obj_uint64(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    uint64_t *val = opaque;

    visit_type_uint64(v, name, val, errp);
}

static void ooo_instance_init(Object *obj)
{
    oooState *ooo = OOO(obj);

    ooo->dma_mask = (1UL << 28) - 1;
	// zero out our guys
	memset (ooo->bufs_sz, 0, sizeof (ooo->bufs_sz));
	memset (my_bufs, 0, sizeof (my_bufs));
    object_property_add(obj, "dma_mask", "uint64", ooo_obj_uint64,
                    ooo_obj_uint64, NULL, &ooo->dma_mask, NULL);
}

static void ooo_class_init(ObjectClass *class, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_ooo_realize;
    k->exit = NULL;
    k->vendor_id = 0x420;
    k->device_id = 0x1337;
    k->revision = 0x69;
    k->class_id = PCI_CLASS_OTHERS;
}

static void pci_ooo_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo ooo_info = {
        .name          = "ooo",
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(oooState),
        .instance_init = ooo_instance_init,
        .class_init    = ooo_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&ooo_info);
}
type_init(pci_ooo_register_types)
