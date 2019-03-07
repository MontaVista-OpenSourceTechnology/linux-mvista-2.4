/*****************************************************************************
 * Broadcom Proprietary and Confidential. © 2016 Broadcom. All rights reserved.
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 *****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <asm/memory.h>
#include <asm/mach/irq.h>

#define IPROC_PCIE_COMPATIBLE "brcm,iproc-pcie"
#define IPROC_MSI_COMPATIBLE "brcm,iproc-msi"

#define PAXB_0_CLK_CONTROL(base)		(base + 0x000)
#define PAXB_0_PAXB_ENDIANNESS(base)		(base + 0x030)
#define PAXB_0_CONFIG_IND_ADDR(base)		(base + 0x120)
#define PAXB_0_CONFIG_IND_DATA(base)		(base + 0x124)
#define PAXB_0_PCIE_SYS_MSI_PAGE(base)		(base + 0x204)
#define PAXB_0_PCIE_SYS_MSI_REQ(base)		(base + 0x340)
#define PAXB_0_PCIE_SYS_HOST_INTR_EN(base)	(base + 0x344)
#define PAXB_0_PCIE_SYS_HOST_INTR_CSR(base)	(base + 0x348)
#define PAXB_0_PCIE_SYS_HOST_INTR_0(base)	(base + 0x350)
#define PAXB_0_PCIE_SYS_HOST_INTR_1(base)	(base + 0x354)
#define PAXB_0_PCIE_SYS_EP_INT_EN0(base)	(base + 0x360)
#define PAXB_0_PCIE_SYS_EP_INT_EN1(base)	(base + 0x364)
#define PAXB_0_PCIE_SYS_EP_INT_CSR0(base)	(base + 0x370)
#define PAXB_0_PCIE_SYS_EP_INT_CSR1(base)	(base + 0x374)

#define PAXB_0_FUNC0_BAR0(base)			(base + 0x410)
#define PAXB_0_FUNC0_BAR2(base)			(base + 0x418)
#define PAXB_0_FUNC0_BAR4(base)			(base + 0x420)
#define PAXB_0_FUNC0_MSI_CAPA(base)		(base + 0x458)
#define PAXB_0_FUNC0_MSI_ADDRL(base)		(base + 0x45c)
#define PAXB_0_FUNC0_MSI_ADDRH(base)		(base + 0x460)
#define PAXB_0_FUNC0_MSI_DATA(base)		(base + 0x464)

#define MSI_IS_ENABLED(val)			(val & 0x10000)
#define MSI_MULTI_MSG(val)			((val & 0xE0000) >> 16)

#define PAXB_0_FUNC0_IMAP0_0(base)		(base + 0xc00)
#define PAXB_0_FUNC0_IMAP1(base)		(base + 0xc80)
#define PAXB_0_FUNC0_IMAP2(base)		(base + 0xcc0)
#define PAXB_0_IARR_1_LOWER(base)		(base + 0xd08)
#define PAXB_0_IARR_2_LOWER(base)		(base + 0xd10)
#define PAXB_0_OARR_0(base)			(base + 0xd20)
#define PAXB_0_OARR_0_UPPER(base)		(base + 0xd24)
#define PAXB_0_OARR_FUNC0_MSI_PAGE(base)	(base + 0xd34)
#define PAXB_0_OMAP_0_LOWER(base)		(base + 0xd40)
#define PAXB_0_OMAP_0_UPPER(base)		(base + 0xd44)

#define PAXB_0_FUNC1_BAR2(base)			(base + 0x518)
#define PAXB_0_FUNC1_IMAP2(base)		(base + 0xcc8)

static void __iomem *iproc_pcie_base;

int test_msi = -1;
module_param(test_msi, int, 0000);

static size_t _dma_mem_size = 0x100000; /* 1MB */
static void *_dma_vbase[2] = { NULL, NULL };
static u32 _dma_pbase[2] = { 0, 0 };

#define EP_OUTBOUND_ADDR			(0x08000000)
static void * __iomem ob_addr;

#define SOC_PCIE_EP_MAX_IRQ			2

static irqreturn_t ep_intr0_handler(unsigned int irq, void *ptr);
static irqreturn_t ep_intr1_handler(unsigned int irq, void *ptr);

typedef irqreturn_t (*ep_intr_handler)(unsigned int irq, void *ptr);

static ep_intr_handler handlers[SOC_PCIE_EP_MAX_IRQ] = {
	ep_intr0_handler,
	ep_intr1_handler
};

typedef struct {
	int id;
	u32 curr_irqs;
} cookie_t;
static cookie_t cookies[SOC_PCIE_EP_MAX_IRQ];

/*
 * Assuming that pcie_enable_msi and request_irq have been called in eHost.
 */
static void
msi_interrupt_generate(u32 fn, u32 mvec)
{
	u32 value;

	pr_debug("@ %s\n", __func__);
	value = readl_relaxed(PAXB_0_FUNC0_MSI_CAPA(iproc_pcie_base));
	if (!MSI_IS_ENABLED(value))
		pr_err("WARNING: msi is not enabled(0x%x)!\n", value);

	value = readl_relaxed(PAXB_0_PCIE_SYS_EP_INT_EN0(iproc_pcie_base));
	pr_debug("EP_INT_EN0 : 0x%x\n", value);
	value |= 0x1;
	writel_relaxed(value, PAXB_0_PCIE_SYS_EP_INT_EN0(iproc_pcie_base));

	value = readl_relaxed(PAXB_0_OARR_FUNC0_MSI_PAGE(iproc_pcie_base));
	pr_debug("OARR_FUNC0_MSI_PAGE : 0x%x\n", value);
	value |= 0x1;
	writel_relaxed(value, PAXB_0_OARR_FUNC0_MSI_PAGE(iproc_pcie_base));

	/* Write MSI_REQ to trigger interrupt */
	value = ((fn & 0x7) << 5) | (mvec & 0x1f);
	pr_debug("MSI_REQ : 0x%x\n", value);
	writel_relaxed(value, PAXB_0_PCIE_SYS_MSI_REQ(iproc_pcie_base));

}

static irqreturn_t ep_intr0_handler(unsigned int irq, void *ptr)
{
	u32 value;
	cookie_t *ck;

	pr_debug("%s: %d irq : %d entry\n", __func__, __LINE__, irq);
	if (ptr) {
		ck = (cookie_t *)ptr;
		pr_debug("irq = %d\n", ck->curr_irqs);
	}

	value = readl_relaxed(PAXB_0_PCIE_SYS_HOST_INTR_CSR(iproc_pcie_base));
	pr_debug("SYS_HOST_INTR_CSR = 0x%x\n", value);
	/* clear INTR0 */
	writel_relaxed(0x1, PAXB_0_PCIE_SYS_HOST_INTR_CSR(iproc_pcie_base));

	return IRQ_HANDLED;
}

static irqreturn_t ep_intr1_handler(unsigned int irq, void *ptr)
{
	u32 value;
	cookie_t *ck;

	pr_debug("%s: %d irq : %d entry\n", __func__, __LINE__, irq);
	if (ptr) {
		ck = (cookie_t *)ptr;
		pr_debug("irq = %d\n", ck->curr_irqs);
	}

	value = readl_relaxed(PAXB_0_PCIE_SYS_HOST_INTR_CSR(iproc_pcie_base));
	pr_debug("SYS_HOST_INTR_CSR = 0x%x\n", value);
	/* clear INTR1 */
	writel_relaxed(0x2, PAXB_0_PCIE_SYS_HOST_INTR_CSR(iproc_pcie_base));

	return IRQ_HANDLED;
}

static void ep_tlp_generate(u64 pcie_addr, u32 data)
{
	/* PCIE addr = OMAP0[35:26] + Bit[25:0] of AXI addr */
	u32 addr = (u32)pcie_addr & 0x3ffffff;
	u32 value;

	ob_addr = ioremap(EP_OUTBOUND_ADDR | addr, 4);
	if (!ob_addr) {
		pr_err("Unable to remap outbound addr 0x%x\n", EP_OUTBOUND_ADDR);
	} else {
		pr_debug("ob_addr = 0x%x, addr = 0x%x\n", (u32)ob_addr, EP_OUTBOUND_ADDR | addr);
		writel_relaxed(EP_OUTBOUND_ADDR | 0x1, PAXB_0_OARR_0(iproc_pcie_base));
		value = readl_relaxed(PAXB_0_OARR_0(iproc_pcie_base));
		pr_debug("PAXB_0_OARR_0 = 0x%x\n", value);
		writel_relaxed((u32)(pcie_addr >> 32), PAXB_0_OARR_0_UPPER(iproc_pcie_base));
		value = readl_relaxed(PAXB_0_OARR_0_UPPER(iproc_pcie_base));
		pr_debug("PAXB_0_OARR_0_UPPER = 0x%x\n", value);
		writel_relaxed((u32)pcie_addr, PAXB_0_OMAP_0_LOWER(iproc_pcie_base));
		value = readl_relaxed(PAXB_0_OMAP_0_LOWER(iproc_pcie_base));
		pr_debug("PAXB_0_OMAP_0_LOWER = 0x%x\n", value);
		writel_relaxed((u32)(pcie_addr >> 32), PAXB_0_OMAP_0_UPPER(iproc_pcie_base));
		value = readl_relaxed(PAXB_0_OMAP_0_UPPER(iproc_pcie_base));
		pr_debug("PAXB_0_OMAP_0_LOWER = 0x%x\n", value);

		value = ioread32(ob_addr);
		pr_debug("value = 0x%x\n", value);

		iowrite32(data, ob_addr);
		value = ioread32(ob_addr);
		pr_debug("value = 0x%x\n", value);
	}
	iounmap(ob_addr);
	ob_addr = NULL;
}

static int __init
pcie_ep_init(void)
{
	struct device_node *np;
	int i, ret;
	u32 value;
	dma_addr_t dma_handle;
	int irq;

	np = of_find_compatible_node(NULL, NULL, IPROC_PCIE_COMPATIBLE);
	if (!np) {
		pr_err("%s: No PCIE node found\n", __func__);
		return -ENODEV;
	}

	iproc_pcie_base = of_iomap(np, 0);
	if (!iproc_pcie_base)
		return -ENOMEM;

	pr_info("PCIe %s base address = 0x%x\n", np->full_name, (u32)iproc_pcie_base);

	for (i = 0; i < SOC_PCIE_EP_MAX_IRQ; i++) {
		if (!(_dma_vbase[i] = dma_alloc_coherent(0, _dma_mem_size, &dma_handle, GFP_KERNEL)) || !dma_handle) {
			pr_err("[%d]Kernel failed to allocate the memory pool of size 0x%lx\n", i, (unsigned long)_dma_mem_size);
		} else {
			_dma_pbase[i] = dma_handle;
			pr_info("_dma_pbase[%d] = 0x%x\n", i, _dma_pbase[i]);
		}
	}

	writel_relaxed(_dma_pbase[0] | 0x1, PAXB_0_FUNC0_IMAP1(iproc_pcie_base));
	value = readl_relaxed(PAXB_0_FUNC0_IMAP1(iproc_pcie_base));
	pr_debug("PAXB_0_FUNC1_IMAP1 = 0x%x\n", value);

	writel_relaxed(_dma_pbase[1] | 0x1, PAXB_0_FUNC0_IMAP2(iproc_pcie_base));
	value = readl_relaxed(PAXB_0_FUNC0_IMAP2(iproc_pcie_base));
	pr_debug("PAXB_0_FUNC0_IMAP2 = 0x%x\n", value);

	/* Overwrite BAR2 size to 1MB */
	writel_relaxed(0x4e0, PAXB_0_CONFIG_IND_ADDR(iproc_pcie_base));
	writel_relaxed(0x35, PAXB_0_CONFIG_IND_DATA(iproc_pcie_base));

	value = readl_relaxed(PAXB_0_IARR_2_LOWER(iproc_pcie_base));
	/* Assign 1MB size */
	value &= ~(0xff);
	value |= 0x1;
	writel_relaxed(value, PAXB_0_IARR_2_LOWER(iproc_pcie_base));

	if (test_msi == 1) {
		pr_info("Testing pcie ep generate msi interrupt to eHost\n");
		msi_interrupt_generate(0, 0);
	}

	np = of_get_next_available_child(np, NULL);
	if (!np) {
		pr_err("%s: No MSI node found\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < SOC_PCIE_EP_MAX_IRQ; i++) {
		irq = of_irq_get(np, i);
		cookies[i].curr_irqs = irq;
		ret = request_irq(irq, (irq_handler_t)handlers[i], IRQF_SHARED, "PCI-EP000-IPROC", &cookies[i]);
		if (ret != 0) {
			pr_info("%s: %d request_irq %d return = %d\n", __func__, __LINE__, irq, ret);
			cookies[i].curr_irqs = 0;
		}
	}

	/* Enable INTR0 and INTR1 */
	pr_debug("%s: enabling: INTR0, INTR1\n", __func__);
	writel_relaxed(0x3, PAXB_0_PCIE_SYS_HOST_INTR_EN(iproc_pcie_base));
	/* Test interrupts from eHost by writing to PAXB_0_PCIE_SYS_HOST_INTR_0/1 */

	/* Test access to other EPs */
	ep_tlp_generate(0x10006000, 0x1234beef);

	return 0;
}

static void __exit
pcie_ep_exit(void)
{
	int i;

	for (i = 0; i < SOC_PCIE_EP_MAX_IRQ; i++) {
		if (_dma_vbase[i]) {
			pr_info("freeing v=%p p=0x%lx size=0x%lx\n", _dma_vbase[0], (unsigned long) _dma_pbase[i], (unsigned long)_dma_mem_size);
			dma_free_coherent(0, _dma_mem_size, _dma_vbase[i], _dma_pbase[i]);
		}
	}
	for (i = 0; i < SOC_PCIE_EP_MAX_IRQ; i++) {
		if (cookies[i].curr_irqs)
			free_irq(cookies[i].curr_irqs, &cookies[i]);
	}
	if (ob_addr)
		iounmap((void *)ob_addr);

	if (iproc_pcie_base)
		iounmap(iproc_pcie_base);
}

module_init(pcie_ep_init);
module_exit(pcie_ep_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("PCIE ep test driver for KT2");
