// SPDX-License-Identifier: (GPL-2.0 OR MIT)
//
// Microsemi SoCs FDMA driver
//
// Copyright (c) 2018 Microsemi

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/slab.h>

#include "dmaengine.h"

#define MSCC_FDMA_DCB_LLP(x)			((x) * 4 + 0x0)

#define MSCC_FDMA_DCB_DATAP(x)			((x) * 4 + 0x28)

#define MSCC_FDMA_DCB_DATAL(x)			((x) * 4 + 0x50)

#define MSCC_FDMA_DCB_DATAL_TOKEN		BIT(17)
#define MSCC_FDMA_DCB_DATAL_DATAL(x)		((x) & GENMASK(15, 0))
#define MSCC_FDMA_DCB_DATAL_DATAL_M		GENMASK(15, 0)

#define MSCC_FDMA_DCB_STAT(x)			((x) * 4 + 0x78)

#define MSCC_FDMA_DCB_STAT_BLOCKO(x)		(((x) << 20) & GENMASK(31, 20))
#define MSCC_FDMA_DCB_STAT_BLOCKO_M		GENMASK(31, 20)
#define MSCC_FDMA_DCB_STAT_BLOCKO_X(x)		(((x) & GENMASK(31, 20)) >> 20)
#define MSCC_FDMA_DCB_STAT_PD			BIT(19)
#define MSCC_FDMA_DCB_STAT_ABORT		BIT(18)
#define MSCC_FDMA_DCB_STAT_EOF			BIT(17)
#define MSCC_FDMA_DCB_STAT_SOF			BIT(16)
#define MSCC_FDMA_DCB_STAT_BLOCKL(x)		((x) & GENMASK(15, 0))
#define MSCC_FDMA_DCB_STAT_BLOCKL_M		GENMASK(15, 0)

#define MSCC_FDMA_DCB_LLP_PREV(x)		((x) * 4 + 0xa0)

#define MSCC_FDMA_CH_STAT			0xc8

#define MSCC_FDMA_CH_SAFE			0xcc

#define MSCC_FDMA_CH_ACTIVATE			0xd0

#define MSCC_FDMA_CH_DISABLE			0xd4

#define MSCC_FDMA_CH_FORCEDIS			0xd8

#define MSCC_FDMA_CH_CNT(x)			((x) * 4 + 0xdc)

#define MSCC_FDMA_CH_CNT_CH_CNT_FRM(x)		(((x) << 16) & GENMASK(31, 16))
#define MSCC_FDMA_CH_CNT_CH_CNT_FRM_M		GENMASK(31, 16)
#define MSCC_FDMA_CH_CNT_CH_CNT_FRM_X(x)	(((x) & GENMASK(31, 16)) >> 16)
#define MSCC_FDMA_CH_CNT_CH_CNT_DCB(x)		(((x) << 8) & GENMASK(15, 8))
#define MSCC_FDMA_CH_CNT_CH_CNT_DCB_M		GENMASK(15, 8)
#define MSCC_FDMA_CH_CNT_CH_CNT_DCB_X(x)	(((x) & GENMASK(15, 8)) >> 8)
#define MSCC_FDMA_CH_CNT_CH_CNT_SIG(x)		((x) & GENMASK(7, 0))
#define MSCC_FDMA_CH_CNT_CH_CNT_SIG_M		GENMASK(7, 0)

#define MSCC_FDMA_CH_INJ_CNT(x)			((x) * 4 + 0x104)

#define MSCC_FDMA_CH_INJ_TICK_RLD(x)		((x) * 4 + 0x124)

#define MSCC_FDMA_CH_INJ_TICK_CNT(x)		((x) * 4 + 0x144)

#define MSCC_FDMA_EVT_ERR			0x164

#define MSCC_FDMA_EVT_ERR_CODE			0x168

#define MSCC_FDMA_INTR_LLP			0x16c

#define MSCC_FDMA_INTR_LLP_ENA			0x170

#define MSCC_FDMA_INTR_FRM			0x174

#define MSCC_FDMA_INTR_FRM_ENA			0x178

#define MSCC_FDMA_INTR_SIG			0x17c

#define MSCC_FDMA_INTR_SIG_ENA			0x180

#define MSCC_FDMA_INTR_ENA			0x184

#define MSCC_FDMA_INTR_IDENT			0x188

#define MSCC_FDMA_CH_CFG(x)			((x) * 4 + 0x18c)

#define MSCC_FDMA_CH_CFG_STAT_IN_DATA_ENA	BIT(7)
#define MSCC_FDMA_CH_CFG_CH_INJ_GRP		BIT(6)
#define MSCC_FDMA_CH_CFG_CH_PRIO(x)		(((x) << 2) & GENMASK(5, 2))
#define MSCC_FDMA_CH_CFG_CH_PRIO_M		GENMASK(5, 2)
#define MSCC_FDMA_CH_CFG_CH_PRIO_X(x)		(((x) & GENMASK(5, 2)) >> 2)
#define MSCC_FDMA_CH_CFG_DONE_STOP_ENA		BIT(1)
#define MSCC_FDMA_CH_CFG_DONEEOF_STOP_ENA	BIT(0)

#define MSCC_FDMA_GCFG				0x1b4

#define MSCC_FDMA_GCFG_FRM_AT_OFF		BIT(12)
#define MSCC_FDMA_GCFG_INJ_RF_WM(x)		(((x) << 7) & GENMASK(11, 7))
#define MSCC_FDMA_GCFG_INJ_RF_WM_M		GENMASK(11, 7)
#define MSCC_FDMA_GCFG_INJ_RF_WM_X(x)		(((x) & GENMASK(11, 7)) >> 7)
#define MSCC_FDMA_GCFG_XTR_RF_WM(x)		(((x) << 3) & GENMASK(6, 3))
#define MSCC_FDMA_GCFG_XTR_RF_WM_M		GENMASK(6, 3)
#define MSCC_FDMA_GCFG_XTR_RF_WM_X(x)		(((x) & GENMASK(6, 3)) >> 3)
#define MSCC_FDMA_GCFG_XTR_AVAIL_EXT_DIS	BIT(2)
#define MSCC_FDMA_GCFG_XTR_PRIO_BP_DIS		BIT(1)
#define MSCC_FDMA_GCFG_PD_IGNORE		BIT(0)

#define MSCC_FDMA_GSTAT				0x1b8

#define MSCC_FDMA_GSTAT_INJ_RF_HIGH(x)		(((x) << 5) & GENMASK(10, 5))
#define MSCC_FDMA_GSTAT_INJ_RF_HIGH_M		GENMASK(10, 5)
#define MSCC_FDMA_GSTAT_INJ_RF_HIGH_X(x)	(((x) & GENMASK(10, 5)) >> 5)
#define MSCC_FDMA_GSTAT_XTR_RF_HIGH(x)		((x) & GENMASK(4, 0))
#define MSCC_FDMA_GSTAT_XTR_RF_HIGH_M		GENMASK(4, 0)

#define MSCC_FDMA_IDLECNT			0x1bc

#define MSCC_FDMA_CONST				0x1c0

#define MSCC_FDMA_CONST_CH_INJ_CNT(x)		(((x) << 8) & GENMASK(15, 8))
#define MSCC_FDMA_CONST_CH_INJ_CNT_M		GENMASK(15, 8)
#define MSCC_FDMA_CONST_CH_INJ_CNT_X(x)		(((x) & GENMASK(15, 8)) >> 8)
#define MSCC_FDMA_CONST_CH_XTR_CNT(x)		((x) & GENMASK(7, 0))
#define MSCC_FDMA_CONST_CH_XTR_CNT_M		GENMASK(7, 0)

struct mscc_fdma_dcb_hw_v2 {
	u32 llp;
	u32 datap;
	u32 datal;
	u32 stat;
};

struct mscc_fdma_dcb {
	struct  mscc_fdma_dcb_hw_v2	hw;
	struct dma_async_tx_descriptor	txd;
	struct list_head		node;
};

struct mscc_fdma {
	struct dma_device	dma;
	void __iomem		*base;
	spinlock_t		lock;
	struct dma_pool		*dcb_pool;
	int			irq;

	unsigned int		nr_pchans;
	struct dma_chan		chans[0];
};

struct dma_async_tx_descriptor	*gtxd[4];

static void dma_writel(struct mscc_fdma *fdma, u32 reg, u32 data)
{
	writel(data, fdma->base + reg);
}

static u32 dma_readl(struct mscc_fdma *fdma, u32 reg)
{
	return readl(fdma->base + reg);
}

static inline struct mscc_fdma *to_mscc_fdma(struct dma_device *dd)
{
	return container_of(dd, struct mscc_fdma, dma);
}

static inline struct mscc_fdma_dcb *txd_to_dcb(struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct mscc_fdma_dcb, txd);
}

static irqreturn_t mscc_fdma_interrupt(int irq, void *dev_id)
{
	struct mscc_fdma *fdma = dev_id;
	u32 ident, llp, frm, chans;

	spin_lock(&fdma->lock); // ABE unecessary locking a a reminder

	ident = dma_readl(fdma, MSCC_FDMA_INTR_IDENT);
	frm = dma_readl(fdma, MSCC_FDMA_INTR_FRM);
	llp = dma_readl(fdma, MSCC_FDMA_INTR_LLP);

	dma_writel(fdma, MSCC_FDMA_INTR_LLP, llp & ident);
	dma_writel(fdma, MSCC_FDMA_INTR_FRM, frm & ident);

	spin_unlock(&fdma->lock);

	chans = ident & (frm | llp);

	while (chans) {
		u32 chan = __fls(chans);
		struct mscc_fdma_dcb *dcb = txd_to_dcb(gtxd[chan]);

		if (dcb->hw.stat & MSCC_FDMA_DCB_STAT_EOF) {
			dma_cookie_complete(gtxd[chan]);
			dmaengine_desc_get_callback_invoke(gtxd[chan], NULL);
		}

		dma_pool_free(fdma->dcb_pool, dcb, dcb->txd.phys);
		chans &= ~(BIT(chan));
	}

	return IRQ_HANDLED;
}

static inline void mscc_fdma_free(struct mscc_fdma *fdma)
{
	pr_err("ABE: %s +%d %s\n", __FILE__, __LINE__, __func__);
}

static enum dma_status
mscc_fdma_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
		    struct dma_tx_state *txstate)
{
	pr_err("ABE: %s +%d %s\n", __FILE__, __LINE__, __func__);
	return DMA_COMPLETE;
}

static void mscc_fdma_issue_pending(struct dma_chan *chan)
{
	return;
}

static int mscc_fdma_terminate_all(struct dma_chan *chan)
{
	pr_err("ABE: %s +%d %s\n", __FILE__, __LINE__, __func__);
	return 0;
}

static int mscc_fdma_alloc_chan_resources(struct dma_chan *chan)
{

	pr_err("ABE: %s +%d %s\n", __FILE__, __LINE__, __func__);
	dma_cookie_init(chan);

	return 0;
}

static void mscc_fdma_free_chan_resources(struct dma_chan *chan)
{
	pr_err("ABE: %s +%d %s\n", __FILE__, __LINE__, __func__);
	return;
}

static dma_cookie_t mscc_fdma_tx_submit(struct dma_async_tx_descriptor *txd)
{
	struct mscc_fdma *fdma = to_mscc_fdma(txd->chan->device);
	dma_cookie_t cookie;

	if (!(dma_readl(fdma, MSCC_FDMA_CH_SAFE) & BIT(txd->chan->chan_id))) {
		pr_err("dropped"); //ABE
		return -EBUSY;
	}

	cookie = dma_cookie_assign(txd);

	gtxd[txd->chan->chan_id] = txd;

	dma_writel(fdma, MSCC_FDMA_DCB_LLP(txd->chan->chan_id), txd->phys);
	dma_writel(fdma, MSCC_FDMA_INTR_LLP, BIT(txd->chan->chan_id));
#if 0
	dma_writel(fdma, MSCC_FDMA_INTR_LLP_ENA, BIT(txd->chan->chan_id) |
		   dma_readl(fdma, MSCC_FDMA_INTR_LLP_ENA));
#endif
	dma_writel(fdma, MSCC_FDMA_INTR_FRM, BIT(txd->chan->chan_id));
	dma_writel(fdma, MSCC_FDMA_INTR_FRM_ENA, BIT(txd->chan->chan_id) |
		   dma_readl(fdma, MSCC_FDMA_INTR_FRM_ENA));
	dma_writel(fdma, MSCC_FDMA_INTR_ENA, BIT(txd->chan->chan_id) |
		   dma_readl(fdma, MSCC_FDMA_INTR_ENA));
	dma_writel(fdma, MSCC_FDMA_CH_ACTIVATE, BIT(txd->chan->chan_id) |
		   dma_readl(fdma, MSCC_FDMA_CH_ACTIVATE));

	return cookie;
}

static struct dma_async_tx_descriptor *
mscc_fdma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			unsigned int sg_len,
			enum dma_transfer_direction direction,
			unsigned long flags, void *context)
{
	struct mscc_fdma *fdma = to_mscc_fdma(chan->device);
	struct mscc_fdma_dcb *prev = NULL, *first = NULL;
	struct scatterlist *sg;
	int i;

	if (!sgl)
		return NULL;

	if (!is_slave_direction(direction)) {
		dev_err(&chan->dev->device, "invalid DMA direction\n");
		return NULL;
	}

	dev_dbg(&chan->dev->device, "%s: sg_len=%d, dir=%s, flags=0x%lx\n",
		__func__, sg_len,
		direction == DMA_MEM_TO_DEV ? "to device" : "from device",
		flags);

	for_each_sg(sgl, sg, sg_len, i) {
		u32 len, addr, o;
		dma_addr_t phys;
		struct mscc_fdma_dcb *dcb;

		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		o = addr & 0x3;

		dcb = dma_pool_zalloc(fdma->dcb_pool, GFP_KERNEL, &phys);
		if (!dcb)
			return NULL; //TODO rollback previous allocations

		dma_async_tx_descriptor_init(&dcb->txd, chan);
		dcb->txd.tx_submit = mscc_fdma_tx_submit;
		dcb->txd.phys = phys;

		dcb->hw.datap = addr & ~0x3;
		dcb->hw.datal = (len + o);
		dcb->hw.stat = MSCC_FDMA_DCB_STAT_BLOCKL(len)
			       | MSCC_FDMA_DCB_STAT_BLOCKO(o);

		if (!first) {
			first = dcb;
			if (direction == DMA_MEM_TO_DEV)
				dcb->hw.stat |= MSCC_FDMA_DCB_STAT_SOF;
		}

		if (prev)
			prev->hw.llp = phys;

		prev = dcb;
	}

	if (direction == DMA_MEM_TO_DEV)
		prev->hw.stat |= MSCC_FDMA_DCB_STAT_EOF;

	return &first->txd;
}

static int mscc_fdma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mscc_fdma *fdma;
	struct resource *res;
	int ret, i, nr_channels;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	ret = of_property_read_u32(np, "dma-channels", &nr_channels);
	if (ret) {
		dev_err(&pdev->dev, "can't get dma-channels\n");
		return ret;
	}

	dev_info(&pdev->dev, "%d dma-channels\n", nr_channels);

	fdma = devm_kzalloc(&pdev->dev, sizeof(*fdma) + nr_channels *
			    sizeof(struct dma_chan), GFP_KERNEL);
	if (!fdma)
		return -ENOMEM;

	fdma->nr_pchans = nr_channels;

	fdma->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fdma->base))
		return PTR_ERR(fdma->base);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	platform_set_drvdata(pdev, fdma);
	spin_lock_init(&fdma->lock);

	dma_cap_set(DMA_SLAVE, fdma->dma.cap_mask);

	fdma->dma.dev = &pdev->dev;
	fdma->dma.device_alloc_chan_resources = mscc_fdma_alloc_chan_resources;
	fdma->dma.device_free_chan_resources = mscc_fdma_free_chan_resources;
	fdma->dma.device_prep_slave_sg = mscc_fdma_prep_slave_sg;
	fdma->dma.device_tx_status = mscc_fdma_tx_status;
	fdma->dma.device_issue_pending = mscc_fdma_issue_pending;
	fdma->dma.device_terminate_all = mscc_fdma_terminate_all;
	fdma->dma.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	fdma->dma.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	fdma->dma.directions = BIT(DMA_MEM_TO_MEM);
	fdma->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	INIT_LIST_HEAD(&fdma->dma.channels);

	dma_writel(fdma, MSCC_FDMA_INTR_ENA, 0);

	fdma->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, fdma->irq, mscc_fdma_interrupt, 0,
			       dev_name(&pdev->dev), fdma);
	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		return ret;
	}

	INIT_LIST_HEAD(&fdma->dma.channels);
	for (i = 0; i < nr_channels; i++) {
		fdma->chans[i].device = &fdma->dma;
		list_add_tail(&fdma->chans[i].device_node,
			      &fdma->dma.channels);
	}

	/* Create a pool of consistent memory blocks for hardware descriptors */
	fdma->dcb_pool = dmam_pool_create(dev_name(fdma->dma.dev), fdma->dma.dev,
					  sizeof(struct mscc_fdma_dcb),
					  __alignof__(struct mscc_fdma_dcb),
					  0);
	if (!fdma->dcb_pool) {
		dev_err(&pdev->dev, "unable to allocate DMA descriptor pool\n");
		return -ENOMEM;
	}

	ret = dma_async_device_register(&fdma->dma);
	if (ret) {
		dev_err(&pdev->dev, "failed to register DMA engine device\n");
		return ret;
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 of_dma_xlate_by_chan_id, fdma);
	if (ret) {
		dev_err(&pdev->dev, "could not register of dma controller\n");
		dma_async_device_unregister(&fdma->dma);
		return ret;
	}

	return 0;
}

static int mscc_fdma_remove(struct platform_device *pdev)
{
	struct mscc_fdma *fdma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&fdma->dma);

	/* Make sure we won't have any further interrupts */
	devm_free_irq(fdma->dma.dev, fdma->irq, fdma);

	mscc_fdma_free(fdma);

	return 0;
}

static const struct of_device_id mscc_fdma_match[] = {
	{ .compatible = "mscc,ocelot-fdma", },
	{ .compatible = "mscc,jaguar2-fdma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mscc_fdma_match);

static struct platform_driver mscc_fdma_driver = {
	.probe	= mscc_fdma_probe,
	.remove	= mscc_fdma_remove,
	.driver = {
		.name = "mscc-fdma",
		.of_match_table = of_match_ptr(mscc_fdma_match),
	},
};

static int mscc_fdma_init(void)
{
	return platform_driver_register(&mscc_fdma_driver);
}
subsys_initcall(mscc_fdma_init);

static void __exit mscc_fdma_exit(void)
{
	platform_driver_unregister(&mscc_fdma_driver);
}
module_exit(mscc_fdma_exit);

MODULE_AUTHOR("Alexandre Belloni <alexandre.belloni@bootlin.com>");
MODULE_DESCRIPTION("Microsemi FDMA driver");
MODULE_LICENSE("Dual MIT/GPL");
