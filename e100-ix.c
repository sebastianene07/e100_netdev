/*
 * e100-ix.c, driver implementation
 *
 * Sebastian Ene <sebastian.ene07@gmail.com>
 */

#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/netdevice.h>
#include <linux/pci.h>

#include "e100-ix.h"

#define LOG_DBG_ETH100(msg, ...)	pr_info 					\
	("["DRIVER_NAME"] "msg"\n", ##__VA_ARGS__)

/*
 * e100 private data
 *
 * @pdev   - PCI device
 * @netdev - network device
 */
struct e100_priv_data {
	struct pci_dev *pdev;
	struct net_device *netdev;

	u8 __iomem *hw_addr;
	struct csr __iomem *csr;
	struct dma_pool *cbs_pool;

	spinlock_t cmd_lock;
	spinlock_t xmit_lock;
	u8 cmd;
	u8 is_xmit_started;

	struct cb *tx_cbs;
	dma_addr_t tx_cbs_dma_addr;
	struct cb *current_tx_cbs;
};

/* Private Function delcaration */
static int e100_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void e100_remove(struct pci_dev *pdev);
static int e100_exec_cmd(struct e100_priv_data *data, u8 cmd,
	dma_addr_t dma_addr);
static void e100_flush_cmd(struct e100_priv_data *data);

/* PCI driver stuctire for Intel E100 Network board */
static const struct pci_device_id e100_pci_driver_ids[] = {
	{ E100_VENDOR, E100_DEVICE, PCI_ANY_ID, PCI_ANY_ID,},
	{ 0},
};

static struct pci_driver et100_pci_driver = {
	.name 		= DRIVER_NAME,
	.id_table	= e100_pci_driver_ids,
	.probe    = e100_probe,
	.remove   = e100_remove,
};

static irqreturn_t e100_intr(int irq, void *private_data)
{
	struct e100_priv_data *data;

	data = (struct e100_priv_data *)private_data;

	/* TODO 6: read STAT/ACK byte from CSR */
	/* TODO 6: return IRQ_NONE if interrupt is not for this device */
	/* TODO 6: handle Frame Reception interrupt
	 * while receving frames
	 * 	allocate skb
	 * 	copy data from Receive Frame Descriptor to skb
	 * 	free current RFD
	 * 	resume receive unit
	 *      push skb up to network stack using netif_rx
	 */

	/* ACK all interrupts */

	return IRQ_HANDLED;
}

static void e100_flush_cmd(struct e100_priv_data *data)
{
	(void)ioread8(&data->csr->scb.status);
}

static int e100_exec_cmd(struct e100_priv_data *data, u8 cmd,
	dma_addr_t dma_addr)
{
	unsigned long flags;
	int times, ret = 0;

	spin_lock_irqsave(&data->cmd_lock, flags);

	/* Previous command is accepted when SCB clears */
	for (times = 0; times < E100_WAIT_SCB_TIMEOUT; times++) {
		if (!ioread8(&data->csr->scb.cmd_lo)) {
			break;
		}

		cpu_relax();
	}

	if (times == E100_WAIT_SCB_TIMEOUT) {
		ret = -ETIMEDOUT;
		goto err_cmd_timeout;
	}

	if (cmd != CU_RESUME)
		iowrite32(dma_addr, &data->csr->scb.gen_ptr);

	iowrite8(cmd, &data->csr->scb.cmd_lo);

err_cmd_timeout:
	spin_unlock_irqrestore(&data->cmd_lock, flags);
	return ret;
}

static int e100_ndo_open(struct net_device *netdev)
{
	struct e100_priv_data *data;
	int ret = 0, i;
	struct cb *cbs;

	data = netdev_priv(netdev);
	LOG_DBG_ETH100("open");

	/* Create TX ring buffer to store CB_RING_LEN Command Blocks */
	data->cbs_pool = dma_pool_create(netdev->name,
		&data->pdev->dev,
		CB_RING_LEN * sizeof(struct cb),
		sizeof(u32),
		0);
	if (!data->cbs_pool) {
		LOG_DBG_ETH100("error DMA pool create");
		return -ENOMEM;
	}

	data->tx_cbs = dma_pool_zalloc(data->cbs_pool, GFP_KERNEL,
		&data->tx_cbs_dma_addr);
	if (!data->tx_cbs) {
		LOG_DBG_ETH100("error alloc DMA pool");
		ret = -ENOMEM;
		goto err_with_dma_pool;
	}

	for (i = 0; i < CB_RING_LEN; ++i) {
		struct cb *cbs = &data->tx_cbs[i];

		cbs->next = i < CB_RING_LEN ? &data->tx_cbs[i + 1] : data->tx_cbs;
		cbs->prev = i == 0 ? &data->tx_cbs[CB_RING_LEN - 1] : &data->tx_cbs[i - 1];
		cbs->dma_addr = data->tx_cbs_dma_addr + i * sizeof(struct cb);
		cbs->link = cpu_to_le32(((i + 1) % CB_RING_LEN) * sizeof(struct cb) +
			data->tx_cbs_dma_addr);
	}

	/* Set the first command to ring buffer to set MAC */
	cbs = &data->tx_cbs[0];
	cbs->command.cmd = cpu_to_le16(CB_SET_INDIVIDUAL_ADDR);
	memcpy(cbs->u.ias, netdev->dev_addr, ETH_ALEN);
	data->cmd = cbs->command.cmd;
	e100_exec_cmd(data, data->cmd, cbs->dma_addr);

	/* TODO 6: Create RX ring buffer to store RFD_RING_LEN */
	/* TODO 6: register interrupt handler */
	/* TODO 6: enable interrupts */

	/* Start command unit */
	cbs = &data->tx_cbs[1];
	cbs->command.cmd = CU_START;
	data->cmd = CU_START;
	data->is_xmit_started = 1;
	e100_exec_cmd(data, data->cmd, cbs->dma_addr);
	data->current_tx_cbs = cbs;

	/* TODO 6: start receive unit */

	/* Allow transmit by calling netif_start_queue */
	netif_start_queue(netdev);
	return ret;

err_with_dma_pool:
	dma_pool_destroy(data->cbs_pool);
	return ret;
}

static int e100_ndo_stop(struct net_device *netdev)
{
	struct e100_priv_data *data;

	data = netdev_priv(netdev);
	LOG_DBG_ETH100("stop");

	/* Stop transmit by calling netif_stop_queue */
	netif_stop_queue(netdev);

	/* TODO 6: disable network interrupts and free irq */

	/* Deallocate TX ring */
	dma_pool_destroy(data->cbs_pool);

	/* TODO 6: deallocate RX ring */
	return 0;
}

/*
 * e100_ndo_start_xmit - transmit skb over netdev
 *
 */
static netdev_tx_t e100_ndo_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct e100_priv_data *data;
	struct cb *cur_cb;
	int is_xmit_started;

	data = netdev_priv(netdev);
	LOG_DBG_ETH100("transmit skb");

	/* TODO 5: reclaim all buffers which were transmitted */

	spin_lock(&data->xmit_lock);

	/* Create new transmit command for current skb */
	cur_cb = data->current_tx_cbs;
	cur_cb->prev->command.suspend = 0;
	cur_cb->command.cmd = CB_TRANSMIT;
	cur_cb->command.suspend = 1;
	is_xmit_started = data->is_xmit_started;

	data->current_tx_cbs = data->current_tx_cbs->next;

	spin_unlock(&data->xmit_lock);

	memcpy(cur_cb->data, skb->data, skb->len);
	cur_cb->u.tcb.tcb_byte_count = skb->len;

	/* resume command unit */
	if (is_xmit_started == 0) {
		data->is_xmit_started = 1;
		data->cmd = CU_START;
		e100_exec_cmd(data, data->cmd, cur_cb->dma_addr);
	} else {
		data->cmd = CU_RESUME;
		e100_exec_cmd(data, data->cmd, cur_cb->dma_addr);
	}

	return NETDEV_TX_OK;
}

struct net_device_ops e100_netdev_ops = {
	.ndo_open = e100_ndo_open,
	.ndo_stop = e100_ndo_stop,
	.ndo_start_xmit = e100_ndo_start_xmit
};

static int e100_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret = 0;
	struct e100_priv_data *e100_priv;
	struct net_device *netdev;

	LOG_DBG_ETH100("probe");

	if (!(netdev = alloc_etherdev(sizeof(struct e100_priv_data)))) {
		LOG_DBG_ETH100("no memory");
		return -ENOMEM;
	}

	netdev->netdev_ops = &e100_netdev_ops;
	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);
	eth_hw_addr_random(netdev);

	/* Get netdevice private data using netdev_priv */
	e100_priv = netdev_priv(netdev);
	e100_priv->netdev = netdev;
	e100_priv->pdev   = pdev;

	dev_set_drvdata(&pdev->dev, e100_priv);
	ret = pci_enable_device(pdev);
	if (ret != 0) {
		LOG_DBG_ETH100("pci enable failed %d", ret);
		goto err_with_alloc;
	}

	/* Reserve PCI I/O and memory resources: */
	if (pci_request_regions(pdev, DRIVER_NAME)) {
		LOG_DBG_ETH100("pci request I/O region");
		ret = -EBUSY;
		goto err_with_enabled_pci;
	}

	/* we will use BAR 1, use pci_resource_flags to check for BAR 1*/
	if (pci_resource_flags(pdev, 1) & IORESOURCE_IO) {
		LOG_DBG_ETH100("pci I/O mapped BAR)");
	}

	/* Check if device supports 32-bit DMA, use pci_set_dma_mask */
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		LOG_DBG_ETH100("%d No usable DMA configuration, aborting", ret);
		goto err_with_req_pci_regions;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	/* map Control Status Register into our address space, use pci_iomap */
	e100_priv->csr = pci_iomap(pdev, 1, sizeof(struct csr));
	LOG_DBG_ETH100("pci BAR1 I/O CSR mapped region 0%x", (unsigned int )e100_priv->csr);

	/* enable DMA by calling pci_set master */
	pci_set_master(pdev);

	/* Register netdevice with the networking subsystem */
	strcpy(netdev->name, "eth%d");
	ret = register_netdev(netdev);
	if (ret) {
		LOG_DBG_ETH100("error %d cannot register net device, aborting", ret);
		goto err_with_pci_dma;
	}

	spin_lock_init(&e100_priv->cmd_lock);
	spin_lock_init(&e100_priv->xmit_lock);
	return 0;

err_with_pci_dma:
	pci_clear_master(pdev);
err_with_pci_iomap:
	pci_iounmap(pdev, e100_priv->csr);
err_with_req_pci_regions:
	pci_release_regions(pdev);
err_with_enabled_pci:
	pci_disable_device(pdev);
err_with_alloc:
	free_netdev(netdev);

	return ret;
}

static void e100_remove(struct pci_dev *pdev)
{
	struct e100_priv_data *data;
	struct net_device *netdev;

	pci_clear_master(pdev);
	data = dev_get_drvdata(&pdev->dev);
	netdev = data->netdev;

	/* unregister netdevice from the networking subsystem */
	unregister_netdev(netdev);

	pci_iounmap(pdev, data->csr);
	pci_release_regions(pdev);
	pci_disable_device(pdev);

	/* free netdevice */
	free_netdev(netdev);
};

static int e100_init(void)
{
	int ret = 0;
	if ((ret = pci_register_driver(&et100_pci_driver)) < 0) {
		pr_info("init failed to register %d", ret);
	}
	LOG_DBG_ETH100("init %d", ret);
	return ret;
}

static void e100_exit(void)
{
	pci_unregister_driver(&et100_pci_driver);
	LOG_DBG_ETH100("exit");
}

module_init(e100_init);
module_exit(e100_exit);

MODULE_DESCRIPTION("e100 network driver");
MODULE_AUTHOR("Sebastian Ene <sebastian.ene07@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, e100_pci_driver_ids);
