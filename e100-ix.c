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

#define LOG_DBG_ETH100(msg, ...)	pr_info ("["DRIVER_NAME"] "msg, ##__VA_ARGS__)

/*
 * e100 private data
 *
 * @pdev   - PCI device
 * @netdev - network device
 */
struct e100_priv_data {
	struct pci_dev *pdev;
	struct net_device *netdev;

	/* TODO 3: device control and configuration
	 * e.g:
	 * 	- CSR register address
	 */
	u8 __iomem *hw_addr;
	struct csr __iomem *csr;
};

/* Private Function delcaration */
static int e100_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void e100_remove(struct pci_dev *pdev);

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

static int e100_ndo_open(struct net_device *netdev)
{
	struct e100_priv_data *data;

	data = netdev_priv(netdev);

	/* TODO 5: Create TX ring buffer to store CB_RING_LEN Command Blocks */
	/* TODO 5: first command to ring buffer to set MAC */
	/* TODO 6: Create RX ring buffer to store RFD_RING_LEN */
	/* TODO 6: register interrupt handler */
	/* TODO 6: enable interrupts */
	/* TODO 5: start command unit */
	/* TODO 6: start receive unit */
	/* TODO 5: allow transmit by calling netif_start_queue */
	return 0;
}

static int e100_ndo_stop(struct net_device *netdev)
{
	struct e100_priv_data *data;

	data = netdev_priv(netdev);

	/* TODO 5: stop transmit by calling netif_stop_queue */
	/* TODO 6: disable network interrupts and free irq */
	/* TODO 5: deallocate TX ring */
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

	data = netdev_priv(netdev);

	/* TODO 5: reclaim all buffers which were transmitted */
	/* TODO 5: create new transmit command for current skb */

	/* TODO 5: resume command unit */
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

	/* TODO 4: allocate netdevice, may use alloc_etherdev
	 *
	 * .. set proper name, irq, netdev_ops
	 * .. set mac address (may use eth_hw_addr_random)
	 */
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
		e100_priv->hw_addr = (unsigned char *)pci_resource_start(pdev, 1);
		LOG_DBG_ETH100("pci BAR1 I/O mapped region 0%x", (unsigned int)e100_priv->hw_addr);
	}

	/* Check if device supports 32-bit DMA, use pci_set_dma_mask */
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		LOG_DBG_ETH100("%d No usable DMA configuration, aborting\n", ret);
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
		goto err_with_pci_iomap;
	}

	return 0;

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
		pr_info("init failed to register %d\n", ret);
	}
	LOG_DBG_ETH100("init %d\n", ret);
	return ret;
}

static void e100_exit(void)
{
	pci_unregister_driver(&et100_pci_driver);
	LOG_DBG_ETH100("exit\n");
}

module_init(e100_init);
module_exit(e100_exit);

MODULE_DESCRIPTION("e100 network driver");
MODULE_AUTHOR("Sebastian Ene <sebastian.ene07@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, e100_pci_driver_ids);
