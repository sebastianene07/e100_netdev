/*
 * e100-ix.c, driver implementation
 *
 * Sebastian Ene <sebastian.ene07@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/irqreturn.h>
#include <linux/netdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/pci.h>

#include "e100-ix.h"

#define LOG_DBG_ETH100(msg, ...)	pr_info ("["DRIVER_NAME"] "msg, ##__VA_ARGS__)

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
}

struct net_device_ops e100_netdev_ops = {
	.ndo_open = e100_ndo_open,
	.ndo_stop = e100_ndo_stop,
	.ndo_start_xmit = e100_ndo_start_xmit
};

static int e100_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	LOG_DBG_ETH100("probe");

	/* TODO 4: allocate netdevice, may use alloc_etherdev
	 *
	 * .. set proper name, irq, netdev_ops
	 * .. set mac address (may use eth_hw_addr_random)
	 */
	/* TODO 4: get netdevice private data using netdev_priv */
	/* TODO 2: hide e100_priv_data into pdev using dev_set_drvdata */

	/* TODO 2: initialize PCI device: use pci_enable_device */
	/* TODO 2: reserve PCI I/O and memory resources: use pci_request_regions */
	/* TODO 2: we will use BAR 1, use pci_resource_flags to check for BAR 1*/
	/* TODO 2: Check if device supports 32-bit DMA, use pci_set_dma_mask */
	/* TODO 2: map Control Status Register into our address space, use pci_iomap */
	/* TODO 2: enable DMA by calling pci_set master */
	/* TODO 4: register netdevice with the networking subsystem */
	return 0;
}

static void e100_remove(struct pci_dev *pdev)
{
	struct e100_priv_data *data;

	/* TODO 2: restore e100_priv_data from pdev using dev_get_drvdata */
	/* TODO 4: unregister netdevice from the networking subsystem */
	/* TODO 2: PCI cleanup
	 * 	* unmap CSR
	 * 	* release PCI regions
	 * 	* disable pci device
	 */
	/* TODO 4: free netdevice */
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
