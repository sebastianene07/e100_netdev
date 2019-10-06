# e100_netdev

Linux network driver for Intel 82559ER controller

## Taking with PCI devices

PCI devices are accessed on the bus using a configuration space which is mapped in 
the main memory or using port adressing. This configuration space is mapped during the boot-up process
or when the driver requests this (usually during probing). 
A device can be accessed on the bus using the following scheme:

```
xxxxxxxx --- yyyyy ---- zzz
(8 bit bus) (5 bit dev) (3 bit function)
```
Extended configuration space uses 4096 bytes instead of 256 and describes
more bits for accessesing the device.

The driver has to program a base adddress register (BAR) to inform the device about
it's address mapping. A device can be uniquely identified by the Device ID (DID) and 
Vendor ID (VID) registers.

## Linux API for PCI device access

PCI drivers need the following steps for device initialization:

1. Enable the device
2. Request MMIO/IOP resources
3. Set the DMA mask size (for both coherent and streaming DMA)
4. Allocate and initialize shared control data (pci_allocate_coherent())
5. Access device configuration space (if needed)
6. Register IRQ handler (request_irq())
7. Initialize non-PCI (i.e. LAN/SCSI/etc parts of the chip)
8. Enable DMA/processing engines.

```
pci_register_driver() /* discover PCI devices in a system */
pci_enable_device()   /* wake up the PCI device */
pci_request_region()  /* Reserve MMIO/IOP resources */

```


This is the IXIA challenge assignment : https://ocw.cs.pub.ro/courses/so2/teme/tema6
