#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

/**
 *	config_drive_for_dma	-	attempt to activate IDE DMA
 *	@drive: the drive to place in DMA mode
 *
 *	If the drive supports at least mode 2 DMA or UDMA of any kind
 *	then attempt to place it into DMA mode. Drives that are known to
 *	support DMA but predate the DMA properties or that are known
 *	to have DMA handling bugs are also set up appropriately based
 *	on the good/bad drive lists.
 */

int config_drive_for_dma(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u16 *id = drive->id;

	if (drive->media != ide_disk) {
		if (hwif->host_flags & IDE_HFLAG_NO_ATAPI_DMA)
			return 0;
	}

	/*
	 * Enable DMA on any drive that has
	 * UltraDMA (mode 0/1/2/3/4/5/6) enabled
	 */
	if ((id[ATA_ID_FIELD_VALID] & 4) &&
	    ((id[ATA_ID_UDMA_MODES] >> 8) & 0x7f))
		return 1;

	/*
	 * Enable DMA on any drive that has mode2 DMA
	 * (multi or single) enabled
	 */
	if (id[ATA_ID_FIELD_VALID] & 2)	/* regular DMA */
		if ((id[ATA_ID_MWDMA_MODES] & 0x404) == 0x404 ||
		    (id[ATA_ID_SWDMA_MODES] & 0x404) == 0x404)
			return 1;

	/* Consult the list of known "good" drives */
	if (ide_dma_good_drive(drive))
		return 1;

	return 0;
}

/**
 *	ide_dma_host_set	-	Enable/disable DMA on a host
 *	@drive: drive to control
 *
 *	Enable/disable DMA on an IDE controller following generic
 *	bus-mastering IDE controller behaviour.
 */

void ide_dma_host_set(ide_drive_t *drive, int on)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 unit = drive->dn & 1;
	u8 dma_stat = hwif->tp_ops->read_sff_dma_status(hwif);

	if (on)
		dma_stat |= (1 << (5 + unit));
	else
		dma_stat &= ~(1 << (5 + unit));

	if (hwif->host_flags & IDE_HFLAG_MMIO)
		writeb(dma_stat,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_STATUS));
	else
		outb(dma_stat, hwif->dma_base + ATA_DMA_STATUS);
}
EXPORT_SYMBOL_GPL(ide_dma_host_set);

/**
 *	ide_build_dmatable	-	build IDE DMA table
 *
 *	ide_build_dmatable() prepares a dma request. We map the command
 *	to get the pci bus addresses of the buffers and then build up
 *	the PRD table that the IDE layer wants to be fed.
 *
 *	Most chipsets correctly interpret a length of 0x0000 as 64KB,
 *	but at least one (e.g. CS5530) misinterprets it as zero (!).
 *	So we break the 64KB entry into two 32KB entries instead.
 *
 *	Returns the number of built PRD entries if all went okay,
 *	returns 0 otherwise.
 *
 *	May also be invoked from trm290.c
 */

int ide_build_dmatable(ide_drive_t *drive, struct request *rq)
{
	ide_hwif_t *hwif = drive->hwif;
	__le32 *table = (__le32 *)hwif->dmatable_cpu;
	unsigned int is_trm290	= (hwif->chipset == ide_trm290) ? 1 : 0;
	unsigned int count = 0;
	int i;
	struct scatterlist *sg;

	hwif->sg_nents = ide_build_sglist(drive, rq);
	if (hwif->sg_nents == 0)
		return 0;

	for_each_sg(hwif->sg_table, sg, hwif->sg_nents, i) {
		u32 cur_addr, cur_len, xcount, bcount;

		cur_addr = sg_dma_address(sg);
		cur_len = sg_dma_len(sg);

		/*
		 * Fill in the dma table, without crossing any 64kB boundaries.
		 * Most hardware requires 16-bit alignment of all blocks,
		 * but the trm290 requires 32-bit alignment.
		 */

		while (cur_len) {
			if (count++ >= PRD_ENTRIES)
				goto use_pio_instead;

			bcount = 0x10000 - (cur_addr & 0xffff);
			if (bcount > cur_len)
				bcount = cur_len;
			*table++ = cpu_to_le32(cur_addr);
			xcount = bcount & 0xffff;
			if (is_trm290)
				xcount = ((xcount >> 2) - 1) << 16;
			else if (xcount == 0x0000) {
				if (count++ >= PRD_ENTRIES)
					goto use_pio_instead;
				*table++ = cpu_to_le32(0x8000);
				*table++ = cpu_to_le32(cur_addr + 0x8000);
				xcount = 0x8000;
			}
			*table++ = cpu_to_le32(xcount);
			cur_addr += bcount;
			cur_len -= bcount;
		}
	}

	if (count) {
		if (!is_trm290)
			*--table |= cpu_to_le32(0x80000000);
		return count;
	}

use_pio_instead:
	printk(KERN_ERR "%s: %s\n", drive->name,
		count ? "DMA table too small" : "empty DMA table?");

	ide_destroy_dmatable(drive);

	return 0; /* revert to PIO for this request */
}
EXPORT_SYMBOL_GPL(ide_build_dmatable);

/**
 *	ide_dma_setup	-	begin a DMA phase
 *	@drive: target device
 *
 *	Build an IDE DMA PRD (IDE speak for scatter gather table)
 *	and then set up the DMA transfer registers for a device
 *	that follows generic IDE PCI DMA behaviour. Controllers can
 *	override this function if they need to
 *
 *	Returns 0 on success. If a PIO fallback is required then 1
 *	is returned.
 */

int ide_dma_setup(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	struct request *rq = hwif->hwgroup->rq;
	unsigned int reading;
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;
	u8 dma_stat;

	if (rq_data_dir(rq))
		reading = 0;
	else
		reading = 1 << 3;

	/* fall back to pio! */
	if (!ide_build_dmatable(drive, rq)) {
		ide_map_sg(drive, rq);
		return 1;
	}

	/* PRD table */
	if (hwif->host_flags & IDE_HFLAG_MMIO)
		writel(hwif->dmatable_dma,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_TABLE_OFS));
	else
		outl(hwif->dmatable_dma, hwif->dma_base + ATA_DMA_TABLE_OFS);

	/* specify r/w */
	if (mmio)
		writeb(reading, (void __iomem *)(hwif->dma_base + ATA_DMA_CMD));
	else
		outb(reading, hwif->dma_base + ATA_DMA_CMD);

	/* read DMA status for INTR & ERROR flags */
	dma_stat = hwif->tp_ops->read_sff_dma_status(hwif);

	/* clear INTR & ERROR flags */
	if (mmio)
		writeb(dma_stat | 6,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_STATUS));
	else
		outb(dma_stat | 6, hwif->dma_base + ATA_DMA_STATUS);

	drive->waiting_for_dma = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(ide_dma_setup);

/**
 *	dma_timer_expiry	-	handle a DMA timeout
 *	@drive: Drive that timed out
 *
 *	An IDE DMA transfer timed out. In the event of an error we ask
 *	the driver to resolve the problem, if a DMA transfer is still
 *	in progress we continue to wait (arguably we need to add a
 *	secondary 'I don't care what the drive thinks' timeout here)
 *	Finally if we have an interrupt we let it complete the I/O.
 *	But only one time - we clear expiry and if it's still not
 *	completed after WAIT_CMD, we error and retry in PIO.
 *	This can occur if an interrupt is lost or due to hang or bugs.
 */

static int dma_timer_expiry(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 dma_stat = hwif->tp_ops->read_sff_dma_status(hwif);

	printk(KERN_WARNING "%s: %s: DMA status (0x%02x)\n",
		drive->name, __func__, dma_stat);

	if ((dma_stat & 0x18) == 0x18)	/* BUSY Stupid Early Timer !! */
		return WAIT_CMD;

	hwif->hwgroup->expiry = NULL;	/* one free ride for now */

	/* 1 dmaing, 2 error, 4 intr */
	if (dma_stat & 2)	/* ERROR */
		return -1;

	if (dma_stat & 1)	/* DMAing */
		return WAIT_CMD;

	if (dma_stat & 4)	/* Got an Interrupt */
		return WAIT_CMD;

	return 0;	/* Status is unknown -- reset the bus */
}

void ide_dma_exec_cmd(ide_drive_t *drive, u8 command)
{
	/* issue cmd to drive */
	ide_execute_command(drive, command, &ide_dma_intr, 2 * WAIT_CMD,
			    dma_timer_expiry);
}
EXPORT_SYMBOL_GPL(ide_dma_exec_cmd);

void ide_dma_start(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 dma_cmd;

	/* Note that this is done *after* the cmd has
	 * been issued to the drive, as per the BM-IDE spec.
	 * The Promise Ultra33 doesn't work correctly when
	 * we do this part before issuing the drive cmd.
	 */
	if (hwif->host_flags & IDE_HFLAG_MMIO) {
		dma_cmd = readb((void __iomem *)(hwif->dma_base + ATA_DMA_CMD));
		/* start DMA */
		writeb(dma_cmd | 1,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_CMD));
	} else {
		dma_cmd = inb(hwif->dma_base + ATA_DMA_CMD);
		outb(dma_cmd | 1, hwif->dma_base + ATA_DMA_CMD);
	}

	wmb();
}
EXPORT_SYMBOL_GPL(ide_dma_start);

/* returns 1 on error, 0 otherwise */
int ide_dma_end(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 mmio = (hwif->host_flags & IDE_HFLAG_MMIO) ? 1 : 0;
	u8 dma_stat = 0, dma_cmd = 0;

	drive->waiting_for_dma = 0;

	if (mmio) {
		/* get DMA command mode */
		dma_cmd = readb((void __iomem *)(hwif->dma_base + ATA_DMA_CMD));
		/* stop DMA */
		writeb(dma_cmd & ~1,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_CMD));
	} else {
		dma_cmd = inb(hwif->dma_base + ATA_DMA_CMD);
		outb(dma_cmd & ~1, hwif->dma_base + ATA_DMA_CMD);
	}

	/* get DMA status */
	dma_stat = hwif->tp_ops->read_sff_dma_status(hwif);

	if (mmio)
		/* clear the INTR & ERROR bits */
		writeb(dma_stat | 6,
		       (void __iomem *)(hwif->dma_base + ATA_DMA_STATUS));
	else
		outb(dma_stat | 6, hwif->dma_base + ATA_DMA_STATUS);

	/* purge DMA mappings */
	ide_destroy_dmatable(drive);
	/* verify good DMA status */
	wmb();
	return (dma_stat & 7) != 4 ? (0x10 | dma_stat) : 0;
}
EXPORT_SYMBOL_GPL(ide_dma_end);

/* returns 1 if dma irq issued, 0 otherwise */
int ide_dma_test_irq(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 dma_stat = hwif->tp_ops->read_sff_dma_status(hwif);

	/* return 1 if INTR asserted */
	if ((dma_stat & 4) == 4)
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(ide_dma_test_irq);

const struct ide_dma_ops sff_dma_ops = {
	.dma_host_set		= ide_dma_host_set,
	.dma_setup		= ide_dma_setup,
	.dma_exec_cmd		= ide_dma_exec_cmd,
	.dma_start		= ide_dma_start,
	.dma_end		= ide_dma_end,
	.dma_test_irq		= ide_dma_test_irq,
	.dma_timeout		= ide_dma_timeout,
	.dma_lost_irq		= ide_dma_lost_irq,
};
EXPORT_SYMBOL_GPL(sff_dma_ops);
