/*
 * The touch screen of the Xiaomi Redmi Note 9 (merlin / merlinx, MT6768 Helio G85):
 * a Novatek NVT-ts-spi (NT36672A / NT36xxx) on SPI0 of the MT6768, as a source of touches
 * for /dev/redwood-touch (sys/redwood_input.h). Activated with boot-arg touchscreen=3.
 *
 * Hardware configuration (from merlin.dts & pinctrl-mtk-mt6768.h):
 *   SPI controller: SPI0 at 0x1100a000
 *   Pins:
 *     GPIO 1:   IRQ line (bank 0 bit 1, active low, pull-up, IOCFG_RM base 5)
 *     GPIO 92:  RST line (bank 2 bit 28, active low, idle high, IOCFG_RM base 5)
 *     GPIO 32:  MI  (bank 1 bit 0, mode 0: GPIO input, IES enabled)
 *     GPIO 33:  CS  (bank 1 bit 1, mode 0: GPIO output, idle high)
 *     GPIO 34:  MO  (bank 1 bit 2, mode 0: GPIO output, idle low)
 *     GPIO 35:  CLK (bank 1 bit 3, mode 0: GPIO output, idle low)
 *   Resolution: 1080 x 2340
 *   Max touches: 10
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/redwood_input.h>
#include <sys/systm.h>
#include "redwood_novatek_fw.h"

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define GPIO_PHYS       0x10005000
#define GPIO_SIZE       0x1000

#define IOCFG_RM_PHYS   0x10002800
#define IOCFG_RM_SIZE   0x1000

#define PANEL_W         1080
#define PANEL_H         2340
#define MAX_FINGERS     10
#define POINT_DATA_LEN  65

static vm_offset_t gpio;
static vm_offset_t iocfg;

#define rd(base, off)           (*(volatile uint32_t *)((base) + (off)))
#define wr(base, off, val)      (*(volatile uint32_t *)((base) + (off)) = (val))

#define GPIO_DIR0_SET   0x004
#define GPIO_DIR0_CLR   0x008
#define GPIO_DIR1_SET   0x014
#define GPIO_DIR1_CLR   0x018
#define GPIO_DIR2_SET   0x024
#define GPIO_DIR2_CLR   0x028

#define GPIO_DOUT1_SET  0x114
#define GPIO_DOUT1_CLR  0x118
#define GPIO_DOUT2_SET  0x124
#define GPIO_DOUT2_CLR  0x128

#define GPIO_DIN0       0x200
#define GPIO_DIN1       0x210

#define GPIO_MODE0_SET  0x304
#define GPIO_MODE0_CLR  0x308
#define GPIO_MODE4_SET  0x344
#define GPIO_MODE4_CLR  0x348
#define GPIO_MODE11_SET 0x3b4
#define GPIO_MODE11_CLR 0x3b8

#define BANK0_IRQ       (1u << 1)
#define BANK2_RST       (1u << 28)
#define BANK1_MI        (1u << 0)
#define BANK1_CS        (1u << 1)
#define BANK1_MO        (1u << 2)
#define BANK1_CLK       (1u << 3)

static inline void
clk_set(bool high)
{
	wr(gpio, high ? GPIO_DOUT1_SET : GPIO_DOUT1_CLR, BANK1_CLK);
}

static inline void
cs_set(bool high)
{
	wr(gpio, high ? GPIO_DOUT1_SET : GPIO_DOUT1_CLR, BANK1_CS);
}

static inline void
mo_set(bool high)
{
	wr(gpio, high ? GPIO_DOUT1_SET : GPIO_DOUT1_CLR, BANK1_MO);
}

static inline void
rst_set(bool high)
{
	wr(gpio, high ? GPIO_DOUT2_SET : GPIO_DOUT2_CLR, BANK2_RST);
}

static inline bool
mi_get(void)
{
	return (rd(gpio, GPIO_DIN1) & BANK1_MI) != 0;
}

static inline bool
irq_get(void)
{
	return (rd(gpio, GPIO_DIN0) & BANK0_IRQ) != 0;
}

static inline void
pin_settle(void)
{
	for (volatile int i = 0; i < 8; i++) {
		__asm__ volatile ("nop");
	}
}

static void
set_pin_mode(unsigned pin, unsigned mode)
{
	unsigned reg_idx = pin / 8;
	unsigned bit_pos = (pin % 8) * 4;
	uint32_t mask = 0x7u << bit_pos;
	uint32_t val = (mode & 0x7u) << bit_pos;

	uint32_t clr_off = 0x308 + reg_idx * 0x10;
	uint32_t set_off = 0x304 + reg_idx * 0x10;

	wr(gpio, clr_off, mask);
	if (val) {
		wr(gpio, set_off, val);
	}
}

/* SPI Mode 0 bitbang transfer (CPOL=0, CPHA=0) */
static void
spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	cs_set(false);
	pin_settle();
	pin_settle();

	for (uint32_t at = 0; at < len; at++) {
		uint8_t out = tx ? tx[at] : 0;
		uint8_t in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			mo_set((out >> bit) & 1);
			pin_settle();

			clk_set(true);
			pin_settle();

			if (mi_get()) {
				in |= (1u << bit);
			}

			clk_set(false);
			pin_settle();
		}
		if (rx) {
			rx[at] = in;
		}
	}
	cs_set(true);
	pin_settle();
	pin_settle();
}

static void
nvt_set_page(uint32_t addr)
{
	uint8_t tx[3];
	tx[0] = 0xFF;   /* Set index/page/addr command */
	tx[1] = (addr >> 15) & 0xFF;
	tx[2] = (addr >> 7) & 0xFF;
	spi_transfer(tx, NULL, 3);
}

static void
nvt_write_addr(uint32_t addr, uint8_t data)
{
	uint8_t tx[2];
	nvt_set_page(addr);
	tx[0] = (addr & 0x7F) | 0x80;   /* SPI_WRITE_MASK */
	tx[1] = data;
	spi_transfer(tx, NULL, 2);
}

static void
nvt_read_addr(uint32_t addr, uint8_t *rx_data, uint32_t len)
{
	uint8_t tx[68] = {0};
	uint8_t rx[68] = {0};
	if (len > 64) {
		len = 64;
	}
	nvt_set_page(addr);
	tx[0] = (addr & 0x7F) & 0x7F;   /* SPI_READ_MASK */
	spi_transfer(tx, rx, len + 2);  /* 1 command byte + 1 dummy byte + data */
	if (rx_data) {
		memcpy(rx_data, &rx[2], len);
	}
}

static void
nvt_set_bld_crc_bank(uint32_t des_addr, uint32_t sram_addr,
		     uint32_t src_addr, uint32_t size,
		     uint32_t crc_addr, uint32_t crc_val)
{
	uint8_t b[4];

	b[0] = (sram_addr >> 8) & 0xFF;
	b[1] = (sram_addr >> 16) & 0xFF;
	b[2] = (sram_addr >> 24) & 0xFF;
	nvt_write_addr(des_addr, b[0]);
	nvt_write_addr(des_addr + 1, b[1]);
	nvt_write_addr(des_addr + 2, b[2]);

	b[0] = size & 0xFF;
	b[1] = (size >> 8) & 0xFF;
	b[2] = (size >> 16) & 0xFF;
	nvt_write_addr(src_addr, b[0]);
	nvt_write_addr(src_addr + 1, b[1]);
	nvt_write_addr(src_addr + 2, b[2]);

	b[0] = crc_val & 0xFF;
	b[1] = (crc_val >> 8) & 0xFF;
	b[2] = (crc_val >> 16) & 0xFF;
	b[3] = (crc_val >> 24) & 0xFF;
	nvt_write_addr(crc_addr, b[0]);
	nvt_write_addr(crc_addr + 1, b[1]);
	nvt_write_addr(crc_addr + 2, b[2]);
	nvt_write_addr(crc_addr + 3, b[3]);
}

static bool
nvt_download_firmware(void)
{
	printf("nvt: downloading Novatek firmware into SRAM (%u bytes)...\n", NOVATEK_FW_LENGTH);

	/* 1. Reset MCU and hold in idle / bootloader state */
	rst_set(false);
	delay(10000);
	rst_set(true);
	delay(25000);

	/* Write Stop MCU command */
	nvt_write_addr(0x3F004, 0x55);
	delay(2000);

	/* Verify Bootloader state (0x3F100 should be 0x05 for NT36672A) */
	uint8_t bld_id = 0;
	nvt_read_addr(0x3F100, &bld_id, 1);
	printf("nvt: bootloader status at 0x3F100: 0x%02x\n", bld_id);

	/* 2. Parse firmware partition table from header */
	const uint8_t *fw = novatek_fw;
	uint32_t info_sec_num = (fw[0x1e] == 0xFF) ? 0 : fw[0x1e];
	if (info_sec_num > 4) {
		printf("nvt: invalid info_sec_num %u\n", info_sec_num);
		return false;
	}
	uint32_t ovly_info = (fw[0x28] & 0x10) >> 4;
	uint32_t ovly_sec_num = ovly_info ? (fw[0x28] & 0x0F) : 0;
	uint32_t total_partitions = 2 + ovly_sec_num + info_sec_num;

	uint32_t ilm_sram = 0, ilm_size = 0, ilm_crc = 0;
	uint32_t dlm_sram = 0, dlm_size = 0, dlm_crc = 0;

	for (uint32_t i = 0; i < total_partitions; i++) {
		uint32_t bin_addr = 0, sram_addr = 0, size = 0, crc = 0;
		if (i < 2) {
			bin_addr = fw[i*12] | ((uint32_t)fw[i*12+1] << 8) | ((uint32_t)fw[i*12+2] << 16) | ((uint32_t)fw[i*12+3] << 24);
			sram_addr = fw[i*12+4] | ((uint32_t)fw[i*12+5] << 8) | ((uint32_t)fw[i*12+6] << 16) | ((uint32_t)fw[i*12+7] << 24);
			size = fw[i*12+8] | ((uint32_t)fw[i*12+9] << 8) | ((uint32_t)fw[i*12+10] << 16) | ((uint32_t)fw[i*12+11] << 24);
			crc = fw[0x18+i*4] | ((uint32_t)fw[0x19+i*4] << 8) | ((uint32_t)fw[0x1A+i*4] << 16) | ((uint32_t)fw[0x1B+i*4] << 24);
			if (i == 0) {
				ilm_sram = sram_addr;
				ilm_size = size;
				ilm_crc = crc;
			} else {
				dlm_sram = sram_addr;
				dlm_size = size;
				dlm_crc = crc;
			}
		} else {
			uint32_t pos = 0x30 + (i - 2) * 0x10;
			sram_addr = fw[pos] | ((uint32_t)fw[pos+1] << 8) | ((uint32_t)fw[pos+2] << 16) | ((uint32_t)fw[pos+3] << 24);
			size = fw[pos+4] | ((uint32_t)fw[pos+5] << 8) | ((uint32_t)fw[pos+6] << 16) | ((uint32_t)fw[pos+7] << 24);
			bin_addr = fw[pos+8] | ((uint32_t)fw[pos+9] << 8) | ((uint32_t)fw[pos+10] << 16) | ((uint32_t)fw[pos+11] << 24);
			crc = fw[pos+12] | ((uint32_t)fw[pos+13] << 8) | ((uint32_t)fw[pos+14] << 16) | ((uint32_t)fw[pos+15] << 24);
		}

		if (size == 0) {
			continue;
		}
		uint32_t actual_size = size + 1;
		if (bin_addr + actual_size > NOVATEK_FW_LENGTH) {
			printf("nvt: part %u exceeds fw length!\n", i);
			return false;
		}

		/* Write partition to SRAM */
		uint32_t written = 0;
		while (written < actual_size) {
			uint32_t len = actual_size - written;
			if (len > 1024) {
				len = 1024;
			}
			uint32_t cur_sram = sram_addr + written;
			nvt_set_page(cur_sram);

			cs_set(false);
			pin_settle();
			pin_settle();

			uint8_t cmd = (cur_sram & 0x7F) | 0x80;
			for (int bit = 7; bit >= 0; bit--) {
				mo_set((cmd >> bit) & 1);
				pin_settle();
				clk_set(true);
				pin_settle();
				clk_set(false);
				pin_settle();
			}
			for (uint32_t j = 0; j < len; j++) {
				uint8_t b = fw[bin_addr + written + j];
				for (int bit = 7; bit >= 0; bit--) {
					mo_set((b >> bit) & 1);
					pin_settle();
					clk_set(true);
					pin_settle();
					clk_set(false);
					pin_settle();
				}
			}
			cs_set(true);
			pin_settle();
			pin_settle();

			written += len;
		}
	}

	/* 3. Configure hardware CRC check registers */
	nvt_set_bld_crc_bank(0x3F128, ilm_sram, 0x3F118, ilm_size, 0x3F100, ilm_crc);
	nvt_set_bld_crc_bank(0x3F12C, dlm_sram, 0x3F130, dlm_size, 0x3F104, dlm_crc);

	/* 4. Enable bootloader hardware CRC */
	uint8_t rx[4] = {0};
	nvt_read_addr(0x3F30E, rx, 1);
	nvt_write_addr(0x3F30E, rx[0] | 0x80);

	/* 5. Clear firmware reset status and enable firmware CRC */
	nvt_write_addr(0x21C60, 0x00);
	nvt_write_addr(0x21C50, 0xAE);

	/* 6. Set Boot Ready Bit to kickstart firmware execution */
	nvt_write_addr(0x3F10D, 0x01);
	delay(10000);

	/* 7. Wait for MCU to enter normal run state (0xA0..0xAF) */
	for (int retry = 0; retry < 50; retry++) {
		uint8_t st = 0;
		nvt_read_addr(0x21C60, &st, 1);
		if (st >= 0xA0 && st <= 0xAF) {
			printf("nvt: firmware running! state=0x%02x\n", st);
			return true;
		}
		delay(10000);
	}
	printf("nvt: firmware boot timeout (EVENT_MAP_RESET_COMPLETE != 0xA0..0xAF)\n");
	return false;
}

static bool down[MAX_FINGERS];
static uint32_t last_x[MAX_FINGERS], last_y[MAX_FINGERS];
static unsigned touch_events;

static void
nvt_recover_and_reinit(uint32_t event_buf_addr)
{
	/* Ensure RST is high */
	rst_set(true);
	delay(2000);

	/* Check firmware status */
	uint8_t fw_info[16] = {0};
	nvt_read_addr(event_buf_addr | 0x78, fw_info, sizeof(fw_info));
	if ((uint8_t)(fw_info[0] + fw_info[1]) != 0xFF || fw_info[0] == 0) {
		printf("nvt: watchdog: firmware inactive (ver=0x%02x ~ver=0x%02x), reloading FW...\n",
		    fw_info[0], fw_info[1]);
		nvt_download_firmware();
	}
	nvt_set_page(event_buf_addr);
	uint8_t dummy_tx[POINT_DATA_LEN + 2] = {0};
	uint8_t dummy_rx[POINT_DATA_LEN + 2] = {0};
	spi_transfer(dummy_tx, dummy_rx, sizeof(dummy_rx));
}

static void
nvt_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 4000, log = 0;
	/* 1 command byte (0x00) + 1 dummy byte + 65 data payload bytes = 67 bytes */
	uint8_t rx[POINT_DATA_LEN + 2] = {0};
	uint8_t tx[POINT_DATA_LEN + 2] = {0};
	uint8_t id_buf[6] = {0};
	uint32_t event_buf_addr = 0x21C00; /* NT36672A default */
	bool last_irq_state = true;
	uint32_t bad_reads = 0;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("touchlog", &log, sizeof(log));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 4000;
	}

	gpio = ml_io_map(GPIO_PHYS, GPIO_SIZE);
	iocfg = ml_io_map(IOCFG_RM_PHYS, IOCFG_RM_SIZE);
	if (!gpio || !iocfg) {
		printf("nvt: ml_io_map failed for GPIO or IOCFG!\n");
		return;
	}

	/*
	 * Enable IES (Input Enable Schmitt-trigger) in IOCFG_RM (base 5):
	 *   Offset 0x064:
	 *     bit 8:  GPIO 1  (IRQ)
	 *     bit 20: GPIO 92 (RST)
	 *     bit 29: GPIO 35 (CLK)
	 *     bit 30: GPIO 33 (CS)
	 *     bit 31: GPIO 32 (MI)
	 *   Offset 0x074:
	 *     bit 0:  GPIO 34 (MO)
	 */
	wr(iocfg, 0x064, (1u << 8) | (1u << 20) | (1u << 29) | (1u << 30) | (1u << 31));
	wr(iocfg, 0x074, 1u << 0);

	/* Clear Pull-Down (PD) on GPIO 1 (IRQ) */
	wr(iocfg, 0x088, 1u << 8);

	/* Set Pull-Up (PU) on GPIO 1 (IRQ) */
	wr(iocfg, 0x0a4, 1u << 8);

	/* Set Schmitt Trigger (SMT) */
	wr(iocfg, 0x104, (1u << 5) | (1u << 18));
	__asm__ volatile ("dsb sy");

	/* Configure Pin Modes via MWR: Mode 0 (GPIO) */
	set_pin_mode(1, 0);     /* GPIO 1: IRQ */
	set_pin_mode(92, 0);    /* GPIO 92: RST */
	set_pin_mode(32, 0);    /* GPIO 32: MI */
	set_pin_mode(33, 0);    /* GPIO 33: CS */
	set_pin_mode(34, 0);    /* GPIO 34: MO */
	set_pin_mode(35, 0);    /* GPIO 35: CLK */

	/* GPIO 92 (RST): output high */
	wr(gpio, GPIO_DIR2_SET, BANK2_RST);
	wr(gpio, GPIO_DOUT2_SET, BANK2_RST);

	/* GPIO 1 (IRQ): input */
	wr(gpio, GPIO_DIR0_CLR, BANK0_IRQ);

	/* Pins 32..35: CS, MO, CLK output; MI input */
	wr(gpio, GPIO_DIR1_SET, BANK1_CS | BANK1_MO | BANK1_CLK);
	wr(gpio, GPIO_DIR1_CLR, BANK1_MI);
	wr(gpio, GPIO_DOUT1_SET, BANK1_CS);             /* CS idle high */
	wr(gpio, GPIO_DOUT1_CLR, BANK1_CLK | BANK1_MO); /* Mode 0: CLK idle low, MO low */
	__asm__ volatile ("dsb sy");

	/* Ensure RST is high */
	rst_set(true);
	delay(10000);

	/* Read chip trim ID from address 0x1F64E */
	nvt_read_addr(0x1F64E, id_buf, sizeof(id_buf));
	printf("nvt: chip trim ID @ 0x1F64E: %02x %02x %02x %02x %02x %02x\n",
	    id_buf[0], id_buf[1], id_buf[2], id_buf[3], id_buf[4], id_buf[5]);

	if (id_buf[0] == 0x0D || id_buf[3] == 0x75) {
		event_buf_addr = 0x22D00;
	} else {
		event_buf_addr = 0x21C00;
	}

	/* Query firmware info (EVENT_MAP_FWINFO = 0x78) */
	uint8_t fw_info[16] = {0};
	nvt_read_addr(event_buf_addr | 0x78, fw_info, sizeof(fw_info));
	printf("nvt: FW info: ver=0x%02x ~ver=0x%02x x_max=%u y_max=%u\n",
	    fw_info[0], fw_info[1],
	    ((uint32_t)fw_info[4] << 8) | fw_info[5],
	    ((uint32_t)fw_info[6] << 8) | fw_info[7]);

	/* If firmware is not running, download firmware into NT36672A SRAM */
	if ((uint8_t)(fw_info[0] + fw_info[1]) != 0xFF || fw_info[0] == 0) {
		printf("nvt: firmware not active; uploading novatek_ts_fw...\n");
		if (nvt_download_firmware()) {
			memset(fw_info, 0, sizeof(fw_info));
			nvt_read_addr(event_buf_addr | 0x78, fw_info, sizeof(fw_info));
			printf("nvt: FW info after upload: ver=0x%02x ~ver=0x%02x x_max=%u y_max=%u\n",
			    fw_info[0], fw_info[1],
			    ((uint32_t)fw_info[4] << 8) | fw_info[5],
			    ((uint32_t)fw_info[6] << 8) | fw_info[7]);
		}
	}

	/* Point Novatek page pointer to the event buffer */
	nvt_set_page(event_buf_addr);

	/* Dummy read to clear any power-on interrupt */
	memset(tx, 0, sizeof(tx));
	tx[0] = 0x00;
	spi_transfer(tx, rx, sizeof(rx));

	last_irq_state = irq_get();
	printf("nvt: Novatek NT36xxx on SPI0 ready (event_buf=0x%05x), IRQ initial=%u\n",
	    event_buf_addr, last_irq_state);

	for (unsigned tick = 1;; tick++) {
		bool current_irq = irq_get();

		/* Log IRQ state transitions */
		if (current_irq != last_irq_state) {
			if (log) {
				printf("nvt: IRQ %s (%u)\n", current_irq ? "HIGH" : "LOW", current_irq);
			}
			last_irq_state = current_irq;
		}

		/*
		 * Read when IRQ is active, fingers are down, or periodically
		 */
		bool any_down = false;
		for (unsigned i = 0; i < MAX_FINGERS; i++) {
			if (down[i]) {
				any_down = true;
				break;
			}
		}

		if (current_irq || any_down || (tick % 2 == 0)) {
			memset(rx, 0, sizeof(rx));
			memset(tx, 0, sizeof(tx));
			tx[0] = 0x00;   /* Read event buffer command */

			spi_transfer(tx, rx, sizeof(rx));

			/*
			 * Watchdog check for screen off / wake / lost page pointer
			 */
			bool is_all_ff = true;
			for (int k = 0; k < 8; k++) {
				if (rx[k] != 0xFF) {
					is_all_ff = false;
					break;
				}
			}
			bool is_all_zero = true;
			for (int k = 0; k < 8; k++) {
				if (rx[k] != 0x00) {
					is_all_zero = false;
					break;
				}
			}

			if (is_all_ff || (current_irq && is_all_zero)) {
				bad_reads++;
				if (bad_reads >= 4) {
					if (log || bad_reads == 4) {
						printf("nvt: watchdog: lost touch response (all 0x%02x, irq=%u), recovering...\n",
						    is_all_ff ? 0xFF : 0x00, current_irq);
					}
					/* Lift all stuck fingers */
					for (unsigned id = 0; id < MAX_FINGERS; id++) {
						if (down[id]) {
							uint32_t norm_x = last_x[id] * 65535 / (PANEL_W - 1);
							uint32_t norm_y = last_y[id] * 65535 / (PANEL_H - 1);
							redwood_touch_post(REDWOOD_TOUCH_UP, id, norm_x, norm_y);
							down[id] = false;
						}
					}
					nvt_recover_and_reinit(event_buf_addr);
					bad_reads = 0;
					delay(5000);
					continue;
				}
			} else {
				bad_reads = 0;
			}

			/* Periodically refresh page pointer to guarantee it never drifts */
			if (tick % 1000 == 0) {
				nvt_set_page(event_buf_addr);
			}

			/*
			 * rx[0]: command byte echo
			 * rx[1]: dummy byte
			 * rx[2..66]: point_data[0..64]
			 */
			bool seen[MAX_FINGERS] = { false };
			unsigned active_fingers = 0;

			for (unsigned i = 0; i < MAX_FINGERS; i++) {
				uint32_t off = 2 + 6 * i;
				if (off + 3 >= sizeof(rx)) {
					break;
				}
				uint8_t status = rx[off];
				uint8_t type = status & 0x07;
				uint8_t raw_id = status >> 3;

				if (raw_id < 1 || raw_id > MAX_FINGERS) {
					continue;
				}
				uint8_t id = raw_id - 1; /* 1-based to 0-based */

				/*
				 * Novatek NT36xxx touch packet:
				 * status & 0x07: 0x01 = enter (down), 0x02 = moving.
				 */
				if (type == 1 || type == 2) {
					uint32_t x = ((uint32_t)rx[off + 1] << 4) | ((uint32_t)rx[off + 3] >> 4);
					uint32_t y = ((uint32_t)rx[off + 2] << 4) | ((uint32_t)rx[off + 3] & 0x0f);

					/* Sanity check: coordinates must be plausible */
					if (x >= PANEL_W * 2 || y >= PANEL_H * 2) {
						continue;
					}

					if (x >= PANEL_W) {
						x = PANEL_W - 1;
					}
					if (y >= PANEL_H) {
						y = PANEL_H - 1;
					}
					seen[id] = true;
					last_x[id] = x;
					last_y[id] = y;

					uint32_t norm_x = x * 65535 / (PANEL_W - 1);
					uint32_t norm_y = y * 65535 / (PANEL_H - 1);

					uint32_t ev = down[id] ? REDWOOD_TOUCH_MOVE : REDWOOD_TOUCH_DOWN;
					redwood_touch_post(ev, id, norm_x, norm_y);
					if (log >= 2 && (ev == REDWOOD_TOUCH_DOWN || (touch_events % 30 == 0))) {
						printf(">>> TOUCH: finger %u %s at (%u, %u) norm (%u, %u)\n",
						    id, (ev == REDWOOD_TOUCH_MOVE) ? "move" : "down", x, y, norm_x, norm_y);
					}
					down[id] = true;
					touch_events++;
					active_fingers++;
				}
			}

			for (unsigned id = 0; id < MAX_FINGERS; id++) {
				if (down[id] && !seen[id]) {
					uint32_t norm_x = last_x[id] * 65535 / (PANEL_W - 1);
					uint32_t norm_y = last_y[id] * 65535 / (PANEL_H - 1);

					redwood_touch_post(REDWOOD_TOUCH_UP, id, norm_x, norm_y);
					down[id] = false;
					touch_events++;
					if (log >= 2) {
						printf(">>> TOUCH: finger %u lifted at (%u, %u)\n", id, last_x[id], last_y[id]);
					}
				}
			}
		}

		if (tick % 5000 == 0 && log) {
			printf("nvt: tick %u, IRQ=%u, touch events: %u\n",
			    tick, irq_get(), touch_events);
		}

		delay(poll_us);
	}
}

void
redwood_nvt_init(void)
{
	uint32_t which = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &which, sizeof(which)) || which != 3) {
		return;
	}
	if (kernel_thread_start(nvt_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("nvt: cannot start the Novatek touch screen thread\n");
	}
}
