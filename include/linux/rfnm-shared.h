#ifndef INCLUDE_LINUX_RFNM_SHARED_H_
#define INCLUDE_LINUX_RFNM_SHARED_H_

#define RFNM_DAUGHTERBOARD_BREAKOUT (1)
#define RFNM_DAUGHTERBOARD_GRANITA (2)
#define RFNM_DAUGHTERBOARD_LIME (3)

#define RFNM_DAUGHTERBOARD_PRESENT (0x10)
#define RFNM_DAUGHTERBOARD_NOT_FOUND (0x20)
#define RFNM_DAUGHTERBOARD_NOT_CHECKED_YET (0xff)

#define RFNM_BOOTCONFIG_PHYADDR (0xA3400000)


struct __packed rfnm_eeprom_data {
	uint8_t magic_header[4];
	uint8_t pad1[12];
	uint8_t board_id;
	uint8_t board_revision_id;
	uint8_t pad2[5];
	uint8_t serial_number[9];
	uint32_t crc;
};

// 0xff initial status is only guaranteed by uboot mod in the first 4kB
struct __packed rfnm_bootconfig {
	uint8_t daughterboard_present[2];
	uint8_t pcie_clock_ready;
	uint8_t usb_pd_negotiation_in_progress;
	struct rfnm_eeprom_data motherboard_eeprom;
	struct rfnm_eeprom_data daughterboard_eeprom[2];
};

#endif
