// SPDX-License-Identifier: GPL-2.0+

#ifndef INCLUDE_LINUX_RFNM_SHARED_H_
#define INCLUDE_LINUX_RFNM_SHARED_H_

#define RFNM_DAUGHTERBOARD_BREAKOUT (1)
#define RFNM_DAUGHTERBOARD_GRANITA (2)
#define RFNM_DAUGHTERBOARD_LIME (3)
#define RFNM_MOTHERBOARD_BLUE (4)
#define RFNM_DAUGHTERBOARD_YUCCA (5)

#define RFNM_BOARD_ID_TO_USER_READABLE_NAME (char[][30]){"", "Breakout", "Granita", "Lime", "Motherboard"}


#define RFNM_SLOT_PRIMARY (0)
#define RFNM_SLOT_SECONDARY (1)

#define RFNM_DAUGHTERBOARD_PRESENT (0x10)
#define RFNM_DAUGHTERBOARD_NOT_FOUND (0x20)
#define RFNM_DAUGHTERBOARD_NOT_CHECKED_YET (0xff)
#define RFNM_DTB_NODE_NOT_FOUND (-1)

extern int la9310_read_dtb_node_mem_region(const char *node_name, struct resource *get_mem_res);
 
#define RFNM_PACKED_STRUCT( __Declaration__ ) __Declaration__ __attribute__((__packed__))

#define RFNM_LA_BAR0_PHY_ADDR (0x18000000)

#define RFNM_LA_DMEM_PHY_ADDR (RFNM_LA_BAR2_PHY_ADDR + 0x400000)
#define RFNM_LA_IPPU_PHY_ADDR (RFNM_LA_BAR2_PHY_ADDR + 0x500000)



#define RFNM_LA_DCS_PHY_ADDR (RFNM_LA_BAR0_PHY_ADDR + 0x1040000)

#define HSDAC_CFGCTL1 ( 0x210 >> 2 )

#define RFNM_LA_GPOUT_PHY_ADDR (RFNM_LA_BAR0_PHY_ADDR + 0x1000580)

#define GP_OUT_4 ( 4 )
#define GP_OUT_7 ( 7 )

#define RFNM_ADC_MAP (int[]){0x2, 0x4, 0x1, 0x3}
	
#define RFNM_DAC_BUFCNT 0x4000


#define RFNM_TX (0)
#define RFNM_RX (1)


#define MHZ_TO_HZ(MHz) (MHz * 1000 * 1000ul)
#define HZ_TO_MHZ(Hz) (Hz / (1000ul * 1000ul))
#define HZ_TO_KHZ(Hz) (Hz / 1000ul)


#define RFNM_NUM_DCS_FREQ 25
extern uint32_t rfnm_si5510_plan_map[RFNM_NUM_DCS_FREQ][3];




struct __attribute__((__packed__)) rfnm_eeprom_data {
	uint8_t magic_header[4];
	uint8_t pad1[6];
	uint8_t mac_addr[6];
	uint8_t board_id;
	uint8_t board_revision_id;
	uint8_t pad2[5];
	uint8_t serial_number[9];
	uint32_t crc;
};


struct __attribute__((__packed__)) rfnm_eeprom_user_config {
	uint8_t magic_header[4];
	uint32_t dcs_clk_tmp;
	uint32_t crc;
};

// 0xff initial status is only guaranteed by uboot mod in the first 4kB
struct __attribute__((__packed__)) rfnm_bootconfig {
	uint8_t daughterboard_present[2];
  	uint8_t pcie_clock_ready;
	uint8_t usb_pd_negotiation_in_progress;
	struct rfnm_eeprom_data motherboard_eeprom;
	struct rfnm_eeprom_data daughterboard_eeprom[2];
	struct rfnm_eeprom_user_config user_eeprom;
};













#include <linux/rfnm-api.h>





//#define RFNM_MAX_TRX_CH_CNT (8)



	struct __attribute__((__packed__)) fe_s {
		uint32_t latch_val[6];
		uint32_t latch_val_last_written[6];
		uint32_t num_latches[7];
		uint32_t align[1];
		uint32_t load_order[8];
	};

struct rfnm_dgb {
	struct rfnm_api_rx_ch *rx_ch[4];
	struct rfnm_api_tx_ch *tx_ch[4];
	struct rfnm_api_rx_ch *rx_s[4];
	struct rfnm_api_tx_ch *tx_s[4];
	int rx_ch_cnt;
	int tx_ch_cnt;

	uint8_t board_id;
	uint8_t board_revision_id;
	uint8_t serial_number[9];
	//struct rfnm_dgb_dt *rfnm_dgb_dt;


	struct device dev;
	//void *priv;
	int dgb_id;
	void *priv_drv;
	//void *priv_fe;
	struct fe_s fe;
	struct fe_s fe_tdd[2];
	void * rx_ch_set;
	void * rx_ch_get;
	void * tx_ch_set;
	void * tx_ch_get;

	uint8_t dac_ifs;
	uint8_t adc_iqswap[2];
	uint8_t dac_iqswap[2];
	int (*reset)(struct rfnm_dgb *);// Function pointer for reset sequence
};

	struct __attribute__((__packed__)) rfnm_m7_dgb {
		struct fe_s fe;
		struct fe_s fe_tdd[2];
		uint32_t m7_tdd_initialized;
        uint32_t dgb_id;
        uint32_t tdd_available;
	};


void rfnm_dgb_reg_rx_ch(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch, struct rfnm_api_rx_ch * rx_s);
void rfnm_dgb_reg_tx_ch(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch, struct rfnm_api_tx_ch * tx_s);
void rfnm_dgb_reg(struct rfnm_dgb *dgb_dt);
void rfnm_dgb_unreg(struct rfnm_dgb *dgb_dt);
void rfnm_dgb_en_tdd(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch, struct rfnm_api_rx_ch * rx_ch);

void rfnm_populate_dev_hwinfo(struct rfnm_dev_hwinfo *r_hwinfo);


void rfnm_populate_dev_tx_chlist(struct rfnm_dev_tx_ch_list * r_chlist);
void rfnm_populate_dev_rx_chlist(struct rfnm_dev_rx_ch_list * r_chlist);
void rfnm_apply_dev_tx_chlist(struct rfnm_dev_tx_ch_list * r_chlist);
void rfnm_apply_dev_rx_chlist(struct rfnm_dev_rx_ch_list * r_chlist);
void rfnm_populate_dev_set_res(struct rfnm_dev_get_set_result * r_res);
int rfnm_la9310_stream(uint8_t tx, uint8_t *rx);
int rfnm_la9310_stream(uint8_t tx, uint8_t *rx);
void rfnm_populate_dev_status(struct rfnm_dev_status * r_stat);
void rfnm_restart_sm(int hard);



#endif
