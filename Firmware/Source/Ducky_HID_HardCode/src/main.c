//_____  M A I N ___________________________________________________________
//
// Module		: Ducky
// Description	: Simple USB HID Keyboard injection Hardcoded in Firmware
// Date			: April 14, 2013
// Author       : Snake
//__________________________________________________________________________

//_____  I N C L U D E S ___________________________________________________
#include <string.h>
#include "compiler.h"
#include "main.h"
#include "preprocessor.h"
#include "board.h"
#include "ctrl_access.h"
#include "power_clocks_lib.h"
#include "gpio.h"
#include "spi.h"
#include "conf_sd_mmc_spi.h"
#include "fat.h"
#include "file.h"
#include "navigation.h"
#include "conf_usb.h"
#include "udc.h"
#include "udd.h"
#include "led.h"
#include "udi_hid_kbd.h"
#include "sysclk.h"
#include "sleepmgr.h"

//_____ D E C L A R A T I O N S ____________________________________________
#define SEQUENCE_PERIOD 100
// filename
//char *injectFile = "A:\\inject.bin";

// state machine enum
static struct {
	bool b_modifier;
	bool b_down;
	uint8_t u8_value;
}ui_sequence[] = {
	// Display windows menu
{true,true,HID_MODIFIER_LEFT_UI},
	// Launch Windows Command line
{false,true,HID_R},
{false,false,HID_R},
	// Clear modifier
{true,false,HID_MODIFIER_LEFT_UI},
	// Tape sequence "notepad" + return
{false,true,HID_N},
{false,false,HID_N},
{false,true,HID_O},
{false,false,HID_O},
{false,true,HID_T},
{false,false,HID_T},
{false,true,HID_E},
{false,false,HID_E},
{false,true,HID_P},
{false,false,HID_P},
{false,true,HID_A},
{false,false,HID_A},
{false,true,HID_D},
{false,false,HID_D},
{false,true,HID_ENTER},
{false,false,HID_ENTER},
	// Delay to wait "notepad" focus
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
{false,false,0}, // No key (= SEQUENCE_PERIOD delay)
	// Display "Atmel "
{true,true,HID_MODIFIER_RIGHT_SHIFT}, // Enable Maj
{false,true,HID_A},
{false,false,HID_A},
{true,false,HID_MODIFIER_RIGHT_SHIFT}, // Disable Maj
{false,true,HID_T},
{false,false,HID_T},
{false,true,HID_M},
{false,false,HID_M},
{false,true,HID_E},
{false,false,HID_E},
{false,true,HID_L},
{false,false,HID_L},
{false,true,HID_SPACEBAR},
{false,false,HID_SPACEBAR},
	// Display "AVR "
{false,true,HID_CAPS_LOCK}, // Enable caps lock
{false,false,HID_CAPS_LOCK},
{false,true,HID_A},
{false,false,HID_A},
{false,true,HID_V},
{false,false,HID_V},
{false,true,HID_R},
{false,false,HID_R},
{false,true,HID_CAPS_LOCK}, // Disable caps lock
{false,false,HID_CAPS_LOCK},
};

//_____ F U N C T I O N S __________________________________________________

// initializes the SD/MMC memory resources: GPIO, SPI and MMC
//-------------------------------------------------------------------
static void sd_mmc_resources_init(long pba_hz) {
  
	// GPIO pins used for SD/MMC interface
	static const gpio_map_t SD_MMC_SPI_GPIO_MAP = {
		{SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
		{SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
		{SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
		{SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions = {
		.reg          = SD_MMC_SPI_NPCS,
		.baudrate     = SD_MMC_SPI_MASTER_SPEED,  // Defined in conf_sd_mmc_spi.h.
		.bits         = SD_MMC_SPI_BITS,          // Defined in conf_sd_mmc_spi.h.
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};

	// assign I/Os to SPI.
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
						sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

	// initialize as master.
	spi_initMaster(SD_MMC_SPI, &spiOptions);

	// set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

	// enable SPI module.
	spi_enable(SD_MMC_SPI);

	// Initialize SD/MMC driver with SPI clock (PBA).
	sd_mmc_spi_init(spiOptions, pba_hz);
}

// process a USB frame
//-------------------------------------------------------------------
void process_frame(uint16_t framenumber)
{
	bool b_btn_state, sucess;
	static bool btn_last_state = false;
	static bool sequence_running = false;
	static uint8_t u8_sequence_pos = 0;
	uint8_t u8_value;
	static uint16_t cpt_sof = 0;

	if ((framenumber % 1000) == 0) {
		LED_On(LED1);
	}
	if ((framenumber % 1000) == 500) {
		LED_Off(LED1);
	}
	// Scan process running each 2ms
	cpt_sof++;
	if ((cpt_sof % 2) == 0) {
		return;
	}

	// Scan buttons on switch 0 to send keys sequence
	b_btn_state = (!gpio_get_pin_value(GPIO_PUSH_BUTTON_0)) ? true : false;
	if (b_btn_state != btn_last_state) {
		btn_last_state = b_btn_state;
		sequence_running = true;
	}

	// Sequence process running each period
	if (SEQUENCE_PERIOD > cpt_sof) {
		return;
	}
	cpt_sof = 0;

	if (sequence_running) {
		// Send next key
		u8_value = ui_sequence[u8_sequence_pos].u8_value;
		if (u8_value!=0) {
			if (ui_sequence[u8_sequence_pos].b_modifier) {
				if (ui_sequence[u8_sequence_pos].b_down) {
					sucess = udi_hid_kbd_modifier_down(u8_value);
				} else {
					sucess = udi_hid_kbd_modifier_up(u8_value);
				}
			} else {
				if (ui_sequence[u8_sequence_pos].b_down) {
					sucess = udi_hid_kbd_down(u8_value);
				} else {
					sucess = udi_hid_kbd_up(u8_value);
				}
			}
			if (!sucess) {
				return; // Retry it on next schedule
			}
		}
		// Valid sequence position
		u8_sequence_pos++;
		if (u8_sequence_pos >=
		sizeof(ui_sequence) / sizeof(ui_sequence[0])) {
			u8_sequence_pos = 0;
			sequence_running = false;
		}
	}
}

// Main Method - IRQ, CLCK, INIT setup
//-------------------------------------------------------------------
int main(void) {
	
	uint32_t sizeTemp;
	
	// init cpu
	irq_initialize_vectors();
	cpu_irq_enable();

	// init board
	sleepmgr_init();
	sysclk_init();
	board_init();

	// initialize SD/MMC resources: GPIO, SPI.
	sd_mmc_resources_init(FOSC0);
	
	// test if the memory is ready - using the control access memory abstraction layer (/SERVICES/MEMORY/CTRL_ACCESS/)
	if (mem_test_unit_ready(LUN_ID_SD_MMC_SPI_MEM) == CTRL_GOOD) {
		// Get and display the capacity
		mem_read_capacity(LUN_ID_SD_MMC_SPI_MEM, &sizeTemp);
	}
	else {
		//  error - we can't proceed - sit and spin...
		while(true) { LED_On( LED1 ); }
	}
	
	/*nav_reset();
	if( !nav_setcwd( injectFile, true, false ) ) {
		//try to open a://inject.bin else sit here 
		while(true) { 
			LED_On( LED1 );
			for (int i=0; i<10000; i++){}
			LED_Off(LED1); 
		}
	}	*/	
	// Start USB stack
	udc_start();
	udc_attach();
	
	while(true) {
		//do nothing - handle interrupts and events
		//sleepmgr_enter_sleep();
	}
}

//-------------------------------------------------------------------
void main_suspend_action(void)
{
	LED_Off(LED0);
	LED_Off(LED1);
}

//-------------------------------------------------------------------
void main_resume_action(void)
{
}
 
//-------------------------------------------------------------------
void main_sof_action(void)
{
		process_frame( udd_get_frame_number() );

}

//-------------------------------------------------------------------
// If remote wakeup enable/disable is supported insert code below
void main_remotewakeup_enable(void)
{
}

//-------------------------------------------------------------------
void main_remotewakeup_disable(void)
{
}

//-------------------------------------------------------------------
bool main_kbd_enable(void)
{
	//main_b_kbd_enable = true;
	return true;
}

//-------------------------------------------------------------------
bool main_kbd_disable(void)
{
	//main_b_kbd_enable = false; mod 
	return false;
}

//-------------------------------------------------------------------
void main_kbd_change(uint8_t value) 
{	
	//no use in this firmware
}
