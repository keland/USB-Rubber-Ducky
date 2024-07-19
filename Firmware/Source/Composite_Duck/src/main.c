//_____  M A I N ___________________________________________________________
//
// Module		: Ducky
// Description	: Composite Ducky USB HID Keyboard & Mass Storage
// Date			: Jan 1, 2013
// Author       : Snake
// Credit		: ATMEL, Jason Applebaum's Keyscan Method
//__________________________________________________________________________

#include <asf.h>
#include "conf_usb.h"
#include "ui.h"
#include "conf_sd_mmc_spi.h"

// state machine enum
typedef enum injectState {
	state_IDLE,
	state_START_INJECT,
	state_INJECTING,
	state_KEY_DOWN,
	state_KEY_UP,
	state_MOD_DOWN,
	state_MOD_KEY_DOWN,
	state_MOD_KEY_UP,
	state_MOD_UP,
	state_WAIT
} injectState_t;

static bool main_b_keyboard_enable = false;
static bool main_b_msc_enable = false;
char *vidpidFile = "A:\\vidpid.bin";
uint16_t vid;
uint16_t pid;
static uint8_t serial[12];
static uint8_t *serial_p;
static uint8_t serial_len;
static char *injectFile = "A:\\inject.bin";
UDC_DESC_STORAGE usb_dev_desc_t udc_device_desc;
static uint16_t inject_array[4096];

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();

	sysclk_init();
	board_init();
	ui_init();
	ui_powerdown();

 memories_initialization(SD_MMC_SPI_MASTER_SPEED);
 
nav_reset();
if( nav_setcwd( vidpidFile, false, false ) ) {
	file_open(FOPEN_MODE_R);
	file_bof();
	
	vid =  file_getc() | (file_getc() << 8);
	pid =  file_getc() | (file_getc() << 8);
	serial_len = file_getc();
	//serial = (uint8_t *)dlmalloc(serial_len);
	file_read_buf(serial,sizeof(serial_len));
	serial_p = &serial;
	//serial[serial_len]='\0';
	udc_device_desc.idVendor = (vid);
	udc_device_desc.idProduct = (pid);
	udc_device_desc.bcdDevice = 2;
	if (serial_len != NULL){
		//#define USB_DEVICE_GET_SERIAL_NAME_LENGTH serial_len;
		#define USB_DEVICE_SERIAL_NAME serial;
		
	}
	
	file_close();
}

 nav_reset();
 if( nav_setcwd( injectFile, false, false ) ) {
	 file_open(FOPEN_MODE_R);
	 file_seek(0,FS_SEEK_END);
	 uint32_t filesize=file_getpos();
	 file_seek(0,FS_SEEK_SET);
	 int a=0;
	 
	 while(!file_eof()){
		inject_array[a] =  file_getc() | (file_getc() << 8);
		a++;
		if (a==4096){
			return;
		}			
	 }	
	 if (a==(filesize/2)){
		 inject_array[a]= 0xFEFE;
	 }
	 else {
		 inject_array[a]= 0xFEFE;
	}
	file_close();
 }
	//memories_initialization(FOSC0);
	// Start USB stack to authorize VBus monitoring
	udc_start();
	udc_attach();
	 
	// The main loop manages only the power mode
	// because the USB management is done by interrupt
	while (true) {

		if (main_msc_enable) {
			if (!udi_msc_process_trans()) {
				sleepmgr_enter_sleep();
			}
		}else{
			sleepmgr_enter_sleep();
		}
	}
}

void main_vbus_action(bool b_high)
{
	if (b_high) {
		// Attach USB Device
		udc_attach();
	} else {
		// VBUS not present
		udc_detach();
	}
}

void main_suspend_action(void)
{
	ui_powerdown();
}

void main_resume_action(void)
{
	ui_wakeup();
}

void main_sof_action(void)
{
	if ((!main_b_keyboard_enable) )
		return;
	process_frame(udd_get_frame_number());
}

void main_remotewakeup_enable(void)
{
	ui_wakeup_enable();
}

void main_remotewakeup_disable(void)
{
	ui_wakeup_disable();
}

bool main_keyboard_enable(void)
{
	main_b_keyboard_enable = true;
	return true;
}

void main_keyboard_disable(void)
{
	main_b_keyboard_enable = false;
}

bool main_msc_enable(void)
{
	main_b_msc_enable = true;
	return main_b_msc_enable;
}

void main_msc_disable(void)
{
	main_b_msc_enable = false;
	return false;
}


void memories_initialization(long pba_hz)
{
	#if (defined SD_MMC_SPI_MEM) && (SD_MMC_SPI_MEM == ENABLE)
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

	sysclk_enable_peripheral_clock(SD_MMC_SPI);

	// If the SPI used by the SD/MMC is not enabled.
	if (!spi_is_enabled(SD_MMC_SPI)) {
		// Initialize as master.
		spi_initMaster(SD_MMC_SPI, &spiOptions);
		// Set selection mode: variable_ps, pcs_decode, delay.
		spi_selectionMode(SD_MMC_SPI, 0, 0, 0);
		// Enable SPI.
		spi_enable(SD_MMC_SPI);
	}

	// Initialize SD/MMC with SPI PB clock.
	sd_mmc_spi_init(spiOptions,pba_hz);
	#endif  // SD_MMC_SPI_MEM == ENABLE
}

void ui_kbd_led(uint8_t value)
{
	//not used
}

void process_frame(uint16_t framenumber)
{
	static uint8_t cpt_sof = 0;
	static injectState_t state = state_START_INJECT;
	static uint8_t wait = 0;
	static uint16_t debounce = 0;
	static uint16_t injectToken = 0x0000;
	static int a=0;
	
	// scan process running each 2ms
	cpt_sof++;
	if( 2 > cpt_sof )
	return;
	cpt_sof = 0;
	
	// pulse led
	LED_Set_Intensity( LED0, framenumber >> 1 );
	
	// debounce switch
	if( debounce > 0 ) --debounce;
	
	// injection state machine
	switch(state) {

		case state_IDLE:
		// check switch
		LED_Off(LED1);
		if( gpio_get_pin_value(GPIO_JOYSTICK_PUSH) == GPIO_JOYSTICK_PUSH_PRESSED ) {
			
			// debounce
			if( debounce == 0 ) {
				state = state_START_INJECT;
				a=0;
				debounce = 100;
			}
		}
		break;
		
		case state_START_INJECT:
		//file_open(FOPEN_MODE_R);
		state = state_INJECTING;
		break;
		
		case state_INJECTING:
		
		if( a >= (sizeof(inject_array)/sizeof(inject_array[0])) ) {
			//file_close();
			state = state_IDLE;
			break;
		}
		if(injectToken==0x0000) state = state_IDLE;
		injectToken = inject_array[a];
		a++;
		if ((a % 2)==0){
			LED_On(LED1);
		}else{
			LED_Off(LED1);
		}
		if( ( injectToken&0xff ) == 0xFE){
			state=state_IDLE;
			debounce=0;
		}
		else if( ( injectToken&0xff ) == 0x00 ) {
			wait = injectToken>>8;
			state = state_WAIT;
		}
		else if( ( injectToken>>8 ) == 0x00 ) {
			state = state_KEY_DOWN;
		}
		else {
			state = state_MOD_DOWN;
		}
		break;
		
		case state_KEY_DOWN:
		udi_hid_kbd_down(injectToken&0xff);
		state = state_KEY_UP;
		break;

		case state_KEY_UP:
		udi_hid_kbd_up(injectToken&0xff);
		state = state_INJECTING;
		break;
		
		case state_MOD_DOWN:
		udi_hid_kbd_modifier_down(injectToken>>8);
		state = state_MOD_KEY_DOWN;
		break;

		case state_MOD_KEY_DOWN:
		udi_hid_kbd_down(injectToken&0xff);
		state = state_MOD_KEY_UP;
		break;

		case state_MOD_KEY_UP:
		udi_hid_kbd_up(injectToken&0xff);
		state = state_MOD_UP;
		break;
		
		case state_MOD_UP:
		udi_hid_kbd_modifier_up(injectToken>>8);
		state = state_INJECTING;
		break;
		
		case state_WAIT:
		if( --wait == 0 ) {
			state = state_INJECTING;
		}
		break;
		
		default:
		state = state_IDLE;
	}
}