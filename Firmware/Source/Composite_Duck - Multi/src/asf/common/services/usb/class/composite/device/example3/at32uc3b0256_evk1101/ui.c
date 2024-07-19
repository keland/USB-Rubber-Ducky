/**
 * \file
 *
 * \brief User Interface
 *
 * Copyright (c) 2011 - 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <asf.h>
#include "ui.h"

static struct{
	bool b_modifier;
	bool b_down;
	uint8_t u8_value;
} ui_sequence[] = {
	// key sequence here, key_down followed by key_up
	// eg {false,true,key_a}, {false,false,key_a}
};

// Interrupt on "pin change" from switch to do wakeup on USB
// Note:
// This interrupt is enable when the USB host enable remotewakeup feature
// This interrupt wakeup the CPU if this one is in idle mode
ISR(ui_wakeup_isr, AVR32_GPIO0_IRQ_GROUP, 0)
{
	// Clear GPIO interrupts.
	gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
	// Clear External Interrupt Line else Wakeup event always enabled
	eic_clear_interrupt_line(&AVR32_EIC, EXT_INT6);
	// It is a wakeup then send wakeup USB
	udc_remotewakeup();
}


void ui_init(void)
{
	/* register joystick handler on level 0 */
	irqflags_t flags = cpu_irq_save();
	irq_register_handler(ui_wakeup_isr,
			AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_PUSH / 8), 0);
	cpu_irq_restore(flags);

	LED_On(LED0);
	LED_Off(LED1);
}

void ui_powerdown(void)
{
	LED_Off(LED0);
	LED_Off(LED1);
}


//! Structure holding the configuration parameters of the EIC low level driver.
static eic_options_t eic_options = {
	// Enable level-triggered interrupt.
	.eic_mode = EIC_MODE_LEVEL_TRIGGERED,
	// Don't care value because the chosen eic mode is level-triggered.
	.eic_edge = 0,
	// Interrupt will trigger on low-level.
	.eic_level = EIC_LEVEL_LOW_LEVEL,
	// Enable filter.
	.eic_filter = EIC_FILTER_ENABLED,
	// For Wake Up mode, initialize in asynchronous mode
	.eic_async = EIC_ASYNCH_MODE
};


void ui_wakeup_enable(void)
{
	/* configure buttons to produce IT on all state change */
	gpio_enable_pin_interrupt(GPIO_JOYSTICK_PUSH, GPIO_PIN_CHANGE);
	/*
	 * Configure pin change interrupt for asynchronous wake-up (required to
	 * wake up from the STATIC sleep mode).
	 *
	 * First, map the interrupt line to the GPIO pin with the right
	 * peripheral function.
	 */
	gpio_enable_module_pin(GPIO_JOYSTICK_PUSH, AVR32_EIC_EXTINT_6_FUNCTION);

	/*
	 * Enable the internal pull-up resistor on that pin (because the EIC is
	 * configured such that the interrupt will trigger on low-level, see
	 * eic_options.eic_level).
	 */
	gpio_enable_pin_pull_up(GPIO_JOYSTICK_PUSH);

	// Init the EIC controller with the set options.
	eic_options.eic_line = EXT_INT6;
	eic_init(&AVR32_EIC, &eic_options,
			sizeof(eic_options) / sizeof(eic_options_t));
	// Enable External Interrupt Controller Line
	eic_enable_line(&AVR32_EIC, EXT_INT6);
}

void ui_wakeup_disable(void)
{
	eic_disable_line(&AVR32_EIC, EXT_INT6);
	/* Disable joystick input change ITs. */
	gpio_disable_pin_interrupt(GPIO_JOYSTICK_PUSH);
}

void ui_wakeup(void)
{
	LED_On(LED0);
}

void ui_start_read(void)
{
	LED_On(LED1);
}

void ui_stop_read(void)
{
	LED_Off(LED1);
}

void ui_start_write(void)
{
	LED_On(LED0);
}

void ui_stop_write(void)
{
	LED_Off(LED0);
}

void ui_process(uint16_t framenumber)
{
	bool b_btn_state, sucess;
	static bool btn_last_state = false;
	static bool sequence_running = false;
	static uint8_t u8_sequence_pos = 0;
	static uint16_t cpt_sof = 0;
	static uint8_t u8_value=0;

	if ((framenumber % 1000) == 0) {
		LED_On(LED1);
	}
	if ((framenumber % 1000) == 500) {
		LED_Off(LED1);
	}
	// Scan process running each 2ms
	cpt_sof++;
	if ((cpt_sof%2)==0) {
		return;
	}

	// Scan buttons on switch 0 to send keys sequence
	//b_btn_state = (!gpio_get_pin_value(GPIO_PUSH_BUTTON_0)) ? true : false;
	b_btn_state = (!gpio_get_pin_value(GPIO_JOYSTICK_PUSH)) ? true : false;
	if (b_btn_state != btn_last_state) {
		btn_last_state = b_btn_state;
		sequence_running = true;
	}

	// Sequence process running each 200ms
	if (60 > cpt_sof) {
		return;
	}
	cpt_sof = 0;

	if (sequence_running) {
		// Send next key				
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
			// Valid sequence position
			u8_sequence_pos++;
				
			if (u8_sequence_pos>=sizeof(ui_sequence)/sizeof(ui_sequence[0])) {
				u8_sequence_pos=0;
				sequence_running=false;
			}
		
	}

}


/**
 * \defgroup UI User Interface
 *
 * Human interface on EVK1101 :
 * - Led 0 is on when USB line is in IDLE mode, and off in SUSPEND mode
 * - Led 1 blinks when USB host has checked and enabled HID and MSC interfaces
 * - The PB0 open a note pad application on Windows O.S.
 * and send key sequence "Atmel AVR USB Keyboard"
 * - The PB0 can be used to wakeup USB Host in remote wakeup mode.
 *
 */
