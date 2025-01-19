/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * This examples creates a USB Microphone device using the TinyUSB
 * library and captures data from a PDM microphone using a sample
 * rate of 16 kHz, to be sent the to PC.
 * 
 * The USB microphone code is based on the TinyUSB audio_test example.
 * 
 * https://github.com/hathach/tinyusb/tree/master/examples/device/audio_test
 */

#include "usb_microphone.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "apdm.pio.h"

// variables
uint16_t sample_buffer[2][SAMPLE_BUFFER_SIZE] = {0};

volatile struct {
	unsigned char v:1;
} sample_buffer_no;

uint32_t offset = 0;

uint pio_sm;
uint ping_dma_chan;
uint pong_dma_chan;
uint tick_dma_chan;

// callback functions
void on_usb_microphone_tx_ready();

	static void setup_dma_const(int dma_chan, uint32_t * val)
	{
		dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_32);
	    channel_config_set_read_increment(&cfg_dma_chan_config, false);
	    channel_config_set_write_increment(&cfg_dma_chan_config, false);
	    
		int timer = dma_claim_unused_timer(true /* required */);
		dma_timer_set_fraction(timer, SAMPLE_RATE/4000, clock_get_hz(clk_sys)/4000);  // divide system clock by num/denom
		int treq = dma_get_timer_dreq(timer);
		channel_config_set_dreq(&cfg_dma_chan_config, treq);

		dma_channel_set_config(dma_chan, &cfg_dma_chan_config, false);

	    dma_channel_set_write_addr(dma_chan, &pio0->txf[pio_sm], false);
	    dma_channel_set_read_addr(dma_chan, val, false);
	    dma_channel_set_trans_count(dma_chan, 0xFFFFFFFF, false);
	}

	static void setup_dma_cont(uint16_t* buff, int dma_chan, int next_dma_chan)
	{
		dma_channel_config cfg_dma_chan_config = dma_channel_get_default_config(dma_chan);
	    channel_config_set_transfer_data_size(&cfg_dma_chan_config, DMA_SIZE_16);
	    channel_config_set_read_increment(&cfg_dma_chan_config, false);
	    channel_config_set_write_increment(&cfg_dma_chan_config, true);
	    channel_config_set_dreq(&cfg_dma_chan_config, pio_get_dreq(pio0, pio_sm, false));
	    channel_config_set_chain_to(&cfg_dma_chan_config, next_dma_chan);
	    dma_channel_set_write_addr(dma_chan, buff, false);
	    dma_channel_set_read_addr(dma_chan, &((uint16_t*) &pio0->rxf[pio_sm])[0], false);
	    dma_channel_set_trans_count(dma_chan, SAMPLE_BUFFER_SIZE, false);
	    dma_channel_set_config(dma_chan, &cfg_dma_chan_config, false);
	    dma_irqn_set_channel_enabled(0, dma_chan, 1);
	}

	static uint16_t* get_buff()
		{
			return sample_buffer[++sample_buffer_no.v];
		}

	static void __not_in_flash_func(isr)()
	{
		if (dma_irqn_get_channel_status(0, ping_dma_chan)) {
			uint16_t* buff = get_buff();
				setup_dma_cont(buff,ping_dma_chan,pong_dma_chan);
			dma_irqn_acknowledge_channel(0, ping_dma_chan);
		}
		if (dma_irqn_get_channel_status(0, pong_dma_chan)) {
			uint16_t* buff = get_buff();
				setup_dma_cont(buff,pong_dma_chan,ping_dma_chan);
			dma_irqn_acknowledge_channel(0, pong_dma_chan);
		}
	}

int main(void)
{
  // initialize and start the PDM microphone

	pio_sm = pio_claim_unused_sm(pio0,true);
	apdm_input_program_init(pio0,pio_sm,pio_add_program(pio0, &apdm_input_program),15);
	ping_dma_chan = dma_claim_unused_channel(true);
	pong_dma_chan = dma_claim_unused_channel(true);
	tick_dma_chan = dma_claim_unused_channel(true);
    pio_sm_set_enabled(pio0,pio_sm,1);


		{
			int16_t* ibuff = get_buff();
			setup_dma_cont(ibuff,ping_dma_chan,pong_dma_chan);
			ibuff = get_buff();
			setup_dma_cont(ibuff,pong_dma_chan,ping_dma_chan);
			irq_add_shared_handler(DMA_IRQ_0,isr,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
		    dma_irqn_set_channel_enabled(0, ping_dma_chan, 1);
		    dma_irqn_set_channel_enabled(0, pong_dma_chan, 1);
		    dma_channel_acknowledge_irq0(ping_dma_chan);
		    dma_channel_acknowledge_irq0(pong_dma_chan);
		    irq_set_enabled(DMA_IRQ_0, true);
		    dma_channel_start(ping_dma_chan);
		}

	setup_dma_const(tick_dma_chan,&offset);
	dma_channel_start(tick_dma_chan);


  // initialize the USB microphone interface
  usb_microphone_init();
  usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

  while (1) {
    // run the USB microphone task continuously
    usb_microphone_task();
  }

  return 0;
}


void on_usb_microphone_tx_ready()
{
  // Callback from TinyUSB library when all data is ready
  // to be transmitted.
  //
  // Write local buffer to the USB microphone
  usb_microphone_write(sample_buffer[sample_buffer_no.v], sizeof(*sample_buffer));
}
