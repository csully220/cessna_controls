
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <string.h>

#include "csd_usb.h"
#include "csd_gpio.h"

bool csdbg[8];
float csflt[8];
int csint[8];

// Output buffer (must match HID report descriptor)
int8_t outbuf[5];
int8_t swbnk[8] = {0};

double mapOneRangeToAnother(double sourceNumber, double fromA, double fromB, double toA, double toB, int decimalPrecision ) {
    double deltaA = fromB - fromA;
    double deltaB = toB - toA;
    double scale  = deltaB / deltaA;
    double negA   = -1 * fromA;
    double offset = (negA * scale) + toA;
    double finalNumber = (sourceNumber * scale) + offset;
    int calcScale = (int) pow(10, decimalPrecision);
    return (double) round(finalNumber * calcScale) / calcScale;
}

// map x to (a1,a2) from (b1, b2)
int csd_map(int x, int a1, int a2, int b1, int b2) {
    int x1 = (x - b1) * (b2 - b1);
    float x2 = (float)a1 + ((float)(x1)*(float)(a2 - a1));
    return (int)x2;
};


int main(void)
{
	// Initialized explicitly for clarity and debugging
	outbuf[0] = 0; // throttle
	outbuf[1] = 0; // prop
	outbuf[2] = 0; // mixture
	outbuf[3] = 0; // pitch trim
	outbuf[4] = 0; // buttons

	edtc_report.throttle = 0;
	edtc_report.prop = 0;
	edtc_report.mixture = 0;
	edtc_report.pitch_trim = 0;

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	gpio_setup();
	adc_setup();


    /* ADC channels for conversion*/
	uint8_t channel_array[2] = {0};
	/*
	 * This is a somewhat common cheap hack to trigger device re-enumeration
	 * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
	 * setting the pin to output, and driving it explicitly low effectively
	 * "removes" the pullup.  The subsequent USB init will "take over" the
	 * pin, and it will appear as a proper pullup to the host.
	 * The magic delay is somewhat arbitrary, no guarantees on USBIF
	 * compliance here, but "it works" in most places.
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	while (true)
	{
		channel_array[0] = 0;
		adc_set_regular_sequence(ADC1, 1, channel_array);

		//ADC1
		adc_start_conversion_direct(ADC1);
		/* Wait for end of conversion. */
		while (!(adc_eoc(ADC1)))
			continue;
		
		int16_t tk0 = adc_read_regular(ADC1); 
		edtc_report.mixture = (int)mapOneRangeToAnother(tk0, 0 , 4096, -127, 127, 0);
		csint[0] = tk0;
		
		//ADC2
		channel_array[0] = 1;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk1 = adc_read_regular(ADC1);
		edtc_report.prop = (int)mapOneRangeToAnother(tk1, 0 , 4096, -127, 127, 0);
		csint[1] = tk1;

		channel_array[0] = 2;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk2 = adc_read_regular(ADC1);
		edtc_report.throttle = (int)mapOneRangeToAnother(tk2, 0 , 4096, -127, 127, 0);
		csint[2] = tk2;

		channel_array[0] = 3;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk3 = adc_read_regular(ADC1);
		edtc_report.pitch_trim = 30; //(int)mapOneRangeToAnother(tk3, 0 , 2708, -127, 127, 0);
		csint[3] = tk3;

		usbd_poll(usbd_dev);
	}
}

void sys_tick_handler(void)
{
	// index directional switch
	swbnk[0] = !gpio_get(GPIOB, GPIO0); // idx push
	swbnk[1] = 0;
	swbnk[2] = 0;
	swbnk[3] = 0;
	swbnk[4] = 0;
	swbnk[5] = 0;
	swbnk[6] = 0;
	swbnk[7] = 0;
	//swbk1[4] = !gpio_get(GPIOB, GPIO8); // spare
	
	edtc_report.buttons = 0;
	for(int i=0;i<8;i++) {
		edtc_report.buttons |= (swbnk[i] << i);
	}

	memcpy(&outbuf[0], &edtc_report.throttle, 1);
	memcpy(&outbuf[1], &edtc_report.prop, 1);
	memcpy(&outbuf[2], &edtc_report.mixture, 1);
	memcpy(&outbuf[3], &edtc_report.pitch_trim, 1);
	memcpy(&outbuf[4], &edtc_report.buttons, 1);

	usbd_ep_write_packet(usbd_dev, 0x81, &outbuf, 5);

}