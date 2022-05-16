
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <string.h>
#include <math.h>


#include "csd_usb.h"
#include "csd_gpio.h"
#include "servo.h"

#define SRVLAG 24

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
	if(x > b2) x = b2;
	if(x < b1) x = b1;
    int x1 = (x - b1) * (b2 - b1);
    float x2 = (float)a1 + ((float)(x1)*(float)(a2 - a1));

    int xr = (int)x2;
	if(xr > a2) xr = a2;
	if(xr < a1) xr = a1;

    return xr;
};

void csd_delay(int d)
{

     int i;

     for(i = 0; i < d; i++) {
          __asm("nop");
     }
}

int servo_dc = 0; // servo delay counter
const int servo_dd = 200; // servo delay duration

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
	servo_init();
	
	servo_set_position(SERVO_CH1, SERVO_NULL);

    /* ADC channels for conversion*/
	uint8_t channel_array[2] = {0};
	//channel_array[1] = 0;
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
	csd_delay(800000);
	/*for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}*/

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	//enum {MIXTURE}
	
	uint32_t servo_pos_arr[SRVLAG] = {1550};


	while (true)
	{
		
		channel_array[0] = 6;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk0 = adc_read_regular(ADC1); 
		int tmp_mix = (int)mapOneRangeToAnother(tk0, 0 , 4095, -127, 127, 0);
		if(tmp_mix > 127) tmp_mix = 127;
		if(tmp_mix < -127) tmp_mix = -127;
		edtc_report.mixture = tmp_mix;

		channel_array[0] = 3;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk1 = adc_read_regular(ADC1);
		int tmp_prop = (int)mapOneRangeToAnother(tk1, 0 , 4095, -127, 127, 0);
		if(tmp_prop > 127) tmp_prop = 127;
		if(tmp_prop < -127) tmp_prop = -127;
		edtc_report.prop = tmp_prop;

		channel_array[0] = 4;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk2 = adc_read_regular(ADC1);
		int tmp_throttle = (int)mapOneRangeToAnother(tk2, 0 , 4095, -127, 127, 0);
		if(tmp_throttle > 127) tmp_throttle = 127;
		if(tmp_throttle < -127) tmp_throttle = -127;
		edtc_report.throttle = tmp_throttle;

		channel_array[0] = 5;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk3 = adc_read_regular(ADC1);
		int tmp_trim = (int)mapOneRangeToAnother(tk3, 0 , 3200, -127, 127, 0);
		if(tmp_trim > 127) tmp_trim = 127;
		if(tmp_trim < -127) tmp_trim = -127;
		edtc_report.pitch_trim = tmp_trim;

		if(servo_dc >= servo_dd) {
			float et_pos = ((((float)tmp_trim + 127.0)) / 255.0); // elevator trim indicator position
            const int et_max = 1780;
			const int et_min = 1180;
			const int et_neutral = 1550;
			int et_range = et_max - et_min;
			uint32_t servo_pos = (et_pos * et_range) + et_min;

			//#define SERVO_MAX		(2050)
			/**
			 * Min. pos. at 950  us (0.95ms).
			 */
			//#define SERVO_MIN		(950)
			/**
			 * Middle pos. at 1580 us (1.58ms).
			 */
			//#define SERVO_NULL		(1500)
			uint32_t servo_pos_sw[SRVLAG];
			uint32_t servo_pos_now = servo_pos_arr[0];
			//memcpy(servo_pos_sw, servo_pos_arr, SRVLAG);
			for (int i=0; i<SRVLAG; i++) {
				servo_pos_sw[i] = servo_pos_arr[i];
			}

			for (int i=0; i<SRVLAG - 1; i++) {
				servo_pos_arr[i] = servo_pos_sw[i+1];
			}
			servo_pos_arr[SRVLAG-1] = servo_pos;
			//csint[0] = servo_pos_now;
			
			servo_set_position(SERVO_CH1, servo_pos_now);

			servo_dc = 0;
		}
		servo_dc++;

	// index directional switch
	swbnk[0] = !gpio_get(GPIOB, GPIO0); // idx push
	swbnk[1] = !gpio_get(GPIOB, GPIO10);
	swbnk[2] = !gpio_get(GPIOB, GPIO11);
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

		usbd_poll(usbd_dev);
	}
}

void sys_tick_handler(void)
{


	usbd_ep_write_packet(usbd_dev, 0x81, &outbuf, 5);

}