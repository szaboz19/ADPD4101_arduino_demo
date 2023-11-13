#ifndef ADPD4101_APP_CONFIG_H_
#define ADPD4101_APP_CONFIG_H_


#include <stddef.h>
#include "adpd4101.h"


/******************************************************************************/
/*************************** Definitions *******************************/
/******************************************************************************/

#define ADPD410X_ACTIVE_TIMESLOTS 8         // number of timeslots (other name can be nomber of channels)
#define ADPD410X_REG_DEFAULT_NR   17        // some settings after timeslot data (normally ca  4*ACTIVE_TIMESLOTS + 1) it was 32
#define ADPD410X_APP_CODE_ODR_DEFAULT	50    // sampling frequency

#define ADPD4101_SUPPORT


/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/


static struct adpd4101_init_param adpd4101_param = {
	.clk_opt = ADPD410X_INTLFO_INTHFO,
	.ext_lfo_freq = 0
};



/* Timeslots Configuration */
static struct adpd4101_timeslot_init ts_init_tab[] = {
	// ********* TS 1 - A **************//
  {
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x10              // 24 mA
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_12K5,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 1,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=false
	},
    // ********* TS 2 - B **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x21              // 50 mA
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_200K,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 1,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=false
	},
    // ********* TS 3 - C **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x01              // 1.5 mA
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_12K5,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 6,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	},
    // ********* TS 4 - D **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x21              // 50 mA
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_12K5,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 16,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	},
    // ********* TS 5 - E **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x02              // 3 mA
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_12K5,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 16,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	},
    // ********* TS 6 - F **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x21              // 50 mA
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_50K,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 16,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	},
    // ********* TS 7 - G **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x01              // 1.5 mA
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_12K5,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 16,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	},
      // ********* TS 8 - H **************//
	{
		.ts_inputs = {
			.option = ADPD410X_INaDIS_INbCH1,
			.pair = ADPD410X_INP12
		},
		.led1 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x00
			}
		},
		.led2 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_A,
				.led_current_select = 0x10              // 24 mA
			}
		},
		.led3 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.led4 = {
			.fields = {
				.led_output_select = ADPD410X_OUTPUT_B,
				.led_current_select = 0x00
			}
		},
		.enable_ch2 = false,
		.precon_option = ADPD410X_TIA_VREF,
		.afe_trim_opt = ADPD410X_TIA_VREF_1V265,
		.vref_pulse_opt = ADPD410X_TIA_VREF_1V265,
		.chan1 = ADPD410X_TIA_GAIN_100K,
    .chan2 = ADPD410X_TIA_GAIN_50K,
		.pulse4_subtract = 0xA,
		.pulse4_reverse = 0xA,
		.byte_no = 4,
		.dec_factor = 0,
		.repeats_no = 16,
		.integ_num=1,
    .integ_off=32,
    .integ_width=3,
    .LED_off=32, 
    .LED_width=2,
    .zero_adjust=true
	}
};

/* Register Configuration (address + data) */
uint16_t reg_config_default[17][2] = {    // it was [32][2] but only 25 data were written ???what???
	/** General configuration */
	{0x0001, 0x000F},

  // 0X0100 subsample: 0, ch2: ???, sample type: 0 standard, input_R_sel: 00 500 Ohm, timeslot_offset: 0x0 
  // 0x0108 MOD_TYPE - default 0 - continous connection
  // 0x010A & 0x010B - offset times set to 32-32us in ~_app.cpp (maybe fine tuning is necessary)

	/** AFE Path */     // 4 - preconditioning, DA - TIA & BPF & integrator & ADC
	{0x0101, 0x40E6},   // 0x40E6 - TIA, INT, ADC
	{0x0121, 0x40E6},
	{0x0141, 0x40DA},
	{0x0161, 0x40DA},
	{0x0181, 0x40DA},
	{0x01A1, 0x40DA},
	{0x01C1, 0x40DA},
	{0x01E1, 0x40DA},

	/** 3us AFE width double sided */
	{0x010A, 0x0803},   // 0x0803 - bit 11 set to 1 --> set integrator as a buffer
	{0x012A, 0x0803},
	{0x014A, 0x0003},
	{0x016A, 0x0003},
	{0x018A, 0x0003},
	{0x01AA, 0x0003},
	{0x01CA, 0x0003},
	{0x01EA, 0x0003},
};





#endif /* ADPD4101_APP_CONFIG_H_ */
