#ifndef _SMP3RegisterEnum
#define _SMP3RegisterEnum

enum SMP3RegisterEnum
	{
	    gamma0 = 0x00,
	    gamma1 = 0x01,
	    pulse_width_half_amp0 = 0x02,
	    pulse_width_half_amp1 = 0x03,
	    pulse_amplitude0 = 0x04,
	    pulse_amplitude1 = 0x05,
	    temp_of_gen = 0x06,
	    temp0 = 0x07,
	    temp1 = 0x08,
	    dummy_command0 = 0x09,
	    dummy_command1 = 0x0A,
	    saw_level = 0x0B,
	    saw_amplitude = 0x0C,
	    input_coef0 = 0x0D,
	    input_coef1 = 0x0E,
	    measure_points = 0x0F,
	    filter_width = 0x10,
	    measures_count = 0x11,
	    speed_register = 0x12,
	    host_address_register = 0x13,
	    valid_request_register = 0x14,
	    crc_error_register = 0x15,
	    exception_register = 0x16,
	    KGT = 0x17,
	    KGU = 0x18,
	    KST0 = 0x19,
	    KST1 = 0x1A,
	    T0_CAL = 0x1B,
	    T1_CAL = 0x1C,
	    TG_CAL = 0x1D
	};

#endif