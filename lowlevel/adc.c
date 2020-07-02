#include "theZ.h"
#include "clock.h"
#include "uart.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>


void adc_setup()
{
  //ADC
	rcc_periph_clock_enable(RCC_ADC12);
	rcc_periph_clock_enable(RCC_GPIOA);
	//ADC
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_power_off(ADC1);
	adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_61DOT5CYC);
	uint8_t channel_array[] = { 2 }; /* ADC1_IN1 (PA0) */
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
	adc_power_on(ADC1);
}

uint32_t adc_read()
{
  adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)));
  return adc_read_regular(ADC1);
}


void adc_test_loop()
{
  while(1)
  {
    uart_send_string("ADC ");
    uart_send_int(adc_read());
    uart_send_string("\n");
    delay_ms(500);
  }
}
