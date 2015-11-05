#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#define PORT_LED GPIOB
#define PIN_LED GPIO1


#define 	SET_CLOCK()				gpio_set(GPIOA, GPIO0)
#define 	CLEAR_CLOCK()			gpio_clear(GPIOA, GPIO0)
#define 	SET_DATAIN()			gpio_set(GPIOA, GPIO1)
#define 	CLEAR_DATAIN()			gpio_clear(GPIOA, GPIO1)
#define 	SET_STROBE()			gpio_set(GPIOA, GPIO2)
#define 	CLEAR_STROBE()			gpio_clear(GPIOA, GPIO2)
#define		SET_BLANK()				gpio_set(GPIOA, GPIO3)
#define		CLEAR_BLANK()			gpio_clear(GPIOA, GPIO3)

static uint8_t digits[9] = {10,9,8,7,6,5,4,3,2};

static int count = -200000;

static const uint8_t segments[12] = {
	0b11101101,		// '0'
	0b01100000,		// '1'
	0b11001110,		// '2'
	0b11101010,		// '3'
	0b01100011,		// '4'
	0b10101011,		// '5'
	0b10101111,		// '6'
	0b11100000,		// '7'
	0b11101111,		// '8'
	0b11101011,		// '9'
	0b00000000,		// ' '
	0b00000010,		// '-'
};

static void rcc_setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM3);
}

static void gpio_setup(void)
{
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3);
}

static void tim3_setup(void)
{
	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM3_IRQ);

	/* Reset TIM2 peripheral. */
	timer_reset(TIM3);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	//timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_ACTIVE);
	/* Reset prescaler value. */
	timer_set_prescaler(TIM3, 1024);
	//TIM3_PSC = 1024;

	/* Enable preload. */
	timer_disable_preload(TIM3);

	/* Continous mode. */
	timer_continuous_mode(TIM3);

	timer_set_period(TIM3, 20);

	/* Disable outputs. */
	timer_disable_oc_output(TIM3, TIM_OC1);
	timer_disable_oc_output(TIM3, TIM_OC2);
	timer_disable_oc_output(TIM3, TIM_OC3);
	timer_disable_oc_output(TIM3, TIM_OC4);

	/* Configure global mode of line 1. */
	// timer_disable_oc_clear(TIM3, TIM_OC1);
	// timer_disable_oc_preload(TIM3, TIM_OC1);
	// timer_set_oc_slow_mode(TIM3, TIM_OC1);
	// timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_FROZEN);

	//TIM3_CCR1=200;

	/* Counter enable. */
	timer_enable_counter(TIM3);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}


int main(void)
{
	int i;
	rcc_setup();
	gpio_setup();
	tim3_setup();

	//nvic_enable_irq(NVIC_TIM3_IRQ);

	/* Blink the LED (PC8) on the board. */
	while (1) {
		//gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */
		for (i = 0; i < 500000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}


void tim3_isr(void)
{
	static uint8_t cnt = 0;
	uint8_t s;
	uint32_t data_out;
	uint16_t digit;
	uint32_t i;

	if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM3, TIM_SR_CC1IF);

		gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */



		digit = (1 << cnt);
		data_out = digit;
		data_out |= (uint32_t)segments[digits[cnt]] << 10;

		//set_bl(1);
		SET_BLANK();

		i=20;
		do{
			i--;
			//set_di(data_out & (1 << i) );
			if(data_out & (1 << i) )
				SET_DATAIN();
			else
				CLEAR_DATAIN();
			
			//set_clk(1);
			SET_CLOCK();
			//timeout();
			//set_clk(0);
			CLEAR_CLOCK();
			//timeout();
		} while ( i!=0 );
		
		//set_str(1);
		SET_STROBE();
		//timeout();
		//set_str(0);
		CLEAR_STROBE();
		//timeout();
		//set_bl(0);
		CLEAR_BLANK();

		cnt++;
		cnt %=10;
		count++;
	}
}
