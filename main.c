#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>

#define PORT_LED GPIOB
#define PIN_LED GPIO1

/*
#define 	SET_CLOCK()				gpio_set(GPIOA, GPIO0)
#define 	CLEAR_CLOCK()			gpio_clear(GPIOA, GPIO0)
#define 	SET_DATAIN()			gpio_set(GPIOA, GPIO1)
#define 	CLEAR_DATAIN()			gpio_clear(GPIOA, GPIO1)
#define 	SET_STROBE()			gpio_set(GPIOA, GPIO2)
#define 	CLEAR_STROBE()			gpio_clear(GPIOA, GPIO2)
#define		SET_BLANK()				gpio_set(GPIOA, GPIO3)
#define		CLEAR_BLANK()			gpio_clear(GPIOA, GPIO3)
*/

#define 	SET_CLOCK()				gpio_clear(GPIOA, GPIO0)
#define 	CLEAR_CLOCK()			gpio_set(GPIOA, GPIO0)
#define 	SET_DATAIN()			gpio_clear(GPIOA, GPIO1)
#define 	CLEAR_DATAIN()			gpio_set(GPIOA, GPIO1)
#define 	SET_STROBE()			gpio_clear(GPIOA, GPIO2)
#define 	CLEAR_STROBE()			gpio_set(GPIOA, GPIO2)
#define		SET_BLANK()				gpio_clear(GPIOA, GPIO3)
#define		CLEAR_BLANK()			gpio_set(GPIOA, GPIO3)

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

enum BUTTONS {
	BUTTON_1_PRESS = 1,
	BUTTON_1_RELEASE = 8,
	BUTTON_2_PRESS = 2,
	BUTTON_2_RELEASE = 16,
	BUTTON_3_PRESS = 4,
	BUTTON_3_RELEASE = 32
} buttons;

static void rcc_setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_PWR);

	// Enable HSE for the RTC
	RCC_CR |= RCC_CR_HSEON;
	while( (RCC_CR & RCC_CR_HSERDY) == 0)
		__asm__("nop");
}

static void rtc_setup()
{
	pwr_disable_backup_domain_write_protect();

	// Reset RTC domain controller
	RCC_BDCR |= RCC_BDCR_BDRST;
	RCC_BDCR &= ~RCC_BDCR_BDRST;

	// Select HSE
	RCC_BDCR |= RCC_BDCR_RTCSEL_HSE;
	// Enable RTC
	RCC_BDCR |= RCC_BDCR_RTCEN;

	rtc_wait_for_synchro();


	rtc_unlock();
	// Enter init mode
	RTC_ISR |= RTC_ISR_INIT;
	// Wait for init flag
	while( (RTC_ISR & RTC_ISR_INITF) == 0 )
		__asm__("nop");

	// Set the prescaler
	// Using a 8Mhz crystal here.
	rtc_set_prescaler(2000-1, 125-1);

	// Exit init mode
	RTC_ISR &= ~RTC_ISR_INIT;

	rtc_lock();
}

static void gpio_setup(void)
{
	// Outputs
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3);

	// Inputs
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
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
	
	/* Reset prescaler value. */
	timer_set_prescaler(TIM3, 512);

	/* Enable preload. */
	timer_disable_preload(TIM3);

	/* Continous mode. */
	timer_continuous_mode(TIM3);

	timer_set_period(TIM3, 15);

	/* Disable outputs. */
	timer_disable_oc_output(TIM3, TIM_OC1);
	timer_disable_oc_output(TIM3, TIM_OC2);
	timer_disable_oc_output(TIM3, TIM_OC3);
	timer_disable_oc_output(TIM3, TIM_OC4);

	/* Counter enable. */
	timer_enable_counter(TIM3);

	/* Enable commutation interrupt. */
	timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

void clear_digits()
{
	int i;
	for(i=0;i!=9;++i) {
		digits[i] = ' ';
	}
}

void set_hour(uint8_t h)
{
	uint32_t t = 0;
	t=RTC_TR;

	t &= ~( RTC_TR_HU_MASK << RTC_TR_HU_SHIFT);
	t |= ( (h % 10) << RTC_TR_HU_SHIFT);

	t &= ~( RTC_TR_HT_MASK << RTC_TR_HT_SHIFT);
	t |= ( (h / 10) << RTC_TR_HT_SHIFT);



	rtc_unlock();
	RTC_ISR |= RTC_ISR_INIT;
	while( (RTC_ISR & RTC_ISR_INITF) == 0 )
		__asm__("nop");

	RTC_TR = t;

	RTC_ISR &= ~RTC_ISR_INIT;
	rtc_lock();
}

void procces_intput()
{
	static bool b1_triggered = false;
	static bool b2_triggered = false;
	static bool b3_triggered = false;

	if( (buttons & BUTTON_1_PRESS) && (b1_triggered == false) ) {
		set_hour(23);
		gpio_toggle(PORT_LED, PIN_LED);
		b1_triggered = true;
	}

	if(buttons & BUTTON_1_RELEASE) {
		b1_triggered = false;
	}	

	if( (buttons & BUTTON_2_PRESS) && (b2_triggered == false) ) {
		b2_triggered = true;
	}

	if( buttons & BUTTON_2_RELEASE ) {
		b2_triggered = false;
	}

	if( (buttons & BUTTON_3_PRESS) && (b3_triggered == false) ) {
		b3_triggered = true;
	}

	if( buttons & BUTTON_3_RELEASE ) {
		b3_triggered = false;
	}
}


int main(void)
{
	int i;
	rcc_setup();
	rtc_setup();
	gpio_setup();
	tim3_setup();

	while (1) {
		//gpio_toggle(PORT_LED, PIN_LED); /* LED on/off */

		// Buttons
		if (!gpio_get(GPIOA, GPIO5)) {
			buttons |= BUTTON_3_PRESS;
		} else {
			buttons |= BUTTON_3_RELEASE;
		}

		if(!gpio_get(GPIOA, GPIO6)) {
			buttons |= BUTTON_2_PRESS;
		} else {
			buttons |= BUTTON_2_RELEASE;
		}

		if(!gpio_get(GPIOA, GPIO7)) {
			buttons |= BUTTON_1_PRESS;
		} else {
			buttons |= BUTTON_1_RELEASE;
		}

		if ( (buttons ) != 0) {
			procces_intput();
			buttons = 0;
		}

		for (i = 0; i < 10000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
		clear_digits();
		uint8_t s = 0;
		digits[8] = RTC_TR & RTC_TR_SU_MASK;
		digits[7] = (RTC_TR & (RTC_TR_ST_MASK << RTC_TR_ST_SHIFT)) >> RTC_TR_ST_SHIFT;
		digits[6] = 11;
		digits[5] = (RTC_TR & (RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT)) >> RTC_TR_MNU_SHIFT;
		digits[4] = (RTC_TR & (RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT)) >> RTC_TR_MNT_SHIFT;
		digits[3] = 11;
		digits[2] = (RTC_TR & (RTC_TR_HU_MASK << RTC_TR_HU_SHIFT)) >> RTC_TR_HU_SHIFT;
		digits[1] = (RTC_TR & (RTC_TR_HT_MASK << RTC_TR_HT_SHIFT)) >> RTC_TR_HT_SHIFT;
	}

	return 0;
}

inline void timeout()
{
	volatile uint8_t i;
	for(i=0;i!=3;++i)
		__asm__("nop");
}

void tim3_isr(void)
{
	static uint8_t cnt = 0;
	uint8_t s;
	uint32_t data_out;
	uint16_t digit;
	uint32_t i;
	volatile t;

	if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {

		// Clear compare interrupt flag.
		timer_clear_flag(TIM3, TIM_SR_CC1IF);

		//gpio_toggle(PORT_LED, PIN_LED); // LED on/off

		digit = (1 << cnt);
		data_out = digit;
		data_out |= (uint32_t)segments[digits[cnt]] << 10;

		SET_BLANK();
		__asm__("nop");
		__asm__("nop");

		i=20;
		do{
			i--;
			if(data_out & (1 << i) )
				SET_DATAIN();
			else
				CLEAR_DATAIN();
			
				__asm__("nop");
			//set_clk(1);

			SET_CLOCK();

			timeout();
			
			CLEAR_CLOCK();

			timeout();
			
		} while ( i!=0 );
		
		SET_STROBE();
		for (t=0;t!=4;++t)
				__asm__("nop");
		
		CLEAR_STROBE();
		for (t=0;t!=4;++t)
				__asm__("nop");
		
		CLEAR_BLANK();

		cnt++;
		cnt %=10;
		count++;
	}
}

/*
void tim3_isr(void)
{
	static uint8_t cnt = 0;
	uint8_t s;
	uint32_t data_out;
	uint16_t digit;
	uint32_t i;
	volatile t;

	if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {

		// Clear compare interrupt flag.
		timer_clear_flag(TIM3, TIM_SR_CC1IF);

		//gpio_toggle(PORT_LED, PIN_LED); // LED on/off

		digit = (1 << cnt);
		data_out = digit;
		data_out |= (uint32_t)segments[digits[cnt]] << 10;

		SET_BLANK();
		__asm__("nop");
		__asm__("nop");

		i=20;
		do{
			i--;
			if(data_out & (1 << i) )
				SET_DATAIN();
			else
				CLEAR_DATAIN();
			
				__asm__("nop");
			//set_clk(1);
			SET_CLOCK();
			for (t=0;t!=2;++t)
				__asm__("nop");
			
			
			//timeout();
			CLEAR_CLOCK();
			for (t=0;t!=2;++t)
				__asm__("nop");
			//timeout();
		} while ( i!=0 );
		
		SET_STROBE();
		for (t=0;t!=4;++t)
				__asm__("nop");
		
		CLEAR_STROBE();
		for (t=0;t!=4;++t)
				__asm__("nop");
		
		CLEAR_BLANK();

		cnt++;
		cnt %=10;
		count++;
	}
}
*/