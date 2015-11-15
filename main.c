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

struct Time {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} time;

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
	BUTTON_2_PRESS = 2,
	BUTTON_3_PRESS = 4,
	BUTTON_1_RELEASE = 8,
	BUTTON_2_RELEASE = 16,
	BUTTON_3_RELEASE = 32,
	BUTTON_1_IS_PRESSED = 64,
	BUTTON_2_IS_PRESSED = 128,
	BUTTON_3_IS_PRESSED = 256,
	BUTTON_1_IS_RELEASED = 512,
	BUTTON_2_IS_RELEASED = 1024,
	BUTTON_3_IS_RELEASED = 2048

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
	buttons = 0;
	buttons = ( BUTTON_1_IS_RELEASED | BUTTON_2_IS_RELEASED | BUTTON_3_IS_RELEASED);
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

void set_time(uint8_t h, uint8_t m, uint8_t s)
{
	uint32_t t = 0;
	t = RTC_TR;

	t &= ~( RTC_TR_HU_MASK << RTC_TR_HU_SHIFT);
	t |= ( (h % 10) << RTC_TR_HU_SHIFT);

	t &= ~( RTC_TR_HT_MASK << RTC_TR_HT_SHIFT);
	t |= ( (h / 10) << RTC_TR_HT_SHIFT);


	t &= ~( RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT);
	t |= ( (m % 10) << RTC_TR_MNU_SHIFT);

	t &= ~( RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT);
	t |= ( (m / 10) << RTC_TR_MNT_SHIFT);


	t &= ~( RTC_TR_SU_MASK << RTC_TR_SU_SHIFT);
	t |= ( (s % 10) << RTC_TR_SU_SHIFT);

	t &= ~( RTC_TR_ST_MASK << RTC_TR_ST_SHIFT);
	t |= ( (s / 10) << RTC_TR_ST_SHIFT);

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

	if( (buttons & BUTTON_1_PRESS)) {
		set_time( (time.hours + 1) % 24, time.minutes, time.seconds);
		gpio_toggle(PORT_LED, PIN_LED);
	}

	if( buttons & BUTTON_2_PRESS) {
		set_time( time.hours, (time.minutes + 1) % 60, time.seconds);
		gpio_toggle(PORT_LED, PIN_LED);
	}

	if( buttons & BUTTON_3_PRESS ) {
		set_time( time.hours, time.minutes, (time.seconds +1) % 60);
		gpio_toggle(PORT_LED, PIN_LED);
	}

}

void get_buttons_input()
{
	volatile bool bt1 = gpio_get(GPIOA, GPIO7);
	volatile bool bt2 = gpio_get(GPIOA, GPIO6);
	volatile bool bt3 = gpio_get(GPIOA, GPIO5);

	// Buttons
	if ( (bt1 == false) && ( buttons & BUTTON_1_IS_RELEASED) ) {
		buttons |= (BUTTON_1_PRESS | BUTTON_1_IS_PRESSED);
		buttons &= ~(BUTTON_1_IS_RELEASED);

	} else if ( (bt1 == true) && (buttons & BUTTON_1_IS_PRESSED) ){
		buttons |= (BUTTON_1_RELEASE | BUTTON_1_IS_RELEASED);
		buttons &= ~(BUTTON_1_IS_PRESSED);
	}

	if ( (bt2 == false) && ( buttons & BUTTON_2_IS_RELEASED) ) {
		buttons |= (BUTTON_2_PRESS | BUTTON_2_IS_PRESSED);
		buttons &= ~(BUTTON_2_IS_RELEASED);

	} else if ( (bt2 == true) && (buttons & BUTTON_2_IS_PRESSED) ){
		buttons |= (BUTTON_2_RELEASE | BUTTON_2_IS_RELEASED);
		buttons &= ~(BUTTON_2_IS_PRESSED);
	}

	if ( (bt3 == false) && ( buttons & BUTTON_3_IS_RELEASED) ) {
		buttons |=  (BUTTON_3_PRESS | BUTTON_3_IS_PRESSED);
		buttons &= ~(BUTTON_3_IS_RELEASED);

	} else if ( (bt3 == true) && (buttons & BUTTON_3_IS_PRESSED) ){
		buttons |= (BUTTON_3_RELEASE | BUTTON_3_IS_RELEASED);
		buttons &= ~(BUTTON_3_IS_PRESSED);
	}
	procces_intput();
	// Remove single shot button events
	buttons &= ~(BUTTON_3_PRESS | BUTTON_3_RELEASE | BUTTON_2_PRESS | BUTTON_2_RELEASE | BUTTON_1_PRESS | BUTTON_1_RELEASE);
}

void get_time()
{
	uint32_t time_register = RTC_TR;
	time.seconds = time_register & RTC_TR_SU_MASK;
	time.seconds += ((time_register & (RTC_TR_ST_MASK << RTC_TR_ST_SHIFT)) >> RTC_TR_ST_SHIFT) * 10;
	time.minutes = (time_register & (RTC_TR_MNU_MASK << RTC_TR_MNU_SHIFT)) >> RTC_TR_MNU_SHIFT;
	time.minutes += ((time_register & (RTC_TR_MNT_MASK << RTC_TR_MNT_SHIFT)) >> RTC_TR_MNT_SHIFT) *10;
	time.hours = (time_register & (RTC_TR_HU_MASK << RTC_TR_HU_SHIFT)) >> RTC_TR_HU_SHIFT;
	time.hours += ((time_register & (RTC_TR_HT_MASK << RTC_TR_HT_SHIFT)) >> RTC_TR_HT_SHIFT) * 10;
}

void show_time()
{
	digits[8] = time.seconds % 10;
	digits[7] = time.seconds / 10;
	digits[6] = 11;
	digits[5] = time.minutes % 10;
	digits[4] = time.minutes / 10;
	digits[3] = 11;
	digits[2] = time.hours % 10;
	digits[1] = time.hours / 10;
}

int main(void)
{
	int i;
	rcc_setup();
	rtc_setup();
	gpio_setup();
	tim3_setup();


	while (1) {

		get_buttons_input();

		for (i = 0; i < 5000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
		//clear_digits();
		get_time();
		show_time();

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
