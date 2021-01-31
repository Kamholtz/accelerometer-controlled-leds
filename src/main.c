#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>



// #define RUN_EXAMPLE 1

#ifdef RUN_EXAMPLE

#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
// #define LEDC_HS_CH0_GPIO       (GPIO_NUM_34)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (19)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

void app_main(void)
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
#ifdef CONFIG_IDF_TARGET_ESP32
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
#elif defined CONFIG_IDF_TARGET_ESP32S2
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
#endif
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1) {
        printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("2. LEDC fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("4. LEDC set duty = 0 without fade\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}



#else

#define ADC_MAX 4095.0
#define TIMER_RES_MAX 8191
#define NUM_SAMPLES_IN_AVG 50
#define SAMP_DELAY_S 0.01
#define FADE_S 2

#define MIDPOINT ((float) 1.5)
#define MOVEMENT_THRESH ((float) 0.05)
#define V_DIFF_MAX ((float) 1.5)
#define SENSITIVITY_PERC (0.25)

void configure_led_pwm() {

	ledc_timer_config_t timer_config =
	{
		.duty_resolution = LEDC_TIMER_13_BIT,
		.freq_hz = 5000,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.clk_cfg = LEDC_AUTO_CLK
	};
	esp_err_t err = ledc_timer_config(&timer_config);

	printf("Timer error: %d\n", err);

	ledc_channel_config_t channel_config =
	{
		.channel = LEDC_CHANNEL_0,         /*!< LEDC channel (0 - 7) */
		.duty = TIMER_RES_MAX/2,                  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
		.gpio_num = GPIO_NUM_18,                   /*!< the LEDC output gpio_num, if you want to use gpio16, gpio_num = 16 */
		.speed_mode = LEDC_LOW_SPEED_MODE,         /*!< LEDC speed speed_mode, high-speed mode or low-speed mode */
		// .intr_type,     /*!< configure interrupt, Fade interrupt enable  or Fade interrupt disable */
		.timer_sel = LEDC_TIMER_0,         /*!< Select the timer source of channel (0 - 3) */
		.hpoint = 0,                     /*!< LEDC channel hpoint value, the max value is 0xfffff */
	};
	err = ledc_channel_config(&channel_config);

	printf("Channel error: %d\n", err);
}

float get_adc_voltage(int adc_raw) {
	float adc_volt = adc_raw/ADC_MAX * 3.3;
	return adc_volt;
}

void app_main()
{
	// ADC setup
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

	// LEDC PWM setup
	configure_led_pwm();

	// Initialize fade service.
    ledc_fade_func_install(0);

	int duty = 0;

	while (1) {

		float total = 0;

		// Averge
		for (int ii = 0; ii < NUM_SAMPLES_IN_AVG; ii++) {
			int val = adc1_get_raw(ADC1_CHANNEL_0);
			float adc_volt = get_adc_voltage(val);

			total += adc_volt;
		}

		float avg = total/NUM_SAMPLES_IN_AVG;

		// Difference
		float diff = avg - MIDPOINT;
		float diff_abs = fabs(diff);

		printf("AVG: %.2f\n", avg);
		printf("Diff: %.2f\n", diff);
		printf("Diff ABS: %.2f\n", diff_abs);

		if (diff_abs > MOVEMENT_THRESH)
		{
			int duty_add = (diff_abs/V_DIFF_MAX * TIMER_RES_MAX) * SENSITIVITY_PERC;
			printf("Duty Add: %d\n", duty_add);

			duty += duty_add;
		}

		if (duty > TIMER_RES_MAX) {
			duty = TIMER_RES_MAX;
		}

		printf("Duty to set: %d, as percentage: %.2f\n", duty, (((float) duty)/TIMER_RES_MAX) * 100);

		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

		// ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 3000);

        vTaskDelay(SAMP_DELAY_S / portTICK_PERIOD_MS);

		duty -= TIMER_RES_MAX/(FADE_S/SAMP_DELAY_S);

		if (duty < 0) {
			duty = 0;
		}
	}
}
#endif
