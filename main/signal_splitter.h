#pragma once

// GPIO Input pint assignments
#define GPIO_INPUT_SIGNAL (36)              // Projector input signal
#define GPIO_INPUT_MULTI_SINGLE_TOGGLE (32) // BUTTON -- Toggle button to change single/multi option
#define GPIO_INPUT_ADVANCE_FRAME (33)       // BUTTON -- Advance frame for signals
#define GPIO_INPUT_RED_BUTTON (19)          // BUTTON -- red toggle button
#define GPIO_INPUT_GREEN_BUTTON (18)        // BUTTON -- Green toggle button
#define GPIO_INPUT_BLUE_BUTTON (4)          // BUTTON -- Blue toggle button
#define GPIO_INPUT_RED_PWM (39)             // POTENTIOMETER -- Red Analog to Digital Converter(ADC) input range (0 - 4096) 12bits resolution
#define GPIO_INPUT_GREEN_PWM (34)           // POTENTIOMETER -- Green ADC input input range (0 - 4096) 12bits resolution
#define GPIO_INPUT_BLUE_PWM (35)            // POTENTIOMETER -- Blue ADC input input range (0 - 4096) 12bits resolution

// Input pin masking
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_SIGNAL) | (1ULL << GPIO_INPUT_MULTI_SINGLE_TOGGLE) | (1ULL << GPIO_INPUT_ADVANCE_FRAME) | (1ULL << GPIO_INPUT_RED_BUTTON) | (1ULL << GPIO_INPUT_GREEN_BUTTON) | (1ULL << GPIO_INPUT_BLUE_BUTTON) | (1ULL << GPIO_INPUT_RED_PWM) | (1ULL << GPIO_INPUT_GREEN_PWM) | (1ULL << GPIO_INPUT_BLUE_PWM))

// GPIO Output pint assignments
#define GPIO_OUTPUT_SIGNAL (25)    /* Test signal */
#define GPIO_OUTPUT_RED_PWM (23)   // RED OUTPUT SIGNAL -- pwm output signal controlled by red button
#define GPIO_OUTPUT_GREEN_PWM (22) // GREEN OUTPUT SIGNAL -- pwm output signal controlled by green button
#define GPIO_OUTPUT_BLUE_PWM (21)  // BLUE OUTPUT SIGNAL -- pwm output signal controlled by blue button

// Output pin masking
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_RED_PWM) | (1ULL << GPIO_OUTPUT_GREEN_PWM) | (1ULL << GPIO_OUTPUT_BLUE_PWM) | (1ULL << GPIO_OUTPUT_SIGNAL))

#define ESP_INTR_FLAG_DEFAULT 0

// LED Controller parameters
#define LEDC_TIMER LEDC_TIMER_0         // Timer
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // Speed mode
#define LEDC_DUTY_RES LEDC_TIMER_12_BIT // Set duty resolution to 12 bits
#define LEDC_DUTY (2048)                // Set duty to 50%. ((2 ** 12) - 1) * 50% = 2048
#define LEDC_FREQUENCY (10000)          // Frequency in Hertz. Set frequency at 5 kHz

// Timer parameters
#define TIMER_DIVIDER (80)     //  Hardware timer clock divider
#define SYNC_TRIGGER_SIZE (10) // Sample size to sync signal

// Constants
typedef enum
{
    RED_SIGNAL = 0x00,
    GREEN_SIGNAL = 0x01,
    BLUE_SIGNAL = 0x02,
    NO_SIGNAL = 0x03
} out_sig;

void led_on(out_sig);
void led_off(out_sig);
void all_leds_off();
// void green_led_on(void);
// void blue_led_on(void);

void ledc_init(void);
void gpio_init(void);
void timer_setUp(void);

static void button_capture_task(void *arg);
static void advance_frame_task(void *arg);
static void adc_pwm_task(void *arg);
static void multiMode_controller_task(void *arg);

void advance_frame_pressed(bool *singleMode);
void signal_mode_pressed(bool *singleMode);

void red_button_pressed(bool *singleMode);

void green_button_pressed(bool *singleMode);

void blue_button_pressed(bool *singleMode);
