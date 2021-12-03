/* Signal Splitter*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/ledc_reg.h"
#include "hal/ledc_ll.h"
#include "signal_splitter.h"
#include "driver/timer.h"

// Task handles
static TaskHandle_t xTaskAdvanceFrameHandle = NULL;
static TaskHandle_t xTaskButtonCaptureHandle = NULL;
static QueueHandle_t xAdvanceFrameQueue = NULL;

// Iterrupt handlers
static void IRAM_ATTR isr_signal_handler(void *arg);
static void IRAM_ATTR isr_advance_frame_handler(void *arg);
static void IRAM_ATTR isr_signal_mode_handler(void *arg);
static void IRAM_ATTR isr_red_button_handler(void *arg);
static void IRAM_ATTR isr_green_button_handler(void *arg);
static void IRAM_ATTR isr_blue_button_handler(void *arg);

void app_main(void)
{

    // Set the LEDC peripheral configuration
    gpio_init();

    // Create Queue handle
    xAdvanceFrameQueue = xQueueCreate(10, sizeof(uint32_t));

    if (xAdvanceFrameQueue == NULL)
    {
        printf("[ERROR]: Could not be create queue");
    }

    //start gpio task
    xTaskCreate(advance_frame_task, "advance_frame_task", 2048 * 2, NULL, 11, &xTaskAdvanceFrameHandle);
    xTaskCreate(button_capture_task, "button_capture_task", 2048 * 2, NULL, 10, &xTaskButtonCaptureHandle);
    xTaskCreate(adc_pwm_task, "adc_pwm_task", 2048 * 2, NULL, 9, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_SIGNAL, isr_signal_handler, NULL);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_MULTI_SINGLE_TOGGLE, isr_signal_mode_handler, NULL);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_ADVANCE_FRAME, isr_advance_frame_handler, NULL);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_RED_BUTTON, isr_red_button_handler, NULL);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_GREEN_BUTTON, isr_green_button_handler, NULL);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_BLUE_BUTTON, isr_blue_button_handler, NULL);

    // configure led controller parameters
    ledc_init();

    // configure timer
    timer_init();
}

/* 
 * Interrupt function handler for the Projector input signal. It it a hardware interrupt 
 * which gets trigger at the raising edge for the input. It then sends a notification to 
 * the advance frame task.
*/
static void IRAM_ATTR isr_signal_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(xTaskAdvanceFrameHandle,    /* Pointer to task handle to notify */
                       0x01,                       /* Value to update Notification */
                       eSetBits,                   /* Action: eSetBits -> set bits */
                       &xHigherPriorityTaskWoken); /* Will be set to true if succesfully unbloked a task*/
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 * Interrupt function handler for the advance frame BUTTON. It it a hardware interrupt 
 * which gets trigger when the advance BUTTON is pressed. It then sends a notification to 
 * the Button capture task.
*/
static void IRAM_ATTR isr_advance_frame_handler(void *arg)
{
    // Send advance notification to the advace frame task
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(xTaskButtonCaptureHandle,
                       0x01,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 * Interrupt function handler for the BUTTON to change from multi mode to single mode. It it a 
 * hardware interrupt which gets trigger when the multi/single mode BUTTON is pressed. It then sends 
 * a notification to the Button capture task.
*/
static void IRAM_ATTR isr_signal_mode_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t buttonPressed = 0x02;

    xTaskNotifyFromISR(xTaskButtonCaptureHandle,   /* Pointer to task handle to notify */
                       buttonPressed,              /* Value to update Notification */
                       eSetBits,                   /* Action: eSetBits -> set bits */
                       &xHigherPriorityTaskWoken); /* Will be set to true if succesfully unbloked a task*/

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 * Interrupt function handler for the RED BUTTON to toggle the red signal. It it a 
 * hardware interrupt which gets trigger when the RED BUTTON is pressed. It then sends 
 * a notification to the Button capture task.
*/
static void IRAM_ATTR isr_red_button_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t buttonPressed = 0x10;

    xTaskNotifyFromISR(xTaskButtonCaptureHandle,
                       buttonPressed,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 * Interrupt function handler for the GREEN BUTTON to toggle the green signal. It it a 
 * hardware interrupt which gets trigger when the GREEN BUTTON is pressed. It then sends 
 * a notification to the Button capture task.
*/
static void IRAM_ATTR isr_green_button_handler(void *arg)
{
    // send taskNotification to button capture
    // button caprture pointer to handler
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t buttonPressed = 0x20;

    xTaskNotifyFromISR(xTaskButtonCaptureHandle,
                       buttonPressed,
                       eSetBits,
                       &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 * Interrupt function handler for the BLUE BUTTON to toggle the blue signal. It it a 
 * hardware interrupt which gets trigger when the BLUE BUTTON is pressed. It then sends 
 * a notification to the Button capture task.
*/
static void IRAM_ATTR isr_blue_button_handler(void *arg)
{
    // send taskNotification to button capture
    // button caprture pointer to handler
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t buttonPressed = 0x40;

    xTaskNotifyFromISR(xTaskButtonCaptureHandle,
                       buttonPressed,
                       eSetBits,
                       &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* 
 *
 * This function initializes the input and output gpio pins 
 *
*/
void gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt for output pins
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge for all the input pins
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
}

/*
 * This function initializes the LED controller. Here each output pwm signal 
 * comes form its own led channel as follows. 
 * Red signal -- channel 0
 * Greeb signal -- channel 1
 * Blue signal -- channel 2
 * Output frequency for all channels is 5 kHz
*/
void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_OUTPUT_RED_PWM,
        .duty = LEDC_DUTY, /* Set duty to 50% */
        .hpoint = 0};

    // set the red led channel configuration
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = GPIO_OUTPUT_GREEN_PWM;

    // set the green led channel coinfiguration
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = GPIO_OUTPUT_BLUE_PWM;

    // set the blue led channel configuration
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
void timer_init(void)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_MAX,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    //timer_enable_intr(group, timer);

    // example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    // timer_info->timer_group = group;
    // timer_info->timer_idx = timer;
    // timer_info->auto_reload = auto_reload;
    // timer_info->alarm_interval = timer_interval_sec;
    // timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);
}

static void advance_frame_task(void)
{
    /* Reciving notifications:
         * PROJECTOR SIGNAL:        SIGNAL_ISR = 0X01
         * ADVANCE FRAME BUTTON:    ADV_FRAME = 0x02
         * TIME INTERRUPT
    */

    uint32_t ulInterruptStatus = 0x00;
    uint32_t current_frame = 0x03;
    uint64_t timer_val = 0;
    uint64_t last_timer_val = 0;
    uint32_t missed_intervals = SYNC_SAMPLE_SIZE;
    uint64_t sync_sample_array[SYNC_SAMPLE_SIZE];
    bool synchronized = false;
    bool timer_started = false;

    const uint64_t tg_interval_time = 5555;                  // set the frame with to 5,555 cycles for a 1MHz clock to get 180Hz
    const uint64_t allowed_error = (tg_interval_time * 0.1); // set allowed error to 10%
    int64_t error = -tg_interval_time;                       // this variable will hold the error from the interupt

    // set timer to reset every 5655 to 5755 cycles

    for (;;)
    {
        if (xAdvanceFrameQueue != NULL)
        {
            if (xQueueReceive(xAdvanceFrameQueue, &ulInterruptStatus, portMAX_DELAY)) // delay time to one second so that we pause the clock if no signal is recieved
            {
                if (ulInterruptStatus & 0x01)
                {
                    /* Start timer if is not running */
                    if (!timer_started)
                    {
                        timer_start(TIMER_GROUP_0, TIMER_0);
                        timer_started = true;
                    }

                    /* Updatae timer value */
                    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val);

                    /* Synchronize signla if unsynchronized */
                    if (!synchronized)
                    {
                        if (missed_intervals > 0)
                        {
                            sync_sample_array[SYNC_SAMPLE_SIZE - missed_intervals] = timer_val;
                            missed_intervals--;
                        }
                        else
                        {
                            synchronized = synchronize_sig(&last_timer_val, sync_sample_array, array_size);
                        }
                        continue;
                    }

                    if (timer_val < allowed_error || timer_val > (tg_interval_time - allowed_error))
                    {
                        error = timer_val - tg_interval_time;

                        if (abs(error) < allowed_error)
                        {
                            // reset interrupt timer
                            //

                            current_frame = current_frame % 3;
                            current_frame++;
                        }
                    }
                    else if ()
                    {
                    }
                }
                else if (ulInterruptStatus & 0x02)
                {
                }
                else if (ulInterruptStatus & 0x04)
                {
                }
                else
                {
                    printf("[ERROR]: There was an error in advance_frame_task() function");
                }
            }
        }
    }
}

/*
 *
 *
 *
*/
static void advance_frame_task(void *arg)
{
    uint32_t ulInterruptStatus;
    uint32_t current_frame;
    uint32_t notify_msg;
    out_sig sm_led = RED_SIGNAL;
    bool multy_mode = true;

    // double curr_time;
    // double min_error_time;
    ulInterruptStatus = 0x00; // message sent to this task
    current_frame = 0x03;     // 0x01 is red: 0x02 is green: 0x04 is blue

    for (;;)
    {
        /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification. This notification recieves notifications from 
        Timer interrup, signal interrupt and button interrupt */

        /* Reciving notifications:
         * PROJECTOR SIGNAL:        SIGNAL_ISR = 0X01
         * ADVANCE FRAME BUTTON:    ADV_BTN = 0x02
         * SINGLE/MULTIPLE BUTTON:  SM_BTN_S = 0x04
         * multy mode               SM_BTN_M = 0x80
         * LED ON FOR SINGLE MODE:  SM_LED_RED = 0x08
         * LED ON FOR SINGLE MODE:  SM_LED_GREEN = 0x10 
         * LED ON FOR SINGLE MODE:  SM_LED_BLUE = 0x20
         * TIMER INTERRUPT:         TIMER_ISR = 0x40
         * 
         * 
        */

        xTaskNotifyWait(0x00,               /* Don't clear any bits on entry. */
                        ULONG_MAX,          /* Clear all bits on exit. ULONG_MAX will clear all bits on exit */
                        &ulInterruptStatus, /* Receives the notification value. */
                        portMAX_DELAY);     /* Block indefinitely. */

        //printf("Advance frame notificaton: %d\n", ulInterruptStatus);

        // first copy the notification message  and record the time from the timer;
        notify_msg = ulInterruptStatus;
        // get_time_from_timer();

        if (notify_msg & 0x04)
        {
            multy_mode = false;
            //printf("Set multy_mode to:  %x\n", multy_mode);
        }
        else if (notify_msg & 0x80)
        {
            multy_mode = true;
            //printf("Set multy_mode to:  %x\n", multy_mode);
        }
        else if (notify_msg & 0x08)
        {
            sm_led = RED_SIGNAL;
        }
        else if (notify_msg & 0x10)
        {
            sm_led = GREEN_SIGNAL;
        }
        else if (notify_msg & 0x20)
        {
            sm_led = BLUE_SIGNAL;
        }
        else
        {
            if (multy_mode)
            {
                if (notify_msg & 0x01)
                {
                    // check if it has been 180Hz since last frame change
                    // comper timer with 180Hz;
                    all_leds_off();
                    current_frame = current_frame % 3;
                    //printf("[MULTY_MODE]: Frame changed by input signal to:  %x\n", current_frame);
                    if (current_frame == RED_SIGNAL)
                    {
                        led_on(RED_SIGNAL);
                        //printf("turn red on");
                    }
                    else if (current_frame == GREEN_SIGNAL)
                    {
                        led_on(GREEN_SIGNAL);
                    }
                    else if (current_frame == BLUE_SIGNAL)
                    {
                        led_on(BLUE_SIGNAL);
                    }
                    current_frame++;
                }
                else if (notify_msg & 0x02)
                {
                    all_leds_off();
                    current_frame = current_frame % 3;
                    //printf("Frame changed by button to:  %x\n", current_frame);
                    if (current_frame == RED_SIGNAL)
                    {
                        led_on(RED_SIGNAL);
                    }
                    else if (current_frame == GREEN_SIGNAL)
                    {
                        led_on(GREEN_SIGNAL);
                    }
                    else if (current_frame == BLUE_SIGNAL)
                    {
                        led_on(BLUE_SIGNAL);
                    }
                    current_frame++;
                }
                else
                {
                    //printf("Unknown notification !!!!!!!!!\n");
                }
            }
            else
            {
                if (notify_msg & 0x01)
                {
                    // check if it has been 180Hz since last frame change
                    // comper timer with 180Hz;

                    current_frame = current_frame % 3;
                    current_frame++;

                    led_off(sm_led);

                    //printf("[SINGLE_MODE]: Frame changed by input signal to:  %x\n", sm_led);
                    led_on(sm_led);
                }
                else
                {
                    //printf("Unknown behavior: ??????   ---  %x\n", notify_msg);
                }
            }
        }

        /* 
         * This function is notify when the an interrup is triggered. There are three diferent interupts 
         * that triger this task. 
         * Main signal interrupt - this is the projector signal
         * Timer interrupt - this signal gets triggerd every 180Hz
         * Advance frame button interrupt - this is the button to manually advance a frame
         * 
        */

        // First we need to identify the source of the notification

        // if (ulInterruptStatus == SIGNAL_ISR)
        // {
        // Get time elapsed since last interrupt
        // Check whether it is close to the 180Hz cycle
        // check how logn it has been since the last frame change
        // if the interrup is within the 180Hz time then change the frame and reset the timer
        // check the difference last_frame_change - current_interrup_trigger = time_elapsed ----> we want time elapsed to be
        // as close to 180Hz as posible. we can achive this by updating time_elapsed at every interrupt
        // we want to choose the interrupt to change frames as the one that gives the smallest (time_elapsed - 1/(180))
        // we will exept a 10% error for the frame change
        //}
    }
}

// TODO: this function needs to be modify to ensure the pins that are off are not floting and set to low
void led_on(out_sig signal)
{
    if (signal == RED_SIGNAL)
    {
        // do this
        gpio_hold_dis(GPIO_OUTPUT_RED_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0);
    }
    else if (signal == GREEN_SIGNAL)
    {
        // turn on green lingght
        gpio_hold_dis(GPIO_OUTPUT_GREEN_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1);
    }
    else if (signal == BLUE_SIGNAL)
    {
        // turn on blue signal
        gpio_hold_dis(GPIO_OUTPUT_BLUE_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2);
    }
    else
    {
        //printf("There was an error in LED_ON  value: %x \n", signal);
    }
    //printf("\nOn -red before register %x\n", REG_READ(LEDC_LSCH0_CONF0_REG));
}

void led_off(out_sig signal)
{
    if (signal == RED_SIGNAL)
    {
        // do this
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0, false);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0);
        //gpio_set_level(GPIO_OUTPUT_RED_PWM, 0);
        gpio_hold_en(GPIO_OUTPUT_RED_PWM);
    }
    else if (signal == GREEN_SIGNAL)
    {
        // turn on green lingght
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1, false);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1);
        //gpio_set_level(GPIO_OUTPUT_GREEN_PWM, 0);
        gpio_hold_en(GPIO_OUTPUT_GREEN_PWM);
    }
    else if (signal == BLUE_SIGNAL)
    {
        // turn on blue signal
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2, false);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2);
        //gpio_set_level(GPIO_OUTPUT_BLUE_PWM, 0);
        gpio_hold_en(GPIO_OUTPUT_BLUE_PWM);
    }
    else
    {
        //printf("There was an error in LED_ON \n");
    }
    //printf("\nOn -red before register %x\n", REG_READ(LEDC_LSCH0_CONF0_REG));
}

/* Turn all led off. This should happen every 180Hz ***/
void all_leds_off(void)
{
    ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0, false);
    ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1, false);
    ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2, false);

    ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0);
    ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1);
    ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2);
    gpio_hold_en(GPIO_OUTPUT_RED_PWM);
    gpio_hold_en(GPIO_OUTPUT_GREEN_PWM);
    gpio_hold_en(GPIO_OUTPUT_BLUE_PWM);
}

static void adc_pwm_task(void *arg)
{
    /* ADC configuration */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db);
    int redValRed = 0;
    int greenValRed = 0;
    int blueValRed = 0;
    int redCurr = 0;
    int greenCurr = 0;
    int blueCurr = 0;

    for (;;)
    {
        // TODO: try to poll every second if is not being used, other wise poll every 250ms for 10 seconds
        //        make the change from curr reading from previous readin to be about 100 for the check that
        //        happens every second. just incase there is a lot of noise.
        redValRed = adc1_get_raw(ADC1_CHANNEL_3);
        greenValRed = adc1_get_raw(ADC1_CHANNEL_6);
        blueValRed = adc1_get_raw(ADC1_CHANNEL_7);

        if (abs(redValRed - redCurr) > 10)
        {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, redValRed));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
            redCurr = redValRed;
        }
        if (abs(greenValRed - greenCurr) > 10)
        {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, greenValRed));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
            greenCurr = greenValRed;
        }
        if (abs(blueValRed - blueCurr) > 10)
        {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, blueValRed));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));
            blueCurr = blueValRed;
        }
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

/* TODO: Write a description of the function */
void button_capture_task(void *arg)
{
    uint32_t ulInterButtonStatus;
    bool redEnable;
    bool greenEnable;
    bool blueEnable;
    bool singleMode;

    redEnable = true;
    greenEnable = true;
    blueEnable = true;
    singleMode = false;
    ulInterButtonStatus = 0x00;

    for (;;)
    {
        /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification. */
        xTaskNotifyWait(0x00,                 /* Don't clear any bits on entry. */
                        ULONG_MAX,            /* Clear all bits on exit. ULONG_MAX will clear all bits*/
                        &ulInterButtonStatus, /* Receives the notification value. */
                        portMAX_DELAY);       /* Block indefinitely. */

        //printf("\nButton Capture Function: \n");
        if (ulInterButtonStatus & 0x01)
        {
            /* If the advance frame button was pressed check send a 
                notidication to the advance frame task 
            */
            gpio_intr_disable(GPIO_INPUT_ADVANCE_FRAME);
            xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                        0x0a,                    /* Value to update Notification */
                        eSetBits);               /* Action: eSetBits -> set bits */

            vTaskDelay(500 / portTICK_RATE_MS);
            gpio_intr_enable(GPIO_INPUT_ADVANCE_FRAME);
            continue;
        }

        if ((ulInterButtonStatus >> 1) & 0x01)
        {
            gpio_intr_disable(GPIO_INPUT_MULTI_SINGLE_TOGGLE);
            singleMode = !singleMode;
            if (singleMode)
            {
                //gpio_intr_disable(GPIO_INPUT_SIGNAL);   Do not disable signal on sigle mode
                gpio_intr_disable(GPIO_INPUT_RED_BUTTON);    // Disable red button intrrupt
                gpio_intr_disable(GPIO_INPUT_ADVANCE_FRAME); // disable advance frame button interrup
                gpio_hold_dis(GPIO_OUTPUT_RED_PWM);          // disabling the gpio pin level hold on all three outputs
                gpio_hold_dis(GPIO_OUTPUT_GREEN_PWM);
                gpio_hold_dis(GPIO_OUTPUT_BLUE_PWM);
                all_leds_off();
                led_on(RED_SIGNAL); // turn red led on
                redEnable = true;
                greenEnable = false;
                blueEnable = false;
                xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                            0x04,                    /* Value to update Notification */
                            eSetBits);               /* Action: eSetBits -> set bits */
            }
            else
            {
                //printf("changing to multy mode: buton pushed: \n");
                gpio_hold_dis(GPIO_OUTPUT_RED_PWM);
                gpio_hold_dis(GPIO_OUTPUT_GREEN_PWM);
                gpio_hold_dis(GPIO_OUTPUT_BLUE_PWM);
                redEnable = true;
                greenEnable = true;
                blueEnable = true;
                gpio_intr_enable(GPIO_INPUT_SIGNAL);
                gpio_intr_enable(GPIO_INPUT_ADVANCE_FRAME);
                gpio_intr_enable(GPIO_INPUT_RED_BUTTON);
                gpio_intr_enable(GPIO_INPUT_GREEN_BUTTON);
                gpio_intr_enable(GPIO_INPUT_BLUE_BUTTON);
                xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                            0x80,                    /* Value to update Notification */
                            eSetBits);               /* Action: eSetBits -> set bits */
            }

            vTaskDelay(500 / portTICK_RATE_MS);
            gpio_intr_enable(GPIO_INPUT_MULTI_SINGLE_TOGGLE);
            continue;
        }

        if (singleMode)
        {
            if ((ulInterButtonStatus >> 4) & 0x01)
            {
                // red button was pressed
                gpio_intr_disable(GPIO_INPUT_RED_BUTTON);

                if (greenEnable)
                {
                    led_off(GREEN_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_GREEN_BUTTON);
                }
                if (blueEnable)
                {
                    led_off(BLUE_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_BLUE_BUTTON);
                }
                led_on(RED_SIGNAL);
                redEnable = true;
                greenEnable = false;
                blueEnable = false;
                xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                            0x08,                    /* Value to update Notification */
                            eSetBits);               /* Action: eSetBits -> set bits */
            }
            else if ((ulInterButtonStatus >> 5) & 0x01)
            {
                // green button was pressed
                gpio_intr_disable(GPIO_INPUT_GREEN_BUTTON);

                if (redEnable)
                {
                    led_off(RED_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_RED_BUTTON);
                }
                if (blueEnable)
                {
                    led_off(BLUE_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_BLUE_BUTTON);
                }
                led_on(GREEN_SIGNAL);
                redEnable = false;
                greenEnable = true;
                blueEnable = false;
                xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                            0x10,                    /* Value to update Notification */
                            eSetBits);               /* Action: eSetBits -> set bits */
            }
            else if ((ulInterButtonStatus >> 6) & 0x01)
            {
                // blue button was pressed
                gpio_intr_disable(GPIO_INPUT_BLUE_BUTTON);

                if (greenEnable)
                {
                    led_off(GREEN_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_GREEN_BUTTON);
                }
                if (redEnable)
                {
                    led_off(RED_SIGNAL);
                    gpio_intr_enable(GPIO_INPUT_RED_BUTTON);
                }
                led_on(BLUE_SIGNAL);
                redEnable = false;
                greenEnable = false;
                blueEnable = true;
                xTaskNotify(xTaskAdvanceFrameHandle, /* Pointer to task handle to notify */
                            0x20,                    /* Value to update Notification */
                            eSetBits);               /* Action: eSetBits -> set bits */
            }
        }
        else
        {
            if ((ulInterButtonStatus >> 4) & 0x01)
            {
                /* red button was pressed */
                gpio_intr_disable(GPIO_INPUT_RED_BUTTON);
                redEnable = !redEnable;
                if (redEnable)
                {
                    gpio_hold_dis(GPIO_OUTPUT_RED_PWM);
                }
                else
                {
                    led_off(RED_SIGNAL);
                    // ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0, false);
                    // ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0);
                    // gpio_hold_en(GPIO_OUTPUT_RED_PWM);
                }
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_intr_enable(GPIO_INPUT_RED_BUTTON);
            }
            else if ((ulInterButtonStatus >> 5) & 0x01)
            {
                /* green button was pressed */
                gpio_intr_disable(GPIO_INPUT_GREEN_BUTTON);
                greenEnable = !greenEnable;
                if (greenEnable)
                {
                    gpio_hold_dis(GPIO_OUTPUT_GREEN_PWM);
                }
                else
                {
                    led_off(GREEN_SIGNAL);
                    // ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1, false);
                    // ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1);
                    // gpio_hold_en(GPIO_OUTPUT_GREEN_PWM);
                }
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_intr_enable(GPIO_INPUT_GREEN_BUTTON);
            }
            else if ((ulInterButtonStatus >> 6) & 0x01)
            {
                /* blue button was pressed */
                gpio_intr_disable(GPIO_INPUT_BLUE_BUTTON);
                blueEnable = !blueEnable;
                if (blueEnable)
                {
                    gpio_hold_dis(GPIO_OUTPUT_BLUE_PWM);
                }
                else
                {
                    led_off(BLUE_SIGNAL);
                    // ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2, false);
                    // ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2);
                    // gpio_hold_en(GPIO_OUTPUT_BLUE_PWM);
                }
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_intr_enable(GPIO_INPUT_BLUE_BUTTON);
            }
        }
    }
}
