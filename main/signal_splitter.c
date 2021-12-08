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

/* Queue handels */
static QueueHandle_t xButtonCaptureQueue = NULL;
static QueueHandle_t xAdvanceFrameQueue = NULL;
static QueueHandle_t xSingleModeControlQueue = NULL;
static QueueHandle_t xMultiModeControlQueue = NULL;

/* Task handels */
static TaskHandle_t xSingleModeTaskHandle = NULL;
static TaskHandle_t xMultiModeTaskHandle = NULL;

/* Iterrupt Servicies Routing callback functions */
static void IRAM_ATTR isr_signal_handler(void *arg);
static void IRAM_ATTR isr_advance_frame_handler(void *arg);
static void IRAM_ATTR isr_signal_mode_handler(void *arg);
static void IRAM_ATTR isr_red_button_handler(void *arg);
static void IRAM_ATTR isr_green_button_handler(void *arg);
static void IRAM_ATTR isr_blue_button_handler(void *arg);

/**
 * @brief Main app function for setup and initialization happens
 */
void app_main(void)
{

    /* Set the LEDC peripheral configuration */
    gpio_init();

    /* Create Queue handles */
    xAdvanceFrameQueue = xQueueCreate(10, sizeof(uint32_t));
    xButtonCaptureQueue = xQueueCreate(10, sizeof(uint32_t));
    xSingleModeControlQueue = xQueueCreate(10, sizeof(uint32_t));
    xMultiModeControlQueue = xQueueCreate(10, sizeof(uint32_t));

    /* Verify that the queue handles were succesfully created */
    if (xAdvanceFrameQueue == NULL || xButtonCaptureQueue == NULL || xSingleModeControlQueue == NULL || xMultiModeControlQueue == NULL)
    {
        printf("[ERROR]: Could not be create queue");
    }

    /* install gpio isr service */
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

    /* configure led controller parameters */
    ledc_init();

    /* configure timer */
    timer_setUp();

    /* ADC configuration */
    adc1_config_width(ADC_WIDTH_BIT_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db);
    adc1_config_width(ADC_WIDTH_BIT_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);
    adc1_config_width(ADC_WIDTH_BIT_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db);

    /* Create application tasks */
    xTaskCreate(advance_frame_task, "advance_frame_task", 2048 * 2, NULL, 12, NULL);
    xTaskCreate(multiMode_controller_task, "multiMode_controller_task", 2048 * 4, NULL, 11, &xMultiModeTaskHandle);
    xTaskCreate(singleMode_controller_task, "singleMode_controller_task", 2048 * 4, NULL, 11, &xSingleModeTaskHandle);
    xTaskCreate(button_capture_task, "button_capture_task", 2048 * 2, NULL, 10, NULL);
    xTaskCreate(adc_pwm_task, "adc_pwm_task", 2048 * 2, NULL, 9, NULL);
}

/** 
 * @brief Interrupt function handler for the Projector input signal. It it a hardware interrupt 
 *        which gets trigger at the raising edge for the input. It then sends a notification to 
 *        the advance frame task.
 */
static void IRAM_ATTR isr_signal_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    uint32_t sig_msg = 0x01;
    xQueueSendFromISR(xAdvanceFrameQueue, &sig_msg, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/**  
 * @brief Interrupt function handler for the advance frame BUTTON. It it a hardware interrupt 
 *        which gets trigger when the advance BUTTON is pressed. It then sends a notification to 
 *        the Button capture task.
 */
static void IRAM_ATTR isr_advance_frame_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    uint32_t buttonPressed = 0x01;

    xQueueSendFromISR(xButtonCaptureQueue, &buttonPressed, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/** 
 * @brief Interrupt function handler for the BUTTON to change from multi mode to single mode. It it a 
 *        hardware interrupt which gets trigger when the multi/single mode BUTTON is pressed. It then sends 
 *        a notification to the Button capture task.
 */
static void IRAM_ATTR isr_signal_mode_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    uint32_t buttonPressed = 0x02;
    xQueueSendFromISR(xButtonCaptureQueue, &buttonPressed, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/** 
 * @brief Interrupt function handler for the RED BUTTON to toggle the red signal. It it a 
 *        hardware interrupt which gets trigger when the RED BUTTON is pressed. It then sends 
 *        a notification to the Button capture task.
 */
static void IRAM_ATTR isr_red_button_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    uint32_t buttonPressed = 0x10;
    xQueueSendFromISR(xButtonCaptureQueue, &buttonPressed, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/** 
 * @brief Interrupt function handler for the GREEN BUTTON to toggle the green signal. It it a 
 *        hardware interrupt which gets trigger when the GREEN BUTTON is pressed. It then sends 
 *        a notification to the Button capture task.
 */
static void IRAM_ATTR isr_green_button_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    uint32_t buttonPressed = 0x20;
    xQueueSendFromISR(xButtonCaptureQueue, &buttonPressed, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/** 
 * @brief Interrupt function handler for the BLUE BUTTON to toggle the blue signal. It it a 
 *        hardware interrupt which gets trigger when the BLUE BUTTON is pressed. It then sends 
 *        a notification to the Button capture task.
*/
static void IRAM_ATTR isr_blue_button_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    uint32_t buttonPressed = 0x40;
    xQueueSendFromISR(xButtonCaptureQueue, &buttonPressed, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief: This function initializes the input and output gpio pins 
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

/**
 * @brief This function initializes the LED controller. Here each output pwm signal 
 *        comes form its own led channel as follows. 
 * 
 * @b Red_signal --> Linked to channel 0
 * @b Greeb_signal --> Linked to channel 1
 * @b Blue_signal --> Linked to channel 2
 * @p Output_frequency --> For all channels is 10 kHz
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
 * @param group Timer Group number is 0
 * @param timer timer ID, index is 0
 * @param auto_reload whether auto-reload on alarm event
 */
void timer_setUp(void)
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
}

/**
 * @brief This task converts the analog input from the potentiometer to digital numbers. 
 *        The function polls the input every half a sencond and checks for changes greater 
 *        10 and updates the corresponding duty cycle for the led controller signal as necessary. 
 * 
 * @b redValRed Holds the digital value read from the () pin analog input
 * @b greenValRed Holds the digital value read from the () pin analog input
 * @b blueValRed Holds the digital value read from the () pin analog input
 */
static void adc_pwm_task(void *arg)
{
    int redValRed = 0;
    int greenValRed = 0;
    int blueValRed = 0;
    int redCurr = LEDC_DUTY;
    int greenCurr = LEDC_DUTY;
    int blueCurr = LEDC_DUTY;

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
            /* Update red_led pwm signal output */
            //printf("RedPWM: %d\n", abs(redValRed));
            set_update_duty(LEDC_CHANNEL_0, redValRed);
            redCurr = redValRed;
        }

        if (abs(greenValRed - greenCurr) > 10)
        {
            /* Update green_led pwm signal output */
            //printf("GreenPWM: %d\n", abs(greenValRed));
            set_update_duty(LEDC_CHANNEL_1, greenValRed);
            greenCurr = greenValRed;
        }

        if (abs(blueValRed - blueCurr) > 10)
        {
            /* Update blue_led pwm signal output */
            //printf("BluePWM: %d\n", abs(blueValRed));
            set_update_duty(LEDC_CHANNEL_2, blueValRed);
            blueCurr = blueValRed;
        }
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

/**
 * @brief This task converts the analog input from the potentiometer to digital numbers. 
 *        The function polls the input every half a sencond and checks for changes greater 
 *        10 and updates the corresponding pwm signal as necessary. 
 * 
 * @param led_channel LED channel to be updated. 
 * @param value Digital number to update the duty cycle.
 */
void set_update_duty(ledc_channel_t led_channel, int value)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, led_channel, value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channel));
}

/**
 * @brief This task converts the analog input from the potentiometer to digital numbers. 
 *        The function polls the input every half a sencond and checks for changes greater 
 *        10 and updates the corresponding pwm signal as necessary. 
 * 
 * @b current_frame Holds the value of the current active frame.
 * @b missed_intervals Then number of missed invervals before the signal has to be resynchronize.
 * @b timer_val Holds the timer value at which the current signal interrupt is triggered.
 * @b last_timer_val Holds the timer value of the last signal interrupt.
 * @b error Holds the error the signal interval.
 * @b tg_interval_time Is the expected value for the signal interval set to @p 5555 cycles for a 1MHz clock to get 180Hz
 * @b allowed_error This is the allowd error to trigger a change of frame, which is set to 10%
 * @p ulInterruptStatus Holds the value of the mesage send through the queue with the follwing values:
 *              @b PROJECTOR_SIGNAL         @c 0X01
 *              @b ADVANCE_FRAME_BUTTON     @c 0x02
 *              @b SUSPEND_QUEUE            @c 0x03
 *              @b RESUME_QUEUE             @c 0x04
 */
static void advance_frame_task(void *arg)
{
    uint32_t ulInterruptStatus = 0x00;
    uint32_t current_frame = 0x03;
    uint32_t missed_intervals = SYNC_TRIGGER_SIZE;
    uint32_t noise_signal = 0;

    uint64_t timer_val = 0;
    uint64_t last_timer_val = 0;
    int64_t error = 0;
    const uint64_t tg_interval_time = 5555;
    const uint64_t allowed_error = (tg_interval_time * 0.1);

    bool synchronized = false;
    bool timer_started = false;
    bool suspendQueue = false;

    for (;;)
    {
        if (xAdvanceFrameQueue != NULL)
        {
            if (xQueueReceive(xAdvanceFrameQueue, &ulInterruptStatus, (TickType_t)100)) // wait for 100 ticks before stopping the timer
            {
                if (ulInterruptStatus == 0x01)
                {
                    /* Start timer if is not running */
                    if (!timer_started)
                    {
                        timer_start(TIMER_GROUP_0, TIMER_0);
                        //printf("*** timer started ***\n");
                        timer_started = true;
                    }

                    /* Updatae timer value */
                    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val);

                    /* Synchronize signal if unsynchronized */
                    if (!synchronized)
                    {
                        last_timer_val = timer_val;
                        current_frame += missed_intervals;
                        current_frame = current_frame % 3;
                        missed_intervals = 0;
                        error = tg_interval_time;
                        synchronized = true;
                        //printf("Synch current frame: %d\n", current_frame);
                        if (!suspendQueue)
                        {
                            xQueueSend(xMultiModeControlQueue, &current_frame, (TickType_t)10);
                        }
                        continue;
                    }

                    uint32_t interval_width = timer_val - last_timer_val; // interval from last confirmed time and most reason clock value
                    //printf("frame interval:  %d\n", interval_width);
                    //printf("Timer value  %lld\n", timer_val);

                    //printf("timer val:  %lld\n", timer_val);

                    if (interval_width >= (tg_interval_time - allowed_error) && interval_width <= (tg_interval_time + allowed_error))
                    {
                        /* If the interval with is with in the expected error, we update the frame and notify the led controller 
                            The frame is only updated once per interval */

                        if (missed_intervals == 1)
                        {
                            current_frame++;
                        }

                        current_frame++;
                        current_frame = current_frame % 3;

                        if (!suspendQueue)
                        {
                            xQueueSend(xMultiModeControlQueue, &current_frame, (TickType_t)10);
                        }

                        error = interval_width - tg_interval_time;
                        last_timer_val = timer_val;
                        missed_intervals = 0;
                        // printf("Error val:  %lld\n", error);
                    }
                    else if (interval_width < allowed_error)
                    {
                        /* Check if the current signal is better and update the las timer value, if it does */
                        if (abs(error + interval_width) < abs(error))
                        {
                            error = error + interval_width;
                            last_timer_val = timer_val;
                        }
                        noise_signal++; // Keep track of noise signal for test purposes
                    }
                    else if (interval_width > (tg_interval_time + allowed_error))
                    {
                        /* if the we are out of sync missed intervals will grow and set synchronized to false */

                        missed_intervals += (interval_width / tg_interval_time);
                        interval_width = interval_width % tg_interval_time;
                        last_timer_val = timer_val - interval_width;
                        if (missed_intervals > SYNC_TRIGGER_SIZE)
                        {
                            missed_intervals++;
                            synchronized = false;
                        }
                        //printf("missed iterval:  %d\n", missed_intervals);
                        //printf("Current frame:  %d\n", current_frame);
                    }
                    else
                    {
                        noise_signal++;
                        //printf("[MESSAGE]: Noise signal:  %d \n", noise_signal);
                        //printf("frame interval:  %d\n", interval_width);
                    }
                }
                else if (ulInterruptStatus == 0x02)
                {
                    /* advance frame from button press */
                    current_frame++;
                    current_frame = current_frame % 3;
                    if (!suspendQueue)
                    {
                        xQueueSend(xMultiModeControlQueue, &current_frame, (TickType_t)10);
                    }
                }
                else if (ulInterruptStatus == 0x03)
                {
                    suspendQueue = true;
                }
                else if (ulInterruptStatus == 0x04)
                {
                    suspendQueue = false;
                }
                else
                {
                    printf("[ERROR]: There was an error in advance_frame_task() function");
                }
            }
            else
            {
                if (timer_started)
                {
                    timer_pause(TIMER_GROUP_0, TIMER_0);
                    timer_started = false;
                    //printf("*** timer has stoped ***\n");
                }
            }
        }
    }
}

/**
 * @brief The main function of this task is to control the multi mode led output signal.
 *        It recives messages from advance_frame_task and button_capture_task in order to 
 *        turn on and off the corresponding signals and to activate or deactivate a signal.
 * 
 * @b activeLed Holds the value of the frame that is currently on. 
 * @b {COLOR}LedDis If value is true, the corresponding color value is disable and output will alaws ben low.
 * @p ledMessage Holds the message value recived from other task as follows:
 *              @b RED_LED_ON           @c 0x00
 *              @b GREEN_LED_ON         @c 0x01
 *              @b BLEU_LED_ON          @c 0x02
 *              @b RED_EN_DIS           @c 0x10
 *              @b GREEN_EN_DIS         @c 0x11
 *              @b BLUE_EN_DIS          @c 0x12
 *              @b START_MULTI_MODE     @c 0x20
 */
static void multiMode_controller_task(void *arg)
{
    uint32_t ledMessage = 0x0;
    out_sig activeLed = RED_SIGNAL;
    bool redLedDis = false;
    bool greenLedDis = false;
    bool blueLedDis = false;

    for (;;)
    {
        if (xMultiModeControlQueue != NULL)
        {
            if (xQueueReceive(xMultiModeControlQueue, &ledMessage, portMAX_DELAY))
            {
                //printf("recived message: %x\n", ledMessage);
                if (ledMessage == 0x00)
                {
                    // // TESTING
                    // if (activeLed != BLUE_SIGNAL)
                    // {
                    //     printf("*****************Frame BLUE skipped");
                    // }
                    // // END TESTING
                    if (activeLed != NO_SIGNAL)
                    {
                        if (!redLedDis)
                        {
                            led_off(activeLed);
                            activeLed = RED_SIGNAL;
                            led_on(activeLed);
                        }
                        else
                        {
                            led_off(activeLed);
                            activeLed = NO_SIGNAL;
                        }
                    }
                    else
                    {
                        if (!redLedDis)
                        {
                            activeLed = RED_SIGNAL;
                            led_on(activeLed);
                        }
                    }
                }
                else if (ledMessage == 0x01)
                {
                    // if (activeLed != RED_SIGNAL)
                    // {
                    //     printf("*****************Frame RED skipped");
                    // }
                    if (activeLed != NO_SIGNAL)
                    {
                        if (!greenLedDis)
                        {
                            led_off(activeLed);
                            activeLed = GREEN_SIGNAL;
                            led_on(activeLed);
                        }
                        else
                        {
                            led_off(activeLed);
                            activeLed = NO_SIGNAL;
                        }
                    }
                    else
                    {
                        if (!greenLedDis)
                        {
                            activeLed = GREEN_SIGNAL;
                            led_on(activeLed);
                        }
                    }
                }
                else if (ledMessage == 0x02)
                {
                    // if (activeLed != GREEN_SIGNAL)
                    // {
                    //     printf("*****************Frame GREEN skipped");
                    // }
                    if (activeLed != NO_SIGNAL)
                    {
                        if (!blueLedDis)
                        {
                            led_off(activeLed);
                            activeLed = BLUE_SIGNAL;
                            led_on(activeLed);
                        }
                        else
                        {
                            led_off(activeLed);
                            activeLed = NO_SIGNAL;
                        }
                    }
                    else
                    {
                        if (!blueLedDis)
                        {
                            activeLed = BLUE_SIGNAL;
                            led_on(activeLed);
                        }
                    }
                }
                else if (ledMessage == 0x10)
                {
                    redLedDis = !redLedDis;
                }
                else if (ledMessage == 0x11)
                {
                    greenLedDis = !greenLedDis;
                }
                else if (ledMessage == 0x12)
                {
                    blueLedDis = !blueLedDis;
                }
                else if (ledMessage == 0x20)
                {
                    all_leds_off();
                    activeLed = NO_SIGNAL;
                    redLedDis = false;
                    greenLedDis = false;
                    blueLedDis = false;
                }
            }
        }
    }
}

/**
 * @brief The main function of this task is to operate the signal in single mode. This only involves 
 *        only one led output signal. The user chooses from three different outputs. This will produce
 *        a single ouput from one of the output signals with frequency @c LED_FREQUENCY
 * 
 * @b activeLed Holds the value of the active led currently on.
 * @p ledMessage Holds the message value recived from other task as follows:
 *              @b RED_LED_ON           @c 0x00
 *              @b GREEN_LED_ON         @c 0x01
 *              @b BLEU_LED_ON          @c 0x02
 *              @b START_SINGLE_MODE    @c 0x03
 *        
*/
static void singleMode_controller_task(void *arg)
{
    uint32_t ledMessage = 0x00;
    out_sig activeLed = RED_SIGNAL;

    for (;;)
    {
        if (xSingleModeControlQueue != NULL)
        {
            if (xQueueReceive(xSingleModeControlQueue, &ledMessage, portMAX_DELAY))
            {
                if (ledMessage == 0x00 && activeLed != RED_SIGNAL)
                {
                    led_off(activeLed);
                    activeLed = RED_SIGNAL;
                    led_on(activeLed);
                }
                else if (ledMessage == 0x01 && activeLed != GREEN_SIGNAL)
                {
                    led_off(activeLed);
                    activeLed = GREEN_SIGNAL;
                    led_on(activeLed);
                }
                else if (ledMessage == 0x02 && activeLed != BLUE_SIGNAL)
                {
                    led_off(activeLed);
                    activeLed = BLUE_SIGNAL;
                    led_on(activeLed);
                }
                else if (ledMessage == 03)
                {
                    all_leds_off();
                    activeLed = RED_SIGNAL;
                    led_on(activeLed);
                }
                else
                {
                    printf("[ERROR]: There was an error in single_mode taskn\n");
                }
            }
        }
    }
}

/**
 * @brief It turns on the specified led ouput signal.
 * 
 * @param signal The desired signal to be turned on.
*/
void led_on(out_sig signal)
{
    if (signal == RED_SIGNAL)
    {
        // turn on red light
        //printf("in led_on RED\n");
        gpio_hold_dis(GPIO_OUTPUT_RED_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_0);
    }
    else if (signal == GREEN_SIGNAL)
    {
        // turn on green light
        gpio_hold_dis(GPIO_OUTPUT_GREEN_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_1);
    }
    else if (signal == BLUE_SIGNAL)
    {
        // turn on blue signal
        //printf("in led_on BLUE\n");
        gpio_hold_dis(GPIO_OUTPUT_BLUE_PWM);
        ledc_ll_set_sig_out_en(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2, true);
        ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, LEDC_CHANNEL_2);
    }
    else
    {
        printf("There was an error in LED_ON  value: %x \n", signal);
    }
    //printf("\nOn -red before register %x\n", REG_READ(LEDC_LSCH0_CONF0_REG));
}

/**
 * @brief It turns off the specified led ouput signal.
 * 
 * @param signal The desired signal to be turned off.
*/
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
        // turn on green light
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
        printf("There was an error in LED_ON \n");
    }
    //printf("\nOn -red before register %x\n", REG_READ(LEDC_LSCH0_CONF0_REG));
}

/**
 * @brief It turns off all the led ouput signals.
*/
void all_leds_off(void)
{
    led_off(RED_SIGNAL);
    led_off(GREEN_SIGNAL);
    led_off(BLUE_SIGNAL);
}

/**
 * @brief This task captures all the button functionos of the device and sends a
 *        messages to the corresponding task function to process the request.
 * 
 * @b singleMode Holds wether the system is in single or multi mode
 * @p ulInterButtonStatus It holds the message sent from the corresponding button interrupt as follows:
 *              @b ADVANCE_FRAME_BTN        @p 0x01
 *              @b SINGLE_MULTI_MODE_BTN    @p 0x02
 *              @b RED_BTN                  @p 0x10
 *              @b GREE_BTN                 @p 0x20
 *              @b BLUE_BTN                 @p 0x40
*/
void button_capture_task(void *arg)
{
    uint32_t ulInterButtonStatus = 0x00;
    bool singleMode = false;
    vTaskSuspend(xSingleModeTaskHandle);

    for (;;)
    {
        /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification. */

        if (xButtonCaptureQueue != NULL)
        {
            if (xQueueReceive(xButtonCaptureQueue, &ulInterButtonStatus, portMAX_DELAY))
            {
                //printf("\nButton Capture Function: \n");
                if (ulInterButtonStatus == 0x01)
                {
                    /* call advance frame function, which decides weather to advance the frame or not*/
                    advance_frame_pressed(&singleMode);
                }
                else if (ulInterButtonStatus == 0x02)
                {
                    /* call signal mode function to set single of multi mode */
                    signal_mode_pressed(&singleMode);
                }
                else if (ulInterButtonStatus == 0x10)
                {
                    /* calls red button pressed to */
                    red_button_pressed(&singleMode);
                }
                else if (ulInterButtonStatus == 0x20)
                {
                    green_button_pressed(&singleMode);
                }
                else if (ulInterButtonStatus == 0x40)
                {
                    blue_button_pressed(&singleMode);
                }
            }
        }
    }
}

/**
 * @brief Helper function for when advancde frame button is pressed. It mainly decideds when to send
 *        a messaga to advance frame task.
 * 
 * @param singleMode Is a pointer to the system state, which is single or multi mode.
*/
void advance_frame_pressed(bool *singleMode)
{
    gpio_intr_disable(GPIO_INPUT_ADVANCE_FRAME);
    uint32_t msg = 0x02;
    if (!(*singleMode))
    {
        //printf("************ advance frame **********\n");
        if (xAdvanceFrameQueue != NULL)
        {
            xQueueSend(xAdvanceFrameQueue, &msg, (TickType_t)10);
        }
    }

    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_ADVANCE_FRAME);
}

/**
 * @brief Helper function for when signal mode button is pressed. I helps switch from single and multi mode
 *        and updates the singleMode variable accordingly. It also suspends and resums the single and multi mode task
 *        functions.
 * 
 * @param singleMode Holds the state of the system, which is single or multi mode.
*/
void signal_mode_pressed(bool *singleMode)
{
    gpio_intr_disable(GPIO_INPUT_MULTI_SINGLE_TOGGLE);
    *singleMode = !(*singleMode);
    if (*singleMode)
    {
        /* stop sending messages from advance_frame function to MultiMode task*/
        uint32_t suspendQueue = 0x03;
        if (xQueueSend(xAdvanceFrameQueue, &suspendQueue, (TickType_t)100))
        {
            vTaskSuspend(xMultiModeTaskHandle);
            vTaskResume(xSingleModeTaskHandle);
            uint32_t startSingleMode = 0x03;
            if (!xQueueSend(xSingleModeControlQueue, &startSingleMode, (TickType_t)10))
            {
                printf("Could not send message to queue: signal_mode_pressed \n");
            }
        }
        else
        {
            *singleMode = !(*singleMode);
            printf("could not send message to queue\n");
        }
    }
    else
    {
        /* resume sending messages from advance_frame to MultiMode task */
        uint32_t enableQueue = 0x04;
        if (xQueueSend(xAdvanceFrameQueue, &enableQueue, (TickType_t)100))
        {
            vTaskSuspend(xSingleModeTaskHandle);
            vTaskResume(xMultiModeTaskHandle);
            uint32_t startMultiMode = 0x20;
            if (!xQueueSend(xMultiModeControlQueue, &startMultiMode, (TickType_t)100))
            {
                printf("Could not send message to queue: signal_mode_pressed \n");
            }
        }
        else
        {
            *singleMode = !(*singleMode);
            printf("could not send message to queue\n");
        }
    }

    vTaskDelay(300 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_MULTI_SINGLE_TOGGLE);
}

/**
 * @brief This function is activated when the red button is pressed. It sends a message to the fucntion task 
 *        corresponding to the system curren output mode (single/multi mode).
 * 
 * @b singleMode Holds the state of the system, which is single or multi mode.     
*/
void red_button_pressed(bool *singleMode)
{
    gpio_intr_disable(GPIO_INPUT_RED_BUTTON);

    if (*singleMode)
    {
        //printf("**************red pressed SINGLE MODE\n");
        // send message to single_mode_task
        uint32_t activateLED = 0x00;
        xQueueSend(xSingleModeControlQueue, &activateLED, (TickType_t)10);
    }
    else
    {
        //printf("**************red pressed MULTI_MODE\n");
        uint32_t enableDisable = 0x10;
        xQueueSend(xMultiModeControlQueue, &enableDisable, (TickType_t)10);
    }
    vTaskDelay(300 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_RED_BUTTON);
}

/**
 * @brief This function is activated when the red button is pressed. It sends a message to the fucntion task 
 *        corresponding to the system curren output mode (single/multi mode).
 * 
 * @b singleMode Holds the state of the system, which is single or multi mode.     
*/
void green_button_pressed(bool *singleMode)
{
    gpio_intr_disable(GPIO_INPUT_GREEN_BUTTON);

    if (*singleMode)
    {
        //printf("*****************green pressed SINGLE MODE\n");
        // send message to single_mode_task
        uint32_t activateLED = 0x01;
        xQueueSend(xSingleModeControlQueue, &activateLED, (TickType_t)10);
    }
    else
    {
        //printf("*******************GREEN pressed MULTI_MODE\n");
        uint32_t enableDisable = 0x11;
        xQueueSend(xMultiModeControlQueue, &enableDisable, (TickType_t)10);
    }
    vTaskDelay(300 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_GREEN_BUTTON);
}

/**
 * @brief This function is activated when the red button is pressed. It sends a message to the fucntion task 
 *        corresponding to the system curren output mode (single/multi mode).
 * 
 * @b singleMode Holds the state of the system, which is single or multi mode.     
*/
void blue_button_pressed(bool *singleMode)
{

    gpio_intr_disable(GPIO_INPUT_BLUE_BUTTON);
    if (*singleMode)
    {
        //printf("******************blue pressed SINGLE MODE\n");
        // send message to single_mode_task
        uint32_t activateLED = 0x02;
        xQueueSend(xSingleModeControlQueue, &activateLED, (TickType_t)10);
    }
    else
    {
        //printf("*******************BLUE pressed MULTI_MODE\n");
        uint32_t enableDisable = 0x12;
        xQueueSend(xMultiModeControlQueue, &enableDisable, (TickType_t)10);
    }
    vTaskDelay(300 / portTICK_RATE_MS);
    gpio_intr_enable(GPIO_INPUT_BLUE_BUTTON);
}