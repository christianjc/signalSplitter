# Signal Splitter - README

## Description:

Signal Splitter is an application for controlling LED signals based on external input and button presses. The application uses FreeRTOS to manage tasks and queues to facilitate inter-task communication. It takes input from various sources, such as hardware interrupts and analog potentiometer readings, to control the LED signals with different colors and modes.

## Features:

1. Signal Splitting: The application can receive an external input signal (Projector input signal) and synchronize it to control the LED signals accordingly.

2. Frame Advancement: The application supports a button that allows users to manually advance the LED signals to the next frame.

3. Multi/Single Mode Toggle: Users can switch between Multi and Single modes using a dedicated button to control the LED signals differently in each mode.

4. Color Control: The application allows the user to control the LED signals' intensity and color using analog potentiometers.

5. LED Controller: The LEDC (LED Controller) peripheral is configured to control the LED signals with different colors and intensities.

## Usage:

1. Connect the LED signals to the corresponding GPIO pins.
2. Connect the hardware interrupts for the input signal, mode toggle button, and frame advancement button to appropriate GPIO pins.
3. Connect the analog potentiometers to the ADC channels to control the LED signal colors.

## Configuration:

1. Modify the GPIO pin configurations in the `gpio_init()` function to match your hardware setup for LED signals and button inputs.

2. Adjust the LEDC configuration in the `ledc_init()` function based on your desired PWM frequency and duty cycle for each LED signal.

3. Configure the ADC channels in the `adc_pwm_task()` function to match the potentiometer connections for controlling LED colors.

## Tasks and Interrupt Handlers:

1. `advance_frame_task`: This task handles synchronization of the LED signals with the Projector input signal. It synchronizes the LED frames and advances them based on the input signal and button presses.

2. `multiMode_controller_task`: This task controls the LED signals in Multi-Mode, toggling different colors based on the input and button presses.

3. `adc_pwm_task`: This task reads the analog potentiometer values to control the LED signal colors. It updates the PWM duty cycle for each color channel accordingly.

4. `isr_signal_handler`: Hardware interrupt handler for the Projector input signal.

5. `isr_advance_frame_handler`: Hardware interrupt handler for the frame advancement button.

6. `isr_signal_mode_handler`: Hardware interrupt handler for the mode toggle button.

7. `isr_red_button_handler`: Hardware interrupt handler for the red LED toggle button.

8. `isr_green_button_handler`: Hardware interrupt handler for the green LED toggle button.

9. `isr_blue_button_handler`: Hardware interrupt handler for the blue LED toggle button.

**Note:**
Please make sure to modify the GPIO pin configurations, ADC channels, and LEDC configurations according to your hardware setup. Also, adjust the queue sizes and priorities as needed for your application.

## License:

This code is provided under the MIT License. Feel free to modify and use it as needed for your projects.

## Contributing:

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, feel free to create a pull request or open an issue. Let's build better software together!
