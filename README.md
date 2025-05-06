Vibration Measurement Device Prototype Based on STM32L073RZT6
This project successfully designed and implemented a prototype of a vibration measurement device using the STM32L073RZT6 microcontroller. The device is capable of real-time collection of vibration data, its processing, and clear presentation of results to the user via an integrated 16x2 LCD screen and a USB-UART interface, allowing for both local data monitoring and transmission to a computer for further analysis. The MPU6050 accelerometer ensured sufficient measurement accuracy.

Future Development Directions
Although the main objectives were achieved, the development process highlighted several potential areas for improvement:

Signal Filtering: To achieve more accurate RMS value calculations, it is recommended to implement digital filtering algorithms to reduce data noise and account for potential rounding errors.
More Reliable Calibration: Instead of the current method of averaging initial values, more comprehensive bias compensation methods should be explored and implemented to further reduce measurement errors.
Measurement Range Management: Future development could include a user interface (e.g., buttons) for controlling the measurement range (2g for precise small amplitudes, 16g for intensive measurements) and saving the selected range in non-volatile memory.
Manual Recalibration: Users should have the ability to initiate manual recalibration via a button, especially if the device is moved to a different location.
Power Saving: If the device is to be powered by a battery, optimizing power consumption is essential.
Practical Applications
The developed prototype of the vibration measurement device has a wide range of potential applications in various fields:

Industry: Using the device in industrial equipment could enable vibration monitoring and early identification of potential mechanical failures.
Construction: Monitoring vibrations in structures and buildings, by analyzing unusual vibration patterns, could help detect structural changes or damage.
Medicine: Vibration sensors can be applied to assess patients' physical activity, and monitor respiratory and heart rates through biomechanical signals.

