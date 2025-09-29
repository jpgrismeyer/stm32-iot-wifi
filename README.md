# STM32 IoT – UART JSON Sensor Output

This project uses an *STM32L475* microcontroller to read data from the onboard sensor:

- *HTS221* → Temperature

The data is sent over *UART1* in *JSON format*, ready to be processed by a Python script or any IoT platform.

## 🛠 Features
- Temperature reading via *I2C2*.
- Sensor initialization with *HAL drivers*.
- Data transmission through *UART* in JSON format:
  ```json
  {"temp_c": 27.02}

•	Error handling: if a reading fails, the value is sent as null.

📂 Project structure
	•	Core/Src/main.c → Peripheral configuration and main loop.
	•	Core/Inc → Project headers.
	•	serial_hts221_logger.py → Python script to read and display the data from the serial port.

🚀 How to run
	1.	Build and flash the firmware with STM32CubeIDE.
	2.	Connect the board via USB and open the serial port (115200 8N1).

3.	Run on your PC:
python serial_hts221_logger.py

Next steps
	•	Add pressure reading with the LPS22HB sensor.
	•	Send data to a REST API or MQTT broker.
	•	Real-time visualization using matplotlib.

⸻

👨‍💻 Author: Juan Pablo Grismeyer
📍 Gröbenzell, Bavaria – Germany
🔗 https://github.com/jpgrismeyer/stm32-projects