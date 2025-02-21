**Mini weather station** - seasurement of temperature, humidity and pressure with SD Card Recording

**Project description:**
The project is based on the STM32L053C8T6 microcontroller and implements the measurement of temperature, humidity and atmospheric pressure using DHT11 and BME280 sensors. The data is sent to a computer via the USART interface and simultaneously saved to an SD card.

![mini weather station - terminal](https://github.com/user-attachments/assets/ee25e604-6fff-4b8a-ad25-5b9164ada993)

![mini weather station - karta sd](https://github.com/user-attachments/assets/76388d26-8f20-4d0b-a23d-c2905521c7da)


**Main functions:**
- Reading temperature and humidity from DHT11 sensor
- Reading temperature, humidity and pressure from BME280 (I2C communication)
- Saving the read data to an SD card (via SPI interface)
- Sending measurements to a computer via USART



**Technologies Used:**
-  C++, C
- STM32 HAL Library
- I2C for BME280 sensor
- SPI for communication with SD card
- Serial Communication (UART)

The way to read data from the DHT11 was created based on: https://controllerstech.com/using-dht11-sensor-with-stm32/
