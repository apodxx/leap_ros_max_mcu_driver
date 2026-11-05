# IMU WiFi-SPI log transport

Build MDK-ARM/STM32H743.uvprojx and flash MDK-ARM/STM32H743/STM32H743.hex.
USB CDC is initialized before WiFi and I2C for local fault diagnostics.
Startup diagnostics, register read errors, initialization results and
ACC/GYRO/TEMP text still use WiFi-SPI UDP, with a best-effort USB mirror.
USB output is skipped until enumeration completes and while a transfer is busy.

## Configuration

Edit Drivers/User/Inc/wifi_spi.h for SSID, password, target PC IPv4 address,
target UDP port (8081), and module local port (6666).  The committed values are
placeholders; set them to your own network and verify the PC address.

SPI4 mode 3: SCK=PE12, MISO=PE13, MOSI=PE14, CS=PG0,
INT=PG1 (polled input), RST=PE15. Use common ground and 3.3 V logic.
IMU remains on I2C1, SCL=PB8 and SDA=PB9.
IMU CS=PB1 is driven high; SA0=PC5 stays low. Diagnostic v2 removes the
full address scan and reads WHO_AM_I (0x0F) at 0x6A, then 0x6B if needed.
Each candidate starts with a reset of the STM32 I2C1 peripheral, not the
sensor. No bus-clear pulses are generated. Low SCL/SDA skips the read.
Logs capture HAL status, error bits, ISR, state and pin levels. A successful
0x70 ID selects the address for initialization; other results skip writes.

## PC receiver

Open upper_computer_QT, start UDP listening on 8081, then reset the MCU.
Allow inbound UDP in the PC firewall and ensure the module can reach the PC.
The existing data log displays these messages as raw text; raw acceleration
and angular velocity do not update the yaw chart without attitude estimation.

The MCU waits for WiFi association and socket setup before IMU diagnostics.
Failed network setup is retried with an LED toggle and a 2-second delay.
USB reports the failing stage on each retry. The post-reset delay matches the
working WiFi example (500 ms after WIFI_SPI_Init).
Keil Watch variables: boot_stage (1=clock, 2=LED/USB initialization,
3=WiFi initialization, 4=IMU GPIO, 5=I2C initialization, 6=IMU diagnostics),
wifi_log_stage (1=reset, 2=version, 3=MAC, 4=join AP, 5=IP, 6=UDP socket,
7=ready), wifi_log_attempt, wifi_log_error.
An IMU initialization failure emits a status line every 2 seconds.
Local transmit errors reconnect and retry the current message. UDP has no
receiver acknowledgement: a stopped PC listener or lost packet is not detected.
Start the listener before reset to capture startup logs.
