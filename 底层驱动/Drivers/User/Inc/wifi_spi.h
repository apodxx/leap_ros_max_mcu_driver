#ifndef __WIFI_SPI_H
#define __WIFI_SPI_H
#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi4;

/* Fill in the Wi-Fi/host values for your own setup.  Do not commit real
   credentials; this file is part of a public reference project. */
#define WIFI_SPI_SSID "YOUR_WIFI_SSID"
#define WIFI_SPI_PASSWORD "YOUR_WIFI_PASSWORD"
#define WIFI_SPI_TARGET_IP "192.168.1.100"
/* Must match the QT upper computer's UDP listen port (mainwindow.cpp
   defaults the "UDP 端口" field to 8081, not 8080). */
#define WIFI_SPI_TARGET_PORT "8081"
#define WIFI_SPI_LOCAL_PORT "6666"

/* Largest payload the module accepts in a single send burst. */
#define WIFI_SPI_MAX_TRANSFER 4088U

/* Module information, filled in by WIFI_SPI_ModuleInit(). */
extern char wifi_spi_version[12];
extern char wifi_spi_mac_addr[20];
extern char wifi_spi_ip_addr_port[25];

/* Low level bring-up: SPI4 + CS/INT/RST GPIO + hardware reset pulse. */
void WIFI_SPI_Init(void);

/* Full module bring-up: version, MAC, then join the given access point.
   Pass password = 0 for an open network.  Returns 0 on success. */
uint8_t WIFI_SPI_ModuleInit(const char *ssid, const char *password);
uint8_t WIFI_SPI_IsReady(void);

uint8_t WIFI_SPI_Reset(void);

/* Raw link-level helpers, kept for electrical diagnostics. */
HAL_StatusTypeDef WIFI_SPI_Transfer(uint8_t tx, uint8_t *rx);
HAL_StatusTypeDef WIFI_SPI_TransmitReceive(uint8_t *tx, uint8_t *rx, uint16_t length);

uint8_t WIFI_SPI_GetVersion(char *out, uint16_t size);
uint8_t WIFI_SPI_GetMAC(char *out, uint16_t size);
uint8_t WIFI_SPI_GetIP(char *out, uint16_t size);

uint8_t WIFI_SPI_Scan(char *out, uint16_t size);
uint8_t WIFI_SPI_ScanDelayed(char *out, uint16_t size, uint32_t delay_ms);

uint8_t WIFI_SPI_Connect(const char *ssid, const char *password);
uint8_t WIFI_SPI_SocketConnect(const char *type, const char *ip, const char *port, const char *local_port);
uint8_t WIFI_SPI_SocketDisconnect(void);
uint8_t WIFI_SPI_UdpSendNow(void);

/* Returns the number of bytes that could NOT be sent (0 = everything sent). */
uint32_t WIFI_SPI_SendBuffer(const uint8_t *data, uint32_t length);
void WIFI_SPI_SendString(const char *string);

/* Returns the number of bytes actually copied into buffer. */
uint16_t WIFI_SPI_Read(uint8_t *buffer, uint16_t length);

/* Diagnostics. */
uint8_t WIFI_SPI_GetLastReply(void);
uint16_t WIFI_SPI_GetLastLength(void);
uint16_t WIFI_SPI_GetLastPacketLength(void);
uint8_t WIFI_SPI_GetLastScanPackets(void);
void WIFI_SPI_PrintLastRaw(const char *label);
uint32_t WIFI_SPI_GetIntEvents(void);

#endif
