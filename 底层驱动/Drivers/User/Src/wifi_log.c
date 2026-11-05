#include "wifi_log.h"
#include "wifi_spi.h"
#include "led.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t log_ready;
static uint32_t failed_datagrams;
/* Watch in Keil: 1=reset, 2=version, 3=MAC, 4=join, 5=IP, 6=socket, 7=ready. */
volatile uint32_t wifi_log_stage;
volatile uint32_t wifi_log_attempt;
volatile uint32_t wifi_log_error;

static void usb_diagnostic(const char *text)
{
    static uint8_t buffer[512];
    USBD_CDC_HandleTypeDef *cdc;
    size_t length;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
        hUsbDeviceFS.pClassData == NULL) return;
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (cdc->TxState != 0) return;
    length = strlen(text);
    if (length > sizeof(buffer)) length = sizeof(buffer);
    memcpy(buffer, text, length);
    CDC_Transmit_FS(buffer, (uint16_t)length);
}

void WIFI_LogInit(void)
{
    log_ready = 0;
    char diagnostic[160];
    {
        wifi_log_attempt++;
        wifi_log_stage = 1;
        usb_diagnostic("WiFi: resetting SPI module\r\n");
        WIFI_SPI_Init();
        /* Match the working reference's delay after hardware reset. */
        HAL_Delay(500);
        wifi_log_stage = 2;
        usb_diagnostic("WiFi: reading version\r\n");
        wifi_log_error = WIFI_SPI_GetVersion(wifi_spi_version, sizeof(wifi_spi_version));
        if (wifi_log_error) goto retry;
        wifi_log_stage = 3;
        WIFI_SPI_GetMAC(wifi_spi_mac_addr, sizeof(wifi_spi_mac_addr));
        wifi_log_stage = 4;
        usb_diagnostic("WiFi: joining access point\r\n");
        wifi_log_error = WIFI_SPI_Connect(WIFI_SPI_SSID, WIFI_SPI_PASSWORD);
        if (wifi_log_error) goto retry;
        wifi_log_stage = 5;
        WIFI_SPI_GetIP(wifi_spi_ip_addr_port, sizeof(wifi_spi_ip_addr_port));
        wifi_log_stage = 6;
        usb_diagnostic("WiFi: opening UDP socket\r\n");
        wifi_log_error = WIFI_SPI_SocketConnect("UDP", WIFI_SPI_TARGET_IP,
                                               WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if (wifi_log_error) goto retry;
        wifi_log_stage = 7;
        log_ready = 1;
        usb_diagnostic("WiFi: UDP socket ready\r\n");
        return;
retry:
        snprintf(diagnostic, sizeof(diagnostic),
                 "WiFi attempt=%lu failed stage=%lu error=%lu reply=0x%02X INT=%u\r\n",
                 (unsigned long)wifi_log_attempt, (unsigned long)wifi_log_stage,
                 (unsigned long)wifi_log_error, WIFI_SPI_GetLastReply(),
                 (unsigned int)HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1));
        usb_diagnostic(diagnostic);
        LED1_Toggle;
        return;
    }
}

void WIFI_LogPrintf(const char *format, ...)
{
    char text[512];
    va_list args;
    int length;
    if (!log_ready) return;
    va_start(args, format);
    length = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (length <= 0) return;
    if ((unsigned int)length >= sizeof(text)) length = sizeof(text) - 1;
    usb_diagnostic(text);
    /* Commit only complete payloads. UDP provides no delivery acknowledgement. */
    while (WIFI_SPI_SendBuffer((const uint8_t *)text, (uint32_t)length) != 0 ||
        WIFI_SPI_UdpSendNow() != 0) {
        failed_datagrams++;
        WIFI_LogInit();
    }
}
