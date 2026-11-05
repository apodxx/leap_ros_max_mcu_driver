#include "wifi_spi.h"
#include "main.h"
#include "wifi_log.h"
#include <string.h>
#include <stdio.h>

#define CS_PORT   GPIOG
#define CS_PIN    GPIO_PIN_0
#define INT_PORT  GPIOG
#define INT_PIN   GPIO_PIN_1
#define RST_PORT  GPIOE
#define RST_PIN   GPIO_PIN_15

/* Protocol command / reply codes, per SeekFree WIFI6B21-SPI datasheet. */
#define CMD_RESET            0x01
#define CMD_DATA             0x02
#define CMD_UDP_SEND         0x03
#define CMD_CLOSE_SOCKET     0x04
#define CMD_SET_WIFI         0x10
#define CMD_SET_SOCKET       0x11
#define CMD_WIFI_SCAN        0x12
#define CMD_GET_VERSION      0x20
#define CMD_GET_MAC          0x21
#define CMD_GET_IP           0x22
#define REPLY_OK             0x80
#define REPLY_DATA_START     0x90
#define REPLY_DATA_END       0x91
#define REPLY_VERSION        0xA0
#define REPLY_MAC_ADDR       0xA1
#define REPLY_IP_ADDR        0xA2

#define WAIT_MS              1000U
#define CONNECT_WAIT_MS      10000U
#define SOCKET_CONNECT_WAIT_MS 50000U

/* Per-command wait budget for each INT handshake phase, matching the
   reference driver's wifi_spi_wait_idle() timeouts: WiFi join and scan
   can take seconds to associate, and a TCP socket connect to a remote
   host can take tens of seconds, so those need far longer than the
   default 1 s budget used for simple GET/SET commands. */
static uint32_t wifi_command_wait_ms(uint8_t cmd)
{
  if (cmd == CMD_SET_WIFI || cmd == CMD_WIFI_SCAN) return CONNECT_WAIT_MS;
  if (cmd == CMD_SET_SOCKET) return SOCKET_CONNECT_WAIT_MS;
  return WAIT_MS;
}

/* One SPI burst carries at most this many payload bytes; the module echoes
   any pending inbound data back on MISO during the same burst. */
#define CHUNK 32U

typedef struct { uint8_t command; uint8_t reserve; uint16_t length; } wifi_head_t;

SPI_HandleTypeDef hspi4;

char wifi_spi_version[12] = {0};
char wifi_spi_mac_addr[20] = {0};
char wifi_spi_ip_addr_port[25] = {0};

static volatile uint32_t wifi_int_events;
static uint8_t  wifi_last_reply;
static uint16_t wifi_last_length;
static uint16_t wifi_last_packet_length;
static uint8_t  wifi_last_scan_packets;
static uint8_t  wifi_last_raw[CHUNK];
static uint8_t  wifi_last_raw_length;
static uint8_t  wifi_socket_ready;

/* Unsolicited inbound bytes (REPLY_DATA_START/END frames) land here until
   WIFI_SPI_Read() drains them. */
#define RX_FIFO_SIZE 512U
static uint8_t  rx_fifo[RX_FIFO_SIZE];
static uint16_t rx_fifo_head, rx_fifo_tail, rx_fifo_used;

static void rx_fifo_push(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len && rx_fifo_used < RX_FIFO_SIZE; i++)
  {
    rx_fifo[rx_fifo_tail] = data[i];
    rx_fifo_tail = (uint16_t)((rx_fifo_tail + 1U) % RX_FIFO_SIZE);
    rx_fifo_used++;
  }
}

static uint16_t rx_fifo_pop(uint8_t *out, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len && rx_fifo_used; i++)
  {
    out[i] = rx_fifo[rx_fifo_head];
    rx_fifo_head = (uint16_t)((rx_fifo_head + 1U) % RX_FIFO_SIZE);
    rx_fifo_used--;
  }
  return i;
}

static void cs_low(void)  { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET); }

/* The module drives INT high once it is ready to clock the next SPI frame. */
static uint8_t wifi_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
  {
    if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == GPIO_PIN_SET) return 1;
  }
  return 0;
}

/* Stashes diagnostics and, for REPLY_DATA_* frames, queues the payload. */
static void capture_reply(const wifi_head_t *r, const uint8_t *payload)
{
  wifi_last_reply = r->command;
  wifi_last_packet_length = r->length;
  if (r->command == REPLY_DATA_START || r->command == REPLY_DATA_END)
  {
    uint16_t n = r->length;
    if (n > CHUNK) n = CHUNK;
    if (n) rx_fifo_push(payload, n);
  }
}

/* -------------------------------------------------------------------------
 * Two-phase command transaction, matching the SeekFree WIFI-SPI protocol:
 *   1) CS low, clock out the command header (+ optional payload), CS high
 *   2) wait for INT, CS low, clock a DATA/0 poll frame to read the module's
 *      reply header (+ up to CHUNK bytes of reply payload), CS high
 * `pad_request_to_32` matches the "get parameter" request framing, which
 * always occupies a full 32-byte SPI burst even with no payload.
 * ---------------------------------------------------------------------- */
static uint8_t wifi_command(uint8_t cmd, const uint8_t *payload, uint16_t len,
                             uint8_t pad_request_to_32,
                             uint8_t *reply, uint16_t reply_cap, uint8_t expected)
{
  wifi_head_t h, r;
  uint8_t request[4U + CHUNK] = {0};
  uint8_t poll_tx[4U + CHUNK] = {0};
  uint8_t poll_rx[4U + CHUNK] = {0};
  uint16_t request_length, n;

  uint32_t wait_ms = wifi_command_wait_ms(cmd);

  wifi_last_reply = 0;
  wifi_last_length = 0;
  wifi_last_raw_length = 0;

  if (!wifi_ready(wait_ms)) return 1;

  h.command = cmd; h.reserve = 0; h.length = len;
  memcpy(request, &h, sizeof(h));
  if (len) memcpy(request + sizeof(h), payload, len);
  request_length = (uint16_t)(sizeof(h) + len);
  if (pad_request_to_32 && request_length < sizeof(request)) request_length = (uint16_t)sizeof(request);

  cs_low();
  if (HAL_SPI_Transmit(&hspi4, request, request_length, 1000) != HAL_OK) { cs_high(); return 1; }
  cs_high();

  if (!wifi_ready(wait_ms)) return 1;

  h.command = CMD_DATA; h.reserve = 0; h.length = 0;
  memcpy(poll_tx, &h, sizeof(h));

  cs_low();
  if (HAL_SPI_TransmitReceive(&hspi4, poll_tx, poll_rx, sizeof(poll_tx), 1000) != HAL_OK) { cs_high(); return 1; }
  cs_high();

  memcpy(&r, poll_rx, sizeof(r));
  capture_reply(&r, poll_rx + sizeof(r));
  wifi_last_length = r.length;
  memcpy(wifi_last_raw, poll_rx + sizeof(r), CHUNK);
  wifi_last_raw_length = CHUNK;

  if (expected && r.command != expected) return 1;

  if (reply && reply_cap)
  {
    n = (r.length < reply_cap - 1U) ? r.length : (uint16_t)(reply_cap - 1U);
    if (n > CHUNK) n = CHUNK;
    memset(reply, 0, reply_cap);
    if (n) memcpy(reply, poll_rx + sizeof(r), n);
  }
  return 0;
}

void WIFI_SPI_Init(void)
{
  wifi_socket_ready = 0;
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  /* WIFI6B21-SPI requires SPI mode 3 (CPOL=1, CPHA=1). Mode 0 shifts every
     received bit by one position, which is what the left-shift symptom in
     earlier revisions of this file was actually caused by. */
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK) Error_Handler();

  cs_high();

  /* WIFI6B21 RST is active low. */
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
}

uint8_t WIFI_SPI_Reset(void)
{
  wifi_head_t h;
  if (!wifi_ready(WAIT_MS)) return 1;
  h.command = CMD_RESET; h.reserve = 0; h.length = 0xA5A5U;
  cs_low();
  if (HAL_SPI_Transmit(&hspi4, (uint8_t *)&h, sizeof(h), 1000) != HAL_OK) { cs_high(); return 1; }
  cs_high();
  /* The reset command intentionally has no response packet. */
  HAL_Delay(1500);
  return wifi_ready(CONNECT_WAIT_MS) ? 0 : 1;
}

HAL_StatusTypeDef WIFI_SPI_Transfer(uint8_t tx, uint8_t *rx)
{
  HAL_StatusTypeDef s;
  cs_low();
  s = HAL_SPI_TransmitReceive(&hspi4, &tx, rx, 1, 1000);
  cs_high();
  return s;
}

HAL_StatusTypeDef WIFI_SPI_TransmitReceive(uint8_t *tx, uint8_t *rx, uint16_t n)
{
  HAL_StatusTypeDef s;
  cs_low();
  s = HAL_SPI_TransmitReceive(&hspi4, tx, rx, n, 1000);
  cs_high();
  return s;
}

uint8_t WIFI_SPI_GetVersion(char *o, uint16_t n)
{
  return (n == 0) || wifi_command(CMD_GET_VERSION, 0, 0, 1, (uint8_t *)o, n, REPLY_VERSION);
}
uint8_t WIFI_SPI_GetMAC(char *o, uint16_t n)
{
  return (n == 0) || wifi_command(CMD_GET_MAC, 0, 0, 1, (uint8_t *)o, n, REPLY_MAC_ADDR);
}
uint8_t WIFI_SPI_GetIP(char *o, uint16_t n)
{
  return (n == 0) || wifi_command(CMD_GET_IP, 0, 0, 1, (uint8_t *)o, n, REPLY_IP_ADDR);
}

static uint8_t wifi_spi_scan_impl(char *out, uint16_t size, uint32_t delay_ms)
{
  wifi_head_t h, r;
  uint8_t frame_tx[4U + CHUNK] = {0};
  uint8_t frame_rx[4U + CHUNK] = {0};
  uint16_t used = 0, n;

  wifi_last_reply = 0; wifi_last_length = 0; wifi_last_packet_length = 0;
  wifi_last_scan_packets = 0; wifi_last_raw_length = 0;

  if (size == 0 || wifi_command(CMD_WIFI_SCAN, 0, 0, 0, 0, 0, REPLY_OK)) return 1;
  if (delay_ms) HAL_Delay(delay_ms);
  if (!wifi_ready(CONNECT_WAIT_MS)) return 1;

  memset(out, 0, size);
  do
  {
    h.command = CMD_DATA; h.reserve = 0; h.length = 0;
    memcpy(frame_tx, &h, sizeof(h));
    cs_low();
    if (HAL_SPI_TransmitReceive(&hspi4, frame_tx, frame_rx, sizeof(frame_tx), 1000) != HAL_OK) { cs_high(); return 1; }
    cs_high();

    memcpy(&r, frame_rx, sizeof(r));
    wifi_last_reply = r.command;
    wifi_last_packet_length = r.length;
    wifi_last_scan_packets++;
    memcpy(wifi_last_raw, frame_rx + sizeof(r), CHUNK);
    wifi_last_raw_length = CHUNK;

    if (r.command != REPLY_DATA_START && r.command != REPLY_DATA_END) return 1;

    n = r.length;
    if (n > CHUNK) n = CHUNK;
    if (n > (uint16_t)(size - 1U - used)) n = (uint16_t)(size - 1U - used);
    memcpy(out + used, frame_rx + sizeof(r), n);
    used = (uint16_t)(used + n);
    wifi_last_length = used;

    if (r.command == REPLY_DATA_START && !wifi_ready(CONNECT_WAIT_MS)) return 1;
  } while (r.command == REPLY_DATA_START);

  return 0;
}

uint8_t WIFI_SPI_Scan(char *out, uint16_t size) { return wifi_spi_scan_impl(out, size, 0); }
uint8_t WIFI_SPI_ScanDelayed(char *out, uint16_t size, uint32_t delay_ms) { return wifi_spi_scan_impl(out, size, delay_ms); }

uint8_t WIFI_SPI_Connect(const char *ssid, const char *pass)
{
  char p[128];
  int n = (pass && pass[0]) ? snprintf(p, sizeof(p), "%s\r\n%s\r\n", ssid, pass)
                             : snprintf(p, sizeof(p), "%s\r\n", ssid);
  if (n < 0 || n >= (int)sizeof(p)) return 1;
  return wifi_command(CMD_SET_WIFI, (uint8_t *)p, (uint16_t)n, 0, 0, 0, REPLY_OK);
}

uint8_t WIFI_SPI_SocketConnect(const char *type, const char *ip, const char *port, const char *local)
{
  char p[128];
  int n = snprintf(p, sizeof(p), "%s\r\n%s\r\n%s\r\n%s\r\n", type, ip, port, local);
  if (n < 0 || n >= (int)sizeof(p)) return 1;
  return wifi_command(CMD_SET_SOCKET, (uint8_t *)p, (uint16_t)n, 0, 0, 0, REPLY_OK);
}

uint8_t WIFI_SPI_SocketDisconnect(void)
{
  return wifi_command(CMD_CLOSE_SOCKET, 0, 0, 1, 0, 0, 0);
}

uint8_t WIFI_SPI_UdpSendNow(void)
{
  return wifi_command(CMD_UDP_SEND, 0, 0, 0, 0, 0, REPLY_OK);
}

/* Sends one <=32-byte data burst and captures whatever the module echoes
   back in the same transaction (outgoing SPI_DATA and any inbound data the
   module has queued travel in the same clocked burst). */
static uint8_t send_chunk(const uint8_t *data, uint16_t len)
{
  wifi_head_t h, r;
  uint8_t tx[4U + CHUNK] = {0};
  uint8_t rx[4U + CHUNK] = {0};

  if (!wifi_ready(WAIT_MS)) return 1;

  h.command = CMD_DATA; h.reserve = 0; h.length = len;
  memcpy(tx, &h, sizeof(h));
  if (len) memcpy(tx + sizeof(h), data, len);

  cs_low();
  if (HAL_SPI_TransmitReceive(&hspi4, tx, rx, sizeof(tx), 1000) != HAL_OK) { cs_high(); return 1; }
  cs_high();

  memcpy(&r, rx, sizeof(r));
  capture_reply(&r, rx + sizeof(r));
  return 0;
}

uint32_t WIFI_SPI_SendBuffer(const uint8_t *data, uint32_t length)
{
  while (length)
  {
    uint16_t n = (length > CHUNK) ? CHUNK : (uint16_t)length;
    if (send_chunk(data, n)) return length;
    data += n;
    length -= n;
  }
  return 0;
}

void WIFI_SPI_SendString(const char *string)
{
  WIFI_SPI_SendBuffer((const uint8_t *)string, (uint32_t)strlen(string));
}

uint16_t WIFI_SPI_Read(uint8_t *buffer, uint16_t length)
{
  uint16_t got = rx_fifo_pop(buffer, length);
  wifi_head_t h, r;
  uint8_t tx[4U + CHUNK] = {0};
  uint8_t rx[4U + CHUNK] = {0};

  if (got >= length) return got;

  /* Opportunistically give the module a chance to push one more frame. */
  if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == GPIO_PIN_SET)
  {
    h.command = CMD_DATA; h.reserve = 0; h.length = 0;
    memcpy(tx, &h, sizeof(h));
    cs_low();
    if (HAL_SPI_TransmitReceive(&hspi4, tx, rx, sizeof(tx), 1000) == HAL_OK)
    {
      cs_high();
      memcpy(&r, rx, sizeof(r));
      capture_reply(&r, rx + sizeof(r));
      got = (uint16_t)(got + rx_fifo_pop(buffer + got, (uint16_t)(length - got)));
    }
    else
    {
      cs_high();
    }
  }
  return got;
}

uint8_t WIFI_SPI_GetLastReply(void) { return wifi_last_reply; }
uint16_t WIFI_SPI_GetLastLength(void) { return wifi_last_length; }
uint16_t WIFI_SPI_GetLastPacketLength(void) { return wifi_last_packet_length; }
uint8_t WIFI_SPI_GetLastScanPackets(void) { return wifi_last_scan_packets; }

void WIFI_SPI_PrintLastRaw(const char *label)
{
  char line[160];
  uint8_t i;
  int offset = snprintf(line, sizeof(line), "%s raw MISO (%u bytes):", label, wifi_last_raw_length);
  for (i = 0; i < wifi_last_raw_length && offset > 0 && offset < (int)sizeof(line); i++)
    offset += snprintf(line + offset, sizeof(line) - (uint16_t)offset, " %02X", wifi_last_raw[i]);
  WIFI_LogPrintf("%s\r\n", line);
}

uint32_t WIFI_SPI_GetIntEvents(void)
{
  uint32_t n;
  __disable_irq();
  n = wifi_int_events;
  wifi_int_events = 0;
  __enable_irq();
  return n;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) { if (pin == INT_PIN) wifi_int_events++; }

uint8_t WIFI_SPI_ModuleInit(const char *ssid, const char *password)
{
  wifi_socket_ready = 0;
  if (WIFI_SPI_GetVersion(wifi_spi_version, sizeof(wifi_spi_version))) return 1;
  WIFI_SPI_GetMAC(wifi_spi_mac_addr, sizeof(wifi_spi_mac_addr));
  if (!ssid) return 0;
  if (WIFI_SPI_Connect(ssid, password)) return 1;
  WIFI_SPI_GetIP(wifi_spi_ip_addr_port, sizeof(wifi_spi_ip_addr_port));
  if (WIFI_SPI_SocketConnect("UDP", WIFI_SPI_TARGET_IP,
                             WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT)) return 1;
  wifi_socket_ready = 1;
  return 0;
}

uint8_t WIFI_SPI_IsReady(void) { return wifi_socket_ready; }
