/******************************************************************************
 * File         : BeckerPort.cpp
 * Description  : See BeckerPort.h
 ******************************************************************************/

// NOTE: We need access to the USB Soft Host so that we can Pause it as during
// certain WiFi activities or else the following Panic will occur:
//
// Guru Meditation Error: Core  1 panic'ed (Cache disabled but cached memory region accessed).
//
// The crash/panic happens because Wi-Fi functions (like NVS flash calibration
// writes) are disabling the flash cache, while the USB Soft Host library's
// high-priority interrupt is firing at the exact same time and attempting to
// read code/data from flash memory.
//

#include "BeckerPort.h"
#include "SD_MMC.h"
#include <WiFi.h>

#include <ESP32-USB-Soft-Host.h>

// Ring buffer size -- must be a power of two.
#define BECKER_BUF_SIZE 1024
#define BECKER_BUF_MASK (BECKER_BUF_SIZE - 1)

typedef struct
{
  volatile uint8_t  data[BECKER_BUF_SIZE];
  volatile uint16_t head;   // next slot the producer will write
  volatile uint16_t tail;   // next slot the consumer will read
} BeckerRing_t;

// RxRing : network task (producer) -> ISR (consumer)   [server -> CoCo]
// TxRing : ISR (producer) -> network task (consumer)   [CoCo -> server]
static BeckerRing_t RxRing;
static BeckerRing_t TxRing;

static WiFiClient BeckerClient;
static volatile bool BeckerConnected = false;
static volatile bool BeckerReconnectRequested = false;

#define BECKER_CFG_FILE "/becker.cfg"

// Compiled-in defaults, used the first time (no /becker.cfg on the SD card
// yet) or if the saved file is missing/corrupt. Edit from the F12 menu
// afterwards -- no need to touch these again.
BeckerConfig_t BeckerConfig =
{
  "YOUR_WIFI_SSID",
  "YOUR_WIFI_PASSWORD",
  "192.168.1.100",
  65504
};

// Single-producer/single-consumer ring -- safe without locks as long as
// each side only ever touches its own end (head for the producer,
// tail for the consumer). uint16_t read/modify/write is atomic on the
// Xtensa core for aligned access, which is all we need here.

static inline bool IRAM_ATTR Ring_Push(volatile BeckerRing_t *r, uint8_t b)
{
  uint16_t next = (r->head + 1) & BECKER_BUF_MASK;
  if (next == r->tail)
  {
    return false; // full -- caller decides how to handle it
  }
  r->data[r->head] = b;
  r->head = next;
  return true;
}

static inline bool IRAM_ATTR Ring_Pop(volatile BeckerRing_t *r, uint8_t *b)
{
  if (r->head == r->tail)
  {
    return false; // empty
  }
  *b = r->data[r->tail];
  r->tail = (r->tail + 1) & BECKER_BUF_MASK;
  return true;
}

static inline bool IRAM_ATTR Ring_IsEmpty(volatile BeckerRing_t *r)
{
  return r->head == r->tail;
}

// ============================================================================
// ISR-side API -- called from ManagePeripherals_Read/Write inside onCPUTimer.
// Must stay fast: no blocking, no socket calls, no Serial prints.
// ============================================================================

uint8_t IRAM_ATTR BeckerPort_ReadStatus(void)
{
  // Bit 1 (mask 0x02) = 1: a byte is available to read at $FF42.
  // Confirmed against the CoCo-side ADDASSEM polling loop:
  //   LDA #2 / BITA $FF41 / BEQ @Loop1 / LDA $FF42
  // which spins while (A AND [FF41]) == 0, i.e. while bit 1 is clear.
  return Ring_IsEmpty(&RxRing) ? 0x00 : 0x02;
}

uint8_t IRAM_ATTR BeckerPort_ReadData(void)
{
  uint8_t b = 0;
  Ring_Pop(&RxRing, &b); // if empty, returns 0 -- driver should check status first
  return b;
}

void IRAM_ATTR BeckerPort_WriteData(uint8_t value)
{
  Ring_Push(&TxRing, value); // dropped only if the CoCo massively outruns WiFi
}

bool BeckerPort_IsConnected(void)
{
  return BeckerConnected;
}

// ============================================================================
// Config persistence -- mirrors the project's existing SaveConfigToSD /
// LoadConfigFromSD pattern (raw fixed-size struct on the SD card).
// ============================================================================

void BeckerPort_LoadConfig(void)
{
  File f = SD_MMC.open(BECKER_CFG_FILE, FILE_READ);
  if (!f)
  {
    Serial.println("[Becker] No saved config found, using defaults.");
    return;
  }

  size_t n = f.read((uint8_t *)&BeckerConfig, sizeof(BeckerConfig));
  f.close();

  if (n != sizeof(BeckerConfig))
  {
    Serial.println("[Becker] Config file size mismatch, using defaults.");
    return;
  }

  // Defensive NUL-termination in case of a corrupt/foreign file.
  BeckerConfig.SSID[sizeof(BeckerConfig.SSID) - 1] = '\0';
  BeckerConfig.Password[sizeof(BeckerConfig.Password) - 1] = '\0';
  BeckerConfig.ServerIP[sizeof(BeckerConfig.ServerIP) - 1] = '\0';

  Serial.println("[Becker] Config loaded from SD.");
}

void BeckerPort_SaveConfig(void)
{
  File f = SD_MMC.open(BECKER_CFG_FILE, FILE_WRITE);
  if (!f)
  {
    Serial.println("[Becker] Failed to open config file for writing.");
    return;
  }
  f.write((uint8_t *)&BeckerConfig, sizeof(BeckerConfig));
  f.close();
  Serial.println("[Becker] Config saved to SD.");
}

void BeckerPort_ApplyConfig(void)
{
  // WiFi.* calls are fine to make from any task/core -- they go through
  // the Arduino WiFi wrapper's own event loop, not raw socket state.
  WiFi.disconnect(true);
  delay(10);

  // See description at top of file for reason for pausing USB Soft Host
  USH.TimerPause();

  WiFi.begin(BeckerConfig.SSID, BeckerConfig.Password);

  for (auto i=0; i<10; ++i)
  {
    if (WiFi.status() == WL_CONNECTED)
      break;
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    Serial.println("WiFi failed to connect");
  }
  else
  {
    Serial.println("WiFi connected");
  }

  USH.TimerResume();

  // The actual TCP socket object (BeckerClient) is only ever touched by
  // BeckerNetworkTask itself, to avoid two cores mutating it at once.
  // This flag tells that task to drop its current connection and pick
  // up the (possibly changed) ServerIP/Port on its next pass.
  BeckerReconnectRequested = true;
}

// ============================================================================
// Networking task -- normal FreeRTOS task, NOT ISR context. Owns the actual
// TCP socket and is the only thing that ever calls into WiFiClient.
// ============================================================================

static void BeckerNetworkTask(void *pvParameters)
{
  uint8_t txBuf[128];
  uint8_t rxBuf[128];

  for (;;)
  {
    if (BeckerReconnectRequested)
    {
      BeckerClient.stop();
      BeckerConnected = false;
      BeckerReconnectRequested = false;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
      BeckerConnected = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (!BeckerClient.connected())
    {
      BeckerConnected = false;
      if (BeckerClient.connect(BeckerConfig.ServerIP, BeckerConfig.Port))
      {
        // Wait a brief moment (10–50ms) for the LWIP socket descriptor to finish settling
        unsigned long startMilli = millis();
        while (!BeckerClient.connected() && (millis() - startMilli < 1000))
        {
          delay(10);
        }

        // Now check if it is officially connected and stable
        if (!BeckerClient.connected())
        {
          Serial.printf("BECKER not CONNECTED1, Error: %d\n", errno);
          vTaskDelay(pdMS_TO_TICKS(1000)); // retry in 1s
          continue;
        }

        Serial.println("BECKER CONNECTED");
        BeckerClient.setNoDelay(true); // Becker traffic is latency sensitive, small packets
        BeckerConnected = true;
      }
      else
      {
        Serial.printf("BECKER not CONNECTED2, Error: %d\n", errno);
        vTaskDelay(pdMS_TO_TICKS(1000)); // retry in 1s
        continue;
      }
    }

    // ---- Drain TxRing (CoCo -> server) ----
    int txCount = 0;
    uint8_t b;
    while (txCount < (int)sizeof(txBuf) && Ring_Pop(&TxRing, &b))
    {
      txBuf[txCount++] = b;
    }
    if (txCount > 0)
    {
      BeckerClient.write(txBuf, txCount);
    }

    // ---- Fill RxRing (server -> CoCo) ----
    int avail = BeckerClient.available();
    if (avail > 0)
    {
      int toRead = avail > (int)sizeof(rxBuf) ? (int)sizeof(rxBuf) : avail;
      int n = BeckerClient.read(rxBuf, toRead);
      for (int i = 0; i < n; i++)
      {
        if (!Ring_Push(&RxRing, rxBuf[i]))
        {
          // RxRing full -- leave the rest sitting in the socket's own
          // receive buffer, we'll pick it up on the next pass.
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // short yield, keeps latency low without hogging core 0
  }
}

void BeckerPort_Init(void)
{
  RxRing.head = RxRing.tail = 0;
  TxRing.head = TxRing.tail = 0;

  BeckerPort_LoadConfig();

  // See description at top of file for reason for pausing USB Soft Host
  USH.TimerPause();

  WiFi.mode(WIFI_STA);

  USH.TimerResume();

  xTaskCreatePinnedToCore(
      BeckerNetworkTask,
      "BeckerNet",
      4096,
      NULL,
      1,    // lower priority than VideoCore
      NULL,
      0     // core 0 -- keeps it off the core running VideoCore + the CPU timer ISR
  );
}
