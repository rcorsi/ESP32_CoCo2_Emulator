/******************************************************************************
 * File         : BeckerPort.h
 * Description  : Becker Port (Becker/DriveWire-style TCP interface) at
 *                $FF41 (status) / $FF42 (data), mirroring VCC's becker.dll
 *                behavior so an existing Becker-aware PC-side bridge app
 *                can talk to this emulator the same way it talks to VCC.
 *
 * IMPORTANT: The read/write hooks (BeckerPort_ReadStatus/ReadData/WriteData)
 * are called from ManagePeripherals_Read/Write, which run inside the
 * CPU timer ISR (onCPUTimer). They must stay fast and non-blocking --
 * no socket calls, no delay(), no locks that can be held by a lower
 * priority task. All real network I/O happens in BeckerNetworkTask,
 * a normal FreeRTOS task, and the two sides only ever touch a pair of
 * lock-free single-producer/single-consumer ring buffers.
 ******************************************************************************/

#ifndef __BECKER_PORT_H
#define __BECKER_PORT_H

#include <Arduino.h>

// WiFi SSID/password, bridge-app server IP/port -- editable from the F12
// emulator menu (WiFi / Becker Port setup) and persisted to /becker.cfg
// on the SD card, the same way disk drive assignments are persisted.
typedef struct
{
  char     SSID[33];
  char     Password[65];
  char     ServerIP[16];
  uint16_t Port;
} BeckerConfig_t;

extern BeckerConfig_t BeckerConfig;

// Call once from setup(). Loads config from SD (or falls back to the
// compiled-in defaults), starts WiFi (non-blocking), and starts the
// network task.
void BeckerPort_Init(void);

// Load/save BeckerConfig to/from "/becker.cfg" on the SD card.
void BeckerPort_LoadConfig(void);
void BeckerPort_SaveConfig(void);

// Call after the menu changes BeckerConfig (SSID/Password/ServerIP/Port)
// to drop the current connection and reconnect with the new settings.
// Safe to call from any task/core -- the actual socket teardown happens
// inside BeckerNetworkTask itself.
void BeckerPort_ApplyConfig(void);

// ISR-safe accessors -- call only from ManagePeripherals_Read/Write.
uint8_t IRAM_ATTR BeckerPort_ReadStatus(void);
uint8_t IRAM_ATTR BeckerPort_ReadData(void);
void IRAM_ATTR BeckerPort_WriteData(uint8_t value);

// For the emulator menu / debug prints.
bool BeckerPort_IsConnected(void);

#endif
