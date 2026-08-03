#pragma once
#include <Arduino.h>

// PDS
static constexpr uint32_t UART_DBG_BAUD        = 1'000'000UL; // 1 Mbps
#define UART_DBG Serial3

static constexpr uint32_t PARAM_SLOW_MAGIC        = 0xCAFEFEEDUL;
static constexpr int      PARAM_SLOW_FLOAT_COUNT  = 50;
static constexpr int      PARAM_SLOW_BOOL_COUNT   = 50;
static constexpr int      PARAM_HEADER_BYTES      = 8;   // magic(4) + seq(4), fuer beide Pakettypen gleich
static constexpr int      PARAM_SLOW_PACKET_BYTES =
    PARAM_HEADER_BYTES + PARAM_SLOW_FLOAT_COUNT * 4 + PARAM_SLOW_BOOL_COUNT;      // 258

static constexpr uint32_t PARAM_FAST_MAGIC        = 0xFA57DA7AUL;
static constexpr int      PARAM_FAST_FLOAT_COUNT  = 5;
static constexpr int      PARAM_FAST_PACKET_BYTES =
    PARAM_HEADER_BYTES + PARAM_FAST_FLOAT_COUNT * 4;                             // 28

static constexpr uint32_t PARAM_SLOW_TIMEOUT_MS = 1000;
static constexpr uint32_t PARAM_FAST_TIMEOUT_MS = 150;

static constexpr uint32_t CHANNEL_DESC_MAGIC         = 0xDE5C0001UL;
static constexpr uint32_t CHANNEL_DESC_REQUEST_MAGIC = 0xDE5C00F0UL;

static constexpr int CHANNEL_DESC_CHUNK_PAYLOAD_MAX = 250;   // Bytes JSON-Text pro Chunk
static constexpr int CHANNEL_DESC_CHUNK_HEADER_BYTES = 7;    // magic(4) + chunk_idx(1) + chunk_count(1) + payload_len(1)
static constexpr int CHANNEL_DESC_CHUNK_PACKET_BYTES =
    CHANNEL_DESC_CHUNK_HEADER_BYTES + CHANNEL_DESC_CHUNK_PAYLOAD_MAX;   // 257
static constexpr int CHANNEL_DESC_REQUEST_PACKET_BYTES = 4;             // nur Magic

static constexpr size_t CHANNEL_DESC_JSON_BUF_BYTES = 12 * 1024;   // reicht fuer 200+105 Namen + ~30 Overlays
static constexpr uint8_t CHANNEL_NAME_MAXLEN = 24;                 // inkl. Nullterminator
