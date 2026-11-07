// Mirror of Satellite1-XMOS
// satellite-xmos-firmware/audio_pipelines/reference/fixed_delay_array/array_diag.h
// (ARRAY_DIAG_VERSION 1). Keep the two in step; the version byte guards it.
//
// Reading these over Satellite1::transfer() as it is in this tree
// (Satellite1-ESPHome v0.2.1-beta.0, 9a58961):
//
//   A read of N bytes makes the XMOS answer [23 payload-available][servicer
//   status][N data bytes]. transfer() copies N bytes starting at the status
//   byte, so the caller's buffer holds buf[0] = status (0 = success) and
//   buf[1..N-1] = data bytes 0..N-2. The last data byte is not delivered.
//   Asking for N + 1 bytes does not help: the XMOS servicer then rejects the
//   length.
//
//   - xmos_array_diag_status_t: the missing byte is the top byte of
//     t1_heap_min_free, which is always 0 (a tile has 512 KiB of RAM).
//   - xmos_array_diag_select_t: the missing byte is reserved_end.
//
//   Later Satellite1-ESPHome versions strip the status byte inside transfer();
//   xmos_array_read() below is written for this tree only.
#pragma once

#include <cstdint>
#include <cstring>

static constexpr uint8_t XMOS_ARRAY_DIAG_RESID = 233;
static constexpr uint8_t XMOS_ARRAY_DIAG_VERSION = 1;
static constexpr uint8_t XMOS_ARRAY_DIAG_CMD_GET_STATUS = 0x80 | 0;
static constexpr uint8_t XMOS_ARRAY_DIAG_CMD_SET_TAP = 1;
static constexpr uint32_t XMOS_ARRAY_DIAG_PUBLISH_FRAMES = 16;  // the XMOS publishes every 16 frames (240 ms)
static constexpr float XMOS_REF_TICKS_PER_MS = 100000.0f;        // 100 MHz reference clock

struct __attribute__((packed)) xmos_array_diag_status_t {
  uint8_t version;
  uint8_t frontend_mode;   // 1 = pair select, 2 = beams
  uint8_t selected;        // pair (0 = E/W, 1 = N/S) or beam (0..7, 45 degree steps)
  uint8_t ref_mix;         // 0 = (L+R)/2, 1 = left, 2 = right
  uint32_t frames;
  uint32_t switch_count;
  int16_t raw_rms_cdb[4];
  int16_t raw_peak_cdb[4];
  uint32_t raw_clip_count[4];
  int16_t aec_rms_cdb[4];
  int16_t aec_erle_cdb[4];
  int16_t frontend_rms_cdb;
  int16_t output_rms_cdb;
  int16_t score_cdb[8];
  uint16_t vnr_x10000;
  uint8_t ref_active;
  uint8_t tap_mode;        // 0xFF: tap not compiled in
  uint8_t tap_a;
  uint8_t tap_b;
  uint32_t t1_aec_ticks_avg;
  uint32_t t1_aec_ticks_max;
  uint32_t t0_frontend_ticks_avg;
  uint32_t t0_frontend_ticks_max;
  uint32_t t0_ns_ticks_max;
  uint32_t t0_agc_ticks_max;
  uint32_t deadline_misses;
  uint32_t t1_input_late_ticks_max;
  uint32_t t1_dropped_frames_est;
  uint32_t t0_heap_min_free;
  uint32_t t1_heap_min_free;
};

static_assert(sizeof(xmos_array_diag_status_t) == 130, "layout must match the XMOS firmware");

// Selector internals (array_diag_select_t). Only XMOS firmware with the selector
// diagnostics patch answers these; older four-mic builds reject the command.
static constexpr uint8_t XMOS_ARRAY_DIAG_CMD_GET_SELECT = 0x80 | 2;
static constexpr uint8_t XMOS_ARRAY_DIAG_CMD_SET_SELECT = 3;   // dev-tap builds only
static constexpr uint8_t XMOS_ARRAY_DIAG_SELECT_VERSION = 1;

struct __attribute__((packed)) xmos_array_diag_select_t {
  uint8_t version;
  uint8_t gate;                 // 0 = output VNR (default), 1 = input VNR
  uint8_t challenger;           // == selected when nothing is being confirmed
  uint8_t confirm_count;
  uint32_t frames;
  uint32_t frames_since_switch;
  uint16_t input_vnr_x10000;    // selected path before IC, smoothed
  uint16_t output_vnr_x10000;   // selected path after IC, smoothed
  uint16_t window_frames;
  uint16_t window_valid_frames; // frames in the window that updated the scores
  int16_t raw_lead_max;         // beams: centi-dB; pairs: VNR x 10000
  uint8_t raw_lead_max_candidate;
  uint8_t reserved;
  int16_t margin;               // same units as raw_lead_max
  uint16_t confirm_frames;
  uint16_t min_dwell_frames;
  int16_t raw_score[8];         // unsmoothed scores of the last valid frame
  uint8_t reserved_end;         // the byte transfer() drops (see top of file)
};

static_assert(sizeof(xmos_array_diag_select_t) == 47, "layout must match the XMOS firmware");

// Read one diagnostics structure. Returns 0 on success; otherwise the XMOS
// servicer status (non-zero), or 0xFF if the SPI transfer itself failed.
// On success every field except the last byte (see top of file) is filled in.
template<typename Dev, typename T> static inline uint8_t xmos_array_read(Dev &dev, uint8_t command, T *out) {
  static_assert(sizeof(T) >= 2 && sizeof(T) <= 255, "one device-control read");
  uint8_t buf[sizeof(T)];
  memset(buf, 0, sizeof(buf));
  if (!dev->transfer(XMOS_ARRAY_DIAG_RESID, command, buf, sizeof(buf))) {
    return 0xFF;
  }
  if (buf[0] != 0) {
    return buf[0];
  }
  memset(static_cast<void *>(out), 0, sizeof(T));
  memcpy(static_cast<void *>(out), &buf[1], sizeof(T) - 1);
  return 0;
}

// transfer() in this tree does not check the response frame type, so a
// device-control status frame ([1][0][4 status registers][zeros]) can look
// like a successful read. These reject it: a real snapshot has a non-zero
// frame count that is a multiple of the publish period.
static inline bool xmos_array_status_valid(const xmos_array_diag_status_t &st) {
  const uint32_t frames = st.frames;
  return st.version == XMOS_ARRAY_DIAG_VERSION && (st.frontend_mode == 1 || st.frontend_mode == 2) &&
         st.selected < 8 && frames != 0 && (frames % XMOS_ARRAY_DIAG_PUBLISH_FRAMES) == 0;
}

static inline bool xmos_array_select_valid(const xmos_array_diag_select_t &sx) {
  const uint32_t frames = sx.frames;
  return sx.version == XMOS_ARRAY_DIAG_SELECT_VERSION && sx.gate <= 1 && sx.challenger < 8 && frames != 0 &&
         (frames % XMOS_ARRAY_DIAG_PUBLISH_FRAMES) == 0;
}
