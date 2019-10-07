/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_UBX_MESSAGES_H
#define SWIFTNAV_UBX_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#include <ubx/constants.h>

/* return codes for the decoders */
typedef enum rtcm3_rc_e {
  RC_OK = 0,
  RC_MESSAGE_TYPE_MISMATCH = -1,
  RC_INVALID_MESSAGE = -2
} ubx_rc;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  double rcv_tow;
  uint16_t rcv_wn;
  uint8_t leap_second;
  uint8_t num_meas;
  uint8_t rec_status;
  uint8_t version;
  double pseudorange_m[MAX_NUM_SATS];
  double carrier_phase_cycles[MAX_NUM_SATS];
  float doppler_hz[MAX_NUM_SATS];
  uint32_t gnss_id[MAX_NUM_SATS];
  uint32_t sat_id[MAX_NUM_SATS];
  uint32_t sig_id[MAX_NUM_SATS];
  uint32_t freq_id[MAX_NUM_SATS];
  uint32_t lock_time[MAX_NUM_SATS];
  uint32_t cno_dbhz[MAX_NUM_SATS];
  uint32_t pr_std_m[MAX_NUM_SATS];
  uint32_t cp_std_cycles[MAX_NUM_SATS];
  uint32_t doppler_std_hz[MAX_NUM_SATS];
  uint32_t track_state[MAX_NUM_SATS];
} ubx_rawx;

#endif /* SWIFTNAV_UBX_MESSAGES_H */