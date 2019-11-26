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

#ifndef SWIFTNAV_UBX_DECODE_H
#define SWIFTNAV_UBX_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ubx/constants.h>
#include <ubx/ubx_messages.h>

void ubx_checksum(const uint8_t buff[], size_t length, uint8_t *checksum);
ubx_rc ubx_decode_rxm_rawx(const uint8_t buff[], ubx_rxm_rawx *msg_rawx);
ubx_rc ubx_decode_nav_clock(const uint8_t buff[], ubx_nav_clock *msg_nav_clock);
ubx_rc ubx_decode_nav_pvt(const uint8_t buff[], ubx_nav_pvt *msg_nav_pvt);
ubx_rc ubx_decode_mga_gps_eph(const uint8_t buff[],
                              ubx_mga_gps_eph *msg_mga_gps_eph);
ubx_rc ubx_decode_rxm_sfrbx(const uint8_t buff[], ubx_rxm_sfrbx *msg_rxm_sfrbx);
ubx_rc ubx_decode_esf_ins(const uint8_t buff[], ubx_esf_ins *msg_esf_ins);
ubx_rc ubx_decode_esf_meas(const uint8_t buff[], ubx_esf_meas *msg_esf_meas);
ubx_rc ubx_decode_esf_raw(const uint8_t buff[], ubx_esf_raw *msg_esf_raw);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UBX_DECODE_H */
