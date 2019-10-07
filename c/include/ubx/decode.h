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

ubx_rc ubx_decode_rawx(const uint8_t buff[], ubx_rawx *msg_rawx);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UBX_DECODE_H */