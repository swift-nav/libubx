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

/* These encoder functions are provided to allow for easier unit testing however
 * they are not robust to be used in production. Before using with real data, we
 * would need to handle ambiguity rollover and code carrier divergance at least
 */

#ifndef SWIFTNAV_UBX_ENCODE_H
#define SWIFTNAV_UBX_ENCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ubx/ubx_messages.h"
#include <ubx/constants.h>

uint16_t ubx_encode_rawx(const ubx_rawx *msg_rawx, uint8_t buff[]);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UBX_ENCODE_H */