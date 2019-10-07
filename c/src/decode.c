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

#include <ubx/decode.h>

#include <stdio.h>
#include <swiftnav/bits.h>

/** Get bit field from buffer as an unsigned long integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as an unsigned value.
 */
uint64_t ubx_getbitul(const uint8_t *buff, uint32_t pos, uint8_t len) {
  uint64_t bits = 0;

  for (uint32_t i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }

  return bits;
}

/** Get bit field from buffer as a signed integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * This function sign extends the `len` bit field to a signed 64 bit integer.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as a signed value.
 */
int64_t ubx_getbitsl(const uint8_t *buff, uint32_t pos, uint8_t len) {
  int64_t bits = (int64_t)ubx_getbitul(buff, pos, len);

  /* Sign extend, taken from:
   * http://graphics.stanford.edu/~seander/bithacks.html#VariableSignExtend
   */
  int64_t m = ((uint64_t)1) << (len - 1);
  return (bits ^ m) - m;
}

/** Deserialize the ubx_rawx message
 *
 * \param buff incoming data buffer
 * \param msg_rawx UBX rawx message
 * \return UBX return code
 */
ubx_rc ubx_decode_rawx(const uint8_t buff[], ubx_rawx *msg_rawx) {
  assert(msg_rawx);
  uint16_t bit = 0;
  msg_rawx->class_id = getbitu(buff, bit, 8);
  bit += 8;
  msg_rawx->msg_id = getbitu(buff, bit, 8);
  bit += 8;

  if (msg_rawx->class_id != 0x02) {
    printf("%u %u\n", msg_rawx->class_id, 0x02);
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_rawx->msg_id != 0x15) {
    printf("%u %u\n", msg_rawx->msg_id, 0x15);
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  *(int64_t *)&msg_rawx->rcv_tow = ubx_getbitul(buff, bit, 64);
  bit += 64;
  msg_rawx->rcv_wn = getbitu(buff, bit, 16);
  bit += 16;
  msg_rawx->leap_second = getbits(buff, bit, 16);
  bit += 16;
  msg_rawx->num_meas = getbitu(buff, bit, 8);
  bit += 8;
  msg_rawx->rec_status = getbitu(buff, bit, 8);
  bit += 8;
  msg_rawx->version = getbitu(buff, bit, 8);
  bit += 8;
  /*reserved */ getbitu(buff, bit, 4);
  bit += 4;

  for (int i = 0; i < msg_rawx->num_meas; i++) {
    *(int64_t *)&msg_rawx->pseudorange_m[i] = ubx_getbitul(buff, bit, 64);
    bit += 64;
    *(int64_t *)&msg_rawx->carrier_phase_cycles[i] =
        ubx_getbitul(buff, bit, 64);
    bit += 64;
    *(int32_t *)&msg_rawx->doppler_hz[i] = getbitu(buff, bit, 32);
    bit += 32;
    msg_rawx->gnss_id[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->sat_id[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->sig_id[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->freq_id[i] = getbits(buff, bit, 8);
    bit += 8;
    msg_rawx->lock_time[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->cno_dbhz[i] = getbitu(buff, bit, 16);
    bit += 16;
    msg_rawx->pr_std_m[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->cp_std_cycles[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->doppler_std_hz[i] = getbitu(buff, bit, 8);
    bit += 8;
    msg_rawx->track_state[i] = getbitu(buff, bit, 8);
    bit += 8;
    /*reserved */ getbitu(buff, bit, 8);
    bit += 8;
  }
  return RC_OK;
}