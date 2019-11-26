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

#include <swiftnav/bits.h>

// UBX protocol is little-endian. This file assumes that the host system is also
// little endian.
// TODO(yizhe) write endian-independent code

/** Writes checksum over `length` bytes of `buf` into `CK_A` and `CK_B`. The
 * `length` includes class id, msg id, length bytes, and payload. Uses the 8-Bit
 * Fletcher Algorithm.
 *
 * \param buff Buffer containing a ubx message.
 * \param length Length of the ubx message
 * \param CK_A first byte of checksum
 * \param CK_B second byte of checksum
 */
void ubx_checksum(const uint8_t buff[], size_t length, uint8_t *checksum) {
  uint8_t *CK_A = checksum;
  uint8_t *CK_B = checksum + 1;
  *CK_A = 0;
  *CK_B = 0;
  for (size_t i = 0; i < length; i++) {
    *CK_A = *CK_A + buff[i];
    *CK_B = *CK_B + *CK_A;
  }
}

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

/** Write `len` bytes from buffer `buff` into destination `dest`
 * UBX is little-endian, so directly using ubx_getbitul swaps the endian-ness.
 * Assumes host system is little-endian
 *
 * \param buff
 * \param index Index in buffer of start of payload in bytes.
 * \param len Number of bytes to retrieve.
 */
void ubx_get_bytes(const uint8_t *buff, uint32_t index, uint8_t len, u8 *dest) {
  for (int i = 0; i < len; i++) {
    dest[i] = ubx_getbitul(buff, (index + i) * 8, 8);
  }
}

/** Deserialize the ubx_rxm_rawx message
 *
 * \param buff incoming data buffer
 * \param msg_rawx UBX rawx message
 * \return UBX return code
 */
ubx_rc ubx_decode_rxm_rawx(const uint8_t buff[], ubx_rxm_rawx *msg_rawx) {
  assert(msg_rawx);
  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->msg_id);
  byte += 1;

  if (msg_rawx->class_id != 0x02) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_rawx->msg_id != 0x15) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_rawx->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 8, (u8 *)&msg_rawx->rcv_tow);
  byte += 8;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_rawx->rcv_wn);
  byte += 2;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->leap_second);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->num_meas);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->rec_status);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->version);
  byte += 1;
  for (int i = 0; i < 2; i++) {
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->reserved1[i]);
    byte += 1;
  }

  for (int i = 0; i < msg_rawx->num_meas; i++) {
    ubx_get_bytes(buff, byte, 8, (u8 *)&msg_rawx->pseudorange_m[i]);
    byte += 8;
    ubx_get_bytes(buff, byte, 8, (u8 *)&msg_rawx->carrier_phase_cycles[i]);
    byte += 8;
    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_rawx->doppler_hz[i]);
    byte += 4;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->gnss_id[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->sat_id[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->sig_id[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->freq_id[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 2, (u8 *)&msg_rawx->lock_time[i]);
    byte += 2;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->cno_dbhz[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->pr_std_m[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->cp_std_cycles[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->doppler_std_hz[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->track_state[i]);
    byte += 1;
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rawx->reserved2[i]);
    byte += 1;
  }
  return RC_OK;
}

/** Deserialize the ubx_nav_clock message
 *
 * \param buff incoming data buffer
 * \param msg_nav_clock UBX nav clock message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_clock(const uint8_t buff[],
                            ubx_nav_clock *msg_nav_clock) {
  assert(msg_nav_clock);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_clock->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_clock->msg_id);
  byte += 1;

  if (msg_nav_clock->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_clock->msg_id != UBX_MSG_NAV_CLOCK) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_nav_clock->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_clock->i_tow);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_clock->clk_bias);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_clock->clk_drift);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_clock->time_acc);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_clock->freq_acc);
  byte += 4;

  return RC_OK;
}

/** Deserialize the ubx_nav_pvt message
 *
 * \param buff incoming data buffer
 * \param msg_nav_pvt UBX nav pvt message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_pvt(const uint8_t buff[], ubx_nav_pvt *msg_nav_pvt) {
  assert(msg_nav_pvt);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->msg_id);
  byte += 1;

  if (msg_nav_pvt->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_pvt->msg_id != UBX_MSG_NAV_PVT) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_nav_pvt->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->i_tow);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_nav_pvt->year);
  byte += 2;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->month);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->day);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->hour);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->min);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->sec);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->valid);
  byte += 1;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->time_acc);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->nano);
  byte += 4;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->fix_type);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->flags);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->flags2);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->num_sats);
  byte += 1;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->lon);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->lat);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->height);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->height_mean_sea_level);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->horizontal_accuracy);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->vertical_accuracy);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->vel_north);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->vel_east);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->vel_down);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->ground_speed);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->heading_of_motion);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->speed_acc);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->heading_acc);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_nav_pvt->PDOP);
  byte += 2;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->flags3);
  byte += 1;
  /* reserved */
  for (int i = 0; i < 5; i++) {
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_nav_pvt->reserved1[i]);
    byte += 1;
  }

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_nav_pvt->heading_vehicle);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_nav_pvt->magnetic_declination);
  byte += 2;
  ubx_get_bytes(buff, byte, 2,
                (u8 *)&msg_nav_pvt->magnetic_declination_accuracy);
  byte += 2;
  return RC_OK;
}

/** Deserialize the ubx_mga_gps_eph message
 *
 * \param buff incoming data buffer
 * \param ubx_mga_gps_eph UBX mga gps eph message
 * \return UBX return code
 */
ubx_rc ubx_decode_mga_gps_eph(const uint8_t buff[],
                              ubx_mga_gps_eph *msg_mga_gps_eph) {
  assert(msg_mga_gps_eph);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->msg_id);
  byte += 1;

  if (msg_mga_gps_eph->class_id != 0x13) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_mga_gps_eph->msg_id != 0x00) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->msg_type);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->version);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->sat_id);
  byte += 1;
  /* reserved */
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->reserved1);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->fit_interval);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->ura_index);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->sat_health);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->tgd);
  byte += 1;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->iodc);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->toc);
  byte += 2;
  /* reserved */
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->reserved2);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->af2);
  byte += 1;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->af1);
  byte += 2;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->af0);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->crs);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->delta_N);
  byte += 2;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->m0);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->cuc);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->cus);
  byte += 2;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->e);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->sqrt_A);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->toe);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->cic);
  byte += 2;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->omega0);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->cis);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->crc);
  byte += 2;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->i0);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->omega);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_mga_gps_eph->omega_dot);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_mga_gps_eph->i_dot);
  byte += 2;
  /* reserved */
  for (int i = 0; i < 2; i++) {
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_mga_gps_eph->reserved3[i]);
    byte += 1;
  }

  return RC_OK;
}

/** Deserialize the ubx_rxm_sfrbx message
 *
 * \param buff incoming data buffer
 * \param ubx_rxm_sfrbx UBX rxm sfrbx message
 * \return UBX return code
 */
ubx_rc ubx_decode_rxm_sfrbx(const uint8_t buff[],
                            ubx_rxm_sfrbx *msg_rxm_sfrbx) {
  assert(msg_rxm_sfrbx);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->msg_id);
  byte += 1;

  if (msg_rxm_sfrbx->class_id != 0x02) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_rxm_sfrbx->msg_id != 0x13) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_rxm_sfrbx->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->gnss_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->sat_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->reserved1);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->freq_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->num_words);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->channel);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->version);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_rxm_sfrbx->reserved2);
  byte += 1;
  for (int i = 0; i < msg_rxm_sfrbx->num_words; i++) {
    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_rxm_sfrbx->data_words[i]);
    byte += 4;
  }

  return RC_OK;
}

/** Deserialize the ubx_esf_ins message
 *
 * \param buff incoming data buffer
 * \param msg_esf_ins UBX esf ins message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_ins(const uint8_t buff[], ubx_esf_ins *msg_esf_ins) {
  assert(msg_esf_ins);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_ins->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_ins->msg_id);
  byte += 1;

  if (msg_esf_ins->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_ins->msg_id != UBX_MSG_ESF_INS) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_esf_ins->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->bitfield0);
  byte += 4;

  for (int i = 0; i < 4; i++) {
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_ins->reserved1[i]);
    byte += 1;
  }

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->i_tow);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->x_ang_rate);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->y_ang_rate);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->z_ang_rate);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->x_accel);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->y_accel);
  byte += 4;
  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_ins->z_accel);
  byte += 4;

  return RC_OK;
}

/** Deserialize the ubx_esf_meas message
 *
 * \param buff incoming data buffer
 * \param msg_esf_meas UBX esf meas message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_meas(const uint8_t buff[], ubx_esf_meas *msg_esf_meas) {
  assert(msg_esf_meas);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_meas->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_meas->msg_id);
  byte += 1;

  if (msg_esf_meas->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_meas->msg_id != UBX_MSG_ESF_MEAS) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_esf_meas->length);
  byte += 2;

  ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_meas->time_tag);
  byte += 4;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_esf_meas->flags);
  byte += 2;
  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_esf_meas->id);
  byte += 2;

  u8 num_meas = (msg_esf_meas->flags >> 11) & 0x1F;
  for (int i = 0; i < num_meas; i++) {
    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_meas->data[i]);
    byte += 4;
  }

  bool has_calib = msg_esf_meas->flags & 0x8;
  if (has_calib) {
    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_meas->calib_tag);
    byte += 4;
  }

  return RC_OK;
}

/** Deserialize the ubx_esf_raw message
 *
 * \param buff incoming data buffer
 * \param msg_esf_raw UBX esf raw message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_raw(const uint8_t buff[], ubx_esf_raw *msg_esf_raw) {
  assert(msg_esf_raw);

  uint16_t byte = 0;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_raw->class_id);
  byte += 1;
  ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_raw->msg_id);
  byte += 1;

  if (msg_esf_raw->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_raw->msg_id != UBX_MSG_ESF_RAW) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ubx_get_bytes(buff, byte, 2, (u8 *)&msg_esf_raw->length);
  byte += 2;
  int num_measurements = (msg_esf_raw->length - 4) / 8;
  assert(num_measurements <= ESF_DATA_MAX_COUNT);

  for (int i = 0; i < 4; i++) {
    ubx_get_bytes(buff, byte, 1, (u8 *)&msg_esf_raw->reserved1[i]);
    byte += 1;
  }

  for (int i = 0; i < num_measurements; i++) {
    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_raw->data[i]);
    byte += 4;

    ubx_get_bytes(buff, byte, 4, (u8 *)&msg_esf_raw->sensor_time_tag[i]);
    byte += 4;
  }

  return RC_OK;
}
