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

#include "check_suites.h"

#include <ubx/decode.h>
#include <ubx/encode.h>
#include <ubx/ubx_messages.h>

#include <assert.h>
#include <check.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLOAT_EPS 1e-6

void msgrawx_equals(ubx_rawx *msg_in, ubx_rawx *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert(fabs(msg_in->rcv_tow - msg_out->rcv_tow) < FLOAT_EPS);
  ck_assert_uint_eq(msg_in->rcv_wn, msg_out->rcv_wn);
  ck_assert_uint_eq(msg_in->num_meas, msg_out->num_meas);
  ck_assert_uint_eq(msg_in->rec_status, msg_out->rec_status);
  ck_assert_uint_eq(msg_in->version, msg_out->version);

  for (int i = 0; i < msg_in->num_meas; i++) {
    printf("%f %f\n", msg_in->pseudorange_m[i], msg_out->pseudorange_m[i]);
    ck_assert(fabs(msg_in->pseudorange_m[i] - msg_out->pseudorange_m[i]) <
              FLOAT_EPS);
    ck_assert(fabs(msg_in->carrier_phase_cycles[i] -
                   msg_out->carrier_phase_cycles[i]) < FLOAT_EPS);
    ck_assert(fabs(msg_in->doppler_hz[i] - msg_out->doppler_hz[i]) < FLOAT_EPS);
    ck_assert_uint_eq(msg_in->gnss_id[i], msg_out->gnss_id[i]);
    ck_assert_uint_eq(msg_in->sat_id[i], msg_out->sat_id[i]);
    ck_assert_uint_eq(msg_in->sig_id[i], msg_out->sig_id[i]);
    ck_assert_uint_eq(msg_in->freq_id[i], msg_out->freq_id[i]);
    ck_assert_uint_eq(msg_in->lock_time[i], msg_out->lock_time[i]);
    ck_assert_uint_eq(msg_in->cno_dbhz[i], msg_out->cno_dbhz[i]);
    ck_assert_uint_eq(msg_in->pr_std_m[i], msg_out->pr_std_m[i]);
    ck_assert_uint_eq(msg_in->cp_std_cycles[i], msg_out->cp_std_cycles[i]);
    ck_assert_uint_eq(msg_in->doppler_std_hz[i], msg_out->doppler_std_hz[i]);
    ck_assert_uint_eq(msg_in->track_state[i], msg_out->track_state[i]);
  }
}

START_TEST(test_ubx_rawx) {

  ubx_rawx msg;

  msg.class_id = 0x02;
  msg.msg_id = 0x15;
  msg.rcv_tow = 415374000;
  msg.rcv_wn = 2015;
  msg.leap_second = 15;
  msg.num_meas = 3;
  msg.rec_status = 7;
  msg.version = 9;

  msg.pseudorange_m[0] = 2179844.3;
  msg.carrier_phase_cycles[0] = 1234.7;
  msg.doppler_hz[0] = 21794.5;
  msg.gnss_id[0] = 15;
  msg.sat_id[0] = 15;
  msg.sig_id[0] = 15;
  msg.freq_id[0] = 5;
  msg.lock_time[0] = 42;
  msg.cno_dbhz[0] = 44;
  msg.pr_std_m[0] = 7;
  msg.cp_std_cycles[0] = 25;
  msg.doppler_std_hz[0] = 2;
  msg.track_state[0] = 12;

  msg.pseudorange_m[1] = 200.3;
  msg.carrier_phase_cycles[1] = 134.7;
  msg.doppler_hz[1] = 2194.5;
  msg.gnss_id[1] = 13;
  msg.sat_id[1] = 7;
  msg.sig_id[1] = 4;
  msg.freq_id[1] = -6;
  msg.lock_time[1] = 47;
  msg.cno_dbhz[1] = 40;
  msg.pr_std_m[1] = 6;
  msg.cp_std_cycles[1] = 3;
  msg.doppler_std_hz[1] = 1;
  msg.track_state[1] = 9;

  msg.pseudorange_m[2] = 199888.5;
  msg.carrier_phase_cycles[2] = 54321.7;
  msg.doppler_hz[2] = 29863.5;
  msg.gnss_id[2] = 1;
  msg.sat_id[2] = 150;
  msg.sig_id[2] = 12;
  msg.freq_id[2] = 0;
  msg.lock_time[2] = 49;
  msg.cno_dbhz[2] = 33;
  msg.pr_std_m[2] = 7;
  msg.cp_std_cycles[2] = 1;
  msg.doppler_std_hz[2] = 2;
  msg.track_state[2] = 3;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ubx_encode_rawx(&msg, buff);

  ubx_rawx msg_rawx_out;
  int8_t ret = ubx_decode_rawx(buff, &msg_rawx_out);
  ck_assert_int_eq(RC_OK, ret);
  msgrawx_equals(&msg, &msg_rawx_out);

  return;
}

END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("ubx");

  TCase *tc_ubx = tcase_create("ubx");
  tcase_add_test(tc_ubx, test_ubx_rawx);
  suite_add_tcase(s, tc_ubx);

  return s;
}
