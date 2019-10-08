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

void msg_rawx_equals(const ubx_rawx *msg_in, const ubx_rawx *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert(fabs(msg_in->rcv_tow - msg_out->rcv_tow) < FLOAT_EPS);
  ck_assert_uint_eq(msg_in->rcv_wn, msg_out->rcv_wn);
  ck_assert_uint_eq(msg_in->num_meas, msg_out->num_meas);
  ck_assert_uint_eq(msg_in->rec_status, msg_out->rec_status);
  ck_assert_uint_eq(msg_in->version, msg_out->version);

  for (int i = 0; i < msg_in->num_meas; i++) {
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

void msg_nav_pvt_equals(const ubx_nav_pvt *msg_in, const ubx_nav_pvt *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_in->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_in->msg_id);
  ck_assert_uint_eq(msg_in->i_tow, msg_in->i_tow);
  ck_assert_uint_eq(msg_in->year, msg_in->year);
  ck_assert_uint_eq(msg_in->month, msg_in->month);
  ck_assert_uint_eq(msg_in->day, msg_in->day);
  ck_assert_uint_eq(msg_in->hour, msg_in->hour);
  ck_assert_uint_eq(msg_in->min, msg_in->min);
  ck_assert_uint_eq(msg_in->sec, msg_in->sec);
  ck_assert_uint_eq(msg_in->valid, msg_in->valid);
  ck_assert_uint_eq(msg_in->time_accuracy, msg_in->time_accuracy);
  ck_assert_uint_eq(msg_in->nano, msg_in->nano);
  ck_assert_uint_eq(msg_in->fix_type, msg_in->fix_type);
  ck_assert_uint_eq(msg_in->flags, msg_in->flags);
  ck_assert_uint_eq(msg_in->flags2, msg_in->flags2);
  ck_assert_uint_eq(msg_in->flags3, msg_in->flags3);
  ck_assert_uint_eq(msg_in->num_sats, msg_in->num_sats);
  ck_assert_uint_eq(msg_in->lon, msg_in->lon);
  ck_assert_uint_eq(msg_in->lat, msg_in->lat);
  ck_assert_uint_eq(msg_in->height, msg_in->height);
  ck_assert_uint_eq(msg_in->height_mean_sea_level,
                    msg_in->height_mean_sea_level);
  ck_assert_uint_eq(msg_in->horizontal_accuracy, msg_in->horizontal_accuracy);
  ck_assert_uint_eq(msg_in->vertical_accuracy, msg_in->vertical_accuracy);
  ck_assert_uint_eq(msg_in->vel_north, msg_in->vel_north);
  ck_assert_uint_eq(msg_in->vel_east, msg_in->vel_east);
  ck_assert_uint_eq(msg_in->vel_down, msg_in->vel_down);
  ck_assert_uint_eq(msg_in->ground_speed, msg_in->ground_speed);
  ck_assert_uint_eq(msg_in->heading_of_motion, msg_in->heading_of_motion);
  ck_assert_uint_eq(msg_in->PDOP, msg_in->PDOP);
  ck_assert_uint_eq(msg_in->heading_vehicle, msg_in->heading_vehicle);
  ck_assert_uint_eq(msg_in->magnetic_declination, msg_in->magnetic_declination);
  ck_assert_uint_eq(msg_in->magnetic_declination_accuracy,
                    msg_in->magnetic_declination_accuracy);
}

void msg_mga_gps_eph_equals(const ubx_mga_gps_eph *msg_in,
                            const ubx_mga_gps_eph *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->msg_type, msg_out->msg_type);
  ck_assert_uint_eq(msg_in->version, msg_out->version);
  ck_assert_uint_eq(msg_in->sat_id, msg_out->sat_id);
  ck_assert_uint_eq(msg_in->fit_interval, msg_out->fit_interval);
  ck_assert_uint_eq(msg_in->ura_index, msg_out->ura_index);
  ck_assert_uint_eq(msg_in->sat_health, msg_out->sat_health);
  ck_assert_int_eq(msg_in->tgd, msg_out->tgd);
  ck_assert_uint_eq(msg_in->iodc, msg_out->iodc);
  ck_assert_uint_eq(msg_in->toc, msg_out->toc);
  ck_assert_int_eq(msg_in->af2, msg_out->af2);
  ck_assert_int_eq(msg_in->af1, msg_out->af1);
  ck_assert_int_eq(msg_in->af0, msg_out->af0);
  ck_assert_int_eq(msg_in->crs, msg_out->crs);
  ck_assert_int_eq(msg_in->delta_N, msg_out->delta_N);
  ck_assert_int_eq(msg_in->m0, msg_out->m0);
  ck_assert_int_eq(msg_in->cuc, msg_out->cuc);
  ck_assert_int_eq(msg_in->cus, msg_out->cus);
  ck_assert_uint_eq(msg_in->e, msg_out->e);
  ck_assert_uint_eq(msg_in->sqrt_A, msg_out->sqrt_A);
  ck_assert_uint_eq(msg_in->toe, msg_out->toe);
  ck_assert_int_eq(msg_in->cic, msg_out->cic);
  ck_assert_int_eq(msg_in->omega0, msg_out->omega0);
  ck_assert_int_eq(msg_in->cis, msg_out->cis);
  ck_assert_int_eq(msg_in->crc, msg_out->crc);
  ck_assert_int_eq(msg_in->i0, msg_out->i0);
  ck_assert_int_eq(msg_in->omega, msg_out->omega);
  ck_assert_int_eq(msg_in->omega_dot, msg_out->omega_dot);
  ck_assert_int_eq(msg_in->i_dot, msg_out->i_dot);
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
  msg_rawx_equals(&msg, &msg_rawx_out);

  return;
}

END_TEST

START_TEST(test_ubx_mga_gps_eph) {

  ubx_mga_gps_eph msg;

  msg.class_id = 0x13;
  msg.msg_id = 0x00;
  msg.msg_type = 0x01;
  msg.version = 1;
  msg.sat_id = 3;
  msg.fit_interval = 4;
  msg.ura_index = 9;
  msg.sat_health = 8;
  msg.tgd = 15;
  msg.iodc = 9;
  msg.toc = 2512;
  msg.af2 = 100;
  msg.af1 = 200;
  msg.af0 = 300;
  msg.crs = 44;
  msg.delta_N = 45;
  msg.m0 = 46;
  msg.cuc = -5;
  msg.cus = -9;
  msg.e = 4;
  msg.sqrt_A = 0;
  msg.toe = 1981;
  msg.cic = 22;
  msg.omega0 = 200;
  msg.cis = 23;
  msg.crc = 24;
  msg.i0 = 25;
  msg.omega = 34;
  msg.omega_dot = 35;
  msg.i_dot = 36;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ubx_encode_mga_gps_eph(&msg, buff);

  ubx_mga_gps_eph msg_mga_gps_eph_out;
  int8_t ret = ubx_decode_mga_gps_eph(buff, &msg_mga_gps_eph_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_mga_gps_eph_equals(&msg, &msg_mga_gps_eph_out);

  return;
}

END_TEST

START_TEST(test_ubx_nav_pvt) {

  ubx_nav_pvt msg;

  msg.class_id = 0x01;
  msg.msg_id = 0x07;
  msg.i_tow = 433200;
  msg.year = 2018;
  msg.month = 11;
  msg.day = 29;
  msg.hour = 23;
  msg.min = 59;
  msg.sec = 1;
  msg.valid = 0xFF;
  msg.time_accuracy = 432;
  msg.nano = -3579;
  msg.fix_type = 4;
  msg.flags = 0xFA;
  msg.flags2 = 0xB9;
  msg.flags3 = 0x11;
  msg.num_sats = 13;
  msg.lon = 432;
  msg.lat = 123;
  msg.height = 987;
  msg.height_mean_sea_level = 23;
  msg.horizontal_accuracy = 12;
  msg.vertical_accuracy = 14;
  msg.vel_north = 2;
  msg.vel_east = 3;
  msg.vel_down = 4;
  msg.ground_speed = 8;
  msg.heading_of_motion = 12;
  msg.PDOP = 23;
  msg.heading_vehicle = 99;
  msg.magnetic_declination = 1;
  msg.magnetic_declination_accuracy = 11;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ubx_encode_nav_pvt(&msg, buff);

  ubx_nav_pvt msg_nav_pvt_out;
  int8_t ret = ubx_decode_nav_pvt(buff, &msg_nav_pvt_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_nav_pvt_equals(&msg, &msg_nav_pvt_out);

  return;
}

END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("ubx");

  TCase *tc_ubx = tcase_create("ubx");
  tcase_add_test(tc_ubx, test_ubx_rawx);
  tcase_add_test(tc_ubx, test_ubx_nav_pvt);
  tcase_add_test(tc_ubx, test_ubx_mga_gps_eph);
  suite_add_tcase(s, tc_ubx);

  return s;
}
