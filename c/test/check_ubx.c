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

void msg_rawx_equals(const ubx_rxm_rawx *msg_in, const ubx_rxm_rawx *msg_out) {
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

void msg_nav_clock_equals(const ubx_nav_clock *msg_in,
                          const ubx_nav_clock *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_in->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_in->msg_id);
  ck_assert_uint_eq(msg_in->length, msg_in->length);
  ck_assert_uint_eq(msg_in->i_tow, msg_in->i_tow);
  ck_assert_uint_eq(msg_in->clk_bias, msg_in->clk_bias);
  ck_assert_uint_eq(msg_in->clk_drift, msg_in->clk_drift);
  ck_assert_uint_eq(msg_in->time_acc, msg_in->time_acc);
  ck_assert_uint_eq(msg_in->freq_acc, msg_in->freq_acc);
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
  ck_assert_uint_eq(msg_in->time_acc, msg_in->time_acc);
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

void msg_rxm_sfrbx_equals(const ubx_rxm_sfrbx *msg_in,
                          const ubx_rxm_sfrbx *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->gnss_id, msg_out->gnss_id);
  ck_assert_uint_eq(msg_in->sat_id, msg_out->sat_id);
  ck_assert_uint_eq(msg_in->freq_id, msg_out->freq_id);
  ck_assert_uint_eq(msg_in->num_words, msg_out->num_words);
  ck_assert_uint_eq(msg_in->channel, msg_out->channel);
  ck_assert_uint_eq(msg_in->version, msg_out->version);
  for (int i = 0; i < 10; i++) {
    ck_assert_uint_eq(msg_in->data_words[i], msg_out->data_words[i]);
  }
}

void msg_esf_ins_equals(const ubx_esf_ins *msg_in, const ubx_esf_ins *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->length, msg_out->length);
  ck_assert_uint_eq(msg_in->bitfield0, msg_out->bitfield0);
  ck_assert_uint_eq(msg_in->i_tow, msg_out->i_tow);
  ck_assert_uint_eq(msg_in->x_ang_rate, msg_out->x_ang_rate);
  ck_assert_uint_eq(msg_in->y_ang_rate, msg_out->y_ang_rate);
  ck_assert_uint_eq(msg_in->z_ang_rate, msg_out->z_ang_rate);
  ck_assert_uint_eq(msg_in->x_accel, msg_out->x_accel);
  ck_assert_uint_eq(msg_in->y_accel, msg_out->y_accel);
  ck_assert_uint_eq(msg_in->z_accel, msg_out->z_accel);
}

void msg_esf_meas_equals(const ubx_esf_meas *msg_in,
                         const ubx_esf_meas *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->length, msg_out->length);
  ck_assert_uint_eq(msg_in->time_tag, msg_out->time_tag);
  ck_assert_uint_eq(msg_in->flags, msg_out->flags);
  ck_assert_uint_eq(msg_in->id, msg_out->id);

  u8 num_meas = (msg_in->flags >> 11) & 0x1F;
  for (int i = 0; i < num_meas; i++) {
    ck_assert_uint_eq(msg_in->data[i], msg_out->data[i]);
  }

  bool has_calib = msg_in->flags & 0x8;
  if (has_calib) {
    ck_assert_uint_eq(msg_in->calib_tag, msg_out->calib_tag);
  }
}

void msg_esf_raw_equals(const ubx_esf_raw *msg_in, const ubx_esf_raw *msg_out) {
  ck_assert_uint_eq(msg_in->class_id, msg_out->class_id);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->length, msg_out->length);
  for (int i = 0; i < (msg_in->length - 4) / 8; i++) {
    ck_assert_uint_eq(msg_in->data[i], msg_out->data[i]);
    ck_assert_uint_eq(msg_in->sensor_time_tag[i], msg_out->sensor_time_tag[i]);
  }
}

START_TEST(test_ubx_rxm_rawx) {

  ubx_rxm_rawx msg;

  msg.class_id = 0x02;
  msg.msg_id = 0x15;
  msg.rcv_tow = 415374000;
  msg.rcv_wn = 2015;
  msg.leap_second = 15;
  msg.num_meas = 3;
  msg.length = 16 + 32 * msg.num_meas;
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
  ck_assert_uint_eq(ubx_encode_rawx(&msg, buff), 4 + msg.length);

  ubx_rxm_rawx msg_rawx_out;
  int8_t ret = ubx_decode_rxm_rawx(buff, &msg_rawx_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_rawx_equals(&msg, &msg_rawx_out);
}

END_TEST

START_TEST(test_ubx_mga_gps_eph) {

  ubx_mga_gps_eph msg;

  msg.class_id = 0x13;
  msg.msg_id = 0x00;
  msg.length = 68;
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
  ck_assert_uint_eq(ubx_encode_mga_gps_eph(&msg, buff), 4 + msg.length);

  ubx_mga_gps_eph msg_mga_gps_eph_out;
  int8_t ret = ubx_decode_mga_gps_eph(buff, &msg_mga_gps_eph_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_mga_gps_eph_equals(&msg, &msg_mga_gps_eph_out);
}

END_TEST

START_TEST(test_ubx_nav_clock) {
  ubx_nav_clock msg;
  msg.class_id = 0x01;
  msg.msg_id = 0x22;
  msg.length = 20;

  msg.i_tow = 12345;
  msg.clk_bias = -777;
  msg.clk_drift = -777;
  msg.time_acc = 888;
  msg.freq_acc = 999;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ck_assert_uint_eq(ubx_encode_nav_clock(&msg, buff), 4 + msg.length);

  ubx_nav_clock msg_nav_clock_out;
  int8_t ret = ubx_decode_nav_clock(buff, &msg_nav_clock_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_nav_clock_equals(&msg, &msg_nav_clock_out);
}

END_TEST

START_TEST(test_ubx_nav_pvt) {
  ubx_nav_pvt msg;

  msg.class_id = 0x01;
  msg.msg_id = 0x07;
  msg.length = 92;
  msg.i_tow = 433200;
  msg.year = 2018;
  msg.month = 11;
  msg.day = 29;
  msg.hour = 23;
  msg.min = 59;
  msg.sec = 1;
  msg.valid = 0xFF;
  msg.time_acc = 432;
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
  ck_assert_uint_eq(ubx_encode_nav_pvt(&msg, buff), 4 + msg.length);

  ubx_nav_pvt msg_nav_pvt_out;
  int8_t ret = ubx_decode_nav_pvt(buff, &msg_nav_pvt_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_nav_pvt_equals(&msg, &msg_nav_pvt_out);
}

END_TEST

START_TEST(test_ubx_rxm_sfrbx) {

  ubx_rxm_sfrbx msg;

  msg.class_id = 0x02;
  msg.msg_id = 0x13;
  msg.gnss_id = 1;
  msg.sat_id = 2;
  msg.freq_id = 3;
  msg.num_words = 10;
  msg.length = 8 + 4 * msg.num_words;
  msg.channel = 4;
  msg.version = 0x02;
  for (int i = 0; i < 10; i++) {
    msg.data_words[i] = 1 << i;
  }

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ck_assert_uint_eq(ubx_encode_rxm_sfrbx(&msg, buff), 4 + msg.length);

  ubx_rxm_sfrbx msg_rxm_sfrbx_out;
  int8_t ret = ubx_decode_rxm_sfrbx(buff, &msg_rxm_sfrbx_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_rxm_sfrbx_equals(&msg, &msg_rxm_sfrbx_out);
}

END_TEST

START_TEST(test_ubx_esf_ins) {

  ubx_esf_ins msg;

  msg.class_id = 0x10;
  msg.msg_id = 0x15;
  msg.length = 36;

  msg.bitfield0 = 0xA3A3;
  msg.i_tow = 1;
  msg.x_ang_rate = 2;
  msg.y_ang_rate = 3;
  msg.z_ang_rate = 4;
  msg.x_accel = -2;
  msg.y_accel = -3;
  msg.z_accel = -4;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ck_assert_uint_eq(ubx_encode_esf_ins(&msg, buff), 4 + msg.length);

  ubx_esf_ins msg_esf_ins_out;
  int8_t ret = ubx_decode_esf_ins(buff, &msg_esf_ins_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_esf_ins_equals(&msg, &msg_esf_ins_out);
}

END_TEST

START_TEST(test_ubx_esf_meas) {

  ubx_esf_meas msg;

  msg.class_id = 0x10;
  msg.msg_id = 0x02;
  msg.length = 12 + 4 * 2;

  msg.time_tag = 1;
  msg.flags = (2 << 11) | (1 << 3);
  msg.id = 2;

  msg.data[0] = 3;
  msg.data[1] = 4;

  msg.calib_tag = 5;

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ck_assert_uint_eq(ubx_encode_esf_meas(&msg, buff), 4 + msg.length);

  ubx_esf_meas msg_esf_meas_out;
  int8_t ret = ubx_decode_esf_meas(buff, &msg_esf_meas_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_esf_meas_equals(&msg, &msg_esf_meas_out);
}

END_TEST

START_TEST(test_ubx_esf_raw) {

  ubx_esf_raw msg;

  msg.class_id = 0x10;
  msg.msg_id = 0x03;
  msg.length = 4 + 8 * 4;

  for (int i = 0; i < 4; i++) {
    msg.data[i] = 1 << i;
    msg.sensor_time_tag[i] = i;
  }

  uint8_t buff[1024];
  memset(buff, 0, 1024);
  ck_assert_uint_eq(ubx_encode_esf_raw(&msg, buff), 4 + msg.length);

  ubx_esf_raw msg_esf_raw_out;
  int8_t ret = ubx_decode_esf_raw(buff, &msg_esf_raw_out);
  ck_assert_int_eq(RC_OK, ret);
  msg_esf_raw_equals(&msg, &msg_esf_raw_out);
}

END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("ubx");

  TCase *tc_ubx = tcase_create("ubx");
  tcase_add_test(tc_ubx, test_ubx_rxm_rawx);
  tcase_add_test(tc_ubx, test_ubx_nav_clock);
  tcase_add_test(tc_ubx, test_ubx_nav_pvt);
  tcase_add_test(tc_ubx, test_ubx_mga_gps_eph);
  tcase_add_test(tc_ubx, test_ubx_rxm_sfrbx);
  tcase_add_test(tc_ubx, test_ubx_esf_ins);
  tcase_add_test(tc_ubx, test_ubx_esf_meas);
  tcase_add_test(tc_ubx, test_ubx_esf_raw);
  suite_add_tcase(s, tc_ubx);

  return s;
}
