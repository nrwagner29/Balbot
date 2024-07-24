/*
   This file is part of ArduinoIoTBearSSL.

   Copyright 2019 ARDUINO SA (http://www.arduino.cc/)

   This software is released under the GNU General Public License version 3,
   which covers the main part of ArduinoIoTBearSSL.
   The terms of this license can be found at:
   https://www.gnu.org/licenses/gpl-3.0.en.html

   You can be released from the requirements of the above licenses by purchasing
   a commercial license. Buying such a license is mandatory if you want to modify or
   otherwise use the software for commercial activities involving the Arduino
   software without disclosing the source code of your own applications. To purchase
   a commercial license, send an email to license@arduino.cc.

*/

#ifndef _AIOTC_UP_CERT_H_
#define _AIOTC_UP_CERT_H_

/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#include <AIoTC_Config.h>
#if defined(ARDUINO_ARCH_ESP32)

/******************************************************************************
 * CONSTANTS
 ******************************************************************************/

/* This certificate bundle is created using the following certificates:

   * https://www.amazontrust.com/repository/AmazonRootCA1.pem
   * https://www.amazontrust.com/repository/AmazonRootCA2.pem
   * https://www.amazontrust.com/repository/AmazonRootCA3.pem
   * https://www.amazontrust.com/repository/AmazonRootCA4.pem
   * https://www.amazontrust.com/repository/SFSRootCAG2.pem
   * https://certs.secureserver.net/repository/sf-class2-root.crt
   * https://iot.arduino.cc

 */

static const unsigned char x509_crt_bundle[] = {
  0x00, 0x07, 0x00, 0x3b, 0x01, 0x26, 0x30, 0x39, 0x31, 0x0b, 0x30, 0x09,
  0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x0f, 0x30,
  0x0d, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x06, 0x41, 0x6d, 0x61, 0x7a,
  0x6f, 0x6e, 0x31, 0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13,
  0x10, 0x41, 0x6d, 0x61, 0x7a, 0x6f, 0x6e, 0x20, 0x52, 0x6f, 0x6f, 0x74,
  0x20, 0x43, 0x41, 0x20, 0x31, 0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06,
  0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00,
  0x03, 0x82, 0x01, 0x0f, 0x00, 0x30, 0x82, 0x01, 0x0a, 0x02, 0x82, 0x01,
  0x01, 0x00, 0xb2, 0x78, 0x80, 0x71, 0xca, 0x78, 0xd5, 0xe3, 0x71, 0xaf,
  0x47, 0x80, 0x50, 0x74, 0x7d, 0x6e, 0xd8, 0xd7, 0x88, 0x76, 0xf4, 0x99,
  0x68, 0xf7, 0x58, 0x21, 0x60, 0xf9, 0x74, 0x84, 0x01, 0x2f, 0xac, 0x02,
  0x2d, 0x86, 0xd3, 0xa0, 0x43, 0x7a, 0x4e, 0xb2, 0xa4, 0xd0, 0x36, 0xba,
  0x01, 0xbe, 0x8d, 0xdb, 0x48, 0xc8, 0x07, 0x17, 0x36, 0x4c, 0xf4, 0xee,
  0x88, 0x23, 0xc7, 0x3e, 0xeb, 0x37, 0xf5, 0xb5, 0x19, 0xf8, 0x49, 0x68,
  0xb0, 0xde, 0xd7, 0xb9, 0x76, 0x38, 0x1d, 0x61, 0x9e, 0xa4, 0xfe, 0x82,
  0x36, 0xa5, 0xe5, 0x4a, 0x56, 0xe4, 0x45, 0xe1, 0xf9, 0xfd, 0xb4, 0x16,
  0xfa, 0x74, 0xda, 0x9c, 0x9b, 0x35, 0x39, 0x2f, 0xfa, 0xb0, 0x20, 0x50,
  0x06, 0x6c, 0x7a, 0xd0, 0x80, 0xb2, 0xa6, 0xf9, 0xaf, 0xec, 0x47, 0x19,
  0x8f, 0x50, 0x38, 0x07, 0xdc, 0xa2, 0x87, 0x39, 0x58, 0xf8, 0xba, 0xd5,
  0xa9, 0xf9, 0x48, 0x67, 0x30, 0x96, 0xee, 0x94, 0x78, 0x5e, 0x6f, 0x89,
  0xa3, 0x51, 0xc0, 0x30, 0x86, 0x66, 0xa1, 0x45, 0x66, 0xba, 0x54, 0xeb,
  0xa3, 0xc3, 0x91, 0xf9, 0x48, 0xdc, 0xff, 0xd1, 0xe8, 0x30, 0x2d, 0x7d,
  0x2d, 0x74, 0x70, 0x35, 0xd7, 0x88, 0x24, 0xf7, 0x9e, 0xc4, 0x59, 0x6e,
  0xbb, 0x73, 0x87, 0x17, 0xf2, 0x32, 0x46, 0x28, 0xb8, 0x43, 0xfa, 0xb7,
  0x1d, 0xaa, 0xca, 0xb4, 0xf2, 0x9f, 0x24, 0x0e, 0x2d, 0x4b, 0xf7, 0x71,
  0x5c, 0x5e, 0x69, 0xff, 0xea, 0x95, 0x02, 0xcb, 0x38, 0x8a, 0xae, 0x50,
  0x38, 0x6f, 0xdb, 0xfb, 0x2d, 0x62, 0x1b, 0xc5, 0xc7, 0x1e, 0x54, 0xe1,
  0x77, 0xe0, 0x67, 0xc8, 0x0f, 0x9c, 0x87, 0x23, 0xd6, 0x3f, 0x40, 0x20,
  0x7f, 0x20, 0x80, 0xc4, 0x80, 0x4c, 0x3e, 0x3b, 0x24, 0x26, 0x8e, 0x04,
  0xae, 0x6c, 0x9a, 0xc8, 0xaa, 0x0d, 0x02, 0x03, 0x01, 0x00, 0x01, 0x00,
  0x3b, 0x02, 0x26, 0x30, 0x39, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55,
  0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x0f, 0x30, 0x0d, 0x06, 0x03,
  0x55, 0x04, 0x0a, 0x13, 0x06, 0x41, 0x6d, 0x61, 0x7a, 0x6f, 0x6e, 0x31,
  0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x10, 0x41, 0x6d,
  0x61, 0x7a, 0x6f, 0x6e, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41,
  0x20, 0x32, 0x30, 0x82, 0x02, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86,
  0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x02,
  0x0f, 0x00, 0x30, 0x82, 0x02, 0x0a, 0x02, 0x82, 0x02, 0x01, 0x00, 0xad,
  0x96, 0x9f, 0x2d, 0x9c, 0x4a, 0x4c, 0x4a, 0x81, 0x79, 0x51, 0x99, 0xec,
  0x8a, 0xcb, 0x6b, 0x60, 0x51, 0x13, 0xbc, 0x4d, 0x6d, 0x06, 0xfc, 0xb0,
  0x08, 0x8d, 0xdd, 0x19, 0x10, 0x6a, 0xc7, 0x26, 0x0c, 0x35, 0xd8, 0xc0,
  0x6f, 0x20, 0x84, 0xe9, 0x94, 0xb1, 0x9b, 0x85, 0x03, 0xc3, 0x5b, 0xdb,
  0x4a, 0xe8, 0xc8, 0xf8, 0x90, 0x76, 0xd9, 0x5b, 0x4f, 0xe3, 0x4c, 0xe8,
  0x06, 0x36, 0x4d, 0xcc, 0x9a, 0xac, 0x3d, 0x0c, 0x90, 0x2b, 0x92, 0xd4,
  0x06, 0x19, 0x60, 0xac, 0x37, 0x44, 0x79, 0x85, 0x81, 0x82, 0xad, 0x5a,
  0x37, 0xe0, 0x0d, 0xcc, 0x9d, 0xa6, 0x4c, 0x52, 0x76, 0xea, 0x43, 0x9d,
  0xb7, 0x04, 0xd1, 0x50, 0xf6, 0x55, 0xe0, 0xd5, 0xd2, 0xa6, 0x49, 0x85,
  0xe9, 0x37, 0xe9, 0xca, 0x7e, 0xae, 0x5c, 0x95, 0x4d, 0x48, 0x9a, 0x3f,
  0xae, 0x20, 0x5a, 0x6d, 0x88, 0x95, 0xd9, 0x34, 0xb8, 0x52, 0x1a, 0x43,
  0x90, 0xb0, 0xbf, 0x6c, 0x05, 0xb9, 0xb6, 0x78, 0xb7, 0xea, 0xd0, 0xe4,
  0x3a, 0x3c, 0x12, 0x53, 0x62, 0xff, 0x4a, 0xf2, 0x7b, 0xbe, 0x35, 0x05,
  0xa9, 0x12, 0x34, 0xe3, 0xf3, 0x64, 0x74, 0x62, 0x2c, 0x3d, 0x00, 0x49,
  0x5a, 0x28, 0xfe, 0x32, 0x44, 0xbb, 0x87, 0xdd, 0x65, 0x27, 0x02, 0x71,
  0x3b, 0xda, 0x4a, 0xf7, 0x1f, 0xda, 0xcd, 0xf7, 0x21, 0x55, 0x90, 0x4f,
  0x0f, 0xec, 0xae, 0x82, 0xe1, 0x9f, 0x6b, 0xd9, 0x45, 0xd3, 0xbb, 0xf0,
  0x5f, 0x87, 0xed, 0x3c, 0x2c, 0x39, 0x86, 0xda, 0x3f, 0xde, 0xec, 0x72,
  0x55, 0xeb, 0x79, 0xa3, 0xad, 0xdb, 0xdd, 0x7c, 0xb0, 0xba, 0x1c, 0xce,
  0xfc, 0xde, 0x4f, 0x35, 0x76, 0xcf, 0x0f, 0xf8, 0x78, 0x1f, 0x6a, 0x36,
  0x51, 0x46, 0x27, 0x61, 0x5b, 0xe9, 0x9e, 0xcf, 0xf0, 0xa2, 0x55, 0x7d,
  0x7c, 0x25, 0x8a, 0x6f, 0x2f, 0xb4, 0xc5, 0xcf, 0x84, 0x2e, 0x2b, 0xfd,
  0x0d, 0x51, 0x10, 0x6c, 0xfb, 0x5f, 0x1b, 0xbc, 0x1b, 0x7e, 0xc5, 0xae,
  0x3b, 0x98, 0x01, 0x31, 0x92, 0xff, 0x0b, 0x57, 0xf4, 0x9a, 0xb2, 0xb9,
  0x57, 0xe9, 0xab, 0xef, 0x0d, 0x76, 0xd1, 0xf0, 0xee, 0xf4, 0xce, 0x86,
  0xa7, 0xe0, 0x6e, 0xe9, 0xb4, 0x69, 0xa1, 0xdf, 0x69, 0xf6, 0x33, 0xc6,
  0x69, 0x2e, 0x97, 0x13, 0x9e, 0xa5, 0x87, 0xb0, 0x57, 0x10, 0x81, 0x37,
  0xc9, 0x53, 0xb3, 0xbb, 0x7f, 0xf6, 0x92, 0xd1, 0x9c, 0xd0, 0x18, 0xf4,
  0x92, 0x6e, 0xda, 0x83, 0x4f, 0xa6, 0x63, 0x99, 0x4c, 0xa5, 0xfb, 0x5e,
  0xef, 0x21, 0x64, 0x7a, 0x20, 0x5f, 0x6c, 0x64, 0x85, 0x15, 0xcb, 0x37,
  0xe9, 0x62, 0x0c, 0x0b, 0x2a, 0x16, 0xdc, 0x01, 0x2e, 0x32, 0xda, 0x3e,
  0x4b, 0xf5, 0x9e, 0x3a, 0xf6, 0x17, 0x40, 0x94, 0xef, 0x9e, 0x91, 0x08,
  0x86, 0xfa, 0xbe, 0x63, 0xa8, 0x5a, 0x33, 0xec, 0xcb, 0x74, 0x43, 0x95,
  0xf9, 0x6c, 0x69, 0x52, 0x36, 0xc7, 0x29, 0x6f, 0xfc, 0x55, 0x03, 0x5c,
  0x1f, 0xfb, 0x9f, 0xbd, 0x47, 0xeb, 0xe7, 0x49, 0x47, 0x95, 0x0b, 0x4e,
  0x89, 0x22, 0x09, 0x49, 0xe0, 0xf5, 0x61, 0x1e, 0xf1, 0xbf, 0x2e, 0x8a,
  0x72, 0x6e, 0x80, 0x59, 0xff, 0x57, 0x3a, 0xf9, 0x75, 0x32, 0xa3, 0x4e,
  0x5f, 0xec, 0xed, 0x28, 0x62, 0xd9, 0x4d, 0x73, 0xf2, 0xcc, 0x81, 0x17,
  0x60, 0xed, 0xcd, 0xeb, 0xdc, 0xdb, 0xa7, 0xca, 0xc5, 0x7e, 0x02, 0xbd,
  0xf2, 0x54, 0x08, 0x54, 0xfd, 0xb4, 0x2d, 0x09, 0x2c, 0x17, 0x54, 0x4a,
  0x98, 0xd1, 0x54, 0xe1, 0x51, 0x67, 0x08, 0xd2, 0xed, 0x6e, 0x7e, 0x6f,
  0x3f, 0xd2, 0x2d, 0x81, 0x59, 0x29, 0x66, 0xcb, 0x90, 0x39, 0x95, 0x11,
  0x1e, 0x74, 0x27, 0xfe, 0xdd, 0xeb, 0xaf, 0x02, 0x03, 0x01, 0x00, 0x01,
  0x00, 0x3b, 0x00, 0x5b, 0x30, 0x39, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03,
  0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x0f, 0x30, 0x0d, 0x06,
  0x03, 0x55, 0x04, 0x0a, 0x13, 0x06, 0x41, 0x6d, 0x61, 0x7a, 0x6f, 0x6e,
  0x31, 0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x10, 0x41,
  0x6d, 0x61, 0x7a, 0x6f, 0x6e, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43,
  0x41, 0x20, 0x33, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48,
  0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03,
  0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0x29, 0x97, 0xa7, 0xc6, 0x41, 0x7f,
  0xc0, 0x0d, 0x9b, 0xe8, 0x01, 0x1b, 0x56, 0xc6, 0xf2, 0x52, 0xa5, 0xba,
  0x2d, 0xb2, 0x12, 0xe8, 0xd2, 0x2e, 0xd7, 0xfa, 0xc9, 0xc5, 0xd8, 0xaa,
  0x6d, 0x1f, 0x73, 0x81, 0x3b, 0x3b, 0x98, 0x6b, 0x39, 0x7c, 0x33, 0xa5,
  0xc5, 0x4e, 0x86, 0x8e, 0x80, 0x17, 0x68, 0x62, 0x45, 0x57, 0x7d, 0x44,
  0x58, 0x1d, 0xb3, 0x37, 0xe5, 0x67, 0x08, 0xeb, 0x66, 0xde, 0x00, 0x3b,
  0x00, 0x78, 0x30, 0x39, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04,
  0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x0f, 0x30, 0x0d, 0x06, 0x03, 0x55,
  0x04, 0x0a, 0x13, 0x06, 0x41, 0x6d, 0x61, 0x7a, 0x6f, 0x6e, 0x31, 0x19,
  0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x10, 0x41, 0x6d, 0x61,
  0x7a, 0x6f, 0x6e, 0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x41, 0x20,
  0x34, 0x30, 0x76, 0x30, 0x10, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d,
  0x02, 0x01, 0x06, 0x05, 0x2b, 0x81, 0x04, 0x00, 0x22, 0x03, 0x62, 0x00,
  0x04, 0xd2, 0xab, 0x8a, 0x37, 0x4f, 0xa3, 0x53, 0x0d, 0xfe, 0xc1, 0x8a,
  0x7b, 0x4b, 0xa8, 0x7b, 0x46, 0x4b, 0x63, 0xb0, 0x62, 0xf6, 0x2d, 0x1b,
  0xdb, 0x08, 0x71, 0x21, 0xd2, 0x00, 0xe8, 0x63, 0xbd, 0x9a, 0x27, 0xfb,
  0xf0, 0x39, 0x6e, 0x5d, 0xea, 0x3d, 0xa5, 0xc9, 0x81, 0xaa, 0xa3, 0x5b,
  0x20, 0x98, 0x45, 0x5d, 0x16, 0xdb, 0xfd, 0xe8, 0x10, 0x6d, 0xe3, 0x9c,
  0xe0, 0xe3, 0xbd, 0x5f, 0x84, 0x62, 0xf3, 0x70, 0x64, 0x33, 0xa0, 0xcb,
  0x24, 0x2f, 0x70, 0xba, 0x88, 0xa1, 0x2a, 0xa0, 0x75, 0xf8, 0x81, 0xae,
  0x62, 0x06, 0xc4, 0x81, 0xdb, 0x39, 0x6e, 0x29, 0xb0, 0x1e, 0xfa, 0x2e,
  0x5c, 0x00, 0x47, 0x00, 0x5b, 0x30, 0x45, 0x31, 0x0b, 0x30, 0x09, 0x06,
  0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x17, 0x30, 0x15,
  0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x0e, 0x41, 0x72, 0x64, 0x75, 0x69,
  0x6e, 0x6f, 0x20, 0x4c, 0x4c, 0x43, 0x20, 0x55, 0x53, 0x31, 0x0b, 0x30,
  0x09, 0x06, 0x03, 0x55, 0x04, 0x0b, 0x13, 0x02, 0x49, 0x54, 0x31, 0x10,
  0x30, 0x0e, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x07, 0x41, 0x72, 0x64,
  0x75, 0x69, 0x6e, 0x6f, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86,
  0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d,
  0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0x6d, 0x77, 0x6c, 0x5a, 0xcf,
  0x61, 0x1c, 0x7d, 0x44, 0x98, 0x51, 0xf2, 0x5e, 0xe1, 0x02, 0x40, 0x77,
  0xb7, 0x9c, 0xbd, 0x49, 0xa2, 0xa3, 0x8c, 0x4e, 0xab, 0x5e, 0x98, 0xac,
  0x82, 0xfc, 0x69, 0x5b, 0x44, 0x22, 0x77, 0xb4, 0x4d, 0x2e, 0x8e, 0xdf,
  0x2a, 0x71, 0xc1, 0x39, 0x6c, 0xd6, 0x39, 0x14, 0xbd, 0xd9, 0x6b, 0x18,
  0x4b, 0x4b, 0xec, 0xb3, 0xd5, 0xee, 0x42, 0x89, 0x89, 0x55, 0x22, 0x00,
  0x6a, 0x01, 0x24, 0x30, 0x68, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55,
  0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x25, 0x30, 0x23, 0x06, 0x03,
  0x55, 0x04, 0x0a, 0x13, 0x1c, 0x53, 0x74, 0x61, 0x72, 0x66, 0x69, 0x65,
  0x6c, 0x64, 0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67,
  0x69, 0x65, 0x73, 0x2c, 0x20, 0x49, 0x6e, 0x63, 0x2e, 0x31, 0x32, 0x30,
  0x30, 0x06, 0x03, 0x55, 0x04, 0x0b, 0x13, 0x29, 0x53, 0x74, 0x61, 0x72,
  0x66, 0x69, 0x65, 0x6c, 0x64, 0x20, 0x43, 0x6c, 0x61, 0x73, 0x73, 0x20,
  0x32, 0x20, 0x43, 0x65, 0x72, 0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74,
  0x69, 0x6f, 0x6e, 0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 0x74,
  0x79, 0x30, 0x82, 0x01, 0x20, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48,
  0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0d,
  0x00, 0x30, 0x82, 0x01, 0x08, 0x02, 0x82, 0x01, 0x01, 0x00, 0xb7, 0x32,
  0xc8, 0xfe, 0xe9, 0x71, 0xa6, 0x04, 0x85, 0xad, 0x0c, 0x11, 0x64, 0xdf,
  0xce, 0x4d, 0xef, 0xc8, 0x03, 0x18, 0x87, 0x3f, 0xa1, 0xab, 0xfb, 0x3c,
  0xa6, 0x9f, 0xf0, 0xc3, 0xa1, 0xda, 0xd4, 0xd8, 0x6e, 0x2b, 0x53, 0x90,
  0xfb, 0x24, 0xa4, 0x3e, 0x84, 0xf0, 0x9e, 0xe8, 0x5f, 0xec, 0xe5, 0x27,
  0x44, 0xf5, 0x28, 0xa6, 0x3f, 0x7b, 0xde, 0xe0, 0x2a, 0xf0, 0xc8, 0xaf,
  0x53, 0x2f, 0x9e, 0xca, 0x05, 0x01, 0x93, 0x1e, 0x8f, 0x66, 0x1c, 0x39,
  0xa7, 0x4d, 0xfa, 0x5a, 0xb6, 0x73, 0x04, 0x25, 0x66, 0xeb, 0x77, 0x7f,
  0xe7, 0x59, 0xc6, 0x4a, 0x99, 0x25, 0x14, 0x54, 0xeb, 0x26, 0xc7, 0xf3,
  0x7f, 0x19, 0xd5, 0x30, 0x70, 0x8f, 0xaf, 0xb0, 0x46, 0x2a, 0xff, 0xad,
  0xeb, 0x29, 0xed, 0xd7, 0x9f, 0xaa, 0x04, 0x87, 0xa3, 0xd4, 0xf9, 0x89,
  0xa5, 0x34, 0x5f, 0xdb, 0x43, 0x91, 0x82, 0x36, 0xd9, 0x66, 0x3c, 0xb1,
  0xb8, 0xb9, 0x82, 0xfd, 0x9c, 0x3a, 0x3e, 0x10, 0xc8, 0x3b, 0xef, 0x06,
  0x65, 0x66, 0x7a, 0x9b, 0x19, 0x18, 0x3d, 0xff, 0x71, 0x51, 0x3c, 0x30,
  0x2e, 0x5f, 0xbe, 0x3d, 0x77, 0x73, 0xb2, 0x5d, 0x06, 0x6c, 0xc3, 0x23,
  0x56, 0x9a, 0x2b, 0x85, 0x26, 0x92, 0x1c, 0xa7, 0x02, 0xb3, 0xe4, 0x3f,
  0x0d, 0xaf, 0x08, 0x79, 0x82, 0xb8, 0x36, 0x3d, 0xea, 0x9c, 0xd3, 0x35,
  0xb3, 0xbc, 0x69, 0xca, 0xf5, 0xcc, 0x9d, 0xe8, 0xfd, 0x64, 0x8d, 0x17,
  0x80, 0x33, 0x6e, 0x5e, 0x4a, 0x5d, 0x99, 0xc9, 0x1e, 0x87, 0xb4, 0x9d,
  0x1a, 0xc0, 0xd5, 0x6e, 0x13, 0x35, 0x23, 0x5e, 0xdf, 0x9b, 0x5f, 0x3d,
  0xef, 0xd6, 0xf7, 0x76, 0xc2, 0xea, 0x3e, 0xbb, 0x78, 0x0d, 0x1c, 0x42,
  0x67, 0x6b, 0x04, 0xd8, 0xf8, 0xd6, 0xda, 0x6f, 0x8b, 0xf2, 0x44, 0xa0,
  0x01, 0xab, 0x02, 0x01, 0x03, 0x00, 0x9b, 0x01, 0x26, 0x30, 0x81, 0x98,
  0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55,
  0x53, 0x31, 0x10, 0x30, 0x0e, 0x06, 0x03, 0x55, 0x04, 0x08, 0x13, 0x07,
  0x41, 0x72, 0x69, 0x7a, 0x6f, 0x6e, 0x61, 0x31, 0x13, 0x30, 0x11, 0x06,
  0x03, 0x55, 0x04, 0x07, 0x13, 0x0a, 0x53, 0x63, 0x6f, 0x74, 0x74, 0x73,
  0x64, 0x61, 0x6c, 0x65, 0x31, 0x25, 0x30, 0x23, 0x06, 0x03, 0x55, 0x04,
  0x0a, 0x13, 0x1c, 0x53, 0x74, 0x61, 0x72, 0x66, 0x69, 0x65, 0x6c, 0x64,
  0x20, 0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x69, 0x65,
  0x73, 0x2c, 0x20, 0x49, 0x6e, 0x63, 0x2e, 0x31, 0x3b, 0x30, 0x39, 0x06,
  0x03, 0x55, 0x04, 0x03, 0x13, 0x32, 0x53, 0x74, 0x61, 0x72, 0x66, 0x69,
  0x65, 0x6c, 0x64, 0x20, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x73,
  0x20, 0x52, 0x6f, 0x6f, 0x74, 0x20, 0x43, 0x65, 0x72, 0x74, 0x69, 0x66,
  0x69, 0x63, 0x61, 0x74, 0x65, 0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72,
  0x69, 0x74, 0x79, 0x20, 0x2d, 0x20, 0x47, 0x32, 0x30, 0x82, 0x01, 0x22,
  0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01,
  0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00, 0x30, 0x82, 0x01, 0x0a,
  0x02, 0x82, 0x01, 0x01, 0x00, 0xd5, 0x0c, 0x3a, 0xc4, 0x2a, 0xf9, 0x4e,
  0xe2, 0xf5, 0xbe, 0x19, 0x97, 0x5f, 0x8e, 0x88, 0x53, 0xb1, 0x1f, 0x3f,
  0xcb, 0xcf, 0x9f, 0x20, 0x13, 0x6d, 0x29, 0x3a, 0xc8, 0x0f, 0x7d, 0x3c,
  0xf7, 0x6b, 0x76, 0x38, 0x63, 0xd9, 0x36, 0x60, 0xa8, 0x9b, 0x5e, 0x5c,
  0x00, 0x80, 0xb2, 0x2f, 0x59, 0x7f, 0xf6, 0x87, 0xf9, 0x25, 0x43, 0x86,
  0xe7, 0x69, 0x1b, 0x52, 0x9a, 0x90, 0xe1, 0x71, 0xe3, 0xd8, 0x2d, 0x0d,
  0x4e, 0x6f, 0xf6, 0xc8, 0x49, 0xd9, 0xb6, 0xf3, 0x1a, 0x56, 0xae, 0x2b,
  0xb6, 0x74, 0x14, 0xeb, 0xcf, 0xfb, 0x26, 0xe3, 0x1a, 0xba, 0x1d, 0x96,
  0x2e, 0x6a, 0x3b, 0x58, 0x94, 0x89, 0x47, 0x56, 0xff, 0x25, 0xa0, 0x93,
  0x70, 0x53, 0x83, 0xda, 0x84, 0x74, 0x14, 0xc3, 0x67, 0x9e, 0x04, 0x68,
  0x3a, 0xdf, 0x8e, 0x40, 0x5a, 0x1d, 0x4a, 0x4e, 0xcf, 0x43, 0x91, 0x3b,
  0xe7, 0x56, 0xd6, 0x00, 0x70, 0xcb, 0x52, 0xee, 0x7b, 0x7d, 0xae, 0x3a,
  0xe7, 0xbc, 0x31, 0xf9, 0x45, 0xf6, 0xc2, 0x60, 0xcf, 0x13, 0x59, 0x02,
  0x2b, 0x80, 0xcc, 0x34, 0x47, 0xdf, 0xb9, 0xde, 0x90, 0x65, 0x6d, 0x02,
  0xcf, 0x2c, 0x91, 0xa6, 0xa6, 0xe7, 0xde, 0x85, 0x18, 0x49, 0x7c, 0x66,
  0x4e, 0xa3, 0x3a, 0x6d, 0xa9, 0xb5, 0xee, 0x34, 0x2e, 0xba, 0x0d, 0x03,
  0xb8, 0x33, 0xdf, 0x47, 0xeb, 0xb1, 0x6b, 0x8d, 0x25, 0xd9, 0x9b, 0xce,
  0x81, 0xd1, 0x45, 0x46, 0x32, 0x96, 0x70, 0x87, 0xde, 0x02, 0x0e, 0x49,
  0x43, 0x85, 0xb6, 0x6c, 0x73, 0xbb, 0x64, 0xea, 0x61, 0x41, 0xac, 0xc9,
  0xd4, 0x54, 0xdf, 0x87, 0x2f, 0xc7, 0x22, 0xb2, 0x26, 0xcc, 0x9f, 0x59,
  0x54, 0x68, 0x9f, 0xfc, 0xbe, 0x2a, 0x2f, 0xc4, 0x55, 0x1c, 0x75, 0x40,
  0x60, 0x17, 0x85, 0x02, 0x55, 0x39, 0x8b, 0x7f, 0x05, 0x02, 0x03, 0x01,
  0x00, 0x01
};

#elif defined (ARDUINO_EDGE_CONTROL)
  /*
   * https://www.amazontrust.com/repository/AmazonRootCA1.pem
   * https://www.amazontrust.com/repository/AmazonRootCA2.pem
   * https://www.amazontrust.com/repository/AmazonRootCA3.pem
   * https://www.amazontrust.com/repository/AmazonRootCA4.pem
   * https://www.amazontrust.com/repository/SFSRootCAG2.pem
   */
static const char AIoTUPCert[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
"rqXRfboQnoZsG4q5WTP468SQvvG5\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIIFQTCCAymgAwIBAgITBmyf0pY1hp8KD+WGePhbJruKNzANBgkqhkiG9w0BAQwF\n"
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
"b24gUm9vdCBDQSAyMB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTEL\n"
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
"b3QgQ0EgMjCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK2Wny2cSkxK\n"
"gXlRmeyKy2tgURO8TW0G/LAIjd0ZEGrHJgw12MBvIITplLGbhQPDW9tK6Mj4kHbZ\n"
"W0/jTOgGNk3Mmqw9DJArktQGGWCsN0R5hYGCrVo34A3MnaZMUnbqQ523BNFQ9lXg\n"
"1dKmSYXpN+nKfq5clU1Imj+uIFptiJXZNLhSGkOQsL9sBbm2eLfq0OQ6PBJTYv9K\n"
"8nu+NQWpEjTj82R0Yiw9AElaKP4yRLuH3WUnAnE72kr3H9rN9yFVkE8P7K6C4Z9r\n"
"2UXTu/Bfh+08LDmG2j/e7HJV63mjrdvdfLC6HM783k81ds8P+HgfajZRRidhW+me\n"
"z/CiVX18JYpvL7TFz4QuK/0NURBs+18bvBt+xa47mAExkv8LV/SasrlX6avvDXbR\n"
"8O70zoan4G7ptGmh32n2M8ZpLpcTnqWHsFcQgTfJU7O7f/aS0ZzQGPSSbtqDT6Zj\n"
"mUyl+17vIWR6IF9sZIUVyzfpYgwLKhbcAS4y2j5L9Z469hdAlO+ekQiG+r5jqFoz\n"
"7Mt0Q5X5bGlSNscpb/xVA1wf+5+9R+vnSUeVC06JIglJ4PVhHvG/LopyboBZ/1c6\n"
"+XUyo05f7O0oYtlNc/LMgRdg7c3r3NunysV+Ar3yVAhU/bQtCSwXVEqY0VThUWcI\n"
"0u1ufm8/0i2BWSlmy5A5lREedCf+3euvAgMBAAGjQjBAMA8GA1UdEwEB/wQFMAMB\n"
"Af8wDgYDVR0PAQH/BAQDAgGGMB0GA1UdDgQWBBSwDPBMMPQFWAJI/TPlUq9LhONm\n"
"UjANBgkqhkiG9w0BAQwFAAOCAgEAqqiAjw54o+Ci1M3m9Zh6O+oAA7CXDpO8Wqj2\n"
"LIxyh6mx/H9z/WNxeKWHWc8w4Q0QshNabYL1auaAn6AFC2jkR2vHat+2/XcycuUY\n"
"+gn0oJMsXdKMdYV2ZZAMA3m3MSNjrXiDCYZohMr/+c8mmpJ5581LxedhpxfL86kS\n"
"k5Nrp+gvU5LEYFiwzAJRGFuFjWJZY7attN6a+yb3ACfAXVU3dJnJUH/jWS5E4ywl\n"
"7uxMMne0nxrpS10gxdr9HIcWxkPo1LsmmkVwXqkLN1PiRnsn/eBG8om3zEK2yygm\n"
"btmlyTrIQRNg91CMFa6ybRoVGld45pIq2WWQgj9sAq+uEjonljYE1x2igGOpm/Hl\n"
"urR8FLBOybEfdF849lHqm/osohHUqS0nGkWxr7JOcQ3AWEbWaQbLU8uz/mtBzUF+\n"
"fUwPfHJ5elnNXkoOrJupmHN5fLT0zLm4BwyydFy4x2+IoZCn9Kr5v2c69BoVYh63\n"
"n749sSmvZ6ES8lgQGVMDMBu4Gon2nL2XA46jCfMdiyHxtN/kHNGfZQIG6lzWE7OE\n"
"76KlXIx3KadowGuuQNKotOrN8I1LOJwZmhsoVLiJkO/KdYE+HvJkJMcYr07/R54H\n"
"9jVlpNMKVv/1F2Rs76giJUmTtt8AF9pYfl3uxRuw0dFfIRDH+fO6AgonB8Xx1sfT\n"
"4PsJYGw=\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIIBtjCCAVugAwIBAgITBmyf1XSXNmY/Owua2eiedgPySjAKBggqhkjOPQQDAjA5\n"
"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6b24g\n"
"Um9vdCBDQSAzMB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTELMAkG\n"
"A1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJvb3Qg\n"
"Q0EgMzBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABCmXp8ZBf8ANm+gBG1bG8lKl\n"
"ui2yEujSLtf6ycXYqm0fc4E7O5hrOXwzpcVOho6AF2hiRVd9RFgdszflZwjrZt6j\n"
"QjBAMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMB0GA1UdDgQWBBSr\n"
"ttvXBp43rDCGB5Fwx5zEGbF4wDAKBggqhkjOPQQDAgNJADBGAiEA4IWSoxe3jfkr\n"
"BqWTrBqYaGFy+uGh0PsceGCmQ5nFuMQCIQCcAu/xlJyzlvnrxir4tiz+OpAUFteM\n"
"YyRIHN8wfdVoOw==\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIIB8jCCAXigAwIBAgITBmyf18G7EEwpQ+Vxe3ssyBrBDjAKBggqhkjOPQQDAzA5\n"
"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6b24g\n"
"Um9vdCBDQSA0MB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTELMAkG\n"
"A1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJvb3Qg\n"
"Q0EgNDB2MBAGByqGSM49AgEGBSuBBAAiA2IABNKrijdPo1MN/sGKe0uoe0ZLY7Bi\n"
"9i0b2whxIdIA6GO9mif78DluXeo9pcmBqqNbIJhFXRbb/egQbeOc4OO9X4Ri83Bk\n"
"M6DLJC9wuoihKqB1+IGuYgbEgds5bimwHvouXKNCMEAwDwYDVR0TAQH/BAUwAwEB\n"
"/zAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0OBBYEFNPsxzplbszh2naaVvuc84ZtV+WB\n"
"MAoGCCqGSM49BAMDA2gAMGUCMDqLIfG9fhGt0O9Yli/W651+kI0rz2ZVwyzjKKlw\n"
"CkcO8DdZEv8tmZQoTipPNU0zWgIxAOp1AE47xDqUEpHJWEadIRNyp4iciuRMStuW\n"
"1KyLa2tJElMzrdfkviT8tQp21KW8EA==\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIID7zCCAtegAwIBAgIBADANBgkqhkiG9w0BAQsFADCBmDELMAkGA1UEBhMCVVMx\n"
"EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxJTAjBgNVBAoT\n"
"HFN0YXJmaWVsZCBUZWNobm9sb2dpZXMsIEluYy4xOzA5BgNVBAMTMlN0YXJmaWVs\n"
"ZCBTZXJ2aWNlcyBSb290IENlcnRpZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5\n"
"MDkwMTAwMDAwMFoXDTM3MTIzMTIzNTk1OVowgZgxCzAJBgNVBAYTAlVTMRAwDgYD\n"
"VQQIEwdBcml6b25hMRMwEQYDVQQHEwpTY290dHNkYWxlMSUwIwYDVQQKExxTdGFy\n"
"ZmllbGQgVGVjaG5vbG9naWVzLCBJbmMuMTswOQYDVQQDEzJTdGFyZmllbGQgU2Vy\n"
"dmljZXMgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIwDQYJKoZI\n"
"hvcNAQEBBQADggEPADCCAQoCggEBANUMOsQq+U7i9b4Zl1+OiFOxHz/Lz58gE20p\n"
"OsgPfTz3a3Y4Y9k2YKibXlwAgLIvWX/2h/klQ4bnaRtSmpDhcePYLQ1Ob/bISdm2\n"
"8xpWriu2dBTrz/sm4xq6HZYuajtYlIlHVv8loJNwU4PahHQUw2eeBGg6345AWh1K\n"
"Ts9DkTvnVtYAcMtS7nt9rjrnvDH5RfbCYM8TWQIrgMw0R9+53pBlbQLPLJGmpufe\n"
"hRhJfGZOozptqbXuNC66DQO4M99H67FrjSXZm86B0UVGMpZwh94CDklDhbZsc7tk\n"
"6mFBrMnUVN+HL8cisibMn1lUaJ/8viovxFUcdUBgF4UCVTmLfwUCAwEAAaNCMEAw\n"
"DwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYEFJxfAN+q\n"
"AdcwKziIorhtSpzyEZGDMA0GCSqGSIb3DQEBCwUAA4IBAQBLNqaEd2ndOxmfZyMI\n"
"bw5hyf2E3F/YNoHN2BtBLZ9g3ccaaNnRbobhiCPPE95Dz+I0swSdHynVv/heyNXB\n"
"ve6SbzJ08pGCL72CQnqtKrcgfU28elUSwhXqvfdqlS5sdJ/PHLTyxQGjhdByPq1z\n"
"qwubdQxtRbeOlKyWN7Wg0I8VRw7j6IPdj/3vQQF3zCepYoUz8jcI73HPdwbeyBkd\n"
"iEDPfUYd/x7H4c7/I9vG+o1VTqkC50cRRj70/b17KSa7qWFiNyi2LSr2EIZkyXCn\n"
"0q23KXB56jzaYyWf/Wi3MOxw+3WKt21gZ7IeyLnp2KhvAotnDU0mV3HaIPzBSlCN\n"
"sSi6\n"
"-----END CERTIFICATE-----\n";
#else

#endif /* #ifdef ARDUINO_ARCH_ESP32 */

#endif /* _AIOTC_UP_CERT_H_ */