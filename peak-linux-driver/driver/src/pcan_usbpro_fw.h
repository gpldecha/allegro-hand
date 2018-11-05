/*****************************************************************************
 * Copyright (C) 2003-2010  PEAK System-Technik GmbH
 *
 * linux@peak-system.com
 * www.peak-system.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Maintainer(s): Stephane Grosjean (s.grosjean@peak-system.com)
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * pcan_usbpro_fw.h - the PCAN-USB-PRO firmware internal structures
 *
 * $Id: pcan_usbpro_fw.h 615 2010-02-14 22:38:55Z khitschler $
 *
 *****************************************************************************/

#ifndef __pcan_usbpro_fw_h
#define __pcan_usbpro_fw_h

/*
 * USB Vendor Request Data Types
 */

/* Vendor (other) request */
#define USB_VENDOR_REQUEST_INFO				0
#define USB_VENDOR_REQUEST_ZERO				1
#define USB_VENDOR_REQUEST_FKT				2

/* Vendor Request wValue for XXX_INFO */
#define USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER	0
#define USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE		1
#define USB_VENDOR_REQUEST_wVALUE_INFO_uC_CHIPID	2
#define USB_VENDOR_REQUEST_wVALUE_INFO_USB_CHIPID	3
#define USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR		4
#define USB_VENDOR_REQUEST_wVALUE_INFO_CPLD		5
#define USB_VENDOR_REQUEST_wVALUE_INFO_MODE		6
#define USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE		7

/* Vendor Request wValue for XXX_ZERO */
/* Value is Endpoint Number 0-7 */

/* Vendor Request wValue for XXX_FKT */
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_BOOT		0
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG_CAN	1
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG_LIN	2
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG1		3
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG2		4
#define USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED	5

/* ctrl_type value */
#define INTERN_FIRMWARE_INFO_STRUCT_TYPE	0x11223322
#define EXT_FIRMWARE_INFO_STRUCT_TYPE		0x11223344

#define BOOTLOADER_INFO_STRUCT_TYPE		0x11112222
#define uC_CHIPID_STRUCT_TYPE			0
#define USB_CHIPID_STRUCT_TYPE			0
#define DEVICE_NR_STRUCT_TYPE			0x3738393A
#define CPLD_INFO_STRUCT_TYPE			0x1A1A2277

/* USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER vendor request record type */
struct pcan_usbpro_bootloader_info {
	u32	ctrl_type;
	u8	version[4];	/* [0] -> main [1]-> sub [2]-> debug */
	u8	day;
	u8	month;
	u8	year;
	u8	dummy;
	u32	serial_num_high;
	u32	serial_num_low;
	u32	hw_type;
	u32	hw_rev;

}  __attribute__ ((packed));

struct pcan_usbpro_crc_block {
	u32	address;
	u32	len;
	u32	crc;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE vendor request record type */
struct pcan_usbpro_ext_firmware_info {
	u32	ctrl_type;
	u8	version[4];	/* [0] -> main [1]-> sub [2]-> debug */
	u8	day;
	u8	month;
	u8	year;
	u8	dummy;
	u32	fw_type;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_uC_CHIPID vendor request record type */
struct pcan_usbpro_uc_chipid {
	u32	ctrl_type;
	u32	chip_id;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_USB_CHIPID vendor request record type */
struct pcan_usbpro_usb_chipid {
	u32	ctrl_type;
	u32	chip_id;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR vendor request record type */
struct pcan_usbpro_device_nr {
	u32	ctrl_type;
	u32	device_nr;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_CPLD vendor request record type */
struct pcan_usbpro_cpld_info {
	u32	ctrl_type;
	u32	cpld_nr;

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_MODE vendor request record type */
struct pcan_usbpro_info_mode {
	u32	ctrl_type;
	u8	mode[16];
	u8	flags[16];

}  __attribute__ ((packed));

/* USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE vendor request record type */
struct pcan_usbpro_time_mode {
	u32	ctrl_type;
	u16	time_mode;
	u16	flags;

}  __attribute__ ((packed));

/*
 * USB Command Record types
 */
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8                0x80
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4                0x81
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0                0x82
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX              0x83
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX     0x84
#define DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX   0x85
#define DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX                 0x86
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8                0x41
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4                0x42
#define DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0                0x43
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE            0x01
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE            0x02
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE      0x03
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE      0x04
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE          0x05
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR            0x06
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT        0x07
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT     0x08
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP        0x09
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE          0x0a
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE          0x0b
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME          0x0c
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS 0x0D
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER            0x0e
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER            0x0f
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG 0x10
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG     0x11
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR            0x12
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING              0x13
#define DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING              0x14
#define DATA_TYPE_USB2CAN_STRUCT_FKT_STRING                 0x15
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM             0x16
#define DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY    0x17
#define DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM        0x18
#define DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID           0x19
#define DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW          0x1A
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER          0x1B
#define DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED             0x1C

/* Record structures */
struct pcan_usbpro_canmsg_rx {
	u8	data_type;
	u8	client;
	u8	flags;
	u8	len;
	u32	timestamp32;
	u32	id;

	u8	data[8];

}  __attribute__ ((packed));

/* Defines for status */
#define FW_USBPRO_STATUS_MASK_ERROR_S     0x0001
#define FW_USBPRO_STATUS_MASK_BUS_S       0x0002
#define FW_USBPRO_STATUS_MASK_OVERRUN_S   0x0004
#define FW_USBPRO_STATUS_MASK_QOVERRUN_S  0x0008

struct pcan_usbpro_canmsg_status_error_rx {
	u8	data_type;
	u8	channel;
	u16	status;
	u32	timestamp32;
	u32	error_frame;

}  __attribute__ ((packed));

struct pcan_usbpro_calibration_ts_rx {
	u8	data_type;
	u8	dummy[3];
	u32	timestamp64[2];

}  __attribute__ ((packed));

struct pcan_usbpro_buslast_rx {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	buslast_val;
	u32	timestamp32;

}  __attribute__ ((packed));

struct pcan_usbpro_canmsg_tx {
	u8	data_type;
	u8	client;
	u8	flags;
	u8	len;
	u32	id;

	u8	data[8];

}  __attribute__ ((packed));

struct pcan_usbpro_baudrate {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;
	u32	CCBT;

}  __attribute__ ((packed));

struct pcan_usbpro_bus_activity {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	onoff;                     /* 0->off  1->on */

}  __attribute__ ((packed));

struct pcan_usbpro_silent_mode {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	onoff;                     /* 0->off  1->on */

}  __attribute__ ((packed));

struct pcan_usbpro_dev_nr {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;
	u32	serial_num;

}  __attribute__ ((packed));

struct pcan_usbpro_warning_limit {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	warning_limit;

}  __attribute__ ((packed));

struct pcan_usbpro_lookup_explicit {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	id_type;
	u32	id;

}  __attribute__ ((packed));

struct pcan_usbpro_lookup_group {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	id_type;
	u32	id_start;
	u32	id_end;

}  __attribute__ ((packed));

struct pcan_usbpro_filter_mode {
	u8	data_type;
	u8	dummy;
	u16	filter_mode;

}  __attribute__ ((packed));

struct pcan_usbpro_reset_mode {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	reset;

}  __attribute__ ((packed));

struct pcan_usbpro_error_frame {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	mode;

}  __attribute__ ((packed));

struct pcan_usbpro_error_status {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	status;

}  __attribute__ ((packed));

struct pcan_usbpro_set_register {
	u8	data_type;
	u8	irq_off;
	u16	dummy;
	u32	address;
	u32	value;
	u32	mask;

}  __attribute__ ((packed));

struct pcan_usbpro_get_register {
	u8	data_type;
	u8	irq_off;
	u16	dummy;
	u32	address;
	u32	value;

}  __attribute__ ((packed));

struct pcan_usbpro_calibration {
	u8	data_type;
	u8	dummy;
	u16	mode;

}  __attribute__ ((packed));

struct pcan_usbpro_buslast {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u8	dummy;
	u8	mode;
	u16	prescaler;
	u16	sampletimequanta;

}  __attribute__ ((packed));

struct pcan_usbpro_set_string {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u8	offset;
	u8	len;
	u8	data[60];

}  __attribute__ ((packed));

struct pcan_usbpro_get_string {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;

}  __attribute__ ((packed));

struct pcan_usbpro_string {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;
	u8	data[250];

}  __attribute__ ((packed));

struct pcan_usbpro_save_eeprom {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;

}  __attribute__ ((packed));

struct pcan_usbpro_packet_delay {
	u8	data_type;
	u8	dummy;
	u16	delay;

}  __attribute__ ((packed));

struct pcan_usbpro_timestamp_param {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	start_or_end;

}  __attribute__ ((packed));

struct pcan_usbpro_error_gen_id {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	bit_pos;
	u32	id;
	u16	ok_counter;
	u16	error_counter;

}  __attribute__ ((packed));

struct pcan_usbpro_error_gen_now {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	bit_pos;

}  __attribute__ ((packed));

struct pcan_usbpro_softfiler {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	dummy;
	u32	accmask;
	u32	acccode;

}  __attribute__ ((packed));

struct pcan_usbpro_set_can_led {
	u8	data_type;
	u8	channel;                   /* Bit(3..0)-> can channel */
	u16	mode;
	u32	timeout;

}  __attribute__ ((packed));

union pcan_usbpro_rec {
	u8	                           data_type;
	struct pcan_usbpro_canmsg_rx      canmsg_rx;
	struct pcan_usbpro_canmsg_status_error_rx canmsg_status_error_rx;
	struct pcan_usbpro_calibration_ts_rx calibration_ts_rx;
	struct pcan_usbpro_buslast_rx     buslast_rx;
	struct pcan_usbpro_canmsg_tx      canmsg_tx;
	struct pcan_usbpro_baudrate       baudrate;
	struct pcan_usbpro_bus_activity   bus_activity;
	struct pcan_usbpro_silent_mode    silent_mode;
	struct pcan_usbpro_dev_nr         dev_nr;
	struct pcan_usbpro_warning_limit  warning_limit;
	struct pcan_usbpro_lookup_explicit lookup_explicit;
	struct pcan_usbpro_lookup_group   lookup_group;
	struct pcan_usbpro_filter_mode    filer_mode;
	struct pcan_usbpro_reset_mode     reset_mode;
	struct pcan_usbpro_error_frame    error_frame;
	struct pcan_usbpro_error_status   error_status;
	struct pcan_usbpro_set_register	set_register;
	struct pcan_usbpro_get_register   get_register;
	struct pcan_usbpro_calibration    calibration;
	struct pcan_usbpro_buslast        buslast;
	struct pcan_usbpro_set_string     set_string;
	struct pcan_usbpro_get_string     get_string;
	struct pcan_usbpro_string         string;
	struct pcan_usbpro_save_eeprom    save_eeprom;
	struct pcan_usbpro_packet_delay   packet_delay;
	struct pcan_usbpro_timestamp_param timestamp_param;
	struct pcan_usbpro_error_gen_id   error_gen_id;
	struct pcan_usbpro_error_gen_now  error_gen_now;
	struct pcan_usbpro_softfiler      softfiler;
	struct pcan_usbpro_set_can_led    set_can_led;

}  __attribute__ ((packed));

#endif
