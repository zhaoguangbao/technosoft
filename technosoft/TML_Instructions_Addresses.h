/*
 *  TML_Instructions_Addresses.h
 *  Technosoft Instruction and Address Definitions
 *  Author: Alex St. Clair
 *  November 2017
 *
 *  This file contains macros defining various TML instructions and addresses
 */

#ifndef TML_INSTRUCTIONS_ADDRESSES_H
#define TML_INSTRUCTIONS_ADDRESSES_H

// TML Instructions ----------------
#define ENDINIT	            0x0020
#define AXISON	            0x0102
#define READ_16_BIT_RAM		0xB204
#define WRITE_16_BIT_RAM	0x9004
#define RW_16_BIT_RAM_RET	0xD400
#define READ_32_BIT_RAM		0xB205
#define WRITE_32_BIT_RAM	0x9005
#define RW_32_BIT_RAM_RET	0xD500
#define SET_BAUD_RATE		0x0820	// SCIBR
#define SAVE_CONFIG			0x1C08	// SAVE
#define CALL				0x7401
#define SET_ABS_POS			0x8400	// SAP

// Register Addresses --------------
#define AXIS_ADDRESS_R		0x030C	// (uint16_t)
#define STATUS_LO_R			0x090E	// (uint16_t)
#define STATUS_HI_R			0x090F	// (uint16_t)
#define DETAILED_ERROR_R	0x035D	// (uint16_t)
#define MOTION_ERROR_R		0x08FC	// (uint16_t)

// Parameter Addresses -------------
#define APOS_P				0x0228	// (32-bit long)  absolute position
#define ASPD_P              0x022C  // (32-bit fixed) speed
#define CACC_P				0x02A2	// (32-bit fixed) acceleration rate
#define CPOS_P				0x029E	// (32-bit long)  position
#define CSPD_P				0x02A0	// (32-bit fixed) slew speed
#define IQ_P				0x0230	// (int16_t)      measured torque current component

// More ----------------------------
#define ACK_BYTE			0x4F
#define SYNC_BYTE			0x0D

#endif