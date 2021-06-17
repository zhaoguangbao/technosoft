/*
 *  Technosoft.h
 *  Technosoft Driver Definition
 *  Author: Alex St. Clair
 *  November 2017
 *
 *  This file defines an Arduino library (C++ class) for communication with
 *  Technosoft motion controllers.
 */

#ifndef TECHNOSOFT_H
#define TECHNOSOFT_H

#include <stdint.h>
#include <string>

#include "dynamixel_sdk/port_handler.h"

#define MAX_PACKET_SIZE		14

struct TML_PACKET_t {
	uint8_t axis;
	uint16_t opcode;
	uint8_t data[8];
	uint8_t data_size;
};

struct DRIVE_STATUS_t {
	bool fault;
	bool motion_complete;
	bool lsp_event;
	uint16_t status_lo;
	uint16_t status_hi;
};

struct DRIVE_FAULT_t {
	uint16_t status_lo;
	uint16_t status_hi;
	uint16_t detailed_err;
	uint16_t motion_err;
};

enum BAUD_t {
	B9600 = 9600,
	B19200 = 19200,
	B38400 = 38400,
	B56600 = 56600,
	B115200 = 115200,
};

class Technosoft {
public:
    Technosoft(uint8_t target, const std::string &port_name, uint8_t expeditor);
    ~Technosoft() { }

	bool Sync(void);
	bool UpdateBaudRate(BAUD_t new_baud);
	bool SaveConfig(void);
    bool SetSerialPort(const std::string& port_name);
	bool SetAxisOn(void);
	bool SendEndInit(void);

	// Technosoft parameters
	bool SetCommandPosition(uint32_t pos);
	bool SetAcceleration(uint32_t acc);
	bool SetAcceleration(float acc);
	bool SetSlewRate(uint32_t slew_rate);
	bool SetSlewRate(float slew_rate);
	bool ReadTorqueCurrent(void);
	bool UpdateDriveStatus(void);
	bool GenerateFaultInfo(void);

	DRIVE_STATUS_t drive_status;
	DRIVE_FAULT_t drive_fault;
	int16_t torque_current;

	// fixed point: 16 bits integer, 16 bits fractional
	uint32_t Float_To_Fixed(float num);
	float Fixed_To_Float(uint32_t num);

	bool CallFunction(uint16_t address);

protected:
	// functions and data access
	int Read16BitRAM(uint16_t address, uint16_t * result);
	int Write16BitRAM(uint16_t address, uint16_t value);
	int Read32BitRAM(uint16_t address, uint32_t * result);
	int Write32BitRAM(uint16_t address, uint32_t value);

    int SetAbsolutePosition(int32_t pos);
	int32_t ReadAbsolutePosition(void);
	float ReadActualSpeed(void);

	uint8_t target_axis;
	uint8_t expeditor_axis;

private:
	int Perform_Transaction(uint8_t * write_buffer, uint8_t * read_buffer, uint8_t write_size, uint8_t * read_size);
	bool Perform_Transaction(uint8_t * write_buffer, uint8_t write_size);
	uint8_t Serialize_Packet(TML_PACKET_t * packet, uint8_t * buffer);
	int Deserialize_Packet(TML_PACKET_t * packet, uint8_t * buffer, uint8_t size);

    dynamixel::PortHandler * port;
	bool port_initialized;
};

#endif
