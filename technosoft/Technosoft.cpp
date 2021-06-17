/*
 *  Technosoft.cpp
 *  Technosoft Driver Implementation
 *  Author: Alex St. Clair
 *  November 2017
 *
 *  This file implements an Arduino library (C++ class) for communication with
 *  Technosoft motion controllers.
 */

#include "Technosoft.h"
#include "TML_Instructions_Addresses.h"

Technosoft::Technosoft(uint8_t target, const std::string& port_name, uint8_t expeditor) {
	drive_status = {false, false, false, 0, 0};
	drive_fault = {0, 0, 0, 0};
    expeditor_axis = expeditor;
	target_axis = target;
	torque_current = 0;
    port_initialized = SetSerialPort(port_name);
}

// Public member functions --------------------------------------------------------
bool Technosoft::Sync(void) {
	uint8_t sync_byte = SYNC_BYTE;
	uint8_t ret_byte = 0;

	if (!port_initialized) {
		return false;
	}

	// per user manual, try 15 times before throwing error
	for (int i = 0; i < 15; i++) {
        port->writePort(&sync_byte, 1);

        // TODO
        while (port->readPort(&ret_byte, 1) == 1)
        {
            if (ret_byte == SYNC_BYTE)
                return true;
        }
	}

	return false;
}

bool Technosoft::UpdateBaudRate(BAUD_t new_baud) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;
	uint8_t baud_value = 0;

	switch (new_baud) {
	case B9600:
		baud_value = 0;
		break;
	case B19200:
		baud_value = 1;
		break;
	case B38400:
		baud_value = 2;
		break;
	case B56600:
		baud_value = 3;
		break;
	case B115200:
		baud_value = 4;
		break;
	default:
		baud_value = 0;
		break;
	}

	request.axis = expeditor_axis;
	request.opcode = SET_BAUD_RATE;
	request.data[0] = 0x00;
	request.data[1] = baud_value;
	request.data_size = 2;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return false;
	if (!Perform_Transaction(request_buffer, write_size)) return false;

    port->setBaudRate(new_baud);

	// re-sync at the new rate
	return Sync();
}

bool Technosoft::SaveConfig(void) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = SAVE_CONFIG;
	request.data_size = 0;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return false;
	if (!Perform_Transaction(request_buffer, write_size)) return false;

	return true;
}

bool Technosoft::SetSerialPort(const std::string &port_name) {
    // TODO
//	bool valid = false;
    port=dynamixel::PortHandler::getPortHandler(port_name.c_str());
    port->openPort();

    port->setPacketTimeout(100.0); // 100 ms

    return true;
}

bool Technosoft::SetAxisOn(void) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = AXISON;
	request.data_size = 0;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return false;
	if (!Perform_Transaction(request_buffer, write_size)) return false;

	return true;
}

bool Technosoft::SendEndInit(void) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = ENDINIT;
	request.data_size = 0;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return false;
	if (!Perform_Transaction(request_buffer, write_size)) return false;

	return true;
}

// Technosoft parameters ------------------------------------------------------
int Technosoft::SetAbsolutePosition(int32_t pos) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = SET_ABS_POS;
	request.data[0] = (uint8_t) (pos >> 8);
	request.data[1] = (uint8_t) (pos);
	request.data[2] = (uint8_t) (pos >> 24);
	request.data[3] = (uint8_t) (pos >> 16);
	request.data_size = 4;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return -1;
	if (!Perform_Transaction(request_buffer, write_size)) return -2;

	// Read the register for verification
	if (ReadAbsolutePosition() != pos) return -3;

	return 1;
}

bool Technosoft::SetCommandPosition(uint32_t pos) {
	return (Write32BitRAM(CPOS_P, pos) == 1);
}

bool Technosoft::SetAcceleration(uint32_t acc) {
	return (Write32BitRAM(CACC_P, acc) == 1);
}

bool Technosoft::SetAcceleration(float acc) {
	uint32_t fixed_acc = Float_To_Fixed(acc);
	return (Write32BitRAM(CACC_P, fixed_acc) == 1);
}

bool Technosoft::SetSlewRate(uint32_t slew_rate) {
	return (Write32BitRAM(CSPD_P, slew_rate) == 1);
}

bool Technosoft::SetSlewRate(float slew_rate) {
	uint32_t fixed_slew = Float_To_Fixed(slew_rate);
	return (Write32BitRAM(CSPD_P, fixed_slew) == 1);
}

/* Current [A] = IQ * (ImaxPS / 32736)
 * where ImaxPS = power stage peak current (=10 A?)
 *           IQ = torque current in TML parameter "IQ"
 */
bool Technosoft::ReadTorqueCurrent(void) {
	uint16_t temp = 0;
	if (1 != Read16BitRAM(IQ_P, &temp)) {
		return false;
	}

	torque_current = (int16_t) temp;
	return true;
}

int32_t Technosoft::ReadAbsolutePosition(void) {
	uint32_t result = 0xFFFFFFFF;
	Read32BitRAM(APOS_P, &result);
	return result;
}

float Technosoft::ReadActualSpeed(void) {
	uint32_t raw = 0xFFFFFFFF;
	Read32BitRAM(ASPD_P, &raw);
	return ((raw == 0xFFFFFFFF) ? 0.0f : (Fixed_To_Float(raw)));
}

bool Technosoft::UpdateDriveStatus(void) {
	uint16_t temp_stat_lo = 0xFFFF;
	uint16_t temp_stat_hi = 0xFFFF;

	if (Read16BitRAM(STATUS_LO_R, &temp_stat_lo) != 1) return false;
	if (Read16BitRAM(STATUS_HI_R, &temp_stat_hi) != 1) return false;

	drive_status.status_lo = temp_stat_lo;
	drive_status.status_hi = temp_stat_hi;
	drive_status.fault = (temp_stat_hi >> 15) & 1; // bit 15 of hi
	drive_status.motion_complete = (temp_stat_lo >> 10) & 1; // bit 10 of lo
	drive_status.lsp_event = (temp_stat_hi >> 6) & 1; // bit 6 of hi

	return true;
}

bool Technosoft::GenerateFaultInfo(void) {
	uint16_t temp_stat_lo = 0xFFFF;
	uint16_t temp_stat_hi = 0xFFFF;
	uint16_t temp_det_err = 0xFFFF;
	uint16_t temp_mot_err = 0xFFFF;

	if (Read16BitRAM(STATUS_LO_R, &temp_stat_lo) != 1) return false;
	if (Read16BitRAM(STATUS_HI_R, &temp_stat_hi) != 1) return false;
	if (Read16BitRAM(DETAILED_ERROR_R, &temp_det_err) != 1) return false;
	if (Read16BitRAM(MOTION_ERROR_R, &temp_mot_err) != 1) return false;

	drive_fault.status_lo = temp_stat_lo;
	drive_fault.status_hi = temp_stat_hi;
	drive_fault.detailed_err = temp_det_err;
	drive_fault.motion_err = temp_mot_err;

	return true;
}

// functions and data ---------------------------------------------------------
bool Technosoft::CallFunction(uint16_t address) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = CALL;
	request.data[0] = (uint8_t) (address >> 8);
	request.data[1] = (uint8_t) (address);
	request.data_size = 2;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return false;
	if (!Perform_Transaction(request_buffer, write_size)) return false;

	return true;
}

int Technosoft::Read16BitRAM(uint16_t address, uint16_t * result) {
	TML_PACKET_t request;
	TML_PACKET_t response;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t response_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;
	uint8_t response_size = MAX_PACKET_SIZE;
	*result = 0xFFFF; // ensure result gets initialized

	request.axis = target_axis;
	request.opcode = READ_16_BIT_RAM;
	request.data[0] = expeditor_axis >> 4;
	request.data[1] = (expeditor_axis << 4) | 1;
	request.data[2] = (uint8_t) (address >> 8);
	request.data[3] = (uint8_t) (address);
	request.data_size = 4;

	int ret = 0;
	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return -1;
	if ((ret = Perform_Transaction(request_buffer, response_buffer, write_size, &response_size)) != 1) return ret;
	if ((ret = Deserialize_Packet(&response, response_buffer, response_size)) != 1) return ret;

	if (response.opcode != (RW_16_BIT_RAM_RET | target_axis)) return -4;
	if (response.data_size != 4) return -5;

	// check that the correct address was read (data bytes 0, 1)
	uint16_t ret_address = ((uint16_t) response.data[0] << 8) | (uint16_t) response.data[1];
	if (ret_address != address) return -6;

	// grab the result (data bytes 2, 3)
	*result = ((uint16_t) response.data[2] << 8) | ((uint16_t) response.data[3]);
	return 1;
}

int Technosoft::Write16BitRAM(uint16_t address, uint16_t value) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = WRITE_16_BIT_RAM;
	request.data[0] = (uint8_t) (address >> 8);
	request.data[1] = (uint8_t) (address);
	request.data[2] = (uint8_t) (value >> 8);
	request.data[3] = (uint8_t) (value);
	request.data_size = 4;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return -1;
	if (!Perform_Transaction(request_buffer, write_size)) return -2;

	// Read the register for verification
	uint16_t result = 0;
	if (Read16BitRAM(address, &result) != 1) return -3;
	if (result != value) return -4;

	return 1;
}

int Technosoft::Read32BitRAM(uint16_t address, uint32_t * result) {
	TML_PACKET_t request;
	TML_PACKET_t response;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t response_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;
	uint8_t response_size = MAX_PACKET_SIZE;
	*result = 0xFFFFFFFF; // ensure result gets initialized

	request.axis = target_axis;
	request.opcode = READ_32_BIT_RAM;
	request.data[0] = expeditor_axis >> 4;
	request.data[1] = (expeditor_axis << 4) | 1;
	request.data[2] = (uint8_t) (address >> 8);
	request.data[3] = (uint8_t) (address);
	request.data_size = 4;

	int ret = 0;
	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return -1;
	if ((ret = Perform_Transaction(request_buffer, response_buffer, write_size, &response_size)) != 1) return ret;
	if ((ret = Deserialize_Packet(&response, response_buffer, response_size)) != 1) return ret;

	if (response.opcode != (RW_32_BIT_RAM_RET | target_axis)) return -4;
	if (response.data_size != 6) return -5;

	// check that the correct address was read (data bytes 0, 1)
	uint16_t ret_address = ((uint16_t) response.data[0] << 8) | (uint16_t) response.data[1];
	if (ret_address != address) return -6;

	// grab the result (data bytes 2, 3, 4, 5)
	*result = ((uint32_t) response.data[4] << 24) | ((uint32_t) response.data[5] << 16) | ((uint32_t) response.data[2] << 8) | ((uint32_t) response.data[3]);
	return 1;
}

int Technosoft::Write32BitRAM(uint16_t address, uint32_t value) {
	TML_PACKET_t request;
	uint8_t request_buffer[MAX_PACKET_SIZE];
	uint8_t write_size = 0;

	request.axis = target_axis;
	request.opcode = WRITE_32_BIT_RAM;
	request.data[0] = (uint8_t) (address >> 8);
	request.data[1] = (uint8_t) (address);
	request.data[2] = (uint8_t) (value >> 8);
	request.data[3] = (uint8_t) (value);
	request.data[4] = (uint8_t) (value >> 24);
	request.data[5] = (uint8_t) (value >> 16);
	request.data_size = 6;

	if ((write_size = Serialize_Packet(&request, request_buffer)) == 0) return -1;
	if (!Perform_Transaction(request_buffer, write_size)) return -2;

	// Read the register for verification
	uint32_t result = 0;
	if (Read32BitRAM(address, &result) != 1) return -3;
	if (result != value) return -4;

	return 1;
}

// Private member functions ---------------------------------------------------
int Technosoft::Perform_Transaction(uint8_t * write_buffer, uint8_t * read_buffer, uint8_t write_size, uint8_t * read_size) {
	if (!port_initialized) {
		return -14;
	}

	// Write the message
    if (port->writePort(write_buffer, write_size) != write_size) return -10;

	// Check the ack
	uint8_t ack_buffer;
    if (port->readPort(&ack_buffer, 1) == 0) return -11;
	if (ack_buffer != ACK_BYTE) return -12;

	// Read the response
	int read_bytes = 0;
    if ((read_bytes = port->readPort(read_buffer, *read_size)) == 0) return -13;
	*read_size = (uint8_t) read_bytes;

	return 1;
}

bool Technosoft::Perform_Transaction(uint8_t * write_buffer, uint8_t write_size) {
	if (!port_initialized) {
		return false;
	}

	// Write the message
    if (port->writePort(write_buffer, write_size) != write_size) return false;

	// Check the ack
	uint8_t ack_buffer;
    if (port->readPort(&ack_buffer, 1) == 0) return false;
	if (ack_buffer != ACK_BYTE) return false;

	// no response needed
	return true;
}

uint8_t Technosoft::Serialize_Packet(TML_PACKET_t * packet, uint8_t * buffer) {
	if (packet->data_size > 8) {
		// bad packet
		return 0;
	}

	uint8_t packet_size = packet->data_size + 6;

	buffer[0] = packet_size - 2;
	buffer[1] = packet->axis >> 4;
	buffer[2] = packet->axis << 4;
	buffer[3] = (uint8_t) (packet->opcode >> 8);
	buffer[4] = (uint8_t) packet->opcode;

	for (int i = 0; i < packet->data_size; i++) {
		buffer[5 + i] = packet->data[i];
	}

	uint8_t checksum = 0;
	for (int i = 0; i < packet_size - 1; i++) {
		checksum += buffer[i];
	}
	buffer[packet_size - 1] = checksum;

	return packet_size;
}

int Technosoft::Deserialize_Packet(TML_PACKET_t * packet, uint8_t * buffer, uint8_t size) {
	if (size < 6 || size > 14 || (buffer[0] + 2) != size) {
		return -100+size; // bad packet size
	}

	uint8_t checksum = 0;
	for (int i = 0; i < size - 1; i++) {
		checksum += buffer[i];
	}

	if (checksum != buffer[size-1]) {
		return -21; // incorrect checksum
	}

	packet->axis = (buffer[1] << 4) | (buffer[2] >> 4);
	packet->opcode = ((uint16_t) buffer[3] << 8) | (uint16_t) buffer[4];

	packet->data_size = size - 6;
	for (int i = 0; i < packet->data_size; i++) {
		packet->data[i] = buffer[5 + i];
	}

	return 1;
}


uint32_t Technosoft::Float_To_Fixed(float num) {
	uint16_t upper = static_cast<uint16_t>(num);
	uint16_t lower = static_cast<uint16_t>(65536*num);
	return (((uint32_t) upper << 16) | (uint32_t) lower);
}

float Technosoft::Fixed_To_Float(uint32_t num) {
	return ((int16_t) (num >> 16)) + (((float) (num & 0xFFFF)) / 65536.0);
}
