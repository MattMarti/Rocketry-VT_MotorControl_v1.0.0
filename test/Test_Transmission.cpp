
#include <iostream>
#include <iomanip>

#include "config.h"
#include "transmission.h"
#include "Assert.h"
#include "Test_Transmission.h"

int TestTransmission() {
	
	// Initialize
	bool pass = true;
	using namespace std;
	using namespace Test;
	//dmsg("=== Begining Test for Packet Compilation ===");

	// Run tests for each expected output type
	pass &= assert_true(Test_checksum(), "Failed: checksum");
	pass &= assert_true(Test_0x01(), "Failed: 0x01");
	pass &= assert_true(Test_0x02(), "Failed: 0x02");
	pass &= assert_true(Test_0x10(), "Failed: 0x10");
	pass &= assert_true(Test_0x40(), "Failed: 0x40");
	pass &= assert_true(Test_0x51(), "Failed: 0x51");
	pass &= assert_true(Test_0x52(), "Failed: 0x52");

	// Ouput results
	if (pass) {
		passed("Test_transmission");
	}
	else {
		failed("Test_transmission");
	}
	return 0;
}

/**
 * Tests correct computation of checksum
 */
bool Test_checksum() {

	using namespace transmission;
	using namespace Test;

	bool pass = true;

	// Initialize
	std::vector<unsigned char> packet;
	unsigned char c0;
	unsigned char c1;
	unsigned char c0_true = 0x00;
	unsigned char c1_true = 0x00;

	// Test 1
	packet.push_back(0x01);
	packet.push_back(0x02);
	packet.push_back(0x01);
	packet.push_back(0x02);
	c0_true = 0x00;
	c1_true = 0x00;
	xorchecksum(packet, c0, c1);
	pass &= assert_equals((long) c0_true, (long) c0, "Test 1 Incorrect c0");
	pass &= assert_equals((long) c1_true, (long) c1, "Test 1 Incorrect c1");

	// Test 2
    packet.clear();
    packet.push_back(0x04);
	packet.push_back(0x04);
	packet.push_back(0x05);
	packet.push_back(0x02);
	c0_true = 0x01;
	c1_true = 0x06;
	xorchecksum(packet, c0, c1);
	pass &= assert_equals((long) c0_true, (long) c0, "Test 2 Incorrect c0");
	pass &= assert_equals((long) c1_true, (long) c1, "Test 2 Incorrect c1");

	// Test 3
    packet.clear();
	packet.push_back(0x07);
    packet.push_back(0x04);
	packet.push_back(0x01);
	packet.push_back(0x02);
	packet.push_back(0x04);
	c0_true = 0x02;
	c1_true = 0x06;
	xorchecksum(packet, c0, c1);
	pass &= assert_equals((long) c0_true, (long) c0, "Test 3 Incorrect c0");
	pass &= assert_equals((long) c1_true, (long) c1, "Test 3 Incorrect c1");
	
	return true;
}

/**
* Tests packet 0x01
*/
bool Test_0x01()
{
	using namespace Test;
	using namespace transmission;

	bool pass = true;

	// Desired output (Obtained using MATLAB Test cases)
    std::vector<unsigned char> packet { 0xAA, 0x14, 0x06, 0x01 };

	// Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	std::vector<unsigned char> out = buildPacket(0x01);

	// Assert cases
	pass &= assert_equals((long) out.size(), (long) 6, "Incorrect value on output  len");
	for (unsigned int i = 0; i < out.size(); i++) {
		pass &= assert_equals((long) out[i], (long) packet[i], "bad str");
	}

	return pass;
}

/**
* Tests packet 0x02
*/
bool Test_0x02() {
	using namespace Test;
	using namespace transmission;

	bool pass = true;

	// Desired output (Obtained using MATLAB Test cases)
	// unsigned char str[] = { 0xAA, 0x14, 0x06, 0x02, 0x00, 0x00 };
    std::vector<unsigned char> packet { 0xAA, 0x14, 0x06, 0x02 };

    // Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	auto out = buildPacket(0x02);

	// Assert cases
	pass &= assert_equals((long) out.size(), (long) 6, "Incorrect value on output  len");
	for (unsigned int i = 0; i < out.size(); i++) {
		pass &= assert_equals((long) out[i], (long) packet[i], "bad str");
	}

	return pass;
}

/**
* Tests packet 0x10
* This packet outputs the current mode setting of the arduino
*/
bool Test_0x10()
{
	using namespace Test;
	using namespace transmission;
	using namespace cfg;

	bool pass = true;

	// Initialize global data
	state::mode = 2;

	// Desired output (Obtained using MATLAB Test cases)
    std::vector<unsigned char> packet { 0xAA, 0x14, 0x07, 0x10, 0x02 };

	// Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	auto out = buildPacket(0x10);

	// Assert cases
	pass &= assert_equals((long) out.size(), (long) 7, "Incorrect value on output  len");
	for (unsigned int i = 0; i < out.size(); i++) {
		pass &= assert_equals((long) out[i], (long) packet[i], "Incorrect value on output");
	}

	// Try changing mode
	state::mode = 11;
	out = buildPacket(0x10);
	pass &= assert_equals((long) out[4], 11, "Incorrect value on output  4");

	return pass;
}

/**
* Tests packet 0x40
*/
bool Test_0x40()
{
	using namespace Test;
	using namespace transmission;
	using namespace cfg;

	bool pass = true;

	// Initialize global data
	state::time = std::chrono::milliseconds(123456) + start_time;
	state::status = 0;
	state::p1 = 1;
	state::p2 = 2;
	state::t1 = 3;
	state::t2 = 4;
	state::thrust = 5;
	state::new_data = 0x1F;
	state::mode = 2;

	// Desired output (Obtained using MATLAB Test cases)
    std::vector<unsigned char> packet { 170, 20, 22,
		0x40, 0x00,  0x01, 0xE2, 0x40,
		2, 0, 0, 64, 64, 0, 0, 64, 160, 0, 0, 31 };

	// Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	auto out = buildPacket(0x40);

	// Assert cases
	pass &= assert_equals((long) out.size(), (long) 22, "Incorrect value on output len");
	for (unsigned int i = 0; i < out.size(); i++) {
		pass &= assert_equals((long) out[i], (long) packet[i], "bad str");
	}

	return pass;
}

/**
* Tests packet 0x51
*/
bool Test_0x51()
{
	using namespace Test;
	using namespace transmission;
	using namespace cfg;

	bool pass = true;

	// Initialize global data
	state::time = std::chrono::milliseconds(123456) + cfg::start_time;
	state::status = 0;
	state::p1 = 1;
	state::p2 = 2;
	state::t1 = 3;
	state::t2 = 4;
	state::thrust = 5;
	state::new_data = 0x1F;
	state::mode = 2;

	// Desired output (Obtained using MATLAB Test cases)
	std::vector<unsigned char> packet = { 170, 20, 34,
        0x51, 0, 1, 226, 64, 2, 0, 0, 63, 128, 0, 0, 64, 0, 0, 0,
		64, 64, 0, 0, 64, 128, 0, 0, 64, 160, 0, 0,	31 };

	// Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	auto out = buildPacket(0x51);
	
	// Assert cases
	for (unsigned int i = 0; i < out.size(); i++)
    {
		pass &= assert_equals((long) out[i], (long) packet[i], "bad str");
	}
	
	return pass;
}

/**
* Tests packet 0x52
*/
bool Test_0x52() {
	using namespace Test;
	using namespace transmission;
	using namespace state;
	using namespace cfg;

	bool pass = true;

	// Initialize global data
	state::time = std::chrono::milliseconds(123456) + cfg::start_time;
	state::status = 0;
	state::p1 = 1;
	state::p2 = 2;
	state::t1 = 3;
	state::t2 = 4;
	state::thrust = 5;
	state::new_data = 0x1F;
	state::mode = 2;

	// Desired output (Obtained using MATLAB Test cases)
	std::vector<unsigned char> packet = { 170, 20, 38, \
		0x52,             \
		0, 1,    226, 64, \
		2,                \
		0,  0,            \
		63, 128, 0,   0,  \
		64, 0,   0,   0,  \
		64, 64,  0,   0,  \
		64, 128, 0,   0,  \
		64, 160, 0,   0,  \
		64, 192, 0,   0,  \
		31};

	// Do the true checksum
	unsigned char c0;
	unsigned char c1;
	xorchecksum(packet, c0, c1);
	packet.push_back(c0);
	packet.push_back(c1);

	// Test conversion from float to char (positive)
	auto out = buildPacket(0x52);

	// Assert cases
	for (unsigned int i = 0; i < out.size(); i++) {
		pass &= assert_equals((long) out[i], (long) packet[i], "bad str");
	}
	
	return pass;
}
