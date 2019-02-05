#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <logging.h>
#include <Test_all.h>
#include <transmission.h>
#include <hardware.h>
#include <control.h>
#include <config.h>
#include <Assert.h>
#include <comms.h>

// Compiles data to a char array so that it can be pushed to the
// serial port directly. Also this makes it easier to compute a
// checksum on the data
std::vector<unsigned char> transmission::buildPacket(unsigned int type)
{
    // packet and header
    std::vector<unsigned char> packet { 0xAA, 0x14 };

	// Chars used to store float bytes
    uint32_t msec = std::chrono::duration_cast<std::chrono::milliseconds>
        (state::time - cfg::start_time).count();
	
	switch (type)
    {
                   // echo version
        case 0x00: return buildPacket(cfg::version);

        case 0x01: // SD Card Begin of Save File

            packet << (uint8_t) 6 << (uint8_t) 0x01;
            break;

        case 0x02: // SD Card End of Save File

            packet << (uint8_t) 6 << (uint8_t) 0x02;
            break;

        case 0x10:

            packet << (uint8_t) 7 << (uint8_t) 0x10 << (uint8_t) state::mode;
            break;

        case 0x40: // Solid Motor Static Fire Tests 2018-11-11

            packet << (uint8_t) 22 << (uint8_t) 0x40
                << (uint32_t) msec << (uint8_t) state::mode
                << (uint16_t) state::status << (float) state::t1
                << (float) state::thrust << (uint8_t) state::new_data;
            break;

        case 0x51: // Cold flow test data

            packet << (uint8_t) 34 << (uint8_t) 0x51
                << (uint32_t) msec << (uint8_t) state::mode << (uint16_t) state::status
                << (float) state::p1 << (float) state::p2
                << (float) state::t1 << (float) state::t2
                << (float) state::thrust << (uint8_t) state::new_data;
            break;

        case 0x52: // Cold flow test data

            packet << (uint8_t) 38 << (uint8_t) 0x52
                << (uint32_t) msec << (uint8_t) state::mode << (uint16_t) state::status
                << (float) state::p1 << (float) state::p2 << (float) state::t1
                << (float) state::t2 << (float) state::t3 << (float) state::thrust
                << (uint8_t) state::new_data;
            break;

        case 0xB0: // Do Unit Tests

            Test::run_tests(true);
            return buildPacket(Test::results_string);

        default: // Do nothing

            packet.clear();
            return packet;
	}
    appendChecksum(packet);
    return packet;
}

// called upon the successful receipt of a data packet
// the argument data should be stripped of the header,
// length, and checksum bytes
// returns true if successful, false if error encountered
bool transmission::dataReceipt(uint8_t id, const std::vector<uint8_t> &data)
{
    bool ascii = false;

    auto log_packet = transmission::buildPacket(id, data);
    logging::write(log_packet);
    logging::flush();

    switch (id)
    {
                   // echo firmware
        case 0x00: comms::transmit(transmission::buildPacket(0x00));
                   break;

                   /*
        case 0x01: hardware::setLED(true); // turn LED on
                   break;

        case 0x02: hardware::setLED(false); // turn LED off
                   break;
                   */

        // why would we need to clear the input buffer?
        case 0x03: // input_buff.clear(); // clear the input buffer
                   break;

        case 0x04: if (data.size() < 2) break;
                   state::mode = data[1]; // set mode
                   comms::transmit(transmission::buildPacket(0x10));
                   break;

                   // run unit tests
        case 0x05: comms::transmit(transmission::buildPacket(0xB0));
                   break;

                   // reset ping timer
        case 0x06: state::last_ping = std::chrono::steady_clock::now();
                   break;

        case 0x10: // hardware::openStepperMotor(); // open motor
                   break;

        case 0x11: // hardware::closeStepperMotor(); // close motor
                   break;

                   // set parameters
        case 0x20: // cfg::data_period = data[1] + (data[2] << 8);
                   // max_time = data[3];
                   break;

        case 0x21: // XBee.print("DATA_PERIOD: "); // print params
                   // XBee.print(data_period_ms);
                   // XBee.print(" ms\n");
                   // XBee.print("MAX_TIME: ");
                   // XBee.print(max_time);
                   // XBee.print(" s\n");
                   break;

                   // 0x23 - '#'
        case 0x23: ascii = true; // parse ascii message
                   break;

        case 0x36: // if (data.size() < 2) break; // edit params
                   // cfg::DATA_OUT_TYPE = data[1];
                   // if (cfg::DATA_OUT_TYPE == 0x10)
                   //     comms::transmit_string();
                   break;

                   // simulation packet
        case 0x50: if (data.size() < 9) break;
                   state::time = std::chrono::steady_clock::now();
                   // cfg::DATA_OUT_TYPE    = data[0];
                   state::mode             = data[1];
                   state::status           = data[2];
                   state::p1               = data[3];
                   state::p2               = data[4];
                   state::t1               = data[5];
                   state::t2               = data[6];
                   state::thrust           = data[7];
                   state::new_data         = data[8];
                   break;

        case 0xFE: if (data.size() < 1) break;
                   control::exit(data[0]); // safe shutdown
                   break;

        case 0xFF: control::reset(); // soft reset
                   break;

        default:   
                   #ifdef DEBUG
                   std::cout << "It's free real estate: 0x"
                       << std::hex << (int) id << std::endl;
                   #endif
                   return false;
                   
    }
  
    if (!ascii) return true;

    std::string msg(data.begin(), data.end());
    #ifdef DEBUG
    std::cout << "Recieved new ASCII message: \""
        << msg << "\"" << std::endl;
    #endif
 
    if (msg == "MARCO")
    {
        state::last_ping = std::chrono::steady_clock::now();
        std::cout << "Polo!" << std::endl;
    }
    else if (msg == "VERSION")
    {
        std::cout << "Current firmware version is "
            << cfg::version << std::endl;
    }
    else if (msg == "SAY HI")
    {
        std::cout << "Hello, world!" << std::endl;
    }
    else if (msg == "BEST SUBTEAM?")
    {
        std::cout << "Software is the best subteam!" << std::endl;
    }
    else if (msg == "WHAT TEAM?")
    {
        std::cout << "WILDCATS" << std::endl;
    }
    else if (msg == "RESET")
    {
        control::reset();
    }
    else if (msg == "SHUTDOWN")
    {
        control::exit(1); // soft shutdown
    }
    else
    {
        #ifdef DEBUG
        std::cout << "Unknown ASCII message: \""
            << msg << "\"" << std::endl;
        #endif
        return false;
    }
	
    return true;
}
