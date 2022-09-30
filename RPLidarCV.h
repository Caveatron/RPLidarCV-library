/*
Copyright (c) 2009 - 2014 RoboPeak Team (http://www.robopeak.com)
Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd. (http://www.slamtec.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
	RPLIDAR Library for Teensy 4
	Designed for use with the Caveatron rev C
	Modified from original RPLIDAR SDK from Slamtec at https://github.com/slamtec/rplidar_sdk
	Tested on A1M8 and S2 RPLIDAR modules - Designed to also work with A2 and A3 series but not tested
	Rev 2022-09-28 - Joe Mitchell
*/


#pragma once

#include "Arduino.h"
#include "inc/rptypes.h"
#include "inc/rplidar_cmd.h"

#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))

struct RPLidarMeasurement
{
    float distance;
    float angle;
    uint8_t quality;
    bool  startBit;
};

class RPLidar
{
public:
    enum {
        RPLIDAR_SERIAL_BAUDRATE = 115200,  
        RPLIDAR_DEFAULT_TIMEOUT = 2000,
    };
	
	enum MotorCtrlSupport
    {
        MotorCtrlNone = 0,
        MotorCtrlPwm = 1,
        MotorCtrlRpm = 2,
    };
    
    RPLidar();  
    ~RPLidar();

    // open the given serial interface and try to connect to the RPLIDAR
    void begin(HardwareSerial &serialobj, uint8_t lidarType);

    // close the currently opened serial interface
    void end();
  
    // check whether the serial interface is opened
    bool isOpen(); 

    // ask the RPLIDAR for its health info
    u_result getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    
    // ask the RPLIDAR for its device info like the serial number
    u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);

    // stop the measurement operation
    u_result stop();

    // start the measurement operation
    u_result startScan(bool force = false, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);
	u_result startScanMode(uint8_t scan_mode, bool force = false, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);
		
	u_result startMotor();
	u_result setMotorSpeed(_u16 speed, _u8 motorCtrlType = MotorCtrlRpm);
	u_result stopMotor();

    // wait for one sample point to arrive in 2K normal mode
    u_result waitPoint(_u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    u_result waitNode(rplidar_response_measurement_node_t * node, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    u_result waitScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT);
    // wait for one packet to arrive in 4K express mode
    u_result waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t & node, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);
	// wait for one packet to arrive in 8K ultra mode
    u_result waitUltraCapsuledNode(rplidar_response_ultra_capsule_measurement_nodes_t & node, _u32 timeout = RPLIDAR_DEFAULT_TIMEOUT*2);
	
	
	// functions to convert capsuled data to individual measurements
    void capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
    void ultraCapsuleToNormal(const rplidar_response_ultra_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
    void denseCapsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
	
    static void convert(const rplidar_response_measurement_node_t& from, rplidar_response_measurement_node_hq_t& to);

    // retrieve currently received sample point
    const RPLidarMeasurement & getCurrentPoint()
    {
        return _currentMeasurement;
    }


    rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
    rplidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
    rplidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
    rplidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;
    bool                                         _is_previous_capsuledataRdy;
    bool                                         _is_previous_HqdataRdy;
	
private:
	MotorCtrlSupport        _isSupportingMotorCtrl;
	bool                    _scan_node_synced;

protected:
    u_result _sendCommand(_u8 cmd, const void * payload = NULL, size_t payloadsize = 0);
    u_result _waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout);
    int recvdata(unsigned char * data, size_t size);
    int waitfordata(size_t data_count, _u32 timeout = -1, size_t * returned_size = NULL);
	
    int currentStartAngle = 0;
	bool lastStartBit = 0;

protected:
    HardwareSerial * _bined_serialdev;  
    RPLidarMeasurement _currentMeasurement;
};
