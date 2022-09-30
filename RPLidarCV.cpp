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

#include "RPLidarCV.h"

RPLidar::RPLidar()
    : _bined_serialdev(NULL)
{
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidar::~RPLidar()
{
    end();
}

// Open the given serial interface and try to connect to the RPLIDAR
// Serial port speed differs depending on RPLIDAR model
// Call this function with the serial port you want to use and the LIDAR type

void RPLidar::begin(HardwareSerial &serialobj, uint8_t lidarType)
{
    if (isOpen()) {
      end(); 
    }
    _bined_serialdev = &serialobj;

	switch(lidarType) {
	case 5:
		_bined_serialdev->begin(1000000);
		break;
	case 4:
		_bined_serialdev->begin(256000);
		break;
	default:
		_bined_serialdev->begin(115200);
		break;
	}
}

// Close the currently opened serial interface
void RPLidar::end()
{
    if (isOpen()) {
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}

// Check whether the serial interface is opened
bool RPLidar::isOpen()
{
    return _bined_serialdev?true:false; 
}

// Ask the RPLIDAR for its health info
// A response of 0 means there are no errors
u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    _u32 currentTs = millis();
    _u32 remainingtime;
  
    _u8 *infobuf = (_u8 *)&healthinfo;
    _u8 recvPos = 0;

    rplidar_ans_header_t response_header;
    u_result  ans;


    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
               
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
        
    }
    return RESULT_OK;
}

// Ask the RPLIDAR for its device info like the model and serial number
u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout )
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *infobuf = (_u8*)&info;
    rplidar_ans_header_t response_header;
    u_result  ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

// Stop the measurement operation
u_result RPLidar::stop()
{
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    u_result ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}


// Legacy measurement operation start
u_result RPLidar::startScan(bool force, _u32 timeout) 
{	
	u_result ans = startScanMode(RPLIDAR_SCAN_MODE_2K, force, timeout);
	if (IS_FAIL(ans)) return ans;
	return RESULT_OK;
}

// Start the measurement operation with options for express scan and boost scan
u_result RPLidar::startScanMode(uint8_t scan_mode, bool force, _u32 timeout)
{
    u_result ans;
	_u8 response_answer_type;
	
	// setup payloads
	uint8_t payload_bytes[5];
	for (int i=1;i<5;i++) payload_bytes[i]=0x00;

    if (!isOpen()) return RESULT_OPERATION_FAIL;
    
    //stop(); //force the previous operation to stop

	{
		// send start scan command depending on scan mode
		switch (scan_mode) {
			case RPLIDAR_SCAN_MODE_2K:
			case RPLIDAR_SCAN_MODE_16K:
				response_answer_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
				ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
				break;
			case RPLIDAR_SCAN_MODE_4K:
				response_answer_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
				payload_bytes[0]=0x00;
				ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &payload_bytes, 5);
				break;
			case RPLIDAR_SCAN_MODE_8K:
				response_answer_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA;
				payload_bytes[0]=0x02;
				ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &payload_bytes, 5);
				break;
			case RPLIDAR_SCAN_MODE_32K:
				response_answer_type = RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED;
				payload_bytes[0]=0x00;
				ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &payload_bytes, 5);
				break;
		}
        if (IS_FAIL(ans)) return ans;

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != response_answer_type) {
            return RESULT_INVALID_DATA;
        }

		_u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
		if (header_size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

    }
    return RESULT_OK;
}


// Wait for one sample point to arrive
u_result RPLidar::waitPoint(_u32 timeout)
{
   _u32 currentTs = millis();
   _u32 remainingtime;
   rplidar_response_measurement_node_t node;
   _u8 *nodebuf = (_u8*)&node;

   _u8 recvPos = 0;

   while ((remainingtime=millis() - currentTs) <= timeout) {
        int currentbyte = _bined_serialdev->read();
        if (currentbyte<0) continue;

        switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte          {
                {
                    _u8 tmp = (currentbyte>>1);
                    if ( (tmp ^ currentbyte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
          }
          nodebuf[recvPos++] = currentbyte;

          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
              // store the data ...
              _currentMeasurement.distance = node.distance_q2/4.0f;
              _currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
              _currentMeasurement.quality = (node.sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
              _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
              return RESULT_OK;
          }
        
   }

   return RESULT_OPERATION_TIMEOUT;
}


u_result RPLidar::waitNode(rplidar_response_measurement_node_t * node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = millis();
    _u8 *nodeBuffer = (_u8*)node;
    _u32 waitTime;

    while ((waitTime = millis() - startTs) <= timeout) {

        int currentByte = _bined_serialdev->read();		
        if (currentByte<0) continue;
		
            switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte
            {
                _u8 tmp = (currentByte >> 1);
                if ((tmp ^ currentByte) & 0x1) {
                    // pass
                }
                else {
                    continue;
                }

            }
            break;
            case 1: // expect the highest bit to be 1
            {
                if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                    // pass
                }
                else {
                    recvPos = 0;
                    continue;
                }
            }
            break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
                return RESULT_OK;
            }

    }
    return RESULT_OPERATION_TIMEOUT;
}

// Wait for scan data to arrive
u_result RPLidar::waitScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
	u_result ans;
    size_t   recvNodeCount = 0;
    _u32     startTs = millis();
    _u32     waitTime;

    while (((waitTime = millis() - startTs) <= timeout) && (recvNodeCount < count)) {
        rplidar_response_measurement_node_t node;
        if (IS_FAIL(ans = waitNode(&node, timeout - waitTime))) {
			return ans;
		}
        nodebuffer[recvNodeCount++] = node;
		
        if (recvNodeCount == count) return RESULT_OK;
    }
	
    count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}

// Wait for an Express mode capsule data packet
u_result RPLidar::waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t & node, _u32 timeout)
{
	int  recvPos = 0;
	_u32 currentTs = millis();
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 remainingtime;

    while ((remainingtime=millis() - currentTs) <= timeout) {

        int currentByte = _bined_serialdev->read();
        if (currentByte<0) continue;

			switch (recvPos) {
			case 0: // expect the sync bit 1
				{
					_u8 tmp = (currentByte>>4);
					if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
						// pass
					} else {
						_is_previous_capsuledataRdy = false;
						continue;
					}

				}
				break;
			case 1: // expect the sync bit 2
				{
					_u8 tmp = (currentByte>>4);
					if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
						// pass
					} else {
						recvPos = 0;
						_is_previous_capsuledataRdy = false;
						continue;
					}
				}
				break;
			}
			nodeBuffer[recvPos++] = currentByte;
			if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
				// calc the checksum ...
				_u8 checksum = 0;
				_u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2<<4));
				for (size_t cpos = offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
					cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
				{
					checksum ^= nodeBuffer[cpos];
				}
				if (recvChecksum == checksum)
				{
					// only consider vaild if the checksum matches...
					// store the data ...

					if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
					{
						// this is the first capsule frame in logic, discard the previous cached data...
						_scan_node_synced = false;
						_is_previous_capsuledataRdy = false;
						return RESULT_OK;
					}
					return RESULT_OK;
				}
				_is_previous_capsuledataRdy = false;
				return RESULT_INVALID_DATA;
			}

    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

// Wait for an Ultra mode capsule data packet
u_result RPLidar::waitUltraCapsuledNode(rplidar_response_ultra_capsule_measurement_nodes_t & node, _u32 timeout)
{
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *nodeBuffer = (_u8*)&node;

    _u8 recvPos = 0;

    while ((remainingtime=millis() - currentTs) <= timeout) {

        int currentByte = _bined_serialdev->read();
        if (currentByte<0) continue;

		switch (recvPos) {
		case 0: // expect the sync bit 1
			{
				_u8 tmp = (currentByte>>4);
				if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
				// pass
				}
				else {
					_is_previous_capsuledataRdy = false;
					continue;
				}
			}
		break;
		case 1: // expect the sync bit 2
		{
			_u8 tmp = (currentByte>>4);
			if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
				// pass
			}
			else {
				recvPos = 0;
				_is_previous_capsuledataRdy = false;
				continue;
			}
		}
		break;
		}
		nodeBuffer[recvPos++] = currentByte;
		if (recvPos == sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
			// calc the checksum ...
			_u8 checksum = 0;
			_u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2 << 4));

			for (size_t cpos = offsetof(rplidar_response_ultra_capsule_measurement_nodes_t, start_angle_sync_q6);
			cpos < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t); ++cpos)
			{
				checksum ^= nodeBuffer[cpos];
			}

			if (recvChecksum == checksum)
			{
				// only consider vaild if the checksum matches...
				if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
				{
					// this is the first capsule frame in logic, discard the previous cached data...
					_is_previous_capsuledataRdy = false;
					return RESULT_OK;
				}
				return RESULT_OK;
			}
			_is_previous_capsuledataRdy = false;
			return RESULT_INVALID_DATA;
		}
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

// Convert an Express Scan data capsule to a normal angle and distance data packet
void RPLidar::capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF)<< 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360<<8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q6[2];
            int syncBit[2];

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3)<<4));
            int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3)<<4));

            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
            syncBit[0] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;


            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
            syncBit[1] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360<<6);
                if (angle_q6[cpos] >= (360<<6)) angle_q6[cpos] -= (360<<6);

                rplidar_response_measurement_node_hq_t node;

                node.angle_z_q14 = _u16((angle_q6[cpos] << 8) / 90);
                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
             }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}


static _u32 _varbitscale_decode(_u32 scaled, _u32 & scaleLevel)
{
    static const _u32 VBS_SCALED_BASE[] = {
        RPLIDAR_VARBITSCALE_X16_DEST_VAL,
        RPLIDAR_VARBITSCALE_X8_DEST_VAL,
        RPLIDAR_VARBITSCALE_X4_DEST_VAL,
        RPLIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const _u32 VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const _u32 VBS_TARGET_BASE[] = {
        (0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i)
    {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

// Convert an Ultra Scan data capsule to a normal angle and distance data packet
void RPLidar::ultraCapsuleToNormal(const rplidar_response_ultra_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_ultracapsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3) / 3;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_ultracapsuledata.ultra_cabins); ++pos)
        {
            int dist_q2[3];
            int angle_q6[3];
            int syncBit[3];


            _u32 combined_x3 = _cached_previous_ultracapsuledata.ultra_cabins[pos].combined_x3;

            // unpack ...
            int dist_major = (combined_x3 & 0xFFF);

            // signed partical integer, using the magic shift here
            // DO NOT TOUCH

            int dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            int dist_predict2 = (((int)combined_x3) >> 22);

            int dist_major2;

            _u32 scalelvl1, scalelvl2;

            // prefetch next ...
            if (pos == _countof(_cached_previous_ultracapsuledata.ultra_cabins) - 1)
            {
                dist_major2 = (capsule.ultra_cabins[0].combined_x3 & 0xFFF);
            }
            else {
                dist_major2 = (_cached_previous_ultracapsuledata.ultra_cabins[pos + 1].combined_x3 & 0xFFF);
            }

            // decode with the var bit scale ...
            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);


            int dist_base1 = dist_major;
            int dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }


            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            } else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            } else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }


            for (int cpos = 0; cpos < 3; ++cpos)
            {

                syncBit[cpos] = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) ? 1 : 0;

                int offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);

                if (dist_q2[cpos] >= (50 * 4))
                {
                    const int k1 = 98361;
                    const int k2 = int(k1 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                rplidar_response_measurement_node_hq_t node;

                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.angle_z_q14 = _u16((angle_q6[cpos] << 8) / 90);
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
            }

        }
    }

    _cached_previous_ultracapsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

// Convert a Dense Scan data capsule to a normal angle and distance data packet
void RPLidar::denseCapsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
	static int lastNodeSyncBit = 0;
	const rplidar_response_dense_capsule_measurement_nodes_t *dense_capsule = reinterpret_cast<const rplidar_response_dense_capsule_measurement_nodes_t*>(&capsule);
	nodeCount = 0;
	if (_is_previous_capsuledataRdy) {
		int diffAngle_q8;
		int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
		int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

		diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
		if (prevStartAngle_q8 > currentStartAngle_q8) {
			diffAngle_q8 += (360 << 8);
		}

		int angleInc_q16 = (diffAngle_q8 << 8) / 40;
		int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
		for (size_t pos = 0; pos < _countof(_cached_previous_dense_capsuledata.cabins); ++pos) {
			int dist_q2;
			int angle_q6;
			int syncBit;
			const int dist = static_cast<const int>(_cached_previous_dense_capsuledata.cabins[pos].distance);
			dist_q2 = dist << 2;
			angle_q6 = (currentAngle_raw_q16 >> 10);

			syncBit = (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < (angleInc_q16<<1)) ? 1 : 0;
			syncBit = (syncBit^ lastNodeSyncBit)&syncBit;//Ensure that syncBit is exactly detected
			if (syncBit) {
				_scan_node_synced = true;
			}

			currentAngle_raw_q16 += angleInc_q16;

			if (angle_q6 < 0) angle_q6 += (360 << 6);
			if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);

			
			rplidar_response_measurement_node_hq_t node;

			node.angle_z_q14 = _u16((angle_q6 << 8) / 90);
			node.flag = (syncBit | ((!syncBit) << 1));
			node.quality = dist_q2 ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
			node.dist_mm_q2 = dist_q2;
			if(_scan_node_synced)
				nodebuffer[nodeCount++] = node;
			lastNodeSyncBit = syncBit;
		}
	}
	else {
		_scan_node_synced = false;
	}

	_cached_previous_dense_capsuledata = *dense_capsule;
	_is_previous_capsuledataRdy = true;
}
		
// Send command packet to LIDAR
u_result RPLidar::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{
    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _bined_serialdev->write( (uint8_t *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _bined_serialdev->write((uint8_t *)&sizebyte, 1);

        // send payload
		_bined_serialdev->write((_u8 *)payload, sizebyte);
		
        // send checksum
        _bined_serialdev->write((uint8_t *)&checksum, 1);

    }

    return RESULT_OK;
}

// Wait for a response to command packets
u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=millis() - currentTs) <= timeout) {
        
        int currentbyte = _bined_serialdev->read();

        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
  
    }

    return RESULT_OPERATION_TIMEOUT;
}

// Convert the lidar data
static void RPLidar::convert(const rplidar_response_measurement_node_t& from, rplidar_response_measurement_node_hq_t& to)
{
    to.angle_z_q14 = (((from.angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
    to.dist_mm_q2 = from.distance_q2;
    to.flag = (from.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to.quality = (from.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
}


//Start the motor at 10 Hz for the S2 RPLIDAR
u_result RPLidar::startMotor()
{
	return setMotorSpeed(600, MotorCtrlRpm);
}

//Stop the motor for the S2 RPLIDAR
u_result RPLidar::stopMotor()
{
	return setMotorSpeed(0, MotorCtrlRpm);
}

//Set the motor speed for the S2 RPLIDAR (speed value is RPM)
u_result RPLidar::setMotorSpeed(_u16 speed, _u8 motorCtrlType)
{
	u_result ans = RESULT_OK;

	switch (motorCtrlType)
	{
	case MotorCtrlNone:
		break;
	case MotorCtrlPwm:
		lidar_payload_motor_pwm_t motor_pwm;
		motor_pwm.pwm_value = speed;
		ans = _sendCommand(RPLIDAR_CMD_SET_MOTOR_PWM, (const _u8 *)&motor_pwm, sizeof(motor_pwm));
		if (!ans) return ans;
		break;
	case MotorCtrlRpm:
		lidar_payload_motor_pwm_t motor_rpm;
		motor_rpm.pwm_value = speed;
		ans = _sendCommand(RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL, (const _u8 *)&motor_rpm, sizeof(motor_rpm));
		if (!ans) return ans;
		break;
	}
	return RESULT_OK;
}
