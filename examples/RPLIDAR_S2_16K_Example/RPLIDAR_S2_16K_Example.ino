//RPLIDAR S2 16K Scan Mode Example using the RPLidarCV library
//Joe Mitchell - 2022-09

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
// It demonstrates the RPLidarCV library for the Caveatron (using the Teensy 4) 
// for the RPLIDAR S2 in 16K scan mode

// You will need to increase your Serial buffer size to 1024 or higher for the port you plan to use 
// I use a buffer size of 2048 for the Caveatron since other functions are occuring while scanning,
// but 1024 is fine for this example.
// To increase your Serial buffer, go to your Arduino program folder, 
// then ->hardware->teensy->avr->cores->teensy4->HardwareSerialx.cpp (where x is the serial port to adjust)
// Change the line #define SERIALx_RX_BUFFER_SIZE from 64 to 1024
// Also, change the LIDAR's serial port on line 34 and 106 to match your setup (Serial1 is currently set)

#include <RPLidarCV.h>

// Create a driver instance 
RPLidar rplidar;

void setup() {
  Serial.begin(250000);
    
  Serial.println("RPLIDAR S2 Example for 16K Scan Rate");
  Serial.println();
  Serial.println("Press any key to begin scanning and 'E' to end scanning");
  Serial.println("(Depending on how your serial monitor is setup, you many also need to press Enter)");

  //Wait for serial key entry to start
  while (!Serial.available()) {}
  Serial.println();

  //->>>>> Change the serial port here and on line 106 if you use a different one
  rplidar.begin(Serial1, 5);
  delay(100);

  //Perform LIDAR health check. If result is '0', LIDAR is ok
  rplidar_response_device_health_t healthinfo;
  if (!IS_OK(rplidar.getHealth(healthinfo, 100))) {
    Serial.println("Get health failed");
    while(1) {};
  } else {
    Serial.print("LIDAR Health Status: ");
    Serial.println(healthinfo.status);
  }

  Serial.println();
  delay(100);

  //Set motor speed to 10 Hz (600 rpm)
  rplidar.setMotorSpeed(600);

  delay(100);

  //Get info about LIDAR
  rplidar_response_device_info_t info;
  if (!IS_OK(rplidar.getDeviceInfo(info, 100))) {
    Serial.println("Get info failed");
    while(1) {};
  } else {
    Serial.println("LIDAR Info");
    Serial.print("Model: ");
    Serial.println(info.model);
    Serial.print("Firmware ver: ");
    Serial.println(info.firmware_version);
    Serial.print("Hardware ver: ");
    Serial.println(info.hardware_version);
  }

  Serial.println();
  delay(1000);

  //Start scanning in 16K mode (stop if scan start fails)
if (!IS_OK(rplidar.startScanMode(RPLIDAR_SCAN_MODE_16K))) {
    Serial.println("LIDAR Failed to Start");
    rplidar.stopMotor();
    rplidar.stop();
    rplidar.end();
    while(1) {};
  } else Serial.println("Start Scan - 16K Scan Mode");

  Serial.println("SCANNING...");
  Serial.println("deg     mm");
  delay(100);
  
  RecordLIDAR_RP_16K();
}

//LIDAR data handling function
void RecordLIDAR_RP_16K() {
  boolean done = false;
  int rot_count = 0;
  float cur_angle_deg;
  int cur_distance_mm;
  rplidar_response_measurement_node_t      local_buf[256];
  size_t                                   count = 256;
  rplidar_response_measurement_node_hq_t   local_scan[2048];
  size_t                                   scan_count = 0;
  u_result                                 ans; 
  memset(local_scan, 0, sizeof(local_scan));
  delay(100);
  
  rplidar.waitScanData(local_buf, count); // // always discard the first data since it may be incomplete

  while (done==false) {

    //Check for serial buffer overflow
    //->>>>> Change the serial port here if you use a different one
    if ((rot_count>2) && (Serial1.available()>1020)) Serial.println("LIDAR OVERFLOW");
    
    // Read LIDAR Data
      if (IS_FAIL(ans=rplidar.waitScanData(local_buf, count))) {
          if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
              Serial.println("LIDAR TIMEOUT");
              break;
          } else {
              // current data is invalid, do not use it.
              continue;
          }
      }

      //Loop through data looking for sync bit to indicate new rotation
      for (size_t pos = 0; pos < count; ++pos)
      {
          if (local_buf[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
          {

              if ((local_scan[0].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                // only publish the data when it contains a full 360 degree scan
  
                rot_count++;

                //Get angle and distance and print all data for the current rotation
                for (int i=0; i<scan_count; i++) {
                    cur_angle_deg = (local_scan[i].angle_z_q14*90.0/16384.0);
                    cur_distance_mm = local_scan[i].dist_mm_q2/4;;
                    Serial.print(cur_angle_deg); Serial.print("\t"); Serial.println(cur_distance_mm);
                }
              }
              scan_count = 0;
          }
          //Reset for next rotation of data
          rplidar_response_measurement_node_hq_t nodeHq;
          rplidar.convert(local_buf[pos], nodeHq);
          local_scan[scan_count++] = nodeHq;
          if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow
      }

      //Check for end scan
      if (Serial.available()){
        char c = Serial.read();
        if ((c=='E')||(c=='e')) done=true;
      }
  }

  //Stop LIDAR
  rplidar.stopMotor();
  rplidar.stop();
  rplidar.end();
  Serial.println("END SCAN");
  Serial.print("Number of Rotations: "); Serial.println(rot_count);
}


void loop() {
}
