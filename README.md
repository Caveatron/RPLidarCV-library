# RPLidarCV Library for Teensy

The RPLidarCV library was created for the Caveatron to interface with the RPLidar modules. It is a significantly pared down version of the [RPLidar Public SDK for C++](https://github.com/Slamtec/rplidar_sdk) that provides the minimal functions needed to obtain data from several LIDAR modules at their supported data rates and support a few basic commands. The functions have been lightly modified for simplification and consistency.

## Supported RPLIDARs

The library has only been validated with the A1M8 and S2, however it is likely that it functions with any of the following:

A1M8, A2M6, A2M7, A2M8, A3, S2

## Supported Platforms

The full library has only been used with the Teensy 4. However, an earlier version of the library was used with the RPLIDAR A1M8 on the Teensy 3.6. It likely also works on similar platforms, however processing the high speed RPLIDAR data requires significant memory and processing capability.

## Usage

Two examples are provided for the S2 - for the 16K and 32K data rates. Examples for the A1M8 will be provided at a later point in time.

Different baud rates are required with these modules and you may wire it to a different serial port, so the begin function provides the ability to select these. The begin function parameters are begin(Serial_port, LIDAR_code); For Serial_port, just Serial1, Serial2, etc. For LIDAR_code, use the following for each supported LIDAR:

A1M8: 3
A2M6: 3
A2M7: 4
A2M8: 3
A3:   4
S2:   5

## Examples

The examples display the angle and distance values for each LIDAR point over the Serial port. Press ENTER in the serial port monitor to start the LIDAR and "E" or "e" to end the scan. The LIDAR health value and several parameters for the LIDAR module are displayed before the scan starts.

