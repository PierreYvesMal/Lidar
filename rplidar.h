/*
 *  RPLIDAR SDK for Mbed
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
#include "mbed.h"
#include "rplidar.h"
#include "inc/rptypes.h"
#include "inc/rplidar_cmd.h"
#include "rplidar_driver.h"
#include <vector>

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
		DEFAULT_TIMEOUT = 500,
	};

	RPLidar();
	~RPLidar();

	// open the given serial interface and try to connect to the RPLIDAR
	//bool begin(BufferedSerial &serialobj);
	void begin(RawSerial& serialobj);
	// close the currently opened serial interface
	void end();

	// check whether the serial interface is opened
  //  bool isOpen();
    virtual u_result checkSupportConfigCommands(bool& outSupport, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getLidarSampleDuration(float& sampleDurationRes, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getMaxDistance(float &maxDistance, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getScanModeAnsType(uint8_t &ansType, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getScanModeName(char* modeName, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, uint32_t timeout = DEFAULT_TIMEOUT);
    virtual u_result getLidarConf(uint32_t type, std::vector<uint8_t> &outputBuf, const std::vector<uint8_t> &reserve = std::vector<uint8_t>(), uint32_t timeout = DEFAULT_TIMEOUT);
    virtual u_result getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getScanModeCount(uint16_t& modeCount, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result checkExpressScanSupported(bool & support, uint32_t timeout = DEFAULT_TIMEOUT);
	// ask the RPLIDAR for its health info
	u_result getHealth(rplidar_response_device_health_t& healthinfo, uint32_t timeout = DEFAULT_TIMEOUT);

	// ask the RPLIDAR for its device info like the serial number
	u_result getDeviceInfo(rplidar_response_device_info_t& info, uint32_t timeout = DEFAULT_TIMEOUT);

	// stop the measurement operation
	u_result stop();

	// start the measurement operation
	u_result startScanNormal(bool force = true, uint32_t timeout = DEFAULT_TIMEOUT * 2);
    u_result startScanExpress(bool force, uint16_t scanMode, uint32_t options = 0, RplidarScanMode* outUsedScanMode = NULL, uint32_t timeout = DEFAULT_TIMEOUT);
    /// Start scan in specific mode
    ///
    /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    /// \param scanMode         The scan mode id (use getAllSupportedScanModes to get supported modes)
    /// \param options          Scan options (please use 0)
    /// \param outUsedScanMode  The scan mode selected by lidar
    // u_result startScanExpress(bool force, uint16_t scanMode, uint32_t options = 0,  uint32_t timeout = DEFAULT_TIMEOUT) = 0;//RplidarScanMode* outUsedScanMode = NULL,
	
    // wait for one sample point to arrive
	u_result waitPoint(uint32_t timeout = DEFAULT_TIMEOUT);
	u_result _sendCommand(uint8_t cmd, const void* payload, size_t payloadsize);
	// retrieve currently received sample point
	//float Data[360];
    RPLidarMeasurement Data[360];
	int ang;
	int angMin, angMax;
	void setAngle(int min, int max);
	void setLidar();
	const RPLidarMeasurement& getCurrentPoint()
	{
		return _currentMeasurement;
	}

protected:
	//    u_result _sendCommand(uint8_t cmd, const void * payload, size_t payloadsize);
	u_result _waitResponseHeader(rplidar_ans_header_t* header, uint32_t timeout);
    bool     _isConnected; 
    bool     _isScanning;
    bool     _isSupportingMotorCtrl;
    void     _disableDataGrabbing();


    uint16_t                    _cached_sampleduration_std;
    uint16_t                    _cached_sampleduration_express;
    uint8_t                     _cached_express_flag;


protected:
	RawSerial* _binded_serialdev;
	RPLidarMeasurement _currentMeasurement;

// <THREAD>
/*private:
    //RPLidar* _lidar;
    static int count;
public:
    void getDataThread(void){
        while(1){
            this->Data[] getData()
        }
    }
*/
// </THREAD>

};
