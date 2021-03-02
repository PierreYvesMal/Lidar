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

#include "rplidar.h"

Timer timers;
RPLidar::RPLidar()
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

// open the given serial interface and try to connect to the RPLIDAR
/*
bool RPLidar::begin(BufferedSerial &serialobj)
{

	//Serial.begin(115200);

	if (isOpen()) {
	  end();
	}
	_binded_serialdev = &serialobj;
  //  _binded_serialdev->end();
	_binded_serialdev->baud(RPLIDAR_SERIAL_BAUDRATE);
}
*/
void RPLidar::begin(RawSerial& serialobj)
{
	_binded_serialdev = &serialobj;
	timers.start();
	_binded_serialdev->baud(RPLIDAR_SERIAL_BAUDRATE);
}
// close the currently opened serial interface
void RPLidar::end()
{/*
	if (isOpen()) {
	   _binded_serialdev->end();
	   _binded_serialdev = NULL;
	}*/
}


// check whether the serial interface is opened
/*
bool RPLidar::isOpen()
{
	return _binded_serialdev?true:false;
}
*/
// ask the RPLIDAR for its health info

u_result RPLidar::getHealth(rplidar_response_device_health_t& healthinfo, uint32_t timeout)
{
	uint32_t startTs = timers.read_ms();
	uint32_t remainingtime;

	uint8_t* infobuf = (uint8_t*)&healthinfo;
	uint8_t recvPos = 0;

	rplidar_ans_header_t response_header;
	u_result  ans;


	//  if (!isOpen()) return RESULT_OPERATION_FAIL;

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

		if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
			return RESULT_INVALID_DATA;
		}

		while ((remainingtime = timers.read_ms() - startTs) <= timeout) {
			int currentbyte = _binded_serialdev->getc();
			if (currentbyte < 0) continue;

			infobuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(rplidar_response_device_health_t)) {
				return RESULT_OK;
			}
		}
	}
	return RESULT_OPERATION_TIMEOUT;
}
// ask the RPLIDAR for its device info like the serial number
u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t& info, uint32_t timeout)
{
	uint8_t  recvPos = 0;
	uint32_t startTs = timers.read_ms();
	uint32_t remainingtime;
	uint8_t* infobuf = (uint8_t*)&info;
	rplidar_ans_header_t response_header;
	u_result  ans;

	//  if (!isOpen()) return RESULT_OPERATION_FAIL;f

	{
		if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO, NULL, 0))) {
			return ans;
		}

		if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
			return ans;
		}

		// verify whether we got a correct header
		if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
			return RESULT_INVALID_DATA;
		}

		if (response_header.size < sizeof(rplidar_response_device_info_t)) {
			return RESULT_INVALID_DATA;
		}

		while ((remainingtime = timers.read_ms() - startTs) <= timeout) {
			int currentbyte = _binded_serialdev->getc();
			if (currentbyte < 0) continue;
			infobuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(rplidar_response_device_info_t)) {
				return RESULT_OK;
			}
		}
	}

	return RESULT_OPERATION_TIMEOUT;
}

// stop the measurement operation
u_result RPLidar::stop()
{
	//    if (!isOpen()) return RESULT_OPERATION_FAIL;
	u_result ans = _sendCommand(RPLIDAR_CMD_STOP, NULL, 0);
	return ans;
}

// start the measurement operation
u_result RPLidar::startScanNormal(bool force, uint32_t timeout)
{
	u_result ans;

	//    if (!isOpen()) return RESULT_OPERATION_FAIL;

	stop(); //force the previous operation to stop


	ans = _sendCommand(force ? RPLIDAR_CMD_FORCE_SCAN : RPLIDAR_CMD_SCAN, NULL, 0);
	if (IS_FAIL(ans)) return ans;

	// waiting for confirmation
	rplidar_ans_header_t response_header;
	if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
		return ans;
	}

	// verify whether we got a correct header
	if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
		return RESULT_INVALID_DATA;
	}

	if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
		return RESULT_INVALID_DATA;
	}

	return RESULT_OK;
}

u_result RPLidar::startScanExpress(bool force, uint16_t scanMode, uint32_t options, RplidarScanMode* outUsedScanMode, uint32_t timeout)
{
    u_result ans;
    // if (!isConnected()) return RESULT_OPERATION_FAIL;
    // if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    if (scanMode == RPLIDAR_CONF_SCAN_COMMAND_STD)
    {
        //return startScan(force, false, 0, outUsedScanMode);
    }

    
    bool ifSupportLidarConf = false;
    ans = checkSupportConfigCommands(ifSupportLidarConf);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (outUsedScanMode)
    {
        outUsedScanMode->id = scanMode;
        if (ifSupportLidarConf)
        {
            ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }

            ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
           

        }
        else
        {
            rplidar_response_sample_rate_t sampleRateTmp;
            ans = getSampleDuration_uS(sampleRateTmp);
            if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

            outUsedScanMode->us_per_sample = sampleRateTmp.express_sample_duration_us;
            outUsedScanMode->max_distance = 16;
            outUsedScanMode->ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy(outUsedScanMode->scan_mode, "Express");
        }
    }

    //get scan answer type to specify how to wait data
    uint8_t scanAnsType;
    if (ifSupportLidarConf)
    {      
        getScanModeAnsType(scanAnsType, scanMode);
    }
    else
    {
        scanAnsType = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
    }

    {
        //rp::hal::AutoLocker l(_lock);

        rplidar_payload_express_scan_t scanReq;
        memset(&scanReq, 0, sizeof(scanReq));
        if (scanMode != RPLIDAR_CONF_SCAN_COMMAND_STD && scanMode != RPLIDAR_CONF_SCAN_COMMAND_EXPRESS)
            scanReq.working_mode = uint8_t(scanMode);
        scanReq.working_flags = options;

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != scanAnsType) {
            return RESULT_INVALID_DATA;
        }

        uint32_t header_size = (response_header.size & RPLIDAR_ANS_HEADER_SIZE_MASK);

        if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 0;
            _isScanning = true;
            //_cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 1;
            _isScanning = true;
            //_cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_HQ) {
            if (header_size < sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            //_cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheHqScanData);
        }
        else
        {
            if (header_size < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            //_cachethread = CLASS_THREAD(RPlidarDriverImplCommon, _cacheUltraCapsuledScanData);
        }

        // if (_cachethread.getHandle() == 0) {
        //     return RESULT_OPERATION_FAIL;
        // }
    }
    return RESULT_OK;
}

u_result RPLidar::getScanModeAnsType(uint8_t &ansType, uint16_t scanModeID, uint32_t timeoutInMs)
{
    u_result ans;
    std::vector<uint8_t> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<uint8_t> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(uint8_t))
    {
        return RESULT_INVALID_DATA;
    }
    const uint8_t *result = reinterpret_cast<const uint8_t*>(&answer[0]);
    ansType = *result;
    return ans;
}

u_result RPLidar::getScanModeName(char* modeName, uint16_t scanModeID, uint32_t timeoutInMs)
{
    u_result ans;
    std::vector<uint8_t> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<uint8_t> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_NAME, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    int len = answer.size();
    if (0 == len) return RESULT_INVALID_DATA;
    memcpy(modeName, &answer[0], len);
    return ans;
}

u_result RPLidar::getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, uint32_t timeoutInMs)
{
    u_result ans;
    bool confProtocolSupported = false;
    ans = checkSupportConfigCommands(confProtocolSupported);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (confProtocolSupported)
    {
        // 1. get scan mode count
        uint16_t modeCount;
        ans = getScanModeCount(modeCount);
        if (IS_FAIL(ans))
        {
            return RESULT_INVALID_DATA;
        }
        // 2. for loop to get all fields of each scan mode
        for (uint16_t i = 0; i < modeCount; i++)
        {
            RplidarScanMode scanModeInfoTmp;
            memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
            scanModeInfoTmp.id = i;
            ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getMaxDistance(scanModeInfoTmp.max_distance, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeName(scanModeInfoTmp.scan_mode, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            outModes.push_back(scanModeInfoTmp);
        }
        return ans;
    }
    else
    {
        rplidar_response_sample_rate_t sampleRateTmp;
        ans = getSampleDuration_uS(sampleRateTmp);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;
        //judge if support express scan
        bool ifSupportExpScan = false;
        ans = checkExpressScanSupported(ifSupportExpScan);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

        RplidarScanMode stdScanModeInfo;
        stdScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_STD;
        stdScanModeInfo.us_per_sample = sampleRateTmp.std_sample_duration_us;
        stdScanModeInfo.max_distance = 16;
        stdScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
        strcpy(stdScanModeInfo.scan_mode, "Standard");
        outModes.push_back(stdScanModeInfo);
        if (ifSupportExpScan)
        {
            RplidarScanMode expScanModeInfo;
            expScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            expScanModeInfo.us_per_sample = sampleRateTmp.express_sample_duration_us;
            expScanModeInfo.max_distance = 16;
            expScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy(expScanModeInfo.scan_mode, "Express");
            outModes.push_back(expScanModeInfo);
        }
        return ans;
    }
    return ans;
}

u_result RPLidar::getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, uint32_t timeout)
{  
    //DEPRECATED_WARN("getSampleDuration_uS", "RplidarScanMode::us_per_sample");

    //if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();
    
    rplidar_response_device_info_t devinfo;
    // 1. fetch the device version first...
    u_result ans = getDeviceInfo(devinfo, timeout);

    rateInfo.express_sample_duration_us = _cached_sampleduration_express;
    rateInfo.std_sample_duration_us = _cached_sampleduration_std;

    if (devinfo.firmware_version < ((0x1<<8) | 17)) {
        // provide fake data...

        return RESULT_OK;
    }


    {
        //rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_SAMPLERATE, NULL, 0))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_SAMPLE_RATE) {
            return RESULT_INVALID_DATA;
        }

        uint32_t header_size = (response_header.size & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_sample_rate_t)) {
            return RESULT_INVALID_DATA;
        }

        // if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        //_chanDev->recvdata(reinterpret_cast<uint8_t *>(&rateInfo), sizeof(rateInfo));
        for(int i=0;i<sizeof(rateInfo);i++)//please check again!!!!!!
	        _binded_serialdev->putc(reinterpret_cast<uint8_t *>(&rateInfo)[i]);
    }
    return RESULT_OK;
}
u_result RPLidar::checkExpressScanSupported(bool & support, uint32_t timeout)
{
    //DEPRECATED_WARN("checkExpressScanSupported(bool&,_u32)", "getAllSupportedScanModes()");

    rplidar_response_device_info_t devinfo;

    support = false;
    u_result ans = getDeviceInfo(devinfo, timeout);

    if (IS_FAIL(ans)) return ans;

    if (devinfo.firmware_version >= ((0x1<<8) | 17)) {
        support = true;
        rplidar_response_sample_rate_t sample_rate;
        getSampleDuration_uS(sample_rate);
        _cached_sampleduration_express = sample_rate.express_sample_duration_us;
        _cached_sampleduration_std = sample_rate.std_sample_duration_us;
    }

    return RESULT_OK;
}
u_result RPLidar::getScanModeCount(uint16_t& modeCount, uint32_t timeoutInMs)
{
    u_result ans;
    std::vector<uint8_t> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_COUNT, answer, std::vector<uint8_t>(), timeoutInMs);

    if (IS_FAIL(ans)) {
        return ans;
    }
    if (answer.size() < sizeof(uint16_t)) {
        return RESULT_INVALID_DATA;
    }
    const uint16_t *p_answer = reinterpret_cast<const uint16_t*>(&answer[0]);
    modeCount = *p_answer;

    return ans;
}

void RPLidar::_disableDataGrabbing()
{
    _isScanning = false;
    //_cachethread.join();
}


u_result RPLidar::getMaxDistance(float &maxDistance, uint16_t scanModeID, uint32_t timeoutInMs)
{
    u_result ans;
    std::vector<uint8_t> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<uint8_t> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(uint32_t))
    {
        return RESULT_INVALID_DATA;
    }
    const uint32_t *result = reinterpret_cast<const uint32_t*>(&answer[0]);
    maxDistance = (float)(*result >> 8);
    return ans;
}

u_result RPLidar::getLidarConf(uint32_t type, std::vector<uint8_t> &outputBuf, const std::vector<uint8_t> &reserve, uint32_t timeout)
{
    rplidar_payload_get_scan_conf_t query;
    memset(&query, 0, sizeof(query));
    query.type = type;
    int sizeVec = reserve.size();

    int maxLen = sizeof(query.reserved) / sizeof(query.reserved[0]);
    if (sizeVec > maxLen) sizeVec = maxLen;

    if (sizeVec > 0)
        memcpy(query.reserved, &reserve[0], reserve.size());

    u_result ans;
    {
        //rp::hal::AutoLocker l(_lock);
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(query)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_GET_LIDAR_CONF) {
            return RESULT_INVALID_DATA;
        }

        uint32_t header_size = (response_header.size & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(type)) {
            return RESULT_INVALID_DATA;
        }

        // if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }

        std::vector<uint8_t> dataBuf;
        dataBuf.resize(header_size);
        //_chanDev->recvdata(reinterpret_cast<uint8_t *>(&dataBuf[0]), header_size);
        for(int i=0;i<header_size;i++)//please check again!!!!!!
	        _binded_serialdev->putc(dataBuf[i]);

        //check if returned type is same as asked type
        uint32_t replyType = -1;
        memcpy(&replyType, &dataBuf[0], sizeof(type));
        if (replyType != type) {
            return RESULT_INVALID_DATA;
        }

        //copy all the payload into &outputBuf
        int payLoadLen = header_size - sizeof(type);

        //do consistency check
        if (payLoadLen <= 0) {
            return RESULT_INVALID_DATA;
        }
        //copy all payLoadLen bytes to outputBuf
        outputBuf.resize(payLoadLen);
        memcpy(&outputBuf[0], &dataBuf[0] + sizeof(type), payLoadLen);
    }
    return ans;
}

u_result RPLidar::checkSupportConfigCommands(bool& outSupport, uint32_t timeoutInMs)
{
    u_result ans;

    rplidar_response_device_info_t devinfo;
    ans = getDeviceInfo(devinfo, timeoutInMs);
    if (IS_FAIL(ans)) return ans;

    // if lidar firmware >= 1.24
    if (devinfo.firmware_version >= ((0x1 << 8) | 24)) {
        outSupport = true;
    }
    return ans;
}

u_result RPLidar::getLidarSampleDuration(float& sampleDurationRes, uint16_t scanModeID, uint32_t timeoutInMs)
{
    u_result ans;
    std::vector<uint8_t> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<uint8_t> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(uint32_t))
    {
        return RESULT_INVALID_DATA;
    }
    const uint32_t *result = reinterpret_cast<const uint32_t*>(&answer[0]);
    sampleDurationRes = (float)(*result >> 8);
    return ans;
}

// wait for one sample point to arrive
u_result RPLidar::waitPoint(uint32_t timeout){
    uint8_t recvPos = 0;
	uint32_t startTs = timers.read_ms();

	uint32_t waitTime;
	rplidar_response_measurement_node_t node;
	uint8_t* nodebuf = (uint8_t*)&node;

	
    uint8_t currentByte;
	while ((waitTime = timers.read_ms() - startTs) <= timeout) {
		if(_binded_serialdev->readable()) 
            currentByte = _binded_serialdev->getc();
        else
            continue;
		
		switch (recvPos) {
		case 0: // expect the sync bit and its reverse in this byte          {
		{
			uint8_t tmp = (currentByte >> 1);
			if ((tmp ^ currentByte) & 0x1) {
				// pass
			}
			else {
				continue;
			}

		}
		break;
		case 1: // expect the Check bit to be 1
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
		nodebuf[recvPos++] = currentByte;

		if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
			// store the data ...
			_currentMeasurement.distance = node.distance_q2 / 4.0f;
			_currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			_currentMeasurement.quality = (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
			_currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
			ang = (int)_currentMeasurement.angle;
			//dis = _currentMeasurement.distance;

            if (ang >= angMin && ang <= angMax) Data[ang] =  _currentMeasurement;
            //.distance;// (float) ang;
            //wait_ms(200);
           /* for(int i = 0; i < size_t(nodebuf); i++)
                Data[i] = nodebuf[i]*/
			//if (ang >= angMin && ang <= angMax)Data[ang] = 12.0;//_currentMeasurement.distance;
//continue;
			return RESULT_OK;
		}


	}
    //return RESULT_OK;     //debug
	return RESULT_OPERATION_TIMEOUT;
}

void RPLidar::setAngle(int min, int max) {
	angMin = min;
	angMax = max;
}
void RPLidar::setLidar() {
	if (!IS_OK(waitPoint())) {
		printf("Connection failed.\n");
        printf("Try to invert Rx and Tx.\n");
        startScanNormal();
        wait_ms(2'000);
        printf("Done \n");
		//startScanNormal(0);
        //startScanExpress(0,0);
        //wait_us(2000000);
	}
    else{
        printf("Connection succeeded.\n");
        startScanNormal(1);
        //wait_ms(2'000);*/

    }
}
u_result RPLidar::_sendCommand(uint8_t cmd, const void* payload, size_t payloadsize){

	rplidar_cmd_packet_t pkt_header;
	rplidar_cmd_packet_t* header = &pkt_header;
	uint8_t checksum = 0;

	if (payloadsize && payload) {
		cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
	}

	header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
	header->cmd_flag = cmd;

	// send header first
	//same as -----> _chanDev->senddata(pkt_header, 2) ;
	_binded_serialdev->putc(header->syncByte);
	_binded_serialdev->putc(header->cmd_flag);

	//  _binded_serialdev->write( (uint8_t *)header, 2);

	if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
		checksum ^= RPLIDAR_CMD_SYNC_BYTE;
		checksum ^= cmd;
		checksum ^= (payloadsize & 0xFF);

		// calc checksum
		for (size_t pos = 0; pos < payloadsize; ++pos) {
			checksum ^= ((uint8_t*)payload)[pos];
		}

		// send size
		uint8_t sizebyte = payloadsize;
		//printf("%d %d !!\n",sizebyte  ,checksum);
		_binded_serialdev->putc(sizebyte);
		//  _binded_serialdev->write((uint8_t *)&sizebyte, 1);

		  // send payload
        for(int i=0;i<sizebyte;i++)//please check again!!!!!!
	        _binded_serialdev->putc(((uint8_t*)payload)[i]);
	    //  _binded_serialdev->write((uint8_t *)&payload, sizebyte,  event_callback_t &callback);
        //_binded_serialdev->write(const uint8_t *buffer, int length, const event_callback_t &callback)
		  
          // send checksum
		_binded_serialdev->putc(checksum);
		//  _binded_serialdev->write((uint8_t *)&checksum, 1);

	}

	return RESULT_OK;
}

u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t* header, uint32_t timeout){
	uint8_t  recvPos = 0;
	uint32_t startTs = timers.read_ms();
	uint32_t remainingtime;
	uint8_t* headerbuf = (uint8_t*)header;
	// printf("%d   ",timers.read_ms());
	while (1) {//(timers.read_ms() - startTs) <= timeout
		uint8_t currentbyte = _binded_serialdev->getc();
		if (currentbyte < 0) continue;
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
	printf("RESULT_OPERATION_TIMEOUT 11\n");
	return RESULT_OPERATION_TIMEOUT;
}


// RPLidar::RPLidar()
//     : _isConnected(false)
//     , _isScanning(false)
//     , _isSupportingMotorCtrl(false)
// {
//     _cached_scan_node_hq_count = 0;
//     _cached_scan_node_hq_count_for_interval_retrieve = 0;
//     _cached_sampleduration_std = LEGACY_SAMPLE_DURATION;
//     _cached_sampleduration_express = LEGACY_SAMPLE_DURATION;
// }
