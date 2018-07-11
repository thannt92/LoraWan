/* \logo
 *             _____ _               _
 *            |_   _| |__   ___  ___(_)___
 *              | | | '_ \ / _ \/ __| / __|
 *              | | | | | |  __/\__ \ \__ \
 *              |_| |_| |_|\___||___/_|___/
 *            |embedded.solutions_________|
 * \endlogo
 *
 * \file       LoraWan.h
 *
 * \brief      LoraWan Module interfaces.
 *
 * \date       10-7-2018
 *
 * \author     ThanNT
 */

#include "LoraWan.h"

LoraWan::LoraWan(Stream* loraStream) {
	mLoraStream = loraStream;
	mLoraStreamBaudRate = 9600; // Default is minimum baudrate.
	mTransceiverSyncState = LoraWan::Async;
	mLoraStreamParserState = LoraWan::SOF;
	mParserDataIndex = 0;
	mIncommingMsgHander = (pf_handler_msg)0;
}

void LoraWan::init(uint32_t streamBaudRate) {
	mLoraStreamBaudRate = streamBaudRate;
}

int LoraWan::update() {
	while (mLoraStream->available()) {
		loraStreamParser((uint8_t)mLoraStream->read());
	}
	return 0;
}

int LoraWan::incommingMsgHandlerRegister(LoraWan::pf_handler_msg func) {
	if (func) {
		mIncommingMsgHander = func;
		return 0;
	}
	return -1;
}

uint8_t LoraWan::getMsgCommandId(uint8_t* msg) {
	return (uint8_t)(*(msg + 1));
}

uint8_t LoraWan::getMsgDataLength(uint8_t* msg) {
	return (uint8_t)(*(msg));
}

uint8_t* LoraWan::getMsgData(uint8_t* msg) {
	if (getMsgDataLength(msg)) {
		return (uint8_t*)(msg + 2);
	}
	return (uint8_t*)0;
}

int LoraWan::setOperationMode(uint8_t operationMode) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_SET_MODE, &operationMode, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::setActiveMode(uint8_t activeMode) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_ACTIVE_MODE, &activeMode, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::setDeviceClass(uint8_t deviceClass) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_CLASS, &deviceClass, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getDeviceClass(uint8_t* deviceClass) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_CLASS, NULL, 0, deviceClass, sizeof(uint8_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setChannelMask(uint16_t channelMark) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_CHAN_MASK, (uint8_t*)&channelMark, sizeof(uint16_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getChannelMask(uint16_t *channelMark) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_CHAN_MASK, NULL, 0, (uint8_t*)channelMark, sizeof(uint16_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setDataRate(uint8_t dataRate) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_DATARATE, &dataRate, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getDataRate(uint8_t *dataRate) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_DATARATE, NULL, 0, dataRate, sizeof(uint8_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setRx2Frequency(uint32_t frequency) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_RX2_CHAN, (uint8_t*)&frequency, sizeof(uint32_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getRx2Frequency(uint32_t *frequency) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_RX2_CHAN, NULL, 0, (uint8_t*)frequency, sizeof(uint32_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setRx2DataRate(uint8_t dataRate) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_RX2_DR, &dataRate, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getRx2DataRate(uint8_t *dataRate) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_RX2_DR, NULL, 0, dataRate, sizeof(uint8_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setTxPower(uint8_t power) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_TX_PWR, &power, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getTxPower(uint8_t *power) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_TX_PWR, NULL, 0, power, sizeof(uint8_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setAntGain(uint8_t antGain) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_ANT_GAIN, &antGain, sizeof(uint8_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getAntGain(uint8_t *antGain) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_ANT_GAIN, NULL, 0, antGain, sizeof(uint8_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setDeviceEUI(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_DEV_EUI, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getDeviceEUI(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_DEV_EUI, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setDeviceAddr(uint32_t deviceAddress) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_DEV_ADDR, (uint8_t*)&deviceAddress, sizeof(uint32_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getDeviceAddr(uint32_t* deviceAddress) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_DEV_ADDR, NULL, 0, (uint8_t*)deviceAddress, sizeof(uint32_t)) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setAppEUI(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_DEV_EUI, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getAppEUI(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_APP_EUI, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setAppKey(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_APP_KEY, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getAppKey(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_APP_KEY, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setNetworkID(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_NET_ID, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getNetworkID(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_NET_ID, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setNetworkSessionKey(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_NKW_SKEY, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getNetworkSessionKey(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_NKW_SKEY, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setAppSessionKey(uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_SET_APP_SKEY, pdata, length, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getAppSessionKey(uint8_t *pdata, uint8_t length) {
	if (loraStreamPushReqResCmdFrame(CMD_ENDD_GET_APP_SKEY, NULL, 0, pdata, length) == 0) {
		return 0;
	}
	return -1;
}

int LoraWan::setSendingConfigParams(uint32_t frequency, uint8_t dataRate, uint8_t power) {
	uint8_t sendBuffer[6] = {0, 0, 0, 0, 0, 0};

	*((uint32_t*)&sendBuffer[0]) = frequency;
	*((uint8_t*)&sendBuffer[4]) = dataRate;
	*((uint8_t*)&sendBuffer[5]) = power;

	if (loraStreamPushReqResCmdFrame(CMD_FW_SET_TX_CONFIG, NULL, 0, sendBuffer, 6)) {
		return 0;
	}
	return -1;
}

int LoraWan::getSendingConfigParams(uint32_t *frequency, uint8_t *dataRate, uint8_t *power) {
	uint8_t sendBuffer[6];
	if (loraStreamPushReqResCmdFrame(CMD_FW_GET_TX_CONFIG, NULL, 0, sendBuffer, 6) == 0) {
		*frequency = *((uint32_t*)&sendBuffer[0]);
		*dataRate = *((uint8_t*)&sendBuffer[4]);
		*power = *((uint8_t*)&sendBuffer[5]);
		return 0;
	}
	*frequency = (uint32_t)0xFFFFFFFF;
	*dataRate = (uint8_t)0xFF;
	*power = (uint8_t)0xFF;
	return -1;
}

int LoraWan::setReceivingConfigParams(uint32_t frequency, uint8_t dataRate) {
	uint8_t sendBuffer[5] = {0, 0, 0, 0, 0};

	*((uint32_t*)&sendBuffer[0]) = frequency;
	*((uint8_t*)&sendBuffer[4]) = dataRate;

	if (loraStreamPushReqResCmdFrame(CMD_FW_SET_RX_CONFIG, NULL, 0, &sendBuffer[0], 5)) {
		return 0;
	}
	return -1;
}

int LoraWan::getReceivingConfigParams(uint32_t *frequency, uint8_t *dataRate) {
	uint8_t sendBuffer[5];
	if (loraStreamPushReqResCmdFrame(CMD_FW_GET_RX_CONFIG, NULL, 0, sendBuffer, 5) == 0) {
		*frequency = *((uint32_t*)&sendBuffer[0]);
		*dataRate = *((uint8_t*)&sendBuffer[4]);
		return 0;
	}
	*frequency = (uint32_t)0xFFFFFFFF;
	*dataRate = (uint8_t)0xFF;
	return -1;
}

int LoraWan::sendUnconfirmedData(uint32_t deviceAddress, uint8_t port, uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	uint8_t* sPframe = (uint8_t*)malloc(7 + length);
	*((uint32_t*)&sPframe[0]) = deviceAddress;
	*((uint8_t*)&sPframe[4]) = 0; //RFU
	*((uint8_t*)&sPframe[5]) = port;
	*((uint8_t*)&sPframe[6]) = length;
	memcpy((uint8_t*)&sPframe[7], pdata, length);

	if (loraStreamPushReqResCmdFrame(CMD_SERV_SEND_UNCONFIRM_MSG, sPframe, 7 + length, &sReturn, sizeof(uint8_t))) {
		free(sPframe);
		return sReturn;
	}
	free(sPframe);
	return -1;
}

int LoraWan::sendConfirmedData(uint32_t deviceAddress, uint8_t port, uint8_t Retries, uint8_t *pdata, uint8_t length) {
	uint8_t sReturn;
	uint8_t* sPframe = (uint8_t*)malloc(8 + length);
	*((uint32_t*)&sPframe[0]) = deviceAddress;
	*((uint8_t*)&sPframe[4]) = 0; //RFU
	*((uint8_t*)&sPframe[5]) = Retries;
	*((uint8_t*)&sPframe[6]) = port;
	*((uint8_t*)&sPframe[7]) = length;
	memcpy((uint8_t*)&sPframe[8], pdata, length);

	if (loraStreamPushReqResCmdFrame(CMD_SERV_SEND_CONFIRM_MSG, sPframe, 8 + length, &sReturn, sizeof(uint8_t))) {
		free(sPframe);
		return sReturn;
	}
	free(sPframe);
	return -1;
}

int LoraWan::requestJoinOTA(uint8_t* deviceEUI, uint8_t* appEUI, uint32_t timeout) {
	uint8_t sPframe[20];
	uint8_t sReturn;

	memcpy(&sPframe[0], deviceEUI, 8);
	memcpy(&sPframe[8], appEUI, 8);
	memcpy(&sPframe[16], (uint8_t*)&timeout, 4);

	if (loraStreamPushReqResCmdFrame(CMD_SERV_SET_JOINT_PERMIT, (uint8_t*)sPframe, 20, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::getJoinOTAPrams(uint8_t *deviceEUI, uint8_t *appEUI, uint32_t* timeout) {
	uint8_t sPframe[20];

	if (loraStreamPushReqResCmdFrame(CMD_SERV_GET_JOINT_PERMIT, NULL, 0, sPframe, 20) == 0) {
		memcpy(deviceEUI, &sPframe[0], 8);
		memcpy(appEUI, &sPframe[8], 8);
		memcpy((uint8_t*)&timeout, &sPframe[16], 4);

		return 0;
	}
	return -1;
}

int LoraWan::removeDevice(uint32_t deviceAddress) {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_SERV_REMOVE_DEVICE, (uint8_t*)&deviceAddress, sizeof(uint32_t), &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

int LoraWan::removeAllDevice() {
	uint8_t sReturn;
	if (loraStreamPushReqResCmdFrame(CMD_SERV_REMOVE_DEVICE_ALL, NULL, 0, &sReturn, sizeof(uint8_t)) == 0) {
		return sReturn;
	}
	return -1;
}

uint8_t LoraWan::cmdFrameCalcFcs(uint8_t *pdata, uint8_t length) {
	uint8_t xorResult = length;
	for (int i = 0; i < length; i++, pdata++) {
		xorResult = xorResult ^ *pdata;
	}
	return xorResult;
}

int LoraWan::loraStreamPushReqCmdFrame(uint8_t cmdID, uint8_t *pdataReq, uint8_t lengthReq) {
	uint8_t* pframe = (uint8_t*)malloc(4 + lengthReq);
	if (pframe) {
		pframe[0] = LoraWan_COMMAND_FRAME_SOF;
		pframe[1] = sizeof(uint8_t) + lengthReq;
		pframe[2] = cmdID;
		memcpy(&pframe[3], pdataReq, lengthReq);
		pframe[pframe[1] + 2] = cmdFrameCalcFcs(&pframe[2], pframe[1]);
		mLoraStream->write(pframe, 4 + lengthReq);
		free(pframe);
		return 0;
	}
	return -1;
}

int LoraWan::loraStreamCheckTimeOut(uint8_t length) {
	uint32_t sAvailableTime = ( length * ( 1 / ( mLoraStreamBaudRate / 10 /* StartBit + 8DataBit + StopBit */ ) ) ) \
			+ LoraWan_COMMAND_TO_OFFSET \
			+ millis();
	while (millis() - sAvailableTime) {
		if (mLoraStream->available() == (int)length) {
			return 0;
		}
	}
	loraStreamFlush();
	return -1;
}

int LoraWan::loraStreamPushReqResCmdFrame(uint8_t cmdID, uint8_t *pdataReq, uint8_t lengthReq, uint8_t *pdataRes, uint8_t lengthRes) {
	uint8_t* pframe = (uint8_t*)malloc(4 + lengthReq);

	if (pframe) {
		pframe[0] = LoraWan_COMMAND_FRAME_SOF;
		pframe[1] = sizeof(uint8_t) + lengthReq;
		pframe[2] = cmdID;

		if (lengthReq) {
			memcpy(&pframe[3], pdataReq, lengthReq);
		}
		pframe[pframe[1] + 2] = cmdFrameCalcFcs(&pframe[2], pframe[1]);
		mLoraStream->write(pframe, 4 + lengthReq);
		free(pframe);

		if (loraStreamCheckTimeOut(4 + lengthRes) == 0) {

			mTransceiverSyncState = Sync;

			for (int i = 0; i < 4 + lengthRes; i++) {
				if (loraStreamParser((uint8_t)mLoraStream->read()) == 1) {
					if (getMsgCommandId(mLoraStreamParserBuffer) != cmdID) {
						return -5;
					}
					else if (lengthRes == getMsgDataLength(mLoraStreamParserBuffer)) {
						memcpy(pdataRes, getMsgData(mLoraStreamParserBuffer), lengthRes);
					}
					else {
						return -6;
					}
				}
			}

			mTransceiverSyncState = Async;

			return 0;
		}

		return -2;
	}

	return -1;
}

int LoraWan::loraStreamParser(uint8_t byteData) {
	int sReturn = 0;

	switch (mLoraStreamParserState) {
	case SOF: { // Start of frame state
		if (LoraWan_COMMAND_FRAME_SOF == byteData) {
			mLoraStreamParserState = LENGTH;
			mLoraStreamParserBuffer[0] = byteData;
		}
		else {
			sReturn = -1; //(-1) Mean parser ignore data.
		}
	}
		break;

	case LENGTH: { // Lenght state
		mLoraStreamParserBuffer[1] = byteData;
		mLoraStreamParserState = CMD_ID;
	}
		break;

	case CMD_ID: { // Command Id state
		mLoraStreamParserBuffer[2] = byteData;
		if (mLoraStreamParserBuffer[1]) {
			mLoraStreamParserState = FCS;
		}
		else {
			mParserDataIndex = 0;
			mLoraStreamParserState = DATA;
		}
	}
		break;

	case DATA: { // Data state
		mLoraStreamParserBuffer[mParserDataIndex++] = byteData;
		if (mParserDataIndex == mLoraStreamParserBuffer[1]) {
			mLoraStreamParserState = FCS;
		}
	}
		break;

	case FCS: { // Frame checksum state
		mLoraStreamParserState = SOF;
		if (cmdFrameCalcFcs(&mLoraStreamParserBuffer[1], 2 + mLoraStreamParserBuffer[1]) == byteData) {
			// handle new message
			if (mTransceiverSyncState == Async) {
				if (mIncommingMsgHander) {
					mIncommingMsgHander(&mLoraStreamParserBuffer[1]);
				}
			}
			else {
				sReturn = 1;
			}
		}
		else {
			//TODO: handle checksum incorrect !
		}
	}
		break;

	default:
		break;
	}

	return sReturn;
}

void LoraWan::loraStreamFlush() {
	uint8_t c;
	while (mLoraStream->available()) {
		c = mLoraStream->read();
		(void)c;
	}
}
