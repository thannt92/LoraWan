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

#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <stdint.h>
#include <string.h>
#include "Arduino.h"
#include "Stream.h"

#define LoraWan_COMMAND_FRAME_SOF	0xEF
#define LoraWan_COMMAND_TO_OFFSET	1000	// (ms)

#define LoraWan_MODE_ENDDEVICE		0x00
#define LoraWan_MODE_FORWARDER		0x01
#define LoraWan_MODE_SERVER			0x02

#define LoraWan_ACTIVE_OTA			0x00
#define LoraWan_ACTIVE_ABP			0x01

#define LoraWan_CLASS_A		0x00
#define LoraWan_CLASS_B		0x01
#define LoraWan_CLASS_C		0x02

#define LoraWan_DR_0		0x00
#define LoraWan_DR_1		0x01
#define LoraWan_DR_2		0x02
#define LoraWan_DR_3		0x03
#define LoraWan_DR_4		0x04
#define LoraWan_DR_5		0x05
#define LoraWan_DR_6		0x06

class LoraWan {
public:
	typedef enum {
		CMD_SET_MODE,

		CMD_ENDD_SET_ACTIVE_MODE,
		CMD_ENDD_OTA_STATUS,
		CMD_ENDD_SET_DEV_EUI ,
		CMD_ENDD_SET_APP_EUI,
		CMD_ENDD_SET_APP_KEY,
		CMD_ENDD_SET_NET_ID,
		CMD_ENDD_SET_DEV_ADDR,
		CMD_ENDD_SET_NKW_SKEY,
		CMD_ENDD_SET_APP_SKEY,
		CMD_ENDD_SET_CLASS,
		CMD_ENDD_SET_CHAN_MASK,
		CMD_ENDD_SET_DATARATE,
		CMD_ENDD_SET_RX2_CHAN,
		CMD_ENDD_SET_RX2_DR,
		CMD_ENDD_SET_TX_PWR,
		CMD_ENDD_SET_ANT_GAIN,
		CMD_ENDD_GET_DEV_EUI,
		CMD_ENDD_GET_APP_EUI,
		CMD_ENDD_GET_APP_KEY,
		CMD_ENDD_GET_NET_ID,
		CMD_ENDD_GET_DEV_ADDR,
		CMD_ENDD_GET_NKW_SKEY,
		CMD_ENDD_GET_APP_SKEY,
		CMD_ENDD_GET_CLASS,
		CMD_ENDD_GET_CHAN_MASK,
		CMD_ENDD_GET_DATARATE,
		CMD_ENDD_GET_RX2_CHAN,
		CMD_ENDD_GET_RX2_DR,
		CMD_ENDD_GET_TX_PWR,
		CMD_ENDD_GET_ANT_GAIN,
		CMD_ENDD_SEND_UNCONFIRM_MSG,
		CMD_ENDD_SEND_CONFIRM_MSG,
		CMD_ENDD_RECV_CONFIRM_MSG,
		CMD_ENDD_RECV_UNCONFIRM_MSG,

		CMD_FW_SET_TX_CONFIG,
		CMD_FW_GET_TX_CONFIG,
		CMD_FW_SET_RX_CONFIG,
		CMD_FW_GET_RX_CONFIG,
		CMD_FW_SEND,
		CMD_FW_RECV,

		CMD_SERV_SET_TX_CONFIG,
		CMD_SERV_GET_TX_CONFIG,
		CMD_SERV_SET_RX_CONFIG,
		CMD_SERV_GET_RX_CONFIG,
		CMD_SERV_SET_NET_PARAM,
		CMD_SERV_GET_NET_PARAM,
		CMD_SERV_SET_JOINT_PERMIT,
		CMD_SERV_GET_JOINT_PERMIT,
		CMD_SERV_RESP_JOINT_PERMIT,
		CMD_SERV_REMOVE_DEVICE,
		CMD_SERV_REMOVE_DEVICE_ALL,
		CMD_SERV_SEND_UNCONFIRM_MSG,
		CMD_SERV_SEND_CONFIRM_MSG,
		CMD_SERV_RECV_UNCONFIRM_MSG,
		CMD_SERV_RECV_CONFIRM_MSG,
		CMD_SERV_RECV_CONFIRM,
	} eCmdId;

	typedef void (*pf_handler_msg)(uint8_t*);

	LoraWan(Stream* loraStream);

	void init(uint32_t streamBaudRate);
	int update();

	// Handle app messages.
	int incommingMsgHandlerRegister(pf_handler_msg func);

	static uint8_t getMsgCommandId(uint8_t* msg); // Return command id of message
	static uint8_t getMsgDataLength(uint8_t* msg); // Return data length of message
	static uint8_t* getMsgData(uint8_t* msg); // Return data pointer

	// Lora method
	int setOperationMode(uint8_t operationMode);
	int setActiveMode(uint8_t activeMode);

	int setDeviceClass(uint8_t deviceClass);
	int getDeviceClass(uint8_t* deviceClass);

	int setChannelMask(uint16_t channelMark);
	int getChannelMask(uint16_t* channelMark);

	int setDataRate(uint8_t dataRate);
	int getDataRate(uint8_t* dataRate);

	int setRx2Frequency(uint32_t frequency);
	int getRx2Frequency(uint32_t* frequency);

	int setRx2DataRate(uint8_t dataRate);
	int getRx2DataRate(uint8_t* dataRate);

	int setTxPower(uint8_t power);
	int getTxPower(uint8_t* power);

	int setAntGain(uint8_t antGain);
	int getAntGain(uint8_t* antGain);


	// LoraWan method
	int setDeviceEUI(uint8_t* pdata, uint8_t length);
	int getDeviceEUI(uint8_t* pdata, uint8_t length);

	int setDeviceAddr(uint32_t deviceAddress);
	int getDeviceAddr(uint32_t* deviceAddress);

	int setAppEUI(uint8_t* pdata, uint8_t length);
	int getAppEUI(uint8_t* pdata, uint8_t length);

	int setAppKey(uint8_t* pdata, uint8_t length);
	int getAppKey(uint8_t* pdata, uint8_t length);

	int setNetworkID(uint8_t* pdata, uint8_t length);
	int getNetworkID(uint8_t* pdata, uint8_t length);

	int setNetworkSessionKey(uint8_t* pdata, uint8_t length);
	int getNetworkSessionKey(uint8_t* pdata, uint8_t length);

	int setAppSessionKey(uint8_t* pdata, uint8_t length);
	int getAppSessionKey(uint8_t* pdata, uint8_t length);

	int setSendingConfigParams(uint32_t frequency, uint8_t dataRate, uint8_t power);
	int getSendingConfigParams(uint32_t* frequency, uint8_t* dataRate, uint8_t* power);

	int setReceivingConfigParams(uint32_t frequency, uint8_t dataRate);
	int getReceivingConfigParams(uint32_t* frequency, uint8_t* dataRate);

	int sendUnconfirmedData(uint32_t deviceAddress, uint8_t port, uint8_t* pdata, uint8_t length);
	int sendConfirmedData(uint32_t deviceAddress, uint8_t port, uint8_t Retries, uint8_t* pdata, uint8_t length);

	// Manage devices (Just for gateway)
	int requestJoinOTA(uint8_t* deviceEUI, uint8_t* appEUI, uint32_t timeout);
	int getJoinOTAPrams(uint8_t* deviceEUI, uint8_t* appEUI, uint32_t* timeout);

	int removeDevice(uint32_t deviceAddress);
	int removeAllDevice();

private:
#define LORA_STREAM_PARSER_BUFFER_SIZE  160

	typedef enum { Async, Sync } eTransceiverSyncState;
	typedef enum {
		SOF,    // Start of frame state
		LENGTH, // Lenght state
		CMD_ID, // Command Id state
		DATA,   // Data state
		FCS     // Frame checksum state
	} eLoraStreamParserState;

	uint8_t mLoraStreamParserBuffer[LORA_STREAM_PARSER_BUFFER_SIZE];
	uint8_t mParserDataIndex;

	eTransceiverSyncState mTransceiverSyncState;
	eLoraStreamParserState mLoraStreamParserState;
	Stream* mLoraStream;
	uint16_t mLoraStreamBaudRate;

	pf_handler_msg mIncommingMsgHander;

	void loraStreamFlush();
	uint8_t cmdFrameCalcFcs(uint8_t* pdata, uint8_t length);
	int loraStreamPushReqCmdFrame(uint8_t cmdID, uint8_t* pdataReq, uint8_t lengthReq);
	int loraStreamCheckTimeOut(uint8_t length);
	int loraStreamPushReqResCmdFrame(uint8_t cmdID, uint8_t* pdataReq, uint8_t lengthReq \
									 , uint8_t* pdataRes, uint8_t lengthRes);
	int loraStreamParser(uint8_t byteData);
};

#endif //__LORAWAN_H__


