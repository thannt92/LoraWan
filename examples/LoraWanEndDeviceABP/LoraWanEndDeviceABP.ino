#include "LoraWan.h"
#include "SoftwareSerial.h"

SoftwareSerial loraStream(2, 3);
LoraWan exLoraWan(&loraStream);

uint32_t exDeviceAddress = 0xAAAA0001;
uint8_t exAppSKey[16]	= {0x08, 0x60, 0x05, 0x03, 0x40, 0xDB, 0x05, 0x00, 0xE4, 0x02, 0xC0, 0x4F, 0x0E, 0x70, 0xAD, 0x0B};
uint8_t exNwkKey[16]	= {0x0D, 0xB0, 0x3B, 0x08, 0x40, 0x50, 0x06, 0x70, 0xFD, 0x00, 0xF0, 0x04, 0x0D, 0xB0, 0xA9, 0x0A};

void setup() {
	Serial.begin(115200);
	loraStream.begin(9600);

	exLoraWan.setOperationMode(LoraWan_MODE_ENDDEVICE);
	exLoraWan.setDeviceClass(LoraWan_CLASS_C);
	exLoraWan.setChannelMask((uint16_t)0x0001); // 433175000 Mhz
	exLoraWan.setDataRate(LoraWan_DR_0); // SF12, BW125
	exLoraWan.setRx2Frequency((uint32_t)433175000); // 433175000 Mhz
	exLoraWan.setRx2DataRate(LoraWan_DR_0); // SF12, BW125
	exLoraWan.setTxPower(0x0F); //20dBm
	exLoraWan.setAntGain(0);

	exLoraWan.setDeviceAddr(exDeviceAddress);
	exLoraWan.setAppSessionKey(exAppSKey, 16);
	exLoraWan.setNetworkSessionKey(exNwkKey, 16);
}

void loop() {
	exLoraWan.update();
}
