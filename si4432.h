/*
 * Si4432 library for Arduino
 *
 * Please note that Library uses standard SS pin for NSEL pin on the chip. This is 53 for Mega, 10 for Uno.
 *
 * made by Ahmet (theGanymedes) Ipkin 2014
 *
 * (c) 2022 nopnop2002
 * (c) 2023 Jens B.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef si4432_H_
#define si4432_H_

#include "Arduino.h"
#include <SPI.h>

/*
 * To perform a send/receive you must:
 *
 * 1- Create an instance
 * 2- Configure parameters with setXXXX() where defaults are not suitable
 * 3- Init SPI with init()
 * 4a- Send with sendPacket()
 * 4b- Receive with startListening() + isPacketReceived() + getPacketReceived()
 *
 * According to the data sheet, you can change any register value and most will get to work after going into IDLE state and back (to RX/TX)
 * (some are hot - changes right away) I didn't test this - but why not? :)
 *
 * Antenna configuration depends on board. If an antenna switch exists that can be controlled via GPIO the GPIO function must be configured
 * after init or reset by the application. As internal conditions (e.g. transmit timeout) can also trigger a reset, implementation is best
 * done by overriding boot(). Here is an example for 1 antenna, 1 switch and separate TX/RX paths:
 *
 * setTransmitPower(7); // disable direct-tie
 * ChangeRegister(REG_GPIO0_CONF,  0b10010); // Tx state output
 * ChangeRegister(REG_GPIO1_CONF,  0b10101); // Rx state output
 * ChangeRegister(REG_GPIO2_CONF, 0b100011); // passive input with 200 kOhm pullup
 *
 * When the packet handler is enabled the following packet configuration is used:
 * - 4 byte preamble (0/1 bit sequence)
 * - 2 byte sync word (0x2DD4)
 * - 2 byte header, defined with setCommsSignature()
 * - 1 byte packet length
 * - variable data bytes
 * - 1 byte CRC-16 (IBM)
 *
 * The default packet configuration can be changed using setConfigCallback() or in the source, see data sheet for more details.
 *
 */

class Si4432 {
public:
	enum ModulationType {
		NONE = 0x00,
		OOK  = 0x01,
		FSK  = 0x02,
		GFSK = 0x03,
	};

	// when intPin is given, interrupts are checked with this pin - rather than SPI polling
	Si4432(uint8_t csPin, uint8_t sdnPin = 0xFF, uint8_t intPin = 0xFF);

	// get interrupt pin, returns 0xFF if not set - may be used to attach ISR
	uint8_t getIntPin() const;
	 // get interrupt flags, see enum INT, also clears pending interrupts - may be used in ISR
	uint16_t getIntStatus();
	// enable interrupt sources, see enum INT - typically used internally
	void enableInt(uint16_t flags);

	// sets the frequency [MHz], 240..930 MHz, default 433 MHz - call before switching to tx or rx mode
	void setFrequency(int frequency);
	// sets the frequency [MHz], 240..930 MHz, default 433 MHz - call before switching to tx or rx mode
	void setFrequency(unsigned long frequency);
	// sets the frequency [MHz], 240..930 MHz, default 433 MHz - call before switching to tx or rx mode
	void setFrequency(double frequency);
	// select a 1 MHz channel rel. to the frequency, 0..255, default 0 - call before switching to tx or rx mode
	void setChannel(byte channel);

	// sets modulation type GFSK or OOK, default GFSK - call before setting baud rate
	void setModulationType(ModulationType modulationType);
	// select Manachester encoding, default disabled
	void setManchesterEncoding(bool enabled, bool inverted = false);
	// sets the bit rate, 1..256 kbps, default 100 kbps - call before switching to tx or rx mode
	void setBaudRate(int kbps);
	// sets the bit rate, 1..256 kbps, default 100 kbps - call before switching to tx or rx mode
	void setBaudRate(uint16_t kbps);
	// sets the bit rate, 0.123 .. 256 kbps, default 100 kbps - call before switching to tx or rx mode
	void setBaudRate(double kbps);

	// enables packet handling, default on - call before init/reset
	void setPacketHandling(bool enabled, bool lsbFirst = false);
	// set packet header value, default 0xDEAD - call before init/reset
	void setCommsSignature(uint16_t signature);

	// blocking, also performs reset and boot to idle mode, spiClock ..10000000
	bool init(SPIClass* spi = &SPI, uint32_t spiClock = 8000000);

	// 0 (min) .. 7 (max), default 7 - call before switching to tx or rx mode
	void setTransmitPower(byte level);
	// make sendPacket block until Tx has completed, default true - call before sendPacket
	void setSendBlocking(bool enabled = true);
	// switches to Tx mode and sends the package, length 0..64, length = 0 repeats last package
	bool sendPacket(uint8_t length = 0, const byte* data = nullptr);
	// wait for Tx to complete, also returns false on transmit timeout
	bool waitTransmitCompleted();

	// switch to Rx mode (don't block)
	void startListening();
	// check for the packet received flags
	bool isPacketReceived();
	// read from FIFO
	void getPacketReceived(uint8_t* length, byte* readData);

	// set idle operation mode to use after completing boot, TX/FIFO or RX/single, StandbyMode/SleepMode/Ready/TuneMode, default Ready - call before init/sendPacket/startListening
	//void setIdleMode(byte mode);
	// freqerr (bit 3) should not be set, see datasheet
	byte getDeviceStatus();

	void readAll();

	void clearTxFIFO();
	void clearRxFIFO();

	void clearFIFO();
	// blocking, also performs boot
	void softReset();
	// blocking for ~17 ms, also performs boot
	void hardReset();

	// non-blocking - wakeup takes ~17 ms, use isClockReady to check status
	void turnOn();
	// non-blocking
	void turnOff();
	// check if oscillator is in ready state
	bool isClockReady();

protected:
	enum AntennaMode {
		RXMode = 0x04, TXMode = 0x08, Ready = 0x01, TuneMode = 0x02
	};

	uint8_t _csPin, _sdnPin, _intPin;

	float _freqCarrier;
	float _kbps;
	uint8_t _freqChannel;

	ModulationType _modulationType;

	//byte _idleMode;
	bool _manchesterEnabled;
	bool _manchesterInverted;
	bool _packetHandlingEnabled;
	bool _lsbFirst;
	bool _sendBlocking;

	SPIClass* _spi;
	uint32_t _spiClock;

	uint16_t _packageSign;

	enum Registers {
		REG_DEV_TYPE = 0x00,
		REG_DEV_VERSION = 0x01,
		REG_DEV_STATUS = 0x02,

		REG_INT_STATUS1 = 0x03,
		REG_INT_STATUS2 = 0x04,
		REG_INT_ENABLE1 = 0x05,
		REG_INT_ENABLE2 = 0x06,
		REG_STATE = 0x07,
		REG_OPERATION_CONTROL = 0x08,

		REG_GPIO0_CONF = 0x0B,
		REG_GPIO1_CONF = 0x0C,
		REG_GPIO2_CONF = 0x0D,
		REG_IOPORT_CONF = 0x0E,

		REG_IF_FILTER_BW = 0x1C,
		REG_AFC_LOOP_GEARSHIFT_OVERRIDE = 0x1D,
		REG_AFC_TIMING_CONTROL = 0x1E,
		REG_CLOCK_RECOVERY_GEARSHIFT = 0x1F,
		REG_CLOCK_RECOVERY_OVERSAMPLING = 0x20,
		REG_CLOCK_RECOVERY_OFFSET2 = 0x21,
		REG_CLOCK_RECOVERY_OFFSET1 = 0x22,
		REG_CLOCK_RECOVERY_OFFSET0 = 0x23,
		REG_CLOCK_RECOVERY_TIMING_GAIN1 = 0x24,
		REG_CLOCK_RECOVERY_TIMING_GAIN0 = 0x25,
		REG_RSSI = 0x26,
		REG_RSSI_THRESHOLD = 0x27,

		REG_AFC_LIMITER = 0x2A,
		REG_AFC_CORRECTION_READ = 0x2B,

		REG_DATAACCESS_CONTROL = 0x30,
		REG_EZMAC_STATUS = 0x31,
		REG_HEADER_CONTROL1 = 0x32,
		REG_HEADER_CONTROL2 = 0x33,
		REG_PREAMBLE_LENGTH = 0x34,
		REG_PREAMBLE_DETECTION = 0x35,
		REG_SYNC_WORD3 = 0x36,
		REG_SYNC_WORD2 = 0x37,
		REG_SYNC_WORD1 = 0x38,
		REG_SYNC_WORD0 = 0x39,
		REG_TRANSMIT_HEADER3 = 0x3A,
		REG_TRANSMIT_HEADER2 = 0x3B,
		REG_TRANSMIT_HEADER1 = 0x3C,
		REG_TRANSMIT_HEADER0 = 0x3D,

		REG_PKG_LEN = 0x3E,

		REG_CHECK_HEADER3 = 0x3F,
		REG_CHECK_HEADER2 = 0x40,
		REG_CHECK_HEADER1 = 0x41,
		REG_CHECK_HEADER0 = 0x42,

		REG_RECEIVED_HEADER3 = 0x47,
		REG_RECEIVED_HEADER2 = 0x48,
		REG_RECEIVED_HEADER1 = 0x49,
		REG_RECEIVED_HEADER0 = 0x4A,

		REG_RECEIVED_LENGTH = 0x4B,

		REG_CHARGEPUMP_OVERRIDE = 0x58,
		REG_DIVIDER_CURRENT_TRIM = 0x59,
		REG_VCO_CURRENT_TRIM = 0x5A,

		REG_AGC_OVERRIDE = 0x69,

		REG_TX_POWER = 0x6D,
		REG_TX_DATARATE1 = 0x6E,
		REG_TX_DATARATE0 = 0x6F,

		REG_MODULATION_MODE1 = 0x70,
		REG_MODULATION_MODE2 = 0x71,

		REG_FREQ_DEVIATION = 0x72,
		REG_FREQ_OFFSET1 = 0x73,
		REG_FREQ_OFFSET2 = 0x74,
		REG_FREQBAND = 0x75,
		REG_FREQCARRIER_H = 0x76,
		REG_FREQCARRIER_L = 0x77,

		REG_FREQCHANNEL = 0x79,
		REG_CHANNEL_STEPSIZE = 0x7A,

		REG_FIFO = 0x7F,

	};

	void boot();
	void switchMode(byte mode);

	void ChangeRegister(Registers reg, byte value);
	byte ReadRegister(Registers reg);

	void BurstWrite(Registers startReg, const byte value[], uint8_t length);
	void BurstRead(Registers startReg, byte value[], uint8_t length);

};

#endif /* si4432_H_ */
