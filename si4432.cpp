#include "si4432.h"

#define MAX_TRANSMIT_TIMEOUT 200 // [ms]
//#define DEBUG // note: readback of registers and serial output will slow down operations

//values here are kept in khz x 10 format (for not to deal with decimals) - look at AN440 page 26 for whole table
const uint16_t IFFilterTable[][2] = { {  322, 0x26 },
                                      { 3355, 0x88 },
                                      { 3618, 0x89 },
                                      { 4202, 0x8A },
                                      { 4684, 0x8B },
                                      {	5188, 0x8C },
                                      { 5770, 0x8D },
                                      { 6207, 0x8E }
                                    };

Si4432::Si4432(uint8_t csPin, uint8_t sdnPin, uint8_t intPin) :
		_csPin(csPin),
		_sdnPin(sdnPin),
		_intPin(intPin),
		_freqCarrier(433.0),
		_kbps(100),
		_freqChannel(0),
		_modulationType(GFSK),
		_idleMode(Ready),
		_transmitPower(7),
		_directTie(true),
		_manchesterEnabled(false),
		_manchesterInverted(false),
		_packetHandlingEnabled(true),
		_lsbFirst(false),
		_sendBlocking(true),
		_spi(nullptr),
		_spiClock(0),
		_packageSign(0xDEAD),
		_sendStart(0),
		_configCallback(nullptr) {
}

void Si4432::setFrequency(int frequency) {
	setFrequency((double)frequency);
}

void Si4432::setFrequency(unsigned long frequency) {
	setFrequency((double)frequency);
}

void Si4432::setFrequency(double frequency) {

	if ((frequency < 240) || (frequency > 930))
		return; // invalid frequency

	_freqCarrier = frequency;

	byte highBand = 0;
	if (frequency >= 480) {
		highBand = 1;
	}

	float fPart = frequency / (10 * (highBand + 1)) - 24;

	uint8_t freqBand = (uint8_t)fPart; // truncate the int

	uint16_t freqCarrier = (fPart - freqBand) * 64000;

	// rx sideband is always on (bit 6)
	byte vals[3] = { (byte)((1 << 6) | (highBand << 5) | (freqBand & 0x3F)),
	                 (byte)(freqCarrier >> 8),
	                 (byte)(freqCarrier & 0xFF)
	               };
	BurstWrite(REG_FREQBAND, vals, 3);
}

void Si4432::setCommsSignature(uint16_t signature) {
	_packageSign = signature;

	ChangeRegister(REG_TRANSMIT_HEADER3, _packageSign >> 8); // header (signature) byte 3 val
	ChangeRegister(REG_TRANSMIT_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val

	ChangeRegister(REG_CHECK_HEADER3, _packageSign >> 8); // header (signature) byte 3 val for receive checks
	ChangeRegister(REG_CHECK_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val for receive checks

#ifdef DEBUG
	Serial.println("Package signature is set!");
#endif
}

bool Si4432::init(SPIClass* spi, uint32_t spiClock) {
	_spi = spi;
	_spiClock = spiClock;

	if (_intPin != 0xFF)
	{
		pinMode(_intPin, INPUT);
	}

	if (_sdnPin != 0xFF) {
		pinMode(_sdnPin, OUTPUT);
		digitalWrite(_sdnPin, LOW); // turn chip on
	} else {
		delay(50); // why wait so long?
	}

	if (_csPin != 0xFF) {
		pinMode(_csPin, OUTPUT);
		digitalWrite(_csPin, HIGH); // set SPI CS pin high, so chip would know we don't use it
	}

	_spi->begin();

#ifdef DEBUG
	Serial.println("SPI initialized");
#endif

	reset();

	// Check Sync Word
	byte syncWord3 = ReadRegister(REG_SYNC_WORD3);
	byte syncWord2 = ReadRegister(REG_SYNC_WORD2);
	byte syncWord1 = ReadRegister(REG_SYNC_WORD1);
	byte syncWord0 = ReadRegister(REG_SYNC_WORD0);

#ifdef DEBUG
	Serial.print("syncWord3=");
	Serial.print(syncWord3, HEX);
	Serial.print(" syncWord2=");
	Serial.print(syncWord2, HEX);
	Serial.print(" syncWord1=");
	Serial.print(syncWord1, HEX);
	Serial.print(" syncWord0=");
	Serial.print(syncWord0, HEX);
	Serial.println();
#endif

	if (syncWord3 != 0x2D || syncWord2 != 0xD4 || syncWord1 != 0 || syncWord0 != 0)
		return false;
	else
		return true;

	if (getDeviceStatus() & 0x4)
	{
		// frequency error
		return false;
	}
}

void Si4432::boot() {
	/*
	 byte currentFix[] = { 0x80, 0x40, 0x7F };
	 BurstWrite(REG_CHARGEPUMP_OVERRIDE, currentFix, 3); // refer to AN440 for reasons
	 */

	ChangeRegister(REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	ChangeRegister(REG_AFC_LIMITER, 0xFF); // write max value - excel file did that.
	ChangeRegister(REG_AGC_OVERRIDE, 0x60); // max gain control
	ChangeRegister(REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C); // turn off AFC

	if (_packetHandlingEnabled) {
		ChangeRegister(REG_DATAACCESS_CONTROL, 0xAD | (_lsbFirst? 1 << 6 : 0)); // enable rx packet handling, enable tx packet handling, enable CRC, use CRC-IBM
		ChangeRegister(REG_HEADER_CONTROL1, 0x0C); // no broadcast address control, enable check headers for bytes 3 & 2
		ChangeRegister(REG_HEADER_CONTROL2, 0x22); // enable headers byte 3 & 2, no fixed package length, sync word 3 & 2
		ChangeRegister(REG_PREAMBLE_LENGTH, 0x08); // 8 * 4 bits = 32 bits (4 bytes) preamble length
		ChangeRegister(REG_PREAMBLE_DETECTION, 0x3A); // validate 7 * 4 bits of preamble in a package
		setCommsSignature(_packageSign); // default signature
	} else {
		ChangeRegister(REG_DATAACCESS_CONTROL, _lsbFirst? 1 << 6 : 0); // disable packet handling
	}

	ChangeRegister(REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 100 * 10 kHz = 1 MHz interval

	// backup current settings
	byte oldTransmitPower = _transmitPower;
	_transmitPower = 0xFF;
	float oldFreqCarrier = _freqCarrier;
	_freqCarrier = 0;
	float oldKbps = _kbps;
	_kbps = 0;
	uint16_t oldFreqChannel = _freqChannel;
	_freqChannel = 0xFFFF;

	if (_configCallback) {
		_configCallback();
	}

	// perform missing config
	if (_transmitPower == 0xFF)
		setTransmitPower(oldTransmitPower, _directTie); // default max power (7), direct-tie enabled
	if (_freqCarrier == 0)
		setFrequency(oldFreqCarrier); // default freq is 433 MHz
	if (_kbps == 0)
		setBaudRate(oldKbps); // default baud rate is 100 kpbs
	if (_freqChannel == 0xFFFF)
		setChannel((byte)oldFreqChannel); // default channel is 0

	switchMode(_idleMode);
}

bool Si4432::sendPacket(uint8_t length, const byte* data) {

	bool result = false;

	if (length <= 64) {
		if (length)	{
			// write data into FIFO
			clearTxFIFO();
			ChangeRegister(REG_PKG_LEN, length);
			BurstWrite(REG_FIFO, data, length);
		}

		// enable interrupt for package sent
		enableInt(INT_PKSENT);

		// read interrupt registers to clean them
		getIntStatus();

		// start transmit (will automatically return to previously assigned mode after transmit is completed)
		switchMode(_idleMode | TXMode);
		_sendStart = millis();

		// wait for transmit to complete
		result = true;
		if (_sendBlocking) {
			result = waitTransmitCompleted();
			if (!result) {
				// transmit timeout, reset
				reset();
			}
		}
	}

	return result;
}

bool Si4432::waitTransmitCompleted() {
	while (millis() - _sendStart < MAX_TRANSMIT_TIMEOUT) {

		// check interrupt pin if configured
		if (_intPin != 0xFF && digitalRead(_intPin)) {
			// no interrupt, wait
			continue;
		}

		// typical state change without interrupt signal:
		// 0x20 (TX enabled: FIFO almost empty = itxffaem) -> 0x80 (TX FIFO empty = ifferr) -> 0xA0 (itxffaem + ifferr) -> 0xA4 (packet send, ipksent)
		uint16_t intStatus = getIntStatus();
#ifdef DEBUG
		Serial.print("sendPacket REG_INT_STATUS=");
		Serial.println(intStatus, HEX);
#endif
		if (intStatus & INT_PKSENT) {
			return true;
		}

		delay(1);
	}

	// timeout
#ifdef DEBUG
	Serial.println("Transmit Timeout --");
#endif

	return false;
}

void Si4432::getPacketReceived(uint8_t* length, byte* readData) {

	*length = ReadRegister(REG_RECEIVED_LENGTH);

	BurstRead(REG_FIFO, readData, *length);

	clearRxFIFO(); // which will also clear the interrupts
}

void Si4432::setChannel(byte channel) {
	_freqChannel = channel;
	ChangeRegister(REG_FREQCHANNEL, channel);
}

void Si4432::switchMode(byte mode) {
#ifdef DEBUG
	Serial.print("switchMode mode=0x");
	Serial.println(mode, HEX);
#endif
	ChangeRegister(REG_STATE, mode); // operation mode
#ifdef XDEBUG
	byte val = getDeviceStatus();
	if (val == 0 || val == 0xFF) {
		Serial.print(val, HEX);
		Serial.println(" -- WHAT THE HELL!!");
	} else if (val & 0x4) {
		// frequency error
		Serial.print("switchMode freqerr");
	}
#endif
}

void Si4432::ChangeRegister(Registers reg, byte value) {
	BurstWrite(reg, &value, 1);

#ifdef DEBUG
	byte _value;
	BurstRead(reg, &_value, 1);
	if (value != _value) {
		Serial.println("ChangeRegister failed");
		Serial.print("reg=0x");
		Serial.print(reg, HEX);
		Serial.print(" value=0x");
		Serial.println(_value, HEX);
	}
#endif
}

void Si4432::setBaudRate(int kbps) {
	setBaudRate((double)kbps);
}

void Si4432::setBaudRate(uint16_t kbps) {
	setBaudRate((double)kbps);
}

void Si4432::setBaudRate(double kbps) {

	// chip normally supports very low bps values, but they are cumbersome to implement - so I just didn't implement lower bps values
	if ((kbps < 1) || (kbps > 256))
		return;

	_kbps = kbps;

	// set modulation (GFSK or OOK) and frequency deviation
	byte modulationMode1 = (kbps < 30 ? 1 << 5 : 0) | (_manchesterInverted? 1 << 2 : 0) | (_manchesterEnabled? 1 << 1 : 0); // txdtrtscale (bit 5), enmaninv (bit2), enmanch (bit 1)
	byte modulationMode2 = _modulationType == OOK? 0x21 : 0x23;  // 0x20 = dtmod = FIFO, 0x01 = modtyp = OOK, 0x03 = modtyp = GFSK
	byte freqDev = round(((kbps <= 10 ? 15 : 150) * 1000.0) / 625.0);	// 15kHz / 150 kHz, msb of kpbs to 3rd bit of register

	byte modulationVals[] = { modulationMode1,
	                          modulationMode2,
	                          freqDev
	                        };
	BurstWrite(REG_MODULATION_MODE1, modulationVals, 3); // 0x70

	// set tx data rate
	uint16_t bpsRegVal = round((kbps * (kbps < 30 ? 1 << 21 : 1 << 16)) / 1000);
#ifdef DEBUG
	Serial.print("kbps: ");
	Serial.println(kbps);
	Serial.print("tx rate reg value: ");
	Serial.println(bpsRegVal, HEX);
#endif
	byte datarateVals[] = { (byte)(bpsRegVal >> 8),
	                        (byte)(bpsRegVal & 0xFF)
	                      };
	BurstWrite(REG_TX_DATARATE1, datarateVals, 2); // 0x6E

	// set rx timings
	uint16_t minBandwidth = (uint16_t)(freqDev << 1) + (uint16_t)round(kbps);
#ifdef DEBUG
	Serial.print("min Bandwidth value: ");
	Serial.println(minBandwidth, HEX);
#endif
	byte IFValue = 0xFF;
	//since the table is ordered (from low to high), just find the 'minimum bandwidth which is greater than required'
	for (byte i = 0; i < 8; ++i) {
		if (IFFilterTable[i][0] >= (minBandwidth * 10)) {
			IFValue = IFFilterTable[i][1];
			break;
		}
	}
#ifdef DEBUG
	Serial.print("Selected IF value: ");
	Serial.println(IFValue, HEX);
#endif

	ChangeRegister(REG_IF_FILTER_BW, IFValue);

	byte dwn3_bypass = (IFValue & 0x80) ? 1 : 0; // if msb is set
	byte ndec_exp = (IFValue >> 4) & 0x07; // only 3 bits

	uint16_t rxOversampling = round((500.0 * (1 + 2 * dwn3_bypass)) / ((pow(2, ndec_exp - 3)) * kbps));

	uint32_t ncOffset = ceil((kbps * (pow(2, ndec_exp + 20))) / (500.0 * (1 + 2 * dwn3_bypass)));

	uint16_t crGain = 2 + ((65535 * (int64_t)kbps) / ((int64_t) rxOversampling * freqDev));
	byte crMultiplier = 0x00;
	if (crGain > 0x7FF) {
		crGain = 0x7FF;
	}
#ifdef DEBUG
	Serial.print("dwn3_bypass value: ");
	Serial.println(dwn3_bypass, HEX);
	Serial.print("ndec_exp value: ");
	Serial.println(ndec_exp, HEX);
	Serial.print("rxOversampling value: ");
	Serial.println(rxOversampling, HEX);
	Serial.print("ncOffset value: ");
	Serial.println(ncOffset, HEX);
	Serial.print("crGain value: ");
	Serial.println(crGain, HEX);
	Serial.print("crMultiplier value: ");
	Serial.println(crMultiplier, HEX);
#endif

	byte timingVals[] = { (byte)(rxOversampling & 0xFF),
	                      (byte)(((rxOversampling & 0x0700) >> 3) | ((ncOffset >> 16) & 0x0F)),
	                      (byte)((ncOffset >> 8) & 0xFF),
	                      (byte)(ncOffset & 0xFF),
	                      (byte)(((crGain & 0x0700) >> 8) | crMultiplier),
	                      (byte)(crGain & 0xFF)
	                    };
	BurstWrite(REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);
}

byte Si4432::ReadRegister(Registers reg) {
	byte val[1];
	BurstRead(reg, val, 1);
	return val[0];
}

void Si4432::BurstWrite(Registers startReg, const byte value[], uint8_t length) {

	byte regVal = (byte) startReg | 0x80; // set MSB

	if (_csPin != 0xFF) {
		digitalWrite(_csPin, LOW);
	}
	_spi->beginTransaction(SPISettings(_spiClock, MSBFIRST, SPI_MODE0));
	_spi->transfer(regVal);

	for (byte i = 0; i < length; ++i) {
#ifdef DEBUG
		Serial.print("  writing: ");
		Serial.print((regVal != 0xFF ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
#endif
		_spi->transfer(value[i]);
	}

	if (_csPin != 0xFF) {
		digitalWrite(_csPin, HIGH);
	}
	_spi->endTransaction();
}

void Si4432::BurstRead(Registers startReg, byte value[], uint8_t length) {

	byte regVal = (byte) startReg & 0x7F; // clear MSB

	if (_csPin != 0xFF) {
		digitalWrite(_csPin, LOW);
	}
	_spi->beginTransaction(SPISettings(_spiClock, MSBFIRST, SPI_MODE0));
	_spi->transfer(regVal);

	for (byte i = 0; i < length; ++i) {
		value[i] = _spi->transfer(0xFF);

#ifdef DEBUG
		Serial.print("  reading: ");
		Serial.print((regVal != 0x7F ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
#endif
	}

	if (_csPin != 0xFF) {
		digitalWrite(_csPin, HIGH);
	}
	_spi->endTransaction();
}

void Si4432::readAll() {

	byte allValues[0x7F];

	BurstRead(REG_DEV_TYPE, allValues, 0x7F);

#ifdef DEBUG
	for (byte i = 0; i < 0x7f; ++i) {
		Serial.print("REG(");
		Serial.print((int) REG_DEV_TYPE + i, HEX);
		Serial.print(") : ");
		Serial.println((int) allValues[i], HEX);
	}
#endif
}

void Si4432::clearTxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x01);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::clearRxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x02);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::clearFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x03);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::softReset() {
	reset(true);
}

void Si4432::hardReset() {
	reset();
}

void Si4432::reset(bool soft) {

#ifdef DEBUG
  	Serial.println("resetting Si4432 ...");
#endif

	if (soft || _sdnPin == 0xFF) {
		// soft reset command
		switchMode(Ready);
		switchMode(Reset);
	} else {
		// toggle shutdown pin off/on
		turnOff();
		delay(1);
		turnOn();
		delay(15);
	}

	// wait for clock to become ready (changing registers will not work without it)
	int noTimeout = 15; // [ms]
	while (!isClockReady() && noTimeout) {
		delay(1);
		noTimeout--;
	}

	if (noTimeout)
	{
#ifdef DEBUG
		Serial.println("Si4432 reset successful");
#endif
		boot();
	}
#ifdef DEBUG
	else
	{
		Serial.println("Si4432 not responding after reset, check wiring");
	}
#endif
}

void Si4432::startListening() {

	clearRxFIFO(); // clear first, so it doesn't overflow if packet is big

	// set interrupts on for package received and CRC error
#ifdef DEBUG
	enableInt(INT_SWDET | INT_PREAVAL | INT_PKVALID | INT_CRCERROR);
#else
	enableInt(INT_PKVALID | INT_CRCERROR);
#endif

	//read interrupt registers to clean them
	getIntStatus();

	switchMode(_idleMode | RXMode);
}

bool Si4432::isPacketReceived() {

	if ((_intPin != 0xFF) && (digitalRead(_intPin) != 0)) {
		return false; // if no interrupt occurred, no packet received is assumed (since startListening will be called prior, this assumption is enough)
	}
	// check for package received status interrupt register
	uint16_t intStat = getIntStatus();
#ifdef DEBUG
	Serial.print("isPacketReceived REG_INT_STATUS=");
	Serial.println(intStat, HEX);

	if (intStat & INT_PREAVAL) { //interrupt occurred, check it && read the Interrupt Status1 register for 'preamble '

		Serial.print("HEY!! HEY!! Valid Preamble detected -- ");

	}

	if (intStat & INT_SWDET) { //interrupt occurred, check it && read the Interrupt Status1 register for 'preamble '

		Serial.print("HEY!! HEY!! SYNC WORD detected -- ");

	}
#endif

	if (intStat & INT_PKVALID) { //interrupt occurred, check it && read the Interrupt Status1 register for 'valid packet'
		switchMode(TuneMode); // if packet came, get out of Rx mode till the packet is read out. Keep PLL on for fast reaction
#ifdef DEBUG
		Serial.print("Packet detected -- ");
		Serial.println(intStat, HEX);
#endif
		return true;
	} else if (intStat & INT_CRCERROR) { // packet crc error
		switchMode(Ready); // get out of Rx mode till buffers are cleared
#ifdef DEBUG
		Serial.print("CRC Error in Packet detected!-- ");
		Serial.println(intStat, HEX);
#endif
		clearRxFIFO();
		switchMode(_idleMode | RXMode); // get back to work
		return false;
	}

	//no relevant interrupt? no packet!

	return false;
}

void Si4432::turnOn() {
	if (_sdnPin != 0xFF)
		digitalWrite(_sdnPin, LOW); // turn on the chip
}

void Si4432::turnOff() {
	if (_sdnPin != 0xFF)
		digitalWrite(_sdnPin, HIGH); // turn off the chip
	if (_csPin != 0xFF)
		digitalWrite(_csPin, HIGH); // set SPI CS pin high, so chip would know we don't use it
}

uint8_t Si4432::getIntPin() const {
	return _intPin;
}

uint16_t Si4432::getIntStatus() {

  uint16_t intStatus;

	BurstRead(REG_INT_STATUS1, (byte*)&intStatus, 2);

  return intStatus;
}

void Si4432::enableInt(uint16_t flags)
{
	BurstWrite(REG_INT_ENABLE1, (byte*)&flags, 2);
}

void Si4432::setModulationType(ModulationType modulationType) {
	_modulationType = modulationType;
}

void Si4432::setManchesterEncoding(bool enabled, bool inverted) {
	_manchesterEnabled = enabled;
	_manchesterInverted = inverted;
}

void Si4432::setPacketHandling(bool enabled, bool lsbFirst) {
	_packetHandlingEnabled = enabled;
	_lsbFirst = lsbFirst;
}

void Si4432::setConfigCallback(void (*callback)()) {
	_configCallback = callback;
}

void Si4432::setTransmitPower(byte level, bool directTie) {
	_transmitPower = min(level, 7);
	_directTie = directTie;
	ChangeRegister(REG_TX_POWER, 0x10 | (directTie? 1 << 4 : 0) | (level & 0x7));
}

void Si4432::setSendBlocking(bool enabled) {
	_sendBlocking = enabled;
}

void Si4432::setIdleMode(byte mode) {
	_idleMode = mode;
}

byte Si4432::getDeviceStatus()
{
	return ReadRegister(REG_DEV_STATUS);
}

bool Si4432::isClockReady()
{
	// notes: - an interrupt status of 0xFF is possible but not probable (see datasheet)
	//        - the value 0xFF is typically returned when the device is not responding
	//        - on power up the typical interrupt sequence is 0 -> 1 (ipor) -> 2 (ichiprdy)
	byte status = ReadRegister(REG_INT_STATUS2);
	return (status != 0xFF) && (status & 0x02);
}
