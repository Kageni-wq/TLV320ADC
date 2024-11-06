
#include "TLV320ADC.h"

TLV320ADC::TLV320ADC()
{
    _initialized = false;
    _currentPage = 0;
}

bool TLV320ADC::begin(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl);
    _initialized = true;
    return true;
}

void TLV320ADC::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(TLV320ADC_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t TLV320ADC::readRegister(uint8_t reg)
{
    Wire.beginTransmission(TLV320ADC_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(TLV320ADC_I2C_ADDR, 1);
    return Wire.read();
}

void TLV320ADC::setBit(uint8_t reg, uint8_t bit, bool value)
{
    uint8_t regValue = readRegister(reg);
    if (value)
    {
        regValue |= (1 << bit);
    }
    else
    {
        regValue &= ~(1 << bit);
    }
    writeRegister(reg, regValue);
}

#include "TLV320ADC.h"

uint8_t TLV320ADC::getChannelBaseRegister(uint8_t channel)
{
    return CH1_CFG0_REG + (channel * 5); // Each channel has 5 configuration registers
}

void TLV320ADC::setInputType(uint8_t channel, InputType type)
{
    uint8_t reg = getChannelBaseRegister(channel);
    setBit(reg, 7, type);
}

void TLV320ADC::setInputSource(uint8_t channel, InputSource source)
{
    uint8_t reg = getChannelBaseRegister(channel);
    uint8_t value = readRegister(reg);
    value &= ~(0b11 << 5);  // Clear bits 6-5
    value |= (source << 5); // Set new source
    writeRegister(reg, value);
}

void TLV320ADC::setChannelGain(uint8_t channel, int8_t gain, bool isNegative)
{
    uint8_t reg = getChannelBaseRegister(channel) + 1; // CFG1 register
    uint8_t gainValue = abs(gain * 2);                 // Convert to 0.5dB steps
    if (gainValue > 84)
        gainValue = 84; // Maximum 42dB

    uint8_t value = (gainValue << 1) | (isNegative ? 1 : 0);
    writeRegister(reg, value);
}

void TLV320ADC::setDigitalVolume(uint8_t channel, float volume)
{
    uint8_t reg = getChannelBaseRegister(channel) + 2; // CFG2 register
    uint8_t volValue;

    if (volume <= -100.0)
        volValue = 1;
    else if (volume >= 27.0)
        volValue = 255;
    else
        volValue = (volume + 100) * 2; // Convert to 0.5dB steps offset

    writeRegister(reg, volValue);
}

void TLV320ADC::setPage(uint8_t page)
{
    writeRegister(ADCX140_PAGE_SELECT, page);
    _currentPage = page;
}

void TLV320ADC::reset()
{
    writeRegister(ADCX140_SW_RESET, 0x01);
    _currentPage = 0;
}

void TLV320ADC::sleep(bool enable)
{
    setBit(ADCX140_SLEEP_CFG, 0, !enable); // SLEEP_ENZ is active low
}

void TLV320ADC::setAnalogSupply(bool useInternalRegulator)
{
    setBit(ADCX140_SLEEP_CFG, 7, useInternalRegulator);
}

void TLV320ADC::setVrefQuickCharge(VrefQuickCharge duration)
{
    uint8_t value = readRegister(ADCX140_SLEEP_CFG);
    value &= ~(0b11 << 3); // Clear VREF_QCHG bits
    value |= (duration << 3);
    writeRegister(ADCX140_SLEEP_CFG, value);
}

void TLV320ADC::setInputCapQuickCharge(InputCapQuickCharge duration)
{
    uint8_t value = readRegister(ADCX140_SHDN_CFG);
    value &= ~(0b11 << 4); // Clear INCAP_QCHG bits
    value |= (duration << 4);
    writeRegister(ADCX140_SHDN_CFG, value);
}

void TLV320ADC::enableI2CBroadcast(bool enable)
{
    setBit(ADCX140_SLEEP_CFG, 2, enable);
}

void TLV320ADC::setASIFormat(ASIFormat format)
{
    uint8_t reg = readRegister(ADCX140_ASI_CFG0);
    reg &= ~(0b11 << 6);
    reg |= (format << 6);
    writeRegister(ADCX140_ASI_CFG0, reg);
}

void TLV320ADC::setWordLength(ASIWordLength length)
{
    uint8_t reg = readRegister(ADCX140_ASI_CFG0);
    reg &= ~(0b11 << 4);
    reg |= (length << 4);
    writeRegister(ADCX140_ASI_CFG0, reg);
}

void TLV320ADC::setFSYNCPolarity(bool invert)
{
    setBit(ADCX140_ASI_CFG0, 3, invert);
}

void TLV320ADC::setBCLKPolarity(bool invert)
{
    setBit(ADCX140_ASI_CFG0, 2, invert);
}

void TLV320ADC::setTXEdge(bool invert)
{
    setBit(ADCX140_ASI_CFG0, 1, invert);
}

void TLV320ADC::setTXFill(bool hiZ)
{
    setBit(ADCX140_ASI_CFG0, 0, hiZ);
}

void TLV320ADC::setTXOffset(uint8_t offset)
{
    if (offset > 31)
        offset = 31;
    uint8_t reg = readRegister(ADCX140_ASI_CFG1);
    reg &= ~0x1F; // Clear lower 5 bits
    reg |= offset;
    writeRegister(ADCX140_ASI_CFG1, reg);
}

void TLV320ADC::setASIMixing(ASIMixSelection mixMode)
{
    uint8_t reg = readRegister(ADCX140_ASI_MIX_CFG);
    reg &= ~(0b11 << 6);
    reg |= (mixMode << 6);
    writeRegister(ADCX140_ASI_MIX_CFG, reg);
}

void TLV320ADC::setASIInputGain(ASIInputGain gain)
{
    uint8_t reg = readRegister(ADCX140_ASI_MIX_CFG);
    reg &= ~(0b11 << 4);
    reg |= (gain << 4);
    writeRegister(ADCX140_ASI_MIX_CFG, reg);
}

uint8_t TLV320ADC::getASIChannelRegister(uint8_t channel)
{
    return ADCX140_ASI_CH1 + channel; // Registers are sequential
}

void TLV320ADC::setChannelSlot(uint8_t channel, uint8_t slot)
{
    if (slot > 63)
        slot = 63; // Maximum slot value is 63
    uint8_t reg = getASIChannelRegister(channel);
    writeRegister(reg, slot & 0x3F); // Write only the lower 6 bits
}

void TLV320ADC::setMicBias(MicBiasValue biasValue)
{
    uint8_t reg = readRegister(ADCX140_BIAS_CFG);
    reg &= ~(0x07 << 4); // Clear MBIAS_VAL bits
    reg |= (biasValue << 4);
    writeRegister(ADCX140_BIAS_CFG, reg);
}

void TLV320ADC::setADCFullScale(ADCFullScale scale)
{
    uint8_t reg = readRegister(ADCX140_BIAS_CFG);
    reg &= ~0x03; // Clear ADC_FSCALE bits
    reg |= scale;
    writeRegister(ADCX140_BIAS_CFG, reg);
}

void TLV320ADC::setDigitalVolumeRuntime(bool enable)
{
    setBit(ADCX140_DSP_CFG0, 7, !enable);
}

void TLV320ADC::setDecimationFilter(DecimationFilter filter)
{
    uint8_t reg = readRegister(ADCX140_DSP_CFG0);
    reg &= ~(0x03 << 4);
    reg |= (filter << 4);
    writeRegister(ADCX140_DSP_CFG0, reg);
}

void TLV320ADC::setChannelSummation(ChannelSum mode)
{
    uint8_t reg = readRegister(ADCX140_DSP_CFG0);
    reg &= ~(0x03 << 2);
    reg |= (mode << 2);
    writeRegister(ADCX140_DSP_CFG0, reg);
}

void TLV320ADC::setHighPassFilter(HPFSelect filter)
{
    uint8_t reg = readRegister(ADCX140_DSP_CFG0);
    reg &= ~0x03;
    reg |= filter;
    writeRegister(ADCX140_DSP_CFG0, reg);
}

void TLV320ADC::setVolumeGanging(bool enable)
{
    setBit(ADCX140_DSP_CFG1, 7, enable);
}

void TLV320ADC::setBiquadConfig(BiquadConfig config)
{
    uint8_t reg = readRegister(ADCX140_DSP_CFG1);
    reg &= ~(0x03 << 5);
    reg |= (config << 5);
    writeRegister(ADCX140_DSP_CFG1, reg);
}

void TLV320ADC::setSoftStepping(bool enable)
{
    setBit(ADCX140_DSP_CFG1, 4, !enable);
}

void TLV320ADC::setAGC(bool enable)
{
    setBit(ADCX140_DSP_CFG1, 3, enable);
}

void TLV320ADC::setAntiClipper(bool enable)
{
    setBit(ADCX140_DSP_CFG1, 0, enable);
}

void TLV320ADC::enableInputChannel(uint8_t channel, bool enable)
{
    if (channel > 3)
        return; // Only channels 0-3 are valid
    setBit(ADCX140_IN_CH_EN, 7 - channel, enable);
}

void TLV320ADC::enableASIOutputChannel(uint8_t channel, bool enable)
{
    if (channel > 3)
        return; // Only channels 0-3 are valid
    setBit(ADCX140_ASI_OUT_CH_EN, 7 - channel, enable);
}

void TLV320ADC::enableMicBias(bool enable)
{
    setBit(ADCX140_PWR_CFG, 7, enable);
}

void TLV320ADC::enableADC(bool enable)
{
    setBit(ADCX140_PWR_CFG, 6, enable);
}

void TLV320ADC::enablePLL(bool enable)
{
    setBit(ADCX140_PWR_CFG, 5, enable);
}

void TLV320ADC::enableDynamicChannelPower(bool enable)
{
    setBit(ADCX140_PWR_CFG, 4, enable);
}

void TLV320ADC::setDynamicMaxChannels(DynamicMaxChannel config)
{
    uint8_t reg = readRegister(ADCX140_PWR_CFG);
    reg &= ~(0x03 << 2); // Clear DYN_MAXCH_SEL bits
    reg |= (config << 2);
    writeRegister(ADCX140_PWR_CFG, reg);
}

void TLV320ADC::enableVAD(bool enable)
{
    setBit(ADCX140_PWR_CFG, 0, enable);
}

bool TLV320ADC::isChannel1PoweredUp()
{
    return (readRegister(ADCX140_DEV_STS0) & (1 << 7)) != 0;
}

bool TLV320ADC::isChannel2PoweredUp()
{
    return (readRegister(ADCX140_DEV_STS0) & (1 << 6)) != 0;
}

DeviceMode TLV320ADC::getDeviceMode()
{
    uint8_t mode = (readRegister(ADCX140_DEV_STS1) >> 5) & 0x07;
    return static_cast<DeviceMode>(mode);
}
