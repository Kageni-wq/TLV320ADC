
#include "TLV320ADC.h"
#include "TLV320ADC_REG.h"

TLV320ADC::TLV320ADC()
{
    _initialized = false;
    _currentPage = 0;
}

/************************************************************************************************************Low Level Functions***************************************************************************************************/

bool TLV320ADC::begin(uint8_t address, TwoWire &wirePort) {
    _initialized = true;
    reset();
    delay(100);
    SleepCFG();
    setPage(0);
    AutoCFG();
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

void TLV320ADC::setPage(uint8_t page)
{
    writeRegister(ADCX140_PAGE_SELECT, page);
    _currentPage = page;
}

void TLV320ADC::reset()
{
    writeRegister(ADCX140_SW_RESET, 0x01);
    _currentPage = 0;
    _sleepConfig.awake = false;
    _powerConfig.PoweredUp = false;
}

void TLV320ADC::sleep(bool enable)
{
    setBit(ADCX140_SLEEP_CFG, 0, !enable); // SLEEP_ENZ is active low
    _sleepConfig.awake = !enable;
}


uint8_t TLV320ADC::getChannelBaseRegister(uint8_t channel)
{
    return ADCX140_CH1_CFG0 + (channel * 5); // Each channel has 5 configuration registers
}

uint8_t TLV320ADC::getASIChannelRegister(uint8_t channel)
{
    return ADCX140_ASI_CH1 + channel; // Registers are sequential
}

/************************************************************************************************************High Level Functions***************************************************************************************************/



int TLV320ADC::SleepCFG(){
    #ifdef I2C_PIN_ENABLED
    _sleepConfig.I2C_Address_Swap_Enabled = true;
    #endif
    
    if(_initialized == false) return MUST_BE_INITIALIZED;
    if(_sleepConfig.awake == true) return ALREADY_AWAKE;

    uint8_t Cfg = 0x00;
    Cfg = (_sleepConfig.AR_Supply << 7) | (_sleepConfig.Vref << 3) | (_sleepConfig.I2C_Address_Swap_Enabled << 2) | (_sleepConfig.awake << 0);
    writeRegister(ADCX140_SLEEP_CFG, Cfg);
    _sleepConfig.awake = true;
    return SUCCESS;
}

int TLV320ADC::EnableInputChannel(uint8_t channel) {
    if(channel > 3) return INVALID_CHANNEL;
    if(_sleepConfig.awake == false) return MUST_BE_AWAKE;
    if(_initialized == false) return MUST_BE_INITIALIZED;
    if(_powerConfig.PoweredUp == true) return MUST_BE_POWERED_DOWN;
    if(_inputChannels[channel].enabled == true) return ALREADY_ENABLED;
    
    InputChannelCFG& cfg = _inputChannels[channel];
    uint8_t baseReg = getChannelBaseRegister(channel);
    
    // CFG0: MIC_INPUT, ANALOG_DIFFERENTIAL by default
    uint8_t cfg0 = 0x00;
    cfg0 = (cfg.type << 7) | (cfg.source << 5) | (cfg.coupling << 4) | (cfg.impedance << 2);
    writeRegister(baseReg, cfg0);
    
    // CFG1: Input Gain, 0dB by default
    uint8_t cfg1 = 0x00;
    if(cfg.gain_db > MAX_INPUT_GAIN) return -2;
    uint8_t DB_Conversion = (cfg.gain_db * 2);
    cfg1 = (DB_Conversion << 1) | (cfg.gain_sign << 0);
    writeRegister(baseReg + 1, cfg1);

    // CFG2: Digital volume, 0dB by default
    if(cfg.DigitalVolume > MAX_INPUT_VOLUME) return -3;
    uint8_t cfg2 = cfg.DigitalVolume;
    writeRegister(baseReg + 2, cfg2);
    
    // Enable channel with fresh register value
    uint8_t enableMask = (1 << (7 - channel));
    writeRegister(ADCX140_IN_CH_EN, enableMask);
    
    cfg.enabled = true;
    return SUCCESS;
}

int TLV320ADC::EnableOutputChannel(uint8_t channel) {
    if(channel > 3) return INVALID_CHANNEL;
    if(_sleepConfig.awake == false) return MUST_BE_AWAKE;
    if(_initialized == false) return MUST_BE_INITIALIZED;
    if(_powerConfig.PoweredUp == true) return MUST_BE_POWERED_DOWN;
    if(_outputChannels[channel].enabled == true) return ALREADY_ENABLED;
    
    // Set Channel Slot 
    uint8_t asiChCfg = channel;
    asiChCfg = _outputChannels[channel].slot;
    writeRegister(getASIChannelRegister(channel), asiChCfg);
    
    // Enable output channel with fresh register value
    uint8_t enableMask = (1 << (7 - channel));
    writeRegister(ADCX140_ASI_OUT_CH_EN, enableMask);
    
    _outputChannels[channel].enabled = true;

    return SUCCESS;
}

int TLV320ADC::PowerUp() {
    if(_initialized == false) return MUST_BE_INITIALIZED;
    if(_sleepConfig.awake == false) return MUST_BE_AWAKE;
    if(_powerConfig.PoweredUp == true) return ALREADY_POWERED_UP;
    
    uint8_t pwrCfg = (_powerConfig.MicBias << 7) | (_powerConfig.ADC << 6) | (_powerConfig.PLL << 5) | (_powerConfig.DynamicCHPwr << 4) | (_powerConfig.DynamicCHs << 3); // MicBias, ADC, PLL bits
    writeRegister(ADCX140_PWR_CFG, pwrCfg);

    _powerConfig.PoweredUp = true;
    return SUCCESS;
}

void TLV320ADC::AutoCFG()
{
    // ASI_CFG0: I2S_MODE, BITS_32 by default
    uint8_t asiCfg0 = 0x30;
    asiCfg0 = (_asiConfig.format << 6) | (_asiConfig.wordLength << 4);
    writeRegister(ADCX140_ASI_CFG0, asiCfg0);

    // ASI_CFG1: MSB Slot 0 offset, 0 by default
    uint8_t asiCfg1 = 0x00;
    asiCfg1 = (_asiConfig.offset << 0);
    writeRegister(ADCX140_ASI_CFG1, asiCfg1);

    // ASI_CFG2: MSB Slot 1 offset, 0 by default
    uint8_t asiCfg2 = 0x00;
    asiCfg2 = (_asiConfig.DisableASIErrorDetection << 5) | (_asiConfig.DisableASIAutoResume << 4);
    writeRegister(ADCX140_ASI_CFG2, asiCfg2);

    uint8_t asiMixCfg = 0x00;
    asiMixCfg = (_asiConfig.mixSelection << 6) | (_asiConfig.inputGain << 4);
    writeRegister(ADCX140_ASI_MIX_CFG, asiMixCfg);

    // Master Mode Congifuration 
    uint8_t masterCfg0 = 0x02;
    masterCfg0 = (_asiConfig.MasterMode << 7);
    writeRegister(ADCX140_MST_CFG1, masterCfg0);
}