#ifndef TLV320ADC_h
#define TLV320ADC_h

#include <Arduino.h>
#include <Wire.h>
#include "TLV320ADC_REG.h"
#include "TLV320ADC_ENUM.h"

#ifdef I2C_PIN_ENABLED
#define TLV320ADC_I2C_ADDR (0x4c)
#else
#define TLV320ADC_I2C_ADDR (0x4e)
#endif

#define MAX_INPUT_GAIN 42
#define MAX_INPUT_VOLUME 255

struct SleepConfig
{
    bool awake = false;
    bool shutdown = false;
    Areg_Supply_Voltage AR_Supply = Internal_1_8V;
    VrefQuickCharge Vref = VREF_QCHG_3_5MS;
    bool I2C_Address_Swap_Enabled = false;
};

struct InputChannelCFG
{
    InputImpedance impedance = IMP_2_5K;
    InputCoupling coupling = AC_COUPLED;
    InputSource source = ANALOG_DIFFERENTIAL;
    InputType type = MIC_INPUT;
    bool AGC_enabled = false;
    uint8_t gain_db = 42;
    InputGainSign gain_sign = POSITIVE;
    uint8_t DigitalVolume = 255;
    bool enabled = false;
};

struct ASIConfig
{
    ASIFormat format = I2S_MODE;
    bool MasterMode = false;
    ASIWordLength wordLength = BITS_32;
    uint8_t offset = 0;
    ASIMixSelection mixSelection = NO_MIXING;
    ASIInputGain inputGain = GAIN_0DB;
    MicBiasValue micBias = VREF;
    bool DisableASIErrorDetection = false;
    bool DisableASIAutoResume = false;
};

struct PowerConfig
{
    bool MicBias = true;
    bool ADC = true;
    bool PLL = true;
    bool DynamicCHPwr = false;
    uint8_t DynamicCHs = CH1_CH2_ONLY;
    bool PoweredUp = false;
};

struct OutputChannelCFG
{
    ChannelSlot slot = L_SLOT_0;
    bool enabled = false;
};

class TLV320ADC
{
public:
    TLV320ADC();
    bool begin(uint8_t address = TLV320ADC_I2C_ADDR, TwoWire &wirePort = Wire);

    // Basic Device Control
    void sleep(bool enable);
    void setPage(uint8_t page);
    void reset();

    // Channel configuration methods
    void setInputType(uint8_t channel, InputType type);
    void setInputSource(uint8_t channel, InputSource source);
    void setInputCoupling(uint8_t channel, InputCoupling coupling);
    void setInputImpedance(uint8_t channel, InputImpedance impedance);
    void enableAGC(uint8_t channel, bool enable);

    // Gain control methods
    void setChannelGain(uint8_t channel, int8_t gain, bool isNegative = false);
    void setDigitalVolume(uint8_t channel, float volume);
    void setGainCalibration(uint8_t channel, float calibration);
    void setPhaseCalibration(uint8_t channel, uint8_t delay);

    // Power configuration
    void setAnalogSupply(bool useInternalRegulator);
    void setVrefQuickCharge(VrefQuickCharge duration);
    void setInputCapQuickCharge(InputCapQuickCharge duration);
    void enableI2CBroadcast(bool enable);

    // ASI Configuration methods
    void setASIFormat(ASIFormat format);
    void setWordLength(ASIWordLength length);
    void setFSYNCPolarity(bool invert);
    void setBCLKPolarity(bool invert);
    void setTXEdge(bool invert);
    void setTXFill(bool hiZ);
    void setTXLSB(bool halfCycle);
    void setTXKeeper(uint8_t keeperMode);
    void setTXOffset(uint8_t offset);
    void setDaisyChain(bool enable);
    void setErrorDetection(bool enable);
    void setErrorAutoResume(bool enable);
    void setChannelSlot(uint8_t channel, uint8_t slot);

    // ASI Mixing Configuration
    void setASIMixing(ASIMixSelection mixMode);
    void setASIInputGain(ASIInputGain gain);
    void setASIInputInversion(bool invert);

    // Bias Configuration
    void setMicBias(MicBiasValue biasValue);
    void setADCFullScale(ADCFullScale scale);

    // DSP Configuration
    void setDigitalVolumeRuntime(bool enable);
    void setDecimationFilter(DecimationFilter filter);
    void setChannelSummation(ChannelSum mode);
    void setHighPassFilter(HPFSelect filter);
    void setVolumeGanging(bool enable);
    void setBiquadConfig(BiquadConfig config);
    void setSoftStepping(bool enable);
    void setAGC(bool enable);
    void setAntiClipper(bool enable);

    // Channel Disable methods
    void DisableInputChannel(uint8_t channel, bool enable);
    void DisableOutputChannel(uint8_t channel, bool enable);

    // Power configuration methods
    void enableMicBias(bool enable);
    void enableADC(bool enable);
    void enablePLL(bool enable);
    void enableDynamicChannelPower(bool enable);
    void setDynamicMaxChannels(DynamicMaxChannel config);
    void enableVAD(bool enable);

    // Status methods
    bool isChannel1PoweredUp();
    bool isChannel2PoweredUp();
    DeviceMode getDeviceMode();

    // High level Congiguration methods
    int SleepCFG();
    int EnableInputChannel(uint8_t channel);
    int EnableOutputChannel(uint8_t channel);
    void AutoCFG();
    int PowerUp();

private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void setBit(uint8_t reg, uint8_t bit, bool value);
    uint8_t getChannelBaseRegister(uint8_t channel);
    uint8_t _currentPage;
    bool _initialized;
    void configureASI0Register(uint8_t value);
    void configureASI1Register(uint8_t value);
    void configureASI2Register(uint8_t value);
    void configureASIMixRegister(uint8_t value);
    uint8_t getASIChannelRegister(uint8_t channel);
    SleepConfig _sleepConfig;            // Store sleep configuration
    InputChannelCFG _inputChannels[4];   // Array to store channel configs
    OutputChannelCFG _outputChannels[4]; // Array to store output configs
    PowerConfig _powerConfig;            // Store power configuration
    ASIConfig _asiConfig;                // Store ASI configuration
};

#endif

