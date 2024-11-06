#ifndef TLV320ADC_h
#define TLV320ADC_h

#include <Arduino.h>
#include <Wire.h>

#ifdef I2C_PIN_ENABLED
#define TLV320ADC_I2C_ADDR (0x4c)
#else
#define TLV320ADC_I2C_ADDR (0x9c)
#endif

// Input type options
enum InputType
{
    MIC_INPUT = 0,
    LINE_INPUT = 1
};

// Input source configuration
enum InputSource
{
    ANALOG_DIFFERENTIAL = 0,
    ANALOG_SINGLE_ENDED = 1,
    DIGITAL_PDM = 2
};

// Input coupling
enum InputCoupling
{
    AC_COUPLED = 0,
    DC_COUPLED = 1
};

// Input impedance
enum InputImpedance
{
    IMP_2_5K = 0,
    IMP_10K = 1,
    IMP_20K = 2
};

enum VrefQuickCharge
{
    VREFCHG_3_MS = 0,
    VREFCHG_10_MS = 1,
    VREFCHG_50_MS = 2,
    VREFCHG_100_MS = 3
};

// ASI Format options
enum ASIFormat
{
    TDM_MODE = 0,
    I2S_MODE = 1,
    LEFT_JUSTIFIED = 2
};

// Word length options
enum ASIWordLength
{
    BITS_16 = 0,
    BITS_20 = 1,
    BITS_24 = 2,
    BITS_32 = 3
};

// ASI Mix Selection
enum ASIMixSelection
{
    NO_MIXING = 0,
    MIX_CH1_CH2_TO_SLOT0 = 1,
    MIX_CH1_CH2_TO_SLOT1 = 2,
    MIX_BOTH_CHANNELS = 3
};

// ASI Input Gain
enum ASIInputGain
{
    GAIN_0DB = 0,
    GAIN_NEG_6DB = 1,
    GAIN_NEG_12DB = 2,
    GAIN_NEG_18DB = 3
};

// Microphone bias value options
enum MicBiasValue
{
    VREF = 0,
    VREF_X_1_096 = 1,
    VCM_IN1M = 2,
    VCM_IN2M = 3,
    VCM_IN1M_IN2M_AVG = 4,
    VCM_INTERNAL = 5,
    AVDD = 6,
    GPI2 = 7
};

// ADC full-scale settings
enum ADCFullScale
{
    VREF_2_75V = 0, // 2VRMS differential, 1VRMS single-ended
    VREF_2_5V = 1,  // 1.818VRMS differential, 0.909VRMS single-ended
    VREF_1_375V = 2 // 1VRMS differential, 0.5VRMS single-ended
};

// DSP Configuration enums
enum DecimationFilter
{
    LINEAR_PHASE = 0,
    LOW_LATENCY = 1,
    ULTRA_LOW_LATENCY = 2
};

enum ChannelSum
{
    SUM_DISABLED = 0,
    SUM_CH1_CH2 = 1
};

enum HPFSelect
{
    CUSTOM_IIR = 0,
    HPF_12HZ = 1,
    HPF_96HZ = 2,
    HPF_384HZ = 3
};

enum BiquadConfig
{
    NO_BIQUADS = 0,
    ONE_BIQUAD = 1,
    TWO_BIQUADS = 2,
    THREE_BIQUADS = 3
};

// Dynamic max channel selection
enum DynamicMaxChannel
{
    CH1_CH2_ONLY = 0,
    CH1_TO_CH4 = 1
};

// Device mode status values
enum DeviceMode
{
    SLEEP_OR_SHUTDOWN = 4,
    ACTIVE_ALL_CHANNELS_OFF = 6,
    ACTIVE_CHANNELS_ON = 7
};

class TLV320ADC
{
public:
    TLV320ADC();
    bool begin(uint8_t address = TLV320ADC_I2C_ADDR, TwoWire &wirePort = Wire);

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

    // Channel enable methods
    void enableInputChannel(uint8_t channel, bool enable);
    void enableASIOutputChannel(uint8_t channel, bool enable);

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
};

#endif
