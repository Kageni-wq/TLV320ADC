#ifndef _TLV320ADC_ENUM_H_
#define _TLV320ADC_ENUM_H_

enum Channel {
    CHANNEL_1 = 0,
    CHANNEL_2 = 1,
    CHANNEL_3 = 2,
    CHANNEL_4 = 3
};

// Error codes 
enum ErrorCode {
    SUCCESS = 0,
    INVALID_CHANNEL = -1,
    INVALID_GAIN = -2,
    INVALID_VOLUME = -3,
    ALREADY_AWAKE = -4,
    ALREADY_POWERED_UP = -5,
    MUST_BE_INITIALIZED = -6,
    MUST_BE_AWAKE = -7,
    MUST_BE_POWERED_DOWN = -8,
    ALREADY_ENABLED = -9,
};

// The analog supply selection from either the internal regulator supply or the external AREG supply. Use Internal 1.8V when AVDD is 3.3V, and use External 1.8V when AVDD is 1.8V and short AREG with AVDD.
enum Areg_Supply_Voltage {
   External_1_8V = 0,
   Internal_1_8V = 1,
};

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

// Input Channel Gain Sign
enum InputGainSign
{
    POSITIVE = 0,
    NEGATIVE = 1
};

// VREF Quick Charge Duration
enum VrefQuickCharge {
    VREF_QCHG_3_5MS = 0,
    VREF_QCHG_10MS = 1,
    VREF_QCHG_50MS = 2,
    VREF_QCHG_100MS = 3
};

// Input Capacitor Quick Charge Duration
enum InputCapQuickCharge {
    INCAP_QCHG_2_5MS = 0,
    INCAP_QCHG_12_5MS = 1,
    INCAP_QCHG_25MS = 2,
    INCAP_QCHG_50MS = 3
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

enum ChannelSlot
{
    L_SLOT_0 = 0,
    L_SLOT_1 = 1,
    L_SLOT_2 = 2,
    L_SLOT_3 = 3,
    R_SLOT_0 = 32,
    R_SLOT_1 = 33,
    R_SLOT_2 = 34,
    R_SLOT_3 = 35
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

// Sample rate appllicable in Master mode only 
enum MasterSampleRate {
    RATE_8KHZ = 0,
    RATE_16KHZ = 1,
    RATE_24KHZ = 2,
    RATE_32KHZ = 3,
    RATE_48KHZ = 4,
    RATE_96KHZ = 5,
    RATE_192KHZ = 6,
    RATE_384KHZ = 7,
    RATE_768KHZ = 8
};

// BCLK to FSYNC ratio  applicable in Master mode only
enum BCLKRatio {
    BCLK_16 = 0,
    BCLK_24 = 1,
    BCLK_32 = 2,
    BCLK_48 = 3,
    BCLK_64 = 4,
    BCLK_96 = 5,
    BCLK_128 = 6,
    BCLK_192 = 7,
    BCLK_256 = 8,
    BCLK_384 = 9,
    BCLK_512 = 10,
    BCLK_1024 = 11,
    BCLK_2048 = 12
};


#endif // _TLV320ADC_ENUM_H_