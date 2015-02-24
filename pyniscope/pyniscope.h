/*
	Copyright (c) 2014 Vincent Paeder
	
	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/*

 Python wrapper for NI-SCOPE driver based on boost-python.

 This wrapper translates the NI-SCOPE C API to python.

*/

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/list.hpp>
using namespace boost::python;
#include <iostream>

#include "niscopew.h"

class pyniscope {
private:
	niscopew scope;
	char* ToChar(std::string strng);
	list WfmInfoExtract(struct niScope_wfmInfo wfmInfo);

public:
	pyniscope();
	//pyniscope(std::string resourceName, bool IDQuery, bool resetDevice);
	//pyniscope(std::string resourceName, bool IDQuery, bool resetDevice, std::string optionString);
	~pyniscope();

	bool Initialize(std::string resourceName, bool IDQuery, bool resetDevice);
	//bool Initialize(std::string resourceName, bool IDQuery, bool resetDevice, std::string optionString);
	bool Close();

	bool AutoSetup();
	bool ConfigureAcquisition(int acquisitionType);
	bool ConfigureHorizontalTiming(double minSampleRate, int minNumPts, double refPosition, int numRecords, bool enforceRealtime);
	bool ConfigureChanCharacteristics(std::string channelList, double inputImpedance, double maxInputFrequency);
	bool ConfigureVertical(std::string channelList, double range, double offset, int coupling, double probeAttenuation, bool enabled);
	int ActualMeasWfmSize(int arrayMeasFunction);
	int ActualNumWfms(std::string channelList);
	int ActualRecordLength();
	int SampleMode();
	double SampleRate();
	bool ConfigureTriggerDigital(std::string triggerSource, int slope, double holdoff, double delay);
	bool ConfigureTriggerEdge(std::string triggerSource, double level, int slope, int triggerCoupling, double holdoff, double delay);
	bool ConfigureTriggerVideo(std::string triggerSource, bool enableDCRestore, int signalFormat, int evt, int lineNumber, int polarity, int triggerCoupling, double holdoff, double delay);
	bool ConfigureTriggerHysteresis(std::string triggerSource, double level, double hysteresis, int slope, int triggerCoupling, double holdoff, double delay);
	bool ConfigureTriggerImmediate();
	bool ConfigureTriggerSoftware(double holdoff, double delay);
	bool ConfigureTriggerWindow(std::string triggerSource, double lowLevel, double highLevel, int windowMode, int triggerCoupling, double holdoff, double delay);
	bool SendSoftwareTriggerEdge(int whichTrigger);
	bool AdjustSampleClockRelativeDelay(double delay);
	bool ConfigureClock(std::string inputClockSource, std::string outputClockSource, std::string clockSyncPulseSource, bool masterEnabled);
	bool ExportSignal(int signal, std::string signalIdentifier, std::string outputTerminal);
	list GetFrequencyResponse(std::string channelName);
	bool ConfigureEqualizationFilterCoefficients(std::string channel, list coefficients);

	bool SetAttributeViInt32(std::string channelList, int attributeID, int value);
	bool SetAttributeViInt64(std::string channelList, int attributeID, int value);
	bool SetAttributeViReal64(std::string channelList, int attributeID, double value);
	bool SetAttributeViString(std::string channelList, int attributeID, std::string value);
	bool SetAttributeViBoolean(std::string channelList, int attributeID, bool value);
	bool SetAttributeViSession(std::string channelList, int attributeID, int value);
	int GetAttributeViInt32(std::string channelList, int attributeID);
	int GetAttributeViInt64(std::string channelList, int attributeID);
	double GetAttributeViReal64(std::string channelList, int attributeID);
	std::string GetAttributeViString(std::string channelList, int attributeID);
	bool GetAttributeViBoolean(std::string channelList, int attributeID);
	int GetAttributeViSession(std::string channelList, int attributeID);
	bool CheckAttributeViInt32(std::string channelList, int attributeID, int value);
	bool CheckAttributeViInt64(std::string channelList, int attributeID, int value);
	bool CheckAttributeViReal64(std::string channelList, int attributeID, double value);
	bool CheckAttributeViString(std::string channelList, int attributeID, std::string value);
	bool CheckAttributeViBoolean(std::string channelList, int attributeID, bool value);
	bool CheckAttributeViSession(std::string channelList, int attributeID, int value);

	bool Abort();
	int AcquisitionStatus();
	bool Commit();
	list Fetch(std::string channelList, double timeout, int numSamples);
	list FetchComplex(std::string channelList, double timeout, int numSamples);
	list FetchComplexBinary16(std::string channelList, double timeout, int numSamples);
	list FetchBinary8(std::string channelList, double timeout, int numSamples);
	list FetchBinary16(std::string channelList, double timeout, int numSamples);
	list FetchBinary32(std::string channelList, double timeout, int numSamples);
	list GetNormalizationCoefficients(std::string channelList);
	list GetScalingCoefficients(std::string channelList);
	bool InitiateAcquisition();
	list Read(std::string channelList, double timeout, int numSamples);

	bool AddWaveformProcessing(std::string channelList, int measFunction);
	bool ClearWaveformMeasurementStats(std::string channelList, int clearableMeasurementFunction);
	bool ClearWaveformProcessing(std::string channelList);
	list FetchArrayMeasurement(std::string channelList, double timeout, int arrayMeasFunction);
	list FetchMeasurement(std::string channelList, double timeout, int scalarMeasFunction);
	list FetchMeasurementStats(std::string channelList, double timeout, int scalarMeasFunction);
	list ReadMeasurement(std::string channelList, double timeout, int scalarMeasFunction);

	bool CalSelfCalibrate(std::string channelList, int option);

	bool ResetDevice();
	bool Disable();
	bool ProbeCompensationSignalStart();
	bool ProbeCompensationSignalStop();
	bool IsDeviceReady(std::string resourceName, std::string channelList);
	bool Reset();
	bool ResetWithDefaults();
	list RevisionQuery();
	list SelfTest();
	int GetStreamEndpointHandle(std::string streamName);
	
	list ErrorHandler(int errorCode);
	std::string GetError(int errorCode);
	std::string GetErrorMessage(int errorCode);

	bool LockSession();
	bool UnlockSession();

	bool ClearError();
	bool ClearInterchangeWarnings();
	bool ConfigureAcquisitionRecord(double timeperRecord, int minNumPoints, double acquisitionStartTime);
	bool ConfigureChannel(std::string channel, double range, double offset, int coupling, double probeAttenuation, bool enabled);
	bool ConfigureEdgeTriggerSource(std::string source, double level, int slope);
	bool ConfigureRefLevels(double low, double mid, double high);
	bool ConfigureTrigger(int triggerType, double holdoff);
	bool ConfigureTriggerCoupling(int coupling);
	bool ConfigureTriggerOutput(int triggerEvent, std::string triggerOutput);
	bool ConfigureTVTriggerLineNumber(int lineNumber);
	bool ConfigureTVTriggerSource(std::string source, int signalFormat, int evt, int polarity);
	list ErrorQuery();
	list FetchWaveform(std::string channel, int waveformSize);
	double FetchWaveformMeasurement(std::string channel, int measFunction);
	std::string GetChannelName(int index);
	std::string GetNextCoercionRecord();
	std::string GetNextInterchangeWarning();
	bool IsInvalidWfmElement(double elementValue);
	list ReadWaveform(std::string channel, int waveformSize, int maxtime);
	double ReadWaveformMeasurement(std::string channel, int measFunction, int maxTime);
	bool ResetInterchangeCheck();
	bool SendSWTrigger();

};

typedef enum {
	AcquisitionType = NISCOPE_ATTR_ACQUISITION_TYPE,
	SampleMode = NISCOPE_ATTR_SAMPLE_MODE,
	BinarySampleWidth = NISCOPE_ATTR_BINARY_SAMPLE_WIDTH,
	Resolution = NISCOPE_ATTR_RESOLUTION,
	FetchRelativeTo = NISCOPE_ATTR_FETCH_RELATIVE_TO,
	FetchOffset = NISCOPE_ATTR_FETCH_OFFSET,
	FetchRecordNumber = NISCOPE_ATTR_FETCH_RECORD_NUMBER,
	FetchNumRecords = NISCOPE_ATTR_FETCH_NUM_RECORDS,
	FetchMeasNumSamples = NISCOPE_ATTR_FETCH_MEAS_NUM_SAMPLES,
	PointsDone = NISCOPE_ATTR_POINTS_DONE,
	RecordsDone = NISCOPE_ATTR_RECORDS_DONE,
	TransferBlockSize = NISCOPE_ATTR_DATA_TRANSFER_BLOCK_SIZE,
	Backlog = NISCOPE_ATTR_BACKLOG,
	RisInAutoSetupEnable = NISCOPE_ATTR_RIS_IN_AUTO_SETUP_ENABLE
} niScope_Acquisition;

typedef enum {
	ChannelEnabled = NISCOPE_ATTR_CHANNEL_ENABLED,
	ChannelTerminalConfiguration = NISCOPE_ATTR_CHANNEL_TERMINAL_CONFIGURATION,
	ProbeAttenuation = NISCOPE_ATTR_PROBE_ATTENUATION,
	Range = NISCOPE_ATTR_VERTICAL_RANGE,
	Offset = NISCOPE_ATTR_VERTICAL_OFFSET,
	InputImpedance = NISCOPE_ATTR_INPUT_IMPEDANCE,
	Coupling = NISCOPE_ATTR_VERTICAL_COUPLING,
	MaxInputFrequency = NISCOPE_ATTR_MAX_INPUT_FREQUENCY,
	FlexFirAntialiasFilterType = NISCOPE_ATTR_FLEX_FIR_ANTIALIAS_FILTER_TYPE,
	DigitalGain = NISCOPE_ATTR_DIGITAL_GAIN,
	DigitalOffset = NISCOPE_ATTR_DIGITAL_OFFSET,
	BandpassFilterEnabled = NISCOPE_ATTR_BANDPASS_FILTER_ENABLED,
	DitherEnabled = NISCOPE_ATTR_DITHER_ENABLED
} niScope_Vertical;

typedef enum {
	DdcCenterFrequency = NISCOPE_ATTR_DDC_CENTER_FREQUENCY,
	DdcDataProcessingMode = NISCOPE_ATTR_DDC_DATA_PROCESSING_MODE,
	DdcEnabled = NISCOPE_ATTR_DDC_ENABLED,
	FetchInterleavedIQData = NISCOPE_ATTR_FETCH_INTERLEAVED_IQ_DATA,
	//FrequencyTranslationEnabled = NISCOPE_ATTR_FREQUENCY_TRANSLATION_ENABLED,
	DdcQSource = NISCOPE_ATTR_DDC_Q_SOURCE,
	DdcFrequencyTranslationPhaseI = NISCOPE_ATTR_DDC_FREQUENCY_TRANSLATION_PHASE_I,
	DdcFrequencyTranslationPhaseQ = NISCOPE_ATTR_DDC_FREQUENCY_TRANSLATION_PHASE_Q,
	EqualizationFilterEnabled = NISCOPE_ATTR_EQUALIZATION_FILTER_ENABLED,
	EqualizationNumCoefficients = NISCOPE_ATTR_EQUALIZATION_NUM_COEFFICIENTS,
	FractionalResampleEnabled = NISCOPE_ATTR_FRACTIONAL_RESAMPLE_ENABLED
} niScope_OnboardSignalProcessing;

typedef enum {
	Enabled = NISCOPE_ATTR_P2P_ENABLED,
	ChannelsToStream = NISCOPE_ATTR_P2P_CHANNELS_TO_STREAM,
	EndpointSize = NISCOPE_ATTR_P2P_ENDPOINT_SIZE,
	SamplesAvailInEndpoint = NISCOPE_ATTR_P2P_SAMPLES_AVAIL_IN_ENDPOINT,
	MostSamplesAvailInEndpoint = NISCOPE_ATTR_P2P_MOST_SAMPLES_AVAIL_IN_ENDPOINT,
	SamplesTransferred = NISCOPE_ATTR_P2P_SAMPLES_TRANSFERRED,
	EndpointOverflow = NISCOPE_ATTR_P2P_ENDPOINT_OVERFLOW,
	FifoEndpointCount = NISCOPE_ATTR_P2P_FIFO_ENDPOINT_COUNT,
	OnboardMemoryEnabled = NISCOPE_ATTR_P2P_ONBOARD_MEMORY_ENABLED
	} niScope_P2P;

typedef enum {
	AcquisitionStartTime = NISCOPE_ATTR_ACQUISITION_START_TIME,
	EnableTimeInterleavedSampling = NISCOPE_ATTR_ENABLE_TIME_INTERLEAVED_SAMPLING,
	HorzNumRecords = NISCOPE_ATTR_HORZ_NUM_RECORDS,
	HorzTimePerRecord = NISCOPE_ATTR_HORZ_TIME_PER_RECORD,
	HorzMinNumPts = NISCOPE_ATTR_HORZ_MIN_NUM_PTS,
	HorzRecordLength = NISCOPE_ATTR_HORZ_RECORD_LENGTH,
	HorzRecordRefPosition = NISCOPE_ATTR_HORZ_RECORD_REF_POSITION,
	MinSampleRate = NISCOPE_ATTR_MIN_SAMPLE_RATE,
	HorzSampleRate = NISCOPE_ATTR_HORZ_SAMPLE_RATE,
	HorzEnforceRealtime = NISCOPE_ATTR_HORZ_ENFORCE_REALTIME,
	RefTrigTdcEnable = NISCOPE_ATTR_REF_TRIG_TDC_ENABLE,
	AllowMoreRecordsThanMemory = NISCOPE_ATTR_ALLOW_MORE_RECORDS_THAN_MEMORY,
	PollInterval = NISCOPE_ATTR_POLL_INTERVAL,
	RisNumAverages = NISCOPE_ATTR_RIS_NUM_AVERAGES,
	RisMethod = NISCOPE_ATTR_RIS_METHOD
} nisScope_Horizontal;

typedef enum {
	TriggerAutoTriggered = NISCOPE_ATTR_TRIGGER_AUTO_TRIGGERED,
	RefTriggerMinimumQuietTime = NISCOPE_ATTR_REF_TRIGGER_MINIMUM_QUIET_TIME,
	RefTriggerDetectorLocation = NISCOPE_ATTR_REF_TRIGGER_DETECTOR_LOCATION,
	StartToRefTriggerHoldoff = NISCOPE_ATTR_START_TO_REF_TRIGGER_HOLDOFF,
	TriggerDelayTime = NISCOPE_ATTR_TRIGGER_DELAY_TIME,
	TriggerCoupling = NISCOPE_ATTR_TRIGGER_COUPLING,
	TriggerHoldoff = NISCOPE_ATTR_TRIGGER_HOLDOFF,
	TriggerHysteresis = NISCOPE_ATTR_TRIGGER_HYSTERESIS,
	TriggerImpedance = NISCOPE_ATTR_TRIGGER_IMPEDANCE,
	TriggerLevel = NISCOPE_ATTR_TRIGGER_LEVEL,
	TriggerModifier = NISCOPE_ATTR_TRIGGER_MODIFIER,
	ExportedRefTriggerOutputTerminal = NISCOPE_ATTR_EXPORTED_REF_TRIGGER_OUTPUT_TERMINAL,
	TriggerSlope = NISCOPE_ATTR_TRIGGER_SLOPE,
	TriggerSource = NISCOPE_ATTR_TRIGGER_SOURCE,
	TriggerType = NISCOPE_ATTR_TRIGGER_TYPE,
	TvTriggerSignalFormat = NISCOPE_ATTR_TV_TRIGGER_SIGNAL_FORMAT,
	TvTriggerLineNumber = NISCOPE_ATTR_TV_TRIGGER_LINE_NUMBER,
	TvTriggerPolarity = NISCOPE_ATTR_TV_TRIGGER_POLARITY,
	TvTriggerEvent = NISCOPE_ATTR_TV_TRIGGER_EVENT,
	EnableDcRestore = NISCOPE_ATTR_ENABLE_DC_RESTORE,
	TriggerWindowLowLevel = NISCOPE_ATTR_TRIGGER_WINDOW_LOW_LEVEL,
	TriggerWindowHighLevel = NISCOPE_ATTR_TRIGGER_WINDOW_HIGH_LEVEL,
	TriggerWindowMode = NISCOPE_ATTR_TRIGGER_WINDOW_MODE
} niScope_Triggering;

typedef enum {
	DeviceTemperature = NISCOPE_ATTR_DEVICE_TEMPERATURE,
	SerialNumber = NISCOPE_ATTR_SERIAL_NUMBER,
//	DaqMXMask = NISCOPE_ATTR_DAQMX_MASK,
	SignalCondGain = NISCOPE_ATTR_SIGNAL_COND_GAIN,
	SignalCondOffset = NISCOPE_ATTR_SIGNAL_COND_OFFSET
} niScope_Device;

typedef enum {
//	ExportedSampleClockOutputTerm = NISCOPE_ATTR_EXPORTED_SAMPLE_CLOCK_OUTPUT_TERM,
	InputClockSource = NISCOPE_ATTR_INPUT_CLOCK_SOURCE,
	OutputClockSource = NISCOPE_ATTR_OUTPUT_CLOCK_SOURCE,
	ClockSyncPulseSource = NISCOPE_ATTR_CLOCK_SYNC_PULSE_SOURCE,
	SampClkTimebaseSrc = NISCOPE_ATTR_SAMP_CLK_TIMEBASE_SRC,
	SampClkTimebaseRate = NISCOPE_ATTR_SAMP_CLK_TIMEBASE_RATE,
	SampClkTimebaseDiv = NISCOPE_ATTR_SAMP_CLK_TIMEBASE_DIV,
	SampClkTimebaseMult = NISCOPE_ATTR_SAMP_CLK_TIMEBASE_MULT,
	RefClkRate = NISCOPE_ATTR_REF_CLK_RATE,
	ExportedSampleClockOutputTerminal = NISCOPE_ATTR_EXPORTED_SAMPLE_CLOCK_OUTPUT_TERMINAL,
	PllLockStatus = NISCOPE_ATTR_PLL_LOCK_STATUS
} niScope_Clocking;

typedef enum {
	FiveVOutOutputTerminal = NISCOPE_ATTR_5V_OUT_OUTPUT_TERMINAL,
	MasterEnable = NISCOPE_ATTR_MASTER_ENABLE,
	AcqArmSource = NISCOPE_ATTR_ACQ_ARM_SOURCE,
	RecordArmSource = NISCOPE_ATTR_RECORD_ARM_SOURCE,
	EndOfAcquisitionEventOutputTerminal = NISCOPE_ATTR_END_OF_ACQUISITION_EVENT_OUTPUT_TERMINAL,
	ExportedStartTriggerOutputTerminal = NISCOPE_ATTR_EXPORTED_START_TRIGGER_OUTPUT_TERMINAL,
	ArmRefTrigSrc = NISCOPE_ATTR_ARM_REF_TRIG_SRC,
	ExportedAdvanceTriggerOutputTerminal = NISCOPE_ATTR_EXPORTED_ADVANCE_TRIGGER_OUTPUT_TERMINAL,
	AdvTrigSrc = NISCOPE_ATTR_ADV_TRIG_SRC,
	EndOfRecordEventOutputTerminal = NISCOPE_ATTR_END_OF_RECORD_EVENT_OUTPUT_TERMINAL,
	ReadyForAdvanceEventOutputTerminal = NISCOPE_ATTR_READY_FOR_ADVANCE_EVENT_OUTPUT_TERMINAL,
	ReadyForStartEventOutputTerminal = NISCOPE_ATTR_READY_FOR_START_EVENT_OUTPUT_TERMINAL,
	RedayForRefEventOutputTerminal = NISCOPE_ATTR_READY_FOR_REF_EVENT_OUTPUT_TERMINAL,
	SlaveTriggerDelay = NISCOPE_ATTR_SLAVE_TRIGGER_DELAY,
	TriggerToStarDelay = NISCOPE_ATTR_TRIGGER_TO_STAR_DELAY,
	TriggerToRtsiDelay = NISCOPE_ATTR_TRIGGER_TO_RTSI_DELAY,
	TriggerToPfiDelay = NISCOPE_ATTR_TRIGGER_TO_PFI_DELAY,
	TriggerFromStarDelay = NISCOPE_ATTR_TRIGGER_FROM_STAR_DELAY,
	TriggerFromRtsiDelay = NISCOPE_ATTR_TRIGGER_FROM_RTSI_DELAY,
	TriggerFromPfiDelay = NISCOPE_ATTR_TRIGGER_FROM_PFI_DELAY
} niScope_Synchronization;

typedef enum {
	MeasOtherChannel = NISCOPE_ATTR_MEAS_OTHER_CHANNEL,
	MeasHysteresisPercent = NISCOPE_ATTR_MEAS_HYSTERESIS_PERCENT,
//	LastAcqHistogramSize = NISCOPE_ATTR_LAST_ACQ_HISTOGRAM_SIZE,
	MeasArrayGain = NISCOPE_ATTR_MEAS_ARRAY_GAIN,
	MeasArrayOffset = NISCOPE_ATTR_MEAS_ARRAY_OFFSET,
	MeasChanLowRefLevel = NISCOPE_ATTR_MEAS_CHAN_LOW_REF_LEVEL,
	MeasChanMidRefLevel = NISCOPE_ATTR_MEAS_CHAN_MID_REF_LEVEL,
	MeasChanHighRefLevel = NISCOPE_ATTR_MEAS_CHAN_HIGH_REF_LEVEL,
	MeasPolynomialInterpolationOrder = NISCOPE_ATTR_MEAS_POLYNOMIAL_INTERPOLATION_ORDER,
	MeasInterpolationSamplingFactor = NISCOPE_ATTR_MEAS_INTERPOLATION_SAMPLING_FACTOR,
	MeasVoltageHistogramSize = NISCOPE_ATTR_MEAS_VOLTAGE_HISTOGRAM_SIZE,
	MeasVoltageHistogramLowVolts = NISCOPE_ATTR_MEAS_VOLTAGE_HISTOGRAM_LOW_VOLTS,
	MeasVoltageHistogramHighVolts = NISCOPE_ATTR_MEAS_VOLTAGE_HISTOGRAM_HIGH_VOLTS,
	MeasTimeHistogramSize = NISCOPE_ATTR_MEAS_TIME_HISTOGRAM_SIZE,
	MeasTimeHistogramHighVolts = NISCOPE_ATTR_MEAS_TIME_HISTOGRAM_HIGH_VOLTS,
	MeasTimeHistogramLowTime = NISCOPE_ATTR_MEAS_TIME_HISTOGRAM_LOW_TIME,
	MeasTimeHistogramHighTime = NISCOPE_ATTR_MEAS_TIME_HISTOGRAM_HIGH_TIME,
	MeasTimeHistogramLowVolts = NISCOPE_ATTR_MEAS_TIME_HISTOGRAM_LOW_VOLTS,
	MeasFilterCutoffFreq = NISCOPE_ATTR_MEAS_FILTER_CUTOFF_FREQ,
	MeasFilterCenterFreq = NISCOPE_ATTR_MEAS_FILTER_CENTER_FREQ,
	MeasFilterWidth = NISCOPE_ATTR_MEAS_FILTER_WIDTH,
	MeasFilterRipple = NISCOPE_ATTR_MEAS_FILTER_RIPPLE,
	MeasFilterTransientWaveformPercent = NISCOPE_ATTR_MEAS_FILTER_TRANSIENT_WAVEFORM_PERCENT,
	MeasFilterType = NISCOPE_ATTR_MEAS_FILTER_TYPE,
	MeasFilterOrder = NISCOPE_ATTR_MEAS_FILTER_ORDER,
	MeasFilterTaps = NISCOPE_ATTR_MEAS_FILTER_TAPS,
	MeasFirFilterWindow = NISCOPE_ATTR_MEAS_FIR_FILTER_WINDOW
} niScope_WaveformMeasurements;

typedef enum {
	MaxRisRate = NISCOPE_ATTR_MAX_RIS_RATE,
	MaxRealTimeSamplingRate = NISCOPE_ATTR_MAX_REAL_TIME_SAMPLING_RATE,
	OnboardMemorySize = NISCOPE_ATTR_ONBOARD_MEMORY_SIZE
} niScope_InstrumentCapabilities;

typedef enum {
	Bandwidth = NISCOPE_ATTR_BANDWIDTH,
	Cache = NISCOPE_ATTR_CACHE,
	ChannelCount = NISCOPE_ATTR_CHANNEL_COUNT,
	DriverSetup = NISCOPE_ATTR_DRIVER_SETUP,
	GroupCapabilities = NISCOPE_ATTR_GROUP_CAPABILITIES,
	InstrumentFirmwareRevision = NISCOPE_ATTR_INSTRUMENT_FIRMWARE_REVISION,
	InstrumentManufacturer = NISCOPE_ATTR_INSTRUMENT_MANUFACTURER,
	InstrumentModel = NISCOPE_ATTR_INSTRUMENT_MODEL,
	InterchangeCheck = NISCOPE_ATTR_INTERCHANGE_CHECK,
	LogicalName = NISCOPE_ATTR_LOGICAL_NAME,
	MeasHighRef = NISCOPE_ATTR_MEAS_HIGH_REF,
	MeasLowRef = NISCOPE_ATTR_MEAS_LOW_REF,
	MeasMidRef = NISCOPE_ATTR_MEAS_MID_REF,
	MeasPercentageMethod = NISCOPE_ATTR_MEAS_PERCENTAGE_METHOD,
	MeasRefLevelUnits = NISCOPE_ATTR_MEAS_REF_LEVEL_UNITS,
	NumChannels = NISCOPE_ATTR_NUM_CHANNELS,
	QueryInstrumentStatus = NISCOPE_ATTR_QUERY_INSTRUMENT_STATUS,
	RangeCheck = NISCOPE_ATTR_RANGE_CHECK,
	RecordCoercions = NISCOPE_ATTR_RECORD_COERCIONS,
	ResourceDescriptor = NISCOPE_ATTR_RESOURCE_DESCRIPTOR,
	Simulate = NISCOPE_ATTR_SIMULATE,
	SpecificDriverClassSpecMajorVersion = NISCOPE_ATTR_SPECIFIC_DRIVER_CLASS_SPEC_MAJOR_VERSION,
	SpecificDriverClassSpecMinorVersion = NISCOPE_ATTR_SPECIFIC_DRIVER_CLASS_SPEC_MINOR_VERSION,
	SpecificDriverDescription = NISCOPE_ATTR_SPECIFIC_DRIVER_DESCRIPTION,
	SpecificDriverPrefix = NISCOPE_ATTR_SPECIFIC_DRIVER_PREFIX,
	SpecificDriverRevision = NISCOPE_ATTR_SPECIFIC_DRIVER_REVISION,
	SpecificDriverVendor = NISCOPE_ATTR_SPECIFIC_DRIVER_VENDOR,
	SupportedInstrumentModels = NISCOPE_ATTR_SUPPORTED_INSTRUMENT_MODELS
} niScope_IviAttribute;

// values enumerators
typedef enum {
	Edge = NISCOPE_VAL_EDGE_TRIGGER,
	Hysteresis = NISCOPE_VAL_HYSTERESIS_TRIGGER,
	Digital = NISCOPE_VAL_DIGITAL_TRIGGER,
	Window = NISCOPE_VAL_WINDOW_TRIGGER,
	Software = NISCOPE_VAL_SOFTWARE_TRIGGER,
	TV = NISCOPE_VAL_TV_TRIGGER,
	Immediate = NISCOPE_VAL_IMMEDIATE_TRIGGER
} niScope_TriggerType;

/* this doesn't work with non-integer enum values (here: char arrays)
typedef enum {
	Immediate = NISCOPE_VAL_IMMEDIATE,
	External = NISCOPE_VAL_EXTERNAL,
	SWTrig = NISCOPE_VAL_SW_TRIG_FUNC,
	TTL0 = NISCOPE_VAL_TTL0,
	TTL1 = NISCOPE_VAL_TTL1,
	TTL2 = NISCOPE_VAL_TTL2,
	TTL3 = NISCOPE_VAL_TTL3,
	TTL4 = NISCOPE_VAL_TTL4,
	TTL5 = NISCOPE_VAL_TTL5,
	TTL6 = NISCOPE_VAL_TTL6,
	TTL7 = NISCOPE_VAL_TTL7,
	ECL0 = NISCOPE_VAL_ECL0,
	ECL1 = NISCOPE_VAL_ECL1,
	PXIStar = NISCOPE_VAL_PXI_STAR,
	RTSI0 = NISCOPE_VAL_RTSI_0,
	RTSI1 = NISCOPE_VAL_RTSI_1,
	RTSI2 = NISCOPE_VAL_RTSI_2,
	RTSI3 = NISCOPE_VAL_RTSI_3,
	RTSI4 = NISCOPE_VAL_RTSI_4,
	RTSI5 = NISCOPE_VAL_RTSI_5,
	RTSI6 = NISCOPE_VAL_RTSI_6,
	RTSI7 = NISCOPE_VAL_RTSI_7,
	PFI0 = NISCOPE_VAL_PFI_0,
	PFI1 = NISCOPE_VAL_PFI_1,
	PFI2 = NISCOPE_VAL_PFI_2
} niScope_TriggerSource;
*/

typedef enum {
	ReadPointer = NISCOPE_VAL_READ_POINTER,
	Pretrigger = NISCOPE_VAL_PRETRIGGER,
	Now = NISCOPE_VAL_NOW,
	Start = NISCOPE_VAL_START,
	Trigger = NISCOPE_VAL_TRIGGER
} niScope_AttrRelativeToValues;

typedef enum {
	NoTriggerMod = NISCOPE_VAL_NO_TRIGGER_MOD,
	Auto = NISCOPE_VAL_AUTO,
	AutoLevel = NISCOPE_VAL_AUTO_LEVEL
} niScope_AttrTriggerModifierValues;

typedef enum {
	AC = NISCOPE_VAL_AC,
	DC = NISCOPE_VAL_DC,
	HFReject = NISCOPE_VAL_HF_REJECT,
	LFReject = NISCOPE_VAL_LF_REJECT,
	ACPlusHFReject = NISCOPE_VAL_AC_PLUS_HF_REJECT
} niScope_AttrTriggerCouplingValues;

typedef enum {
	Positive = NISCOPE_VAL_POSITIVE,
	Negative = NISCOPE_VAL_NEGATIVE
} niScope_AttrTriggerSlopeValues;

typedef enum {
	Normal = NISCOPE_VAL_NORMAL,
	Flexres = NISCOPE_VAL_FLEXRES,
	DDC = NISCOPE_VAL_DDC
} niScope_AttrAcquisitionTypeValues;

typedef enum {
	NoInterpolation = NISCOPE_VAL_NO_INTERPOLATION,
	SineX = NISCOPE_VAL_SINE_X,
	Linear = NISCOPE_VAL_LINEAR
} niScope_AttrInterpolationValues;

typedef enum {
	NoEvent = NISCOPE_VAL_NO_EVENT,
	StopTriggerEvent = NISCOPE_VAL_STOP_TRIGGER_EVENT,
	StartTriggerEvent = NISCOPE_VAL_START_TRIGGER_EVENT
} niScope_TriggerOutputEvent;

typedef enum {
	ExactNumAverages = NISCOPE_VAL_RIS_EXACT_NUM_AVERAGES,
	MinNumAverages = NISCOPE_VAL_RIS_MIN_NUM_AVERAGES,
	Incomplete = NISCOPE_VAL_RIS_INCOMPLETE,
	LimitedBinWidth = NISCOPE_VAL_RIS_LIMITED_BIN_WIDTH
} niScope_RISMethods;

typedef enum {
	TriggerStart = NISCOPE_VAL_SOFTWARE_TRIGGER_START,
	TriggerArmReference = NISCOPE_VAL_SOFTWARE_TRIGGER_ARM_REFERENCE,
	TriggerReference = NISCOPE_VAL_SOFTWARE_TRIGGER_REFERENCE,
	TriggerAdvance = NISCOPE_VAL_SOFTWARE_TRIGGER_ADVANCE
} niScope_SoftwareTriggerTypes;



BOOST_PYTHON_MODULE(pyniscope) {
	class_<pyniscope>("pyniscope", init<>())
		.def("Initialize", &pyniscope::Initialize)
		.def("Close", &pyniscope::Close)
		.def("AutoSetup", &pyniscope::AutoSetup)
		.def("ConfigureAcquisition", &pyniscope::ConfigureAcquisition)
		.def("ConfigureHorizontalTiming", &pyniscope::ConfigureHorizontalTiming)
		.def("ConfigureChanCharacteristics", &pyniscope::ConfigureChanCharacteristics)
		.def("ConfigureVertical", &pyniscope::ConfigureVertical)
		.def("ActualMeasWfmSize", &pyniscope::ActualMeasWfmSize)
		.def("ActualNumWfms", &pyniscope::ActualNumWfms)
		.def("ActualRecordLength", &pyniscope::ActualRecordLength)
		.def("SampleMode", &pyniscope::SampleMode)
		.def("SampleRate", &pyniscope::SampleRate)
		.def("ConfigureTriggerDigital", &pyniscope::ConfigureTriggerDigital)
		.def("ConfigureTriggerEdge", &pyniscope::ConfigureTriggerEdge)
		.def("ConfigureTriggerVideo", &pyniscope::ConfigureTriggerVideo)
		.def("ConfigureTriggerHysteresis", &pyniscope::ConfigureTriggerHysteresis)
		.def("ConfigureTriggerImmediate", &pyniscope::ConfigureTriggerImmediate)
		.def("ConfigureTriggerSoftware", &pyniscope::ConfigureTriggerSoftware)
		.def("ConfigureTriggerWindow", &pyniscope::ConfigureTriggerWindow)
		.def("SendSoftwareTriggerEdge", &pyniscope::SendSoftwareTriggerEdge)
		.def("AdjustSampleClockRelativeDelay", &pyniscope::AdjustSampleClockRelativeDelay)
		.def("ConfigureClock", &pyniscope::ConfigureClock)
		.def("ExportSignal", &pyniscope::ExportSignal)
		.def("GetFrequencyResponse", &pyniscope::GetFrequencyResponse)
		.def("ConfigureEqualizationFilterCoefficients", &pyniscope::ConfigureEqualizationFilterCoefficients)
		.def("SetAttributeViInt32", &pyniscope::SetAttributeViInt32)
		.def("SetAttributeViInt64", &pyniscope::SetAttributeViInt64)
		.def("SetAttributeViReal64", &pyniscope::SetAttributeViReal64)
		.def("SetAttributeViString", &pyniscope::SetAttributeViString)
		.def("SetAttributeViBoolean", &pyniscope::SetAttributeViBoolean)
		.def("SetAttributeViSession", &pyniscope::SetAttributeViSession)
		.def("GetAttributeViInt32", &pyniscope::GetAttributeViInt32)
		.def("GetAttributeViInt64", &pyniscope::GetAttributeViInt64)
		.def("GetAttributeViReal64", &pyniscope::GetAttributeViReal64)
		.def("GetAttributeViString", &pyniscope::GetAttributeViString)
		.def("GetAttributeViBoolean", &pyniscope::GetAttributeViBoolean)
		.def("GetAttributeViSession", &pyniscope::GetAttributeViSession)
		.def("CheckAttributeViInt32", &pyniscope::CheckAttributeViInt32)
		.def("CheckAttributeViInt64", &pyniscope::CheckAttributeViInt64)
		.def("CheckAttributeViReal64", &pyniscope::CheckAttributeViReal64)
		.def("CheckAttributeViString", &pyniscope::CheckAttributeViString)
		.def("CheckAttributeViBoolean", &pyniscope::CheckAttributeViBoolean)
		.def("CheckAttributeViSession", &pyniscope::CheckAttributeViSession)
		.def("Abort", &pyniscope::Abort)
		.def("AcquisitionStatus", &pyniscope::AcquisitionStatus)
		.def("Commit", &pyniscope::Commit)
		.def("Fetch", &pyniscope::Fetch)
		.def("FetchComplex", &pyniscope::FetchComplex)
		.def("FetchComplexBinary16", &pyniscope::FetchComplexBinary16)
		.def("FetchBinary8", &pyniscope::FetchBinary8)
		.def("FetchBinary16", &pyniscope::FetchBinary16)
		.def("FetchBinary32", &pyniscope::FetchBinary32)
		.def("GetNormalizationCoefficients", &pyniscope::GetNormalizationCoefficients)
		.def("GetScalingCoefficients", &pyniscope::GetScalingCoefficients)
		.def("InitiateAcquisition", &pyniscope::InitiateAcquisition)
		.def("Read", &pyniscope::Read)
		.def("AddWaveformProcessing", &pyniscope::AddWaveformProcessing)
		.def("ClearWaveformMeasurementStats", &pyniscope::ClearWaveformMeasurementStats)
		.def("ClearWaveformProcessing", &pyniscope::ClearWaveformProcessing)
		.def("FetchArrayMeasurement", &pyniscope::FetchArrayMeasurement)
		.def("FetchMeasurement", &pyniscope::FetchMeasurement)
		.def("FetchMeasurementStats", &pyniscope::FetchMeasurementStats)
		.def("ReadMeasurement", &pyniscope::ReadMeasurement)
		.def("CalSelfCalibrate", &pyniscope::CalSelfCalibrate)
		.def("ResetDevice", &pyniscope::ResetDevice)
		.def("Disable", &pyniscope::Disable)
		.def("ProbeCompensationSignalStart", &pyniscope::ProbeCompensationSignalStart)
		.def("ProbeCompensationSignalStop", &pyniscope::ProbeCompensationSignalStop)
		.def("IsDeviceReady", &pyniscope::IsDeviceReady)
		.def("Reset", &pyniscope::Reset)
		.def("ResetWithDefaults", &pyniscope::ResetWithDefaults)
		.def("RevisionQuery", &pyniscope::RevisionQuery)
		.def("SelfTest", &pyniscope::SelfTest)
		.def("GetStreamEndpointHandle", &pyniscope::GetStreamEndpointHandle)
		.def("ErrorHandler", &pyniscope::ErrorHandler)
		.def("GetError", &pyniscope::GetError)
		.def("GetErrorMessage", &pyniscope::GetErrorMessage)
		.def("LockSession", &pyniscope::LockSession)
		.def("UnlockSession", &pyniscope::UnlockSession)
		.def("ClearError", &pyniscope::ClearError)
		.def("ClearInterchangeWarnings", &pyniscope::ClearInterchangeWarnings)
		.def("ConfigureAcquisitionRecord", &pyniscope::ConfigureAcquisitionRecord)
		.def("ConfigureChannel", &pyniscope::ConfigureChannel)
		.def("ConfigureEdgeTriggerSource", &pyniscope::ConfigureEdgeTriggerSource)
		.def("ConfigureRefLevels", &pyniscope::ConfigureRefLevels)
		.def("ConfigureTrigger", &pyniscope::ConfigureTrigger)
		.def("ConfigureTriggerCoupling", &pyniscope::ConfigureTriggerCoupling)
		.def("ConfigureTriggerOutput", &pyniscope::ConfigureTriggerOutput)
		.def("ConfigureTVTriggerLineNumber", &pyniscope::ConfigureTVTriggerLineNumber)
		.def("ConfigureTVTriggerSource", &pyniscope::ConfigureTVTriggerSource)
		.def("ErrorQuery", &pyniscope::ErrorQuery)
		.def("FetchWaveform", &pyniscope::FetchWaveform)
		.def("FetchWaveformMeasurement", &pyniscope::FetchWaveformMeasurement)
		.def("GetChannelName", &pyniscope::GetChannelName)
		.def("GetNextCoercionRecord", &pyniscope::GetNextCoercionRecord)
		.def("GetNextInterchangeWarning", &pyniscope::GetNextInterchangeWarning)
		.def("IsInvalidWfmElement", &pyniscope::IsInvalidWfmElement)
		.def("ReadWaveform", &pyniscope::ReadWaveform)
		.def("ReadWaveformMeasurement", &pyniscope::ReadWaveformMeasurement)
		.def("ResetInterchangeCheck", &pyniscope::ResetInterchangeCheck)
		.def("SendSWTrigger", &pyniscope::SendSWTrigger)
		;
	enum_<niScope_Acquisition>("AcquisitionAttribute")
		.value("AcquisitionType", AcquisitionType)
		.value("SampleMode", SampleMode)
		.value("BinarySampleWidth", BinarySampleWidth)
		.value("Resolution", Resolution)
		.value("FetchRelativeTo", FetchRelativeTo)
		.value("FetchOffset", FetchOffset)
		.value("FetchRecordNumber", FetchRecordNumber)
		.value("FetchNumRecords", FetchNumRecords)
		.value("FetchMeasNumSamples", FetchMeasNumSamples)
		.value("PointsDone", PointsDone)
		.value("RecordsDone", RecordsDone)
		.value("TransferBlockSize", TransferBlockSize)
		.value("Backlog", Backlog)
		.value("RisInAutoSetupEnable", RisInAutoSetupEnable);


	enum_<niScope_Vertical>("VerticalAttribute")
		.value("ChannelEnabled", ChannelEnabled)
		.value("ChannelTerminalConfiguration", ChannelTerminalConfiguration)
		.value("ProbeAttenuation", ProbeAttenuation)
		.value("Range", Range)
		.value("Offset", Offset)
		.value("InputImpedance", InputImpedance)
		.value("Coupling", Coupling)
		.value("MaxInputFrequency", MaxInputFrequency)
		.value("FlexFirAntialiasFilterType", FlexFirAntialiasFilterType)
		.value("DigitalGain", DigitalGain)
		.value("DigitalOffset", DigitalOffset)
		.value("BandpassFilterEnabled", BandpassFilterEnabled)
		.value("DitherEnabled", DitherEnabled);

	enum_<niScope_OnboardSignalProcessing>("OnboardSignalProcessingAttribute")
		.value("DdcCenterFrequency", DdcCenterFrequency)
		.value("DdcDataProcessingMode", DdcDataProcessingMode)
		.value("DdcEnabled", DdcEnabled)
		.value("FetchInterleavedIQData", FetchInterleavedIQData)
		//		.value("FrequencyTranslationEnabled", FrequencyTranslationEnabled)
		.value("DdcQSource", DdcQSource)
		.value("DdcFrequencyTranslationPhaseI", DdcFrequencyTranslationPhaseI)
		.value("DdcFrequencyTranslationPhaseQ", DdcFrequencyTranslationPhaseQ)
		.value("EqualizationFilterEnabled", EqualizationFilterEnabled)
		.value("EqualizationNumCoefficients", EqualizationNumCoefficients)
		.value("FractionalResampleEnabled", FractionalResampleEnabled);

	enum_<niScope_P2P>("P2PAttribute")
		.value("Enabled", Enabled)
		.value("ChannelsToStream", ChannelsToStream)
		.value("EndpointSize", EndpointSize)
		.value("SamplesAvailInEndpoint", SamplesAvailInEndpoint)
		.value("MostSamplesAvailInEndpoint", MostSamplesAvailInEndpoint)
		.value("SamplesTransferred", SamplesTransferred)
		.value("EndpointOverflow", EndpointOverflow)
		.value("FifoEndpointCount", FifoEndpointCount)
		.value("OnboardMemoryEnabled", OnboardMemoryEnabled);

	enum_<nisScope_Horizontal>("HorizontalAttribute")
		.value("AcquisitionStartTime", AcquisitionStartTime)
		.value("EnableTimeInterleavedSampling", EnableTimeInterleavedSampling)
		.value("HorzNumRecords", HorzNumRecords)
		.value("HorzTimePerRecord", HorzTimePerRecord)
		.value("HorzMinNumPts", HorzMinNumPts)
		.value("HorzRecordLength", HorzRecordLength)
		.value("HorzRecordRefPosition", HorzRecordRefPosition)
		.value("MinSampleRate", MinSampleRate)
		.value("HorzSampleRate", HorzSampleRate)
		.value("HorzEnforceRealtime", HorzEnforceRealtime)
		.value("RefTrigTdcEnable", RefTrigTdcEnable)
		.value("AllowMoreRecordsThanMemory", AllowMoreRecordsThanMemory)
		.value("PollInterval", PollInterval)
		.value("RisNumAverages", RisNumAverages)
		.value("RisMethod", RisMethod);

	enum_<niScope_Triggering>("TriggeringAttribute")
		.value("TriggerAutoTriggered", TriggerAutoTriggered)
		.value("RefTriggerMinimumQuietTime", RefTriggerMinimumQuietTime)
		.value("RefTriggerDetectorLocation", RefTriggerDetectorLocation)
		.value("StartToRefTriggerHoldoff", StartToRefTriggerHoldoff)
		.value("TriggerDelayTime", TriggerDelayTime)
		.value("TriggerCoupling", TriggerCoupling)
		.value("TriggerHoldoff", TriggerHoldoff)
		.value("TriggerHysteresis", TriggerHysteresis)
		.value("TriggerImpedance", TriggerImpedance)
		.value("TriggerLevel", TriggerLevel)
		.value("TriggerModifier", TriggerModifier)
		.value("ExportedRefTriggerOutputTerminal", ExportedRefTriggerOutputTerminal)
		.value("TriggerSlope", TriggerSlope)
		.value("TriggerSource", TriggerSource)
		.value("TriggerType", TriggerType)
		.value("TvTriggerSignalFormat", TvTriggerSignalFormat)
		.value("TvTriggerLineNumber", TvTriggerLineNumber)
		.value("TvTriggerPolarity", TvTriggerPolarity)
		.value("TvTriggerEvent", TvTriggerEvent)
		.value("EnableDcRestore", EnableDcRestore)
		.value("TriggerWindowLowLevel", TriggerWindowLowLevel)
		.value("TriggerWindowHighLevel", TriggerWindowHighLevel)
		.value("TriggerWindowMode", TriggerWindowMode);

	enum_<niScope_Device>("DeviceAttribute")
		.value("DeviceTemperature", DeviceTemperature)
		.value("SerialNumber", SerialNumber)
		//		.value("DaqMXMask", DaqMXMask)
		.value("SignalCondGain", SignalCondGain)
		.value("SignalCondOffset", SignalCondOffset);

	enum_<niScope_Clocking>("ClockingAttribute")
		//		.value("ExportedSampleClockOutputTerm", ExportedSampleClockOutputTerm)
		.value("InputClockSource", InputClockSource)
		.value("OutputClockSource", OutputClockSource)
		.value("ClockSyncPulseSource", ClockSyncPulseSource)
		.value("SampClkTimebaseSrc", SampClkTimebaseSrc)
		.value("SampClkTimebaseRate", SampClkTimebaseRate)
		.value("SampClkTimebaseDiv", SampClkTimebaseDiv)
		.value("SampClkTimebaseMult", SampClkTimebaseMult)
		.value("RefClkRate", RefClkRate)
		.value("ExportedSampleClockOutputTerminal", ExportedSampleClockOutputTerminal)
		.value("PllLockStatus", PllLockStatus);

	enum_<niScope_Synchronization>("SynchronizationAttribute")
		.value("5VOutOutputTerminal", FiveVOutOutputTerminal)
		.value("MasterEnable", MasterEnable)
		.value("AcqArmSource", AcqArmSource)
		.value("RecordArmSource", RecordArmSource)
		.value("EndOfAcquisitionEventOutputTerminal", EndOfAcquisitionEventOutputTerminal)
		.value("ExportedStartTriggerOutputTerminal", ExportedStartTriggerOutputTerminal)
		.value("ArmRefTrigSrc", ArmRefTrigSrc)
		.value("ExportedAdvanceTriggerOutputTerminal", ExportedAdvanceTriggerOutputTerminal)
		.value("AdvTrigSrc", AdvTrigSrc)
		.value("EndOfRecordEventOutputTerminal", EndOfRecordEventOutputTerminal)
		.value("ReadyForAdvanceEventOutputTerminal", ReadyForAdvanceEventOutputTerminal)
		.value("ReadyForStartEventOutputTerminal", ReadyForStartEventOutputTerminal)
		.value("RedayForRefEventOutputTerminal", RedayForRefEventOutputTerminal)
		.value("SlaveTriggerDelay", SlaveTriggerDelay)
		.value("TriggerToStarDelay", TriggerToStarDelay)
		.value("TriggerToRtsiDelay", TriggerToRtsiDelay)
		.value("TriggerToPfiDelay", TriggerToPfiDelay)
		.value("TriggerFromStarDelay", TriggerFromStarDelay)
		.value("TriggerFromRtsiDelay", TriggerFromRtsiDelay)
		.value("TriggerFromPfiDelay", TriggerFromPfiDelay);

	enum_<niScope_WaveformMeasurements>("WaveformMeasurementsAttribute")
		.value("MeasOtherChannel", MeasOtherChannel)
		.value("MeasHysteresisPercent", MeasHysteresisPercent)
		//		.value("LastAcqHistogramSize", LastAcqHistogramSize)
		.value("MeasArrayGain", MeasArrayGain)
		.value("MeasArrayOffset", MeasArrayOffset)
		.value("MeasChanLowRefLevel", MeasChanLowRefLevel)
		.value("MeasChanMidRefLevel", MeasChanMidRefLevel)
		.value("MeasChanHighRefLevel", MeasChanHighRefLevel)
		.value("MeasPolynomialInterpolationOrder", MeasPolynomialInterpolationOrder)
		.value("MeasInterpolationSamplingFactor", MeasInterpolationSamplingFactor)
		.value("MeasVoltageHistogramSize", MeasVoltageHistogramSize)
		.value("MeasVoltageHistogramLowVolts", MeasVoltageHistogramLowVolts)
		.value("MeasVoltageHistogramHighVolts", MeasVoltageHistogramHighVolts)
		.value("MeasTimeHistogramSize", MeasTimeHistogramSize)
		.value("MeasTimeHistogramHighVolts", MeasTimeHistogramHighVolts)
		.value("MeasTimeHistogramLowTime", MeasTimeHistogramLowTime)
		.value("MeasTimeHistogramHighTime", MeasTimeHistogramHighTime)
		.value("MeasTimeHistogramLowVolts", MeasTimeHistogramLowVolts)
		.value("MeasFilterCutoffFreq", MeasFilterCutoffFreq)
		.value("MeasFilterCenterFreq", MeasFilterCenterFreq)
		.value("MeasFilterWidth", MeasFilterWidth)
		.value("MeasFilterRipple", MeasFilterRipple)
		.value("MeasFilterTransientWaveformPercent", MeasFilterTransientWaveformPercent)
		.value("MeasFilterType", MeasFilterType)
		.value("MeasFilterOrder", MeasFilterOrder)
		.value("MeasFilterTaps", MeasFilterTaps)
		.value("MeasFirFilterWindow", MeasFirFilterWindow);

	enum_<niScope_InstrumentCapabilities>("InstrumentCapabilitiesAttribute")
		.value("MaxRisRate", MaxRisRate)
		.value("MaxRealTimeSamplingRate", MaxRealTimeSamplingRate)
		.value("OnboardMemorySize", OnboardMemorySize);

	enum_<niScope_IviAttribute>("IviAttribute")
		.value("Bandwidth", Bandwidth)
		.value("Cache", Cache)
		.value("ChannelCount", ChannelCount)
		.value("DriverSetup", DriverSetup)
		.value("GroupCapabilities", GroupCapabilities)
		.value("InstrumentFirmwareRevision", InstrumentFirmwareRevision)
		.value("InstrumentManufacturer", InstrumentManufacturer)
		.value("InstrumentModel", InstrumentModel)
		.value("InterchangeCheck", InterchangeCheck)
		.value("LogicalName", LogicalName)
		.value("MeasHighRef", MeasHighRef)
		.value("MeasLowRef", MeasLowRef)
		.value("MeasMidRef", MeasMidRef)
		.value("MeasPercentageMethod", MeasPercentageMethod)
		.value("MeasRefLevelUnits", MeasRefLevelUnits)
		.value("NumChannels", NumChannels)
		.value("QueryInstrumentStatus", QueryInstrumentStatus)
		.value("RangeCheck", RangeCheck)
		.value("RecordCoercions", RecordCoercions)
		.value("ResourceDescriptor", ResourceDescriptor)
		.value("Simulate", Simulate)
		.value("SpecificDriverClassSpecMajorVersion", SpecificDriverClassSpecMajorVersion)
		.value("SpecificDriverClassSpecMinorVersion", SpecificDriverClassSpecMinorVersion)
		.value("SpecificDriverDescription", SpecificDriverDescription)
		.value("SpecificDriverPrefix", SpecificDriverPrefix)
		.value("SpecificDriverRevision", SpecificDriverRevision)
		.value("SpecificDriverVendor", SpecificDriverVendor)
		.value("SupportedInstrumentModels", SupportedInstrumentModels);
}