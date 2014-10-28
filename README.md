pyniscope
=========

Python wrapper for National Instruments high speed digitizer interface.

This wrapper is based on boost-python and is tested to run on Windows 7 with NI-SCOPE 4.1.
I used Microsoft Visual Studio Express 2013 to compile (project files included).

The wrapper is made to wrap as closely as possible the functions provided by the NI-SCOPE C interface. The project is made of a C-to-C++ stage followed by a C++-to-Python stage (I find it more readable in this way).

Architecture
============

Here is a brief list of the functions and other things provided by pyniscope. An extended description can be found in the [NI-SCOPE help files](http://zone.ni.com/reference/en-XX/help/370592W-01/), under *Programming / Reference / NI-SCOPE Function Reference Help*.
The *enum* structures give a convenient way to access the attributes with the Set/Get/Check attribute functions. Data types and descriptions are found in the NI-SCOPE help files.

**pyniscope**
* bool Initialize(str resourceName, bool IDQuery, bool resetDevice)
* bool Close()
  
*Configuration functions*
* bool AutoSetup()
* bool ConfigureAcquisition(int acquisitionType)

*Configuration - Horizontal*
* bool ConfigureHorizontalTiming(float minSampleRate, int minNumPts, float refPosition, int numRecords, bool enforceRealtime)

*Configuration - Vertical*
* bool ConfigureChanCharacteristics(str channelList, float inputImpedance, float maxInputFrequency)
* bool ConfigureVertical(str channelList, float range, float offset, int coupling, float probeAttenuation, bool enabled)

*Configuration - Actual values*
* int ActualMeasWfmSize(int arrayMeasFunction)
* int ActualNumWfms(str channelList)
* int ActualRecordLength()
* int SampleMode()
* float SampleRate()

*Configuration - Trigger*
* bool ConfigureTriggerDigital(str triggerSource, int slope, float holdoff, float delay)
* bool ConfigureTriggerEdge(str triggerSource, float level, int slope, int triggerCoupling, float holdoff, float delay)
* bool ConfigureTriggerVideo(str triggerSource, bool enableDCRestore, int signalFormat, int evt, int lineNumber, int polarity, int triggerCoupling, float holdoff, float delay)
* bool ConfigureTriggerHysteresis(str triggerSource, float level, float hysteresis, int slope, int triggerCoupling, float holdoff, float delay)
* bool ConfigureTriggerImmediate()
* bool ConfigureTriggerSoftware(float holdoff, float delay)
* bool ConfigureTriggerWindow(str triggerSource, float lowLevel, float highLevel, int windowMode, int triggerCoupling, float holdoff, float delay)
* bool SendSoftwareTriggerEdge(int whichTrigger)

*Configuration - Synchronization*
* bool AdjustSampleClockRelativeDelay(float delay)
* bool ConfigureClock(str inputClockSource, str outputClockSource, str clockSyncPulseSource, bool masterEnabled)
* bool ExportSignal(int signal, str signalIdentifier, str outputTerminal)

*Configuration - Onboard Signal Processing*
* list GetFrequencyResponse(str channelName)
* bool ConfigureEqualizationFilterCoefficients(str channel, list coefficients)

*Set/Get/Check attribute functions*

*Set attribute*
* bool SetAttributeViInt32(str channelList, int attributeID, int value)
* bool SetAttributeViInt64(str channelList, int attributeID, int value)
* bool SetAttributeViReal64(str channelList, int attributeID, float value)
* bool SetAttributeViString(str channelList, int attributeID, str value)
* bool SetAttributeViBoolean(str channelList, int attributeID, bool value)
* bool SetAttributeViSession(str channelList, int attributeID, int value)

*Get attribute*
* int GetAttributeViInt32(str channelList, int attributeID)
* int GetAttributeViInt64(str channelList, int attributeID)
* float GetAttributeViReal64(str channelList, int attributeID)
* str GetAttributeViString(str channelList, int attributeID)
* bool GetAttributeViBoolean(str channelList, int attributeID)
* int GetAttributeViSession(str channelList, int attributeID)

*Check attribute*
* bool CheckAttributeViInt32(str channelList, int attributeID, int value)
* bool CheckAttributeViInt64(str channelList, int attributeID, int value)
* bool CheckAttributeViReal64(str channelList, int attributeID, float value)
* bool CheckAttributeViString(str channelList, int attributeID, str value)
* bool CheckAttributeViBoolean(str channelList, int attributeID, bool value)
* bool CheckAttributeViSession(str channelList, int attributeID, int value)

*Acquisition functions*
* bool Abort()
* int AcquisitionStatus()
* bool Commit()
* list Fetch(str channelList, float timeout, int numSamples)
* list FetchComplex(str channelList, float timeout, int numSamples)
* list FetchComplexBinary16(str channelList, float timeout, int numSamples)
* list FetchBinary8(str channelList, float timeout, int numSamples)
* list FetchBinary16(str channelList, float timeout, int numSamples)
* list FetchBinary32(str channelList, float timeout, int numSamples)
* list GetNormalizationCoefficients(str channelList)
* list GetScalingCoefficients(str channelList)
* bool InitiateAcquisition()
* list Read(str channelList, float timeout, int numSamples)

*Measurement functions*
* bool AddWaveformProcessing(str channelList, int measFunction)
* bool ClearWaveformMeasurementStats(str channelList, int clearableMeasurementFunction)
* bool ClearWaveformProcessing(str channelList)
* list FetchArrayMeasurement(str channelList, float timeout, int arrayMeasFunction)
* list FetchMeasurement(str channelList, float timeout, int scalarMeasFunction)
* list FetchMeasurementStats(str channelList, float timeout, int scalarMeasFunction)
* list ReadMeasurement(str channelList, float timeout, int scalarMeasFunction)

*Calibration functions*
* bool CalSelfCalibrate(str channelList, int option)

*Utility functions*
* bool ResetDevice()
* bool Disable()
* bool ProbeCompensationSignalStart()
* bool ProbeCompensationSignalStop()
* bool IsDeviceReady(str resourceName, str channelList)
* bool Reset()
* bool ResetWithDefaults()
* list RevisionQuery()
* list SelfTest()
* int GetStreamEndpointHandle(str streamName)

*Error handling functions*
* list ErrorHandler(int errorCode)
* str GetError(int errorCode)
* str GetErrorMessage(int errorCode)

*Locking functions*
* bool LockSession()
* bool UnlockSession()

*IVI compliance functions*
* bool ClearError()
* bool ClearInterchangeWarnings()
* bool ConfigureAcquisitionRecord(float timeperRecord, int minNumPoints, float acquisitionStartTime)
* bool ConfigureChannel(str channel, float range, float offset, int coupling, float probeAttenuation, bool enabled)
* bool ConfigureEdgeTriggerSource(str source, float level, int slope)
* bool ConfigureRefLevels(float low, float mid, float high)
* bool ConfigureTrigger(int triggerType, float holdoff)
* bool ConfigureTriggerCoupling(int coupling)
* bool ConfigureTriggerOutput(int triggerEvent, str triggerOutput)
* bool ConfigureTVTriggerLineNumber(int lineNumber)
* bool ConfigureTVTriggerSource(str source, int signalFormat, int evt, int polarity)
* list ErrorQuery()
* list FetchWaveform(str channel, int waveformSize)
* list FetchWaveformMeasurement(str channel, int measFunction)
* str GetChannelName(int index)
* str GetNextCoercionRecord()
* str GetNextInterchangeWarning()
* bool IsInvalidWfmElement(float elementValue)
* list ReadWaveform(str channel, int waveformSize, int maxtime)
* list ReadWaveformMeasurement(str channel, int measFunction, int maxTime)
* bool ResetInterchangeCheck()
* bool SendSWTrigger()

*AcquisitionAttribute enum*
* AcquisitionType
* SampleMode
* BinarySampleWidth
* Resolution
* FetchRelativeTo
* FetchOffset
* FetchRecordNumber
* FetchNumRecords
* FetchMeasNumSamples
* PointsDone
* RecordsDone
* TransferBlockSize
* Backlog
* RisInAutoSetupEnable


*VerticalAttribute enum*
* ChannelEnabled
* ChannelTerminalConfiguration
* ProbeAttenuation
* Range
* Offset
* InputImpedance
* Coupling
* MaxInputFrequency
* FlexFirAntialiasFilterType
* DigitalGain
* DigitalOffset
* BandpassFilterEnabled
* DitherEnabled

*OnboardSignalProcessingAttribute enum*
* DdcCenterFrequency
* DdcDataProcessingMode
* DdcEnabled
* FetchInterleavedIQData
* DdcQSource
* DdcFrequencyTranslationPhaseI
* DdcFrequencyTranslationPhaseQ
* EqualizationFilterEnabled
* EqualizationNumCoefficients
* FractionalResampleEnabled

*P2PAttribute enum*
* Enabled
* ChannelsToStream
* EndpointSize
* SamplesAvailInEndpoint
* MostSamplesAvailInEndpoint
* SamplesTransferred
* EndpointOverflow
* FifoEndpointCount
* OnboardMemoryEnabled

*HorizontalAttribute enum*
* AcquisitionStartTime
* EnableTimeInterleavedSampling
* HorzNumRecords
* HorzTimePerRecord
* HorzMinNumPts
* HorzRecordLength
* HorzRecordRefPosition
* MinSampleRate
* HorzSampleRate
* HorzEnforceRealtime
* RefTrigTdcEnable
* AllowMoreRecordsThanMemory
* PollInterval
* RisNumAverages
* RisMethod

*TriggeringAttribute enum*
* TriggerAutoTriggered
* RefTriggerMinimumQuietTime
* RefTriggerDetectorLocation
* StartToRefTriggerHoldoff
* TriggerDelayTime
* TriggerCoupling
* TriggerHoldoff
* TriggerHysteresis
* TriggerImpedance
* TriggerLevel
* TriggerModifier
* ExportedRefTriggerOutputTerminal
* TriggerSlope
* TriggerSource
* TriggerType
* TvTriggerSignalFormat
* TvTriggerLineNumber
* TvTriggerPolarity
* TvTriggerEvent
* EnableDcRestore
* TriggerWindowLowLevel
* TriggerWindowHighLevel
* TriggerWindowMode

*DeviceAttribute enum*
* DeviceTemperature
* SerialNumber
* SignalCondGain
* SignalCondOffset

*ClockingAttribute enum*
* InputClockSource
* OutputClockSource
* ClockSyncPulseSource
* SampClkTimebaseSrc
* SampClkTimebaseRate
* SampClkTimebaseDiv
* SampClkTimebaseMult
* RefClkRate
* ExportedSampleClockOutputTerminal
* PllLockStatus

*SynchronizationAttribute enum*
* 5VOutOutputTerminal
* MasterEnable
* AcqArmSource
* RecordArmSource
* EndOfAcquisitionEventOutputTerminal
* ExportedStartTriggerOutputTerminal
* ArmRefTrigSrc
* ExportedAdvanceTriggerOutputTerminal
* AdvTrigSrc
* EndOfRecordEventOutputTerminal
* ReadyForAdvanceEventOutputTerminal
* ReadyForStartEventOutputTerminal
* RedayForRefEventOutputTerminal
* SlaveTriggerDelay
* TriggerToStarDelay
* TriggerToRtsiDelay
* TriggerToPfiDelay
* TriggerFromStarDelay
* TriggerFromRtsiDelay
* TriggerFromPfiDelay

*WaveformMeasurementsAttribute enum*
* MeasOtherChannel
* MeasHysteresisPercent
* MeasArrayGain
* MeasArrayOffset
* MeasChanLowRefLevel
* MeasChanMidRefLevel
* MeasChanHighRefLevel
* MeasPolynomialInterpolationOrder
* MeasInterpolationSamplingFactor
* MeasVoltageHistogramSize
* MeasVoltageHistogramLowVolts
* MeasVoltageHistogramHighVolts
* MeasTimeHistogramSize
* MeasTimeHistogramHighVolts
* MeasTimeHistogramLowTime
* MeasTimeHistogramHighTime
* MeasTimeHistogramLowVolts
* MeasFilterCutoffFreq
* MeasFilterCenterFreq
* MeasFilterWidth
* MeasFilterRipple
* MeasFilterTransientWaveformPercent
* MeasFilterType
* MeasFilterOrder
* MeasFilterTaps
* MeasFirFilterWindow

*InstrumentCapabilitiesAttribute enum*
* MaxRisRate
* MaxRealTimeSamplingRate
* OnboardMemorySize

*IviAttribute enum*
* Bandwidth
* Cache
* ChannelCount
* DriverSetup
* GroupCapabilities
* InstrumentFirmwareRevision
* InstrumentManufacturer
* InstrumentModel
* InterchangeCheck
* LogicalName
* MeasHighRef
* MeasLowRef
* MeasMidRef
* MeasPercentageMethod
* MeasRefLevelUnits
* NumChannels
* QueryInstrumentStatus
* RangeCheck
* RecordCoercions
* ResourceDescriptor
* Simulate
* SpecificDriverClassSpecMajorVersion
* SpecificDriverClassSpecMinorVersion
* SpecificDriverDescription
* SpecificDriverPrefix
* SpecificDriverRevision
* SpecificDriverVendor
* SupportedInstrumentModels

Usage example
=============

	# digitizer settings
	instr_scope = "PXIData" # instrument address
	channels = "0,1" # channel string (comma-separated channel numbers)
	verticalSpan = 0.2 # vertical span in V
	sampleRate = 1e8 # sample rate in sample/sec
	numPoints = 2000 # number of points to be recorded
	inputImpedance = 1e6 # input impedance in Ohm
	timeout = 1500 # acquisition timeout in ms
	
	# loads digitizer drivers
	from pyniscope import pyniscope
	scope = pyniscope()
	
	# initializes and sets digitizer to the appropriate state
	scope.Initialize(instr_scope, False, False)
	scope.ConfigureVertical(channels, verticalSpan, 0.0, 1, 1.0, True)
	scope.ConfigureChanCharacteristics(channels, inputImpedance, 0.0)
	scope.ConfigureHorizontalTiming(sampleRate, numPoints, 0.0, 1, True)
	
	# sets trigger mode
	scope.ConfigureTriggerSoftware(0.0, 0.0)
	
	# starts digitizing
	scope.InitiateAcquisition()
	scope.SendSWTrigger()
	while scope.AcquisitionStatus()==0:
		pass
	
	data = scope.Fetch(channels, timeout, numPoints)[1]
	
	# reads actual sample rate and builds time vector
	sr = scope.SampleRate()
	tv = array(range(len(data)))/sr
	
	# add here what you want to do with the data

	scope.Close() # closes digitizer connection
