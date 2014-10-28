pyniscope
=========

Python wrapper for National Instruments high speed digitizer interface.

This wrapper is based on boost-python and is tested to run on Windows 7 with NI-SCOPE 4.1.
I used Microsoft Visual Studio Express 2013 to compile (project files included).

The wrapper is made to wrap as closely as possible the functions provided by NI-SCOPE C interface. The project is made of a C-to-C++ stage followed by a C++-to-Python stage (I find it more readable this way).

Architecture
============

Here is a brief list of the functions and other things provided by pyniscope. An extended description may be found in the NI-SCOPE help files.

**pyniscope**
	- bool Initialize(string resourceName, bool IDQuery, bool resetDevice)
	- bool Close()
  
	* Configuration functions *
	- bool AutoSetup()
	- bool ConfigureAcquisition(int acquisitionType)
	* Configuration - Horizontal *
	- bool ConfigureHorizontalTiming(double minSampleRate, int minNumPts, double refPosition, int numRecords, bool enforceRealtime)
	* Configuration - Vertical *
	- bool ConfigureChanCharacteristics(string channelList, double inputImpedance, double maxInputFrequency)
	- bool ConfigureVertical(string channelList, double range, double offset, int coupling, double probeAttenuation, bool enabled)
	* Configuration - Actual values *
	- int ActualMeasWfmSize(int arrayMeasFunction)
	- int ActualNumWfms(string channelList)
	- int ActualRecordLength()
	- int SampleMode()
	- double SampleRate()
	* Configuration - Trigger 
	- bool ConfigureTriggerDigital(string triggerSource, int slope, double holdoff, double delay)
	- bool ConfigureTriggerEdge(string triggerSource, double level, int slope, int triggerCoupling, double holdoff, double delay)
	- bool ConfigureTriggerVideo(string triggerSource, bool enableDCRestore, int signalFormat, int evt, int lineNumber, int polarity, int triggerCoupling, double holdoff, double delay)
	- bool ConfigureTriggerHysteresis(string triggerSource, double level, double hysteresis, int slope, int triggerCoupling, double holdoff, double delay)
	- bool ConfigureTriggerImmediate()
	- bool ConfigureTriggerSoftware(double holdoff, double delay)
	- bool ConfigureTriggerWindow(string triggerSource, double lowLevel, double highLevel, int windowMode, int triggerCoupling, double holdoff, double delay)
	- bool SendSoftwareTriggerEdge(int whichTrigger)
	* Configuration - Synchronization *
	- bool AdjustSampleClockRelativeDelay(double delay)
	- bool ConfigureClock(string inputClockSource, string outputClockSource, string clockSyncPulseSource, bool masterEnabled)
	- bool ExportSignal(int signal, string signalIdentifier, string outputTerminal)
	* Configuration - Onboard Signal Processing *
	- list GetFrequencyResponse(string channelName)
	- bool ConfigureEqualizationFilterCoefficients(string channel, list coefficients)
	
	* Set/Get/Check attribute functions *
	* Set attribute *
	- bool SetAttributeViInt32(string channelList, int attributeID, int value)
	- bool SetAttributeViInt64(string channelList, int attributeID, int value)
	- bool SetAttributeViReal64(string channelList, int attributeID, double value)
	- bool SetAttributeViString(string channelList, int attributeID, string value)
	- bool SetAttributeViBoolean(string channelList, int attributeID, bool value)
	- bool SetAttributeViSession(string channelList, int attributeID, int value)
	* Get attribute *
	- int GetAttributeViInt32(string channelList, int attributeID)
	- int GetAttributeViInt64(string channelList, int attributeID)
	- double GetAttributeViReal64(string channelList, int attributeID)
	- string GetAttributeViString(string channelList, int attributeID)
	- bool GetAttributeViBoolean(string channelList, int attributeID)
	- int GetAttributeViSession(string channelList, int attributeID)
	* Check attribute *
	- bool CheckAttributeViInt32(string channelList, int attributeID, int value)
	- bool CheckAttributeViInt64(string channelList, int attributeID, int value)
	- bool CheckAttributeViReal64(string channelList, int attributeID, double value)
	- bool CheckAttributeViString(string channelList, int attributeID, string value)
	- bool CheckAttributeViBoolean(string channelList, int attributeID, bool value)
	- bool CheckAttributeViSession(string channelList, int attributeID, int value)
	
	* Acquisition functions *
	- bool Abort()
	- int AcquisitionStatus()
	- bool Commit()
	- list Fetch(string channelList, double timeout, int numSamples)
	- list FetchComplex(string channelList, double timeout, int numSamples)
	- list FetchComplexBinary16(string channelList, double timeout, int numSamples)
	- list FetchBinary8(string channelList, double timeout, int numSamples)
	- list FetchBinary16(string channelList, double timeout, int numSamples)
	- list FetchBinary32(string channelList, double timeout, int numSamples)
	- list GetNormalizationCoefficients(string channelList)
	- list GetScalingCoefficients(string channelList)
	- bool InitiateAcquisition()
	- list Read(string channelList, double timeout, int numSamples)

	* Measurement functions *
	- bool AddWaveformProcessing(string channelList, int measFunction)
	- bool ClearWaveformMeasurementStats(string channelList, int clearableMeasurementFunction)
	- bool ClearWaveformProcessing(string channelList)
	- list FetchArrayMeasurement(string channelList, double timeout, int arrayMeasFunction)
	- list FetchMeasurement(string channelList, double timeout, int scalarMeasFunction)
	- list FetchMeasurementStats(string channelList, double timeout, int scalarMeasFunction)
	- list ReadMeasurement(string channelList, double timeout, int scalarMeasFunction)

	* Calibration functions *
	- bool CalSelfCalibrate(string channelList, int option)

	* Utility functions *
	- bool ResetDevice()
	- bool Disable()
	- bool ProbeCompensationSignalStart()
	- bool ProbeCompensationSignalStop()
	- bool IsDeviceReady(string resourceName, string channelList)
	- bool Reset()
	- bool ResetWithDefaults()
	- list RevisionQuery()
	- list SelfTest()
	- int GetStreamEndpointHandle(string streamName)
	
	* Error handling functions *
	- list ErrorHandler(int errorCode)
	- string GetError(int errorCode)
	- string GetErrorMessage(int errorCode)
	
	* Locking functions *
	- bool LockSession()
	- bool UnlockSession()
	
	* IVI compliance functions *
	- bool ClearError()
	- bool ClearInterchangeWarnings()
	- bool ConfigureAcquisitionRecord(double timeperRecord, int minNumPoints, double acquisitionStartTime)
	- bool ConfigureChannel(string channel, double range, double offset, int coupling, double probeAttenuation, bool enabled)
	- bool ConfigureEdgeTriggerSource(string source, double level, int slope)
	- bool ConfigureRefLevels(double low, double mid, double high)
	- bool ConfigureTrigger(int triggerType, double holdoff)
	- bool ConfigureTriggerCoupling(int coupling)
	- bool ConfigureTriggerOutput(int triggerEvent, string triggerOutput)
	- bool ConfigureTVTriggerLineNumber(int lineNumber)
	- bool ConfigureTVTriggerSource(string source, int signalFormat, int evt, int polarity)
	- list ErrorQuery()
	- list FetchWaveform(string channel, int waveformSize)
	- list FetchWaveformMeasurement(string channel, int measFunction)
	- string GetChannelName(int index)
	- string GetNextCoercionRecord()
	- string GetNextInterchangeWarning()
	- bool IsInvalidWfmElement(double elementValue)
	- list ReadWaveform(string channel, int waveformSize, int maxtime)
	- list ReadWaveformMeasurement(string channel, int measFunction, int maxTime)
	- bool ResetInterchangeCheck()
	- bool SendSWTrigger()

	* AcquisitionAttribute enum *
	- AcquisitionType
	- SampleMode
	- BinarySampleWidth
	- Resolution
	- FetchRelativeTo
	- FetchOffset
	- FetchRecordNumber
	- FetchNumRecords
	- FetchMeasNumSamples
	- PointsDone
	- RecordsDone
	- TransferBlockSize
	- Backlog
	- RisInAutoSetupEnable


	* VerticalAttribute enum *
	- ChannelEnabled
	- ChannelTerminalConfiguration
	- ProbeAttenuation
	- Range
	- Offset
	- InputImpedance
	- Coupling
	- MaxInputFrequency
	- FlexFirAntialiasFilterType
	- DigitalGain
	- DigitalOffset
	- BandpassFilterEnabled
	- DitherEnabled

	* OnboardSignalProcessingAttribute enum *
	- DdcCenterFrequency
	- DdcDataProcessingMode
	- DdcEnabled
	- FetchInterleavedIQData
	- DdcQSource
	- DdcFrequencyTranslationPhaseI
	- DdcFrequencyTranslationPhaseQ
	- EqualizationFilterEnabled
	- EqualizationNumCoefficients
	- FractionalResampleEnabled

	* P2PAttribute enum *
	- Enabled
	- ChannelsToStream
	- EndpointSize
	- SamplesAvailInEndpoint
	- MostSamplesAvailInEndpoint
	- SamplesTransferred
	- EndpointOverflow
	- FifoEndpointCount
	- OnboardMemoryEnabled

	* HorizontalAttribute enum *
	- AcquisitionStartTime
	- EnableTimeInterleavedSampling
	- HorzNumRecords
	- HorzTimePerRecord
	- HorzMinNumPts
	- HorzRecordLength
	- HorzRecordRefPosition
	- MinSampleRate
	- HorzSampleRate
	- HorzEnforceRealtime
	- RefTrigTdcEnable
	- AllowMoreRecordsThanMemory
	- PollInterval
	- RisNumAverages
	- RisMethod

	* TriggeringAttribute enum *
	- TriggerAutoTriggered
	- RefTriggerMinimumQuietTime
	- RefTriggerDetectorLocation
	- StartToRefTriggerHoldoff
	- TriggerDelayTime
	- TriggerCoupling
	- TriggerHoldoff
	- TriggerHysteresis
	- TriggerImpedance
	- TriggerLevel
	- TriggerModifier
	- ExportedRefTriggerOutputTerminal
	- TriggerSlope
	- TriggerSource
	- TriggerType
	- TvTriggerSignalFormat
	- TvTriggerLineNumber
	- TvTriggerPolarity
	- TvTriggerEvent
	- EnableDcRestore
	- TriggerWindowLowLevel
	- TriggerWindowHighLevel
	- TriggerWindowMode

	* DeviceAttribute enum *
	- DeviceTemperature
	- SerialNumber
	- SignalCondGain
	- SignalCondOffset

	* ClockingAttribute enum *
	- InputClockSource
	- OutputClockSource
	- ClockSyncPulseSource
	- SampClkTimebaseSrc
	- SampClkTimebaseRate
	- SampClkTimebaseDiv
	- SampClkTimebaseMult
	- RefClkRate
	- ExportedSampleClockOutputTerminal
	- PllLockStatus

	* SynchronizationAttribute enum *
	- 5VOutOutputTerminal
	- MasterEnable
	- AcqArmSource
	- RecordArmSource
	- EndOfAcquisitionEventOutputTerminal
	- ExportedStartTriggerOutputTerminal
	- ArmRefTrigSrc
	- ExportedAdvanceTriggerOutputTerminal
	- AdvTrigSrc
	- EndOfRecordEventOutputTerminal
	- ReadyForAdvanceEventOutputTerminal
	- ReadyForStartEventOutputTerminal
	- RedayForRefEventOutputTerminal
	- SlaveTriggerDelay
	- TriggerToStarDelay
	- TriggerToRtsiDelay
	- TriggerToPfiDelay
	- TriggerFromStarDelay
	- TriggerFromRtsiDelay
	- TriggerFromPfiDelay

	 * WaveformMeasurementsAttribute enum *
	- MeasOtherChannel
	- MeasHysteresisPercent
	- MeasArrayGain
	- MeasArrayOffset
	- MeasChanLowRefLevel
	- MeasChanMidRefLevel
	- MeasChanHighRefLevel
	- MeasPolynomialInterpolationOrder
	- MeasInterpolationSamplingFactor
	- MeasVoltageHistogramSize
	- MeasVoltageHistogramLowVolts
	- MeasVoltageHistogramHighVolts
	- MeasTimeHistogramSize
	- MeasTimeHistogramHighVolts
	- MeasTimeHistogramLowTime
	- MeasTimeHistogramHighTime
	- MeasTimeHistogramLowVolts
	- MeasFilterCutoffFreq
	- MeasFilterCenterFreq
	- MeasFilterWidth
	- MeasFilterRipple
	- MeasFilterTransientWaveformPercent
	- MeasFilterType
	- MeasFilterOrder
	- MeasFilterTaps
	- MeasFirFilterWindow

	* InstrumentCapabilitiesAttribute enum *
	- MaxRisRate
	- MaxRealTimeSamplingRate
	- OnboardMemorySize

	* IviAttribute enum *
	- Bandwidth
	- Cache
	- ChannelCount
	- DriverSetup
	- GroupCapabilities
	- InstrumentFirmwareRevision
	- InstrumentManufacturer
	- InstrumentModel
	- InterchangeCheck
	- LogicalName
	- MeasHighRef
	- MeasLowRef
	- MeasMidRef
	- MeasPercentageMethod
	- MeasRefLevelUnits
	- NumChannels
	- QueryInstrumentStatus
	- RangeCheck
	- RecordCoercions
	- ResourceDescriptor
	- Simulate
	- SpecificDriverClassSpecMajorVersion
	- SpecificDriverClassSpecMinorVersion
	- SpecificDriverDescription
	- SpecificDriverPrefix
	- SpecificDriverRevision
	- SpecificDriverVendor
	- SupportedInstrumentModels
