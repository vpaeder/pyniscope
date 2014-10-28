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

 C++ wrapper for NI-SCOPE driver.

 This wrapper translates the NI-SCOPE C API to a C++ class.

*/

#ifndef _NI_SCOPE_WRAP_H_
#define _NI_SCOPE_WRAP_H_

#pragma comment (lib, "niscope.lib")
#include "niscope.h"

class niscopew {
protected:
	ViSession session;
	bool sessionOpen;

	bool HandleStatus(ViStatus status);

public:
	niscopew();
	niscopew(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice);
	niscopew(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViString optionString);
	~niscopew();

	bool Initialize(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice);
	bool Initialize(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViString optionString);
	bool Close();

	// class Configuration
		bool AutoSetup();
		bool ConfigureAcquisition(ViInt32 acquisitionType);
		bool ConfigureHorizontalTiming(ViReal64 minSampleRate, ViInt32 minNumPts, ViReal64 refPosition, ViInt32 numRecords, ViBoolean enforceRealtime);
		bool ConfigureChanCharacteristics(ViConstString channelList, ViReal64 inputImpedance, ViReal64 maxInputFrequency);
		bool ConfigureVertical(ViConstString channelList, ViReal64 range, ViReal64 offset, ViInt32 coupling, ViReal64 probeAttenuation, ViBoolean enabled);
		bool ActualMeasWfmSize(ViInt32 arrayMeasFunction, ViInt32* measWfmSize);
		bool ActualNumWfms(ViConstString channelList, ViInt32* numWfms);
		bool ActualRecordLength(ViInt32* recordLength);
		bool SampleMode(ViInt32* sampleMode);
		bool SampleRate(ViReal64* sampleRate);
		bool ConfigureTriggerDigital(ViConstString triggerSource, ViInt32 slope, ViReal64 holdoff, ViReal64 delay);
		bool ConfigureTriggerEdge(ViConstString triggerSource, ViReal64 level, ViInt32 slope, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay);
		bool ConfigureTriggerVideo(ViConstString triggerSource, ViBoolean enableDCRestore, ViInt32 signalFormat, ViInt32 evt, ViInt32 lineNumber, ViInt32 polarity, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay);
		bool ConfigureTriggerHysteresis(ViConstString triggerSource, ViReal64 level, ViReal64 hysteresis, ViInt32 slope, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay);
		bool ConfigureTriggerImmediate();
		bool ConfigureTriggerSoftware(ViReal64 holdoff, ViReal64 delay);
		bool ConfigureTriggerWindow(ViConstString triggerSource, ViReal64 lowLevel, ViReal64 highLevel, ViInt32 windowMode, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay);
		bool SendSoftwareTriggerEdge(ViInt32 whichTrigger);
		bool AdjustSampleClockRelativeDelay(ViReal64 delay);
		bool ConfigureClock(ViConstString inputClockSource, ViConstString outputClockSource, ViConstString clockSyncPulseSource, ViBoolean masterEnabled);
		bool ExportSignal(ViInt32 signal, ViConstString signalIdentifier, ViConstString outputTerminal);
		bool GetFrequencyResponse(ViConstString channelName, ViInt32 bufferSize, ViReal64 frequencies[], ViReal64 amplitudes[], ViReal64 phases[], ViInt32* numberOfFrequencies);
		int GetFrequencyResponseBufSize(ViConstString channelName);
		bool ConfigureEqualizationFilterCoefficients(ViConstString channel, ViInt32 numberOfCoefficients, ViReal64* coefficients);

	// class Attributes
		bool SetAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32 value);
		bool SetAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64 value);
		bool SetAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64 value);
		bool SetAttributeViString(ViConstString channelList, ViAttr attributeID, ViConstString value);
		bool SetAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean value);
		bool SetAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession value);
		bool GetAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32* value);
		bool GetAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64* value);
		bool GetAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64* value);
		bool GetAttributeViString(ViConstString channelList, ViAttr attributeID, ViInt32 bufSize, ViChar value[]);
		int GetAttributeViStringBufSize(ViConstString channelList, ViAttr attributeID);
		bool GetAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean* value);
		bool GetAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession* value);
		bool CheckAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32 value);
		bool CheckAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64 value);
		bool CheckAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64 value);
		bool CheckAttributeViString(ViConstString channelList, ViAttr attributeID, ViConstString value);
		bool CheckAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean value);
		bool CheckAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession value);

	// class Acquisition
		bool Abort();
		bool AcquisitionStatus(ViInt32* status);
		bool Commit();
		bool Fetch(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViReal64* wfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchComplex(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, NIComplexNumber* wfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchComplexBinary16(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, NIComplexI16* wfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchBinary8(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt8* wfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchBinary16(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt16* wfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchBinary32(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt32* wfm, struct niScope_wfmInfo* wfmInfo);
		bool GetNormalizationCoefficients(ViConstString channelList, ViInt32 bufferSize, struct niScope_coefficientInfo coefficientInfo[], ViInt32* numberOfCoefficientSets);
		int GetNormalizationCoefficientsBufSize(ViConstString channelList);
		bool GetScalingCoefficients(ViConstString channelList, ViInt32 bufferSize, struct niScope_coefficientInfo coefficientInfo[], ViInt32* numberOfCoefficientSets);
		int GetScalingCoefficientsBufSize(ViConstString channelList);
		bool InitiateAcquisition();
		bool Read(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViReal64* wfm, struct niScope_wfmInfo* wfmInfo);

	// class Measurement
		bool AddWaveformProcessing(ViConstString channelList, ViInt32 measFunction);
		bool ClearWaveformMeasurementStats(ViConstString channelList, ViInt32 clearableMeasurementFunction);
		bool ClearWaveformProcessing(ViConstString channelList);
		bool FetchArrayMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 arrayMeasFunction, ViInt32 measWfmSize, ViReal64* measWfm, struct niScope_wfmInfo* wfmInfo);
		bool FetchMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result);
		bool FetchMeasurementStats(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result, ViReal64* mean, ViReal64* stdev, ViReal64* min, ViReal64* max, ViInt32*numInStats);
		bool ReadMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result);

	// class Calibration
		bool CalSelfCalibrate(ViConstString channelList, ViInt32 option);

	// class Utility
		bool ResetDevice();
		bool Disable();
		bool ProbeCompensationSignalStart();
		bool ProbeCompensationSignalStop();
		bool IsDeviceReady(ViRsrc resourceName, ViConstString channelList, ViBoolean* deviceReady);
		bool Reset();
		bool ResetWithDefaults();
		bool RevisionQuery(ViChar driverRev[IVI_MAX_MESSAGE_BUF_SIZE], ViChar instrRev[IVI_MAX_MESSAGE_BUF_SIZE]);
		bool SelfTest(ViInt16* selfTestResult, ViChar selfTestMessage[IVI_MAX_MESSAGE_BUF_SIZE]);
		bool GetStreamEndpointHandle(ViConstString Stream_Name, ViUInt32 *Writer_Handle);

	// class ErrorHandling
		bool ErrorHandler(ViInt32 errorCode, ViChar errorSource[MAX_FUNCTION_NAME_SIZE], ViChar errorDescription[MAX_ERROR_DESCRIPTION]);
		bool GetError(ViStatus* errorCode, ViInt32 bufferSize, ViChar description[]);
		int GetErrorBufSize(ViStatus* errorCode);
		bool GetErrorMessage(ViStatus errorCode, ViInt32 bufferSize, ViChar errorMessage[]);
		int GetErrorMessageBufSize(ViStatus errorCode);

	// class Locking
		bool LockSession(ViBoolean* callerHasLock);
		bool UnlockSession(ViBoolean* callerHasLock);

	// class IVI
		bool ClearError();
		bool ClearInterchangeWarnings();
		bool ConfigureAcquisitionRecord(ViReal64 timeperRecord, ViInt32 minNumPoints, ViReal64 acquisitionStartTime);
		bool ConfigureChannel(ViConstString channel, ViReal64 range, ViReal64 offset, ViInt32 coupling, ViReal64 probeAttenuation, ViBoolean enabled);
		bool ConfigureEdgeTriggerSource(ViConstString source, ViReal64 level, ViInt32 slope);
		bool ConfigureRefLevels(ViReal64 low, ViReal64 mid, ViReal64 high);
		bool ConfigureTrigger(ViInt32 triggerType, ViReal64 holdoff);
		bool ConfigureTriggerCoupling(ViInt32 coupling);
		bool ConfigureTriggerOutput(ViInt32 triggerEvent, ViConstString triggerOutput);
		bool ConfigureTVTriggerLineNumber(ViInt32 lineNumber);
		bool ConfigureTVTriggerSource(ViConstString source, ViInt32 signalFormat, ViInt32 evt, ViInt32 polarity);
		bool ErrorQuery(ViInt32* errCode, ViChar errMessage[]);
		bool FetchWaveform(ViConstString channel, ViInt32 waveformSize, ViReal64 waveform[], ViInt32* actualPoints, ViReal64* initialX, ViReal64* xIncrement);
		bool FetchWaveformMeasurement(ViConstString channel, ViInt32 measFunction, ViReal64* measurement);
		bool GetChannelName(ViInt32 index, ViInt32 bufferSize, ViChar channelString[]);
		bool GetNextCoercionRecord(ViInt32 bufferSize, ViChar record[]);
		bool GetNextInterchangeWarning(ViInt32 bufferSize, ViChar interchangeWarning[]);
		bool IsInvalidWfmElement(ViReal64 elementValue, ViBoolean* isInvalid);
		bool ReadWaveform(ViConstString channel, ViInt32 waveformSize, ViInt32 maxtime, ViReal64 waveform[], ViInt32* actualPoints, ViReal64* initialX, ViReal64* xIncrement);
		bool ReadWaveformMeasurement(ViConstString channel, ViInt32 measFunction, ViInt32 maxTime, ViReal64* measurement);
		bool ResetInterchangeCheck();
		bool SendSWTrigger();
};

#endif;
