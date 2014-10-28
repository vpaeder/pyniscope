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

#include <iostream>
#include "niscopew.h"

niscopew::niscopew() {
	session = 0;
	sessionOpen = false;
}

niscopew::niscopew(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice) {
	session = 0;
	sessionOpen = false;
	Initialize(resourceName, IDQuery, resetDevice);
}

niscopew::niscopew(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViString optionString) {
	session = 0;
	sessionOpen = false;
	Initialize(resourceName, IDQuery, resetDevice, optionString);
}

niscopew::~niscopew() {
	/*
	Called upon deletion of the niscopew object.
	*/
	Close();
}

bool niscopew::HandleStatus(ViStatus status) {
	if (status < 0) {
		std::cout << "[ERROR]: " << status << std::endl;
		return false;
	}
	if (status > 0) {
		std::cout << "[WARNING]: " << status << std::endl;
	}
	return true;
}

bool niscopew::Initialize(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice) {
	/*
	Initialize IVI instrument driver session.
	*/
	if (!sessionOpen) sessionOpen = HandleStatus(niScope_init(resourceName, IDQuery, resetDevice, &session));
	return sessionOpen;
}

bool niscopew::Initialize(ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViString optionString) {
	/*
	Initialize IVI instrument driver session with options.
	*/
	if (!sessionOpen) sessionOpen = HandleStatus(niScope_InitWithOptions(resourceName, IDQuery, resetDevice, optionString, &session));
	return sessionOpen;
}

bool niscopew::Close() {
	/*
	Close IVI driver session and clean up.
	*/
	bool status = false;
	if (sessionOpen) status = HandleStatus(niScope_close(session));
	if (status) sessionOpen = false;
	return status;
}

bool niscopew::AutoSetup() {
	/*
	Automatically configures the instrument.
	*/
	if (sessionOpen) return HandleStatus(niScope_AutoSetup(session));
	return false;
}

bool niscopew::ConfigureAcquisition(ViInt32 acquisitionType) {
	/*
	Configures how the digitizer acquires data and fills the waveform record.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureAcquisition(session, acquisitionType));
	return false;
}

bool niscopew::ConfigureHorizontalTiming(ViReal64 minSampleRate, ViInt32 minNumPts, ViReal64 refPosition, ViInt32 numRecords, ViBoolean enforceRealtime){
	/*
	Configures the common properties of the horizontal subsystem for a multirecord acquisition in terms of minimum sample rate.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureHorizontalTiming(session, minSampleRate, minNumPts, refPosition, numRecords, enforceRealtime));
	return false;
}

bool niscopew::ConfigureChanCharacteristics(ViConstString channelList, ViReal64 inputImpedance, ViReal64 maxInputFrequency){
	/*
	Configures how the digitizer acquires data and fills the waveform record.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureChanCharacteristics(session, channelList, inputImpedance, maxInputFrequency));
	return false;
}

bool niscopew::ConfigureVertical(ViConstString channelList, ViReal64 range, ViReal64 offset, ViInt32 coupling, ViReal64 probeAttenuation, ViBoolean enabled){
	/*
	Configures the common properties of the horizontal subsystem for a multirecord acquisition in terms of minimum sample rate.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureVertical(session, channelList, range, offset, coupling, probeAttenuation, enabled));
	return false;
}

bool niscopew::ActualMeasWfmSize(ViInt32 arrayMeasFunction, ViInt32* measWfmSize){
	/*
	Returns the total available size of an array measurement acquisition.
	*/
	if (sessionOpen) return HandleStatus(niScope_ActualMeasWfmSize(session, arrayMeasFunction, measWfmSize));
	return false;
}

bool niscopew::ActualNumWfms(ViConstString channelList, ViInt32* numWfms){
	/*
	Helps you to declare appropriately sized waveforms. NI-SCOPE handles the channel list parsing for you.
    */
	if (sessionOpen) return HandleStatus(niScope_ActualNumWfms(session, channelList, numWfms));
	return false;
}

bool niscopew::ActualRecordLength(ViInt32* recordLength){
	/*
	Returns the actual number of points the digitizer acquires for each channel.
	*/
	if (sessionOpen) return HandleStatus(niScope_ActualRecordLength(session, recordLength));
	return false;
}

bool niscopew::SampleMode(ViInt32* sampleMode){
	/*
	Returns the sample mode the digitizer is currently using.
    */
	if (sessionOpen) return HandleStatus(niScope_SampleMode(session, sampleMode));
	return false;
}

bool niscopew::SampleRate(ViReal64* sampleRate){
	/*
	Returns the effective sample rate, in samples per second, of the acquired waveform using the current configuration.
	*/
	if (sessionOpen) return HandleStatus(niScope_SampleRate(session, sampleRate));
	return false;
}

bool niscopew::ConfigureTriggerDigital(ViConstString triggerSource, ViInt32 slope, ViReal64 holdoff, ViReal64 delay){
	/*
	Configures the common properties of a digital trigger.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerDigital(session, triggerSource, slope, holdoff, delay));
	return false;
}

bool niscopew::ConfigureTriggerEdge(ViConstString triggerSource, ViReal64 level, ViInt32 slope, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay){
	/*
	Configures common properties for analog edge triggering.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerEdge(session,triggerSource,level,slope,triggerCoupling,holdoff,delay));
	return false;
}

bool niscopew::ConfigureTriggerVideo(ViConstString triggerSource, ViBoolean enableDCRestore, ViInt32 signalFormat, ViInt32 evt, ViInt32 lineNumber, ViInt32 polarity, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay){
	/*
	Configures the common properties for video triggering, including the signal format, TV event, line number, polarity, and enable DC restore. A video trigger occurs when the digitizer finds a valid video signal sync.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerVideo(session,triggerSource,enableDCRestore,signalFormat,evt,lineNumber,polarity,triggerCoupling,holdoff,delay));
	return false;
}

bool niscopew::ConfigureTriggerHysteresis(ViConstString triggerSource, ViReal64 level, ViReal64 hysteresis, ViInt32 slope, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay){
	/*
	Configures common properties for analog hysteresis triggering.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerHysteresis(session,triggerSource,level,hysteresis,slope,triggerCoupling,holdoff,delay));
	return false;
}

bool niscopew::ConfigureTriggerImmediate(){
	/*
	Configures common properties for immediate triggering. Immediate triggering means the digitizer triggers itself.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerImmediate(session));
	return false;
}

bool niscopew::ConfigureTriggerSoftware(ViReal64 holdoff, ViReal64 delay){
	/*
	Configures common properties for software triggering.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerSoftware(session,holdoff,delay));
	return false;
}

bool niscopew::ConfigureTriggerWindow(ViConstString triggerSource, ViReal64 lowLevel, ViReal64 highLevel, ViInt32 windowMode, ViInt32 triggerCoupling, ViReal64 holdoff, ViReal64 delay){
	/*
	Configures common properties for analog window triggering. A window trigger occurs when a signal enters or leaves a window you specify with the high level or low level parameters.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerWindow(session,triggerSource,lowLevel,highLevel,windowMode,triggerCoupling,holdoff,delay));
	return false;
}

bool niscopew::SendSoftwareTriggerEdge(ViInt32 whichTrigger){
	/*
	Sends the selected trigger to the digitizer.
	*/
	if (sessionOpen) return HandleStatus(niScope_SendSoftwareTriggerEdge(session,whichTrigger));
	return false;
}

bool niscopew::AdjustSampleClockRelativeDelay(ViReal64 delay){
	/*
	Configures the relative sample clock delay. Each time this function is called, the sample clock is delayed by the specified amount of time.
    */
	if (sessionOpen) return HandleStatus(niScope_AdjustSampleClockRelativeDelay(session,delay));
	return false;
}

bool niscopew::ConfigureClock(ViConstString inputClockSource, ViConstString outputClockSource, ViConstString clockSyncPulseSource, ViBoolean masterEnabled){
	/*
	Configures the attributes for synchronizing the digitizer to a reference or sending the digitizer's reference clock output to be used as a synchronizing clock for other digitizers.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureClock(session,inputClockSource,outputClockSource,clockSyncPulseSource,masterEnabled));
	return false;
}

bool niscopew::ExportSignal(ViInt32 signal, ViConstString signalIdentifier, ViConstString outputTerminal){
	/*
	Configures the digitizer to generate a signal that other devices can detect when configured for digital triggering or sharing clocks.
	*/
	if (sessionOpen) return HandleStatus(niScope_ExportSignal(session,signal,signalIdentifier,outputTerminal));
	return false;
}

bool niscopew::GetFrequencyResponse(ViConstString channelName, ViInt32 bufferSize, ViReal64 frequencies[], ViReal64 amplitudes[], ViReal64 phases[], ViInt32* numberOfFrequencies){
	/*
	Gets the frequency response of the digitizer for the current configurations of the channel attributes.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetFrequencyResponse(session,channelName,bufferSize,frequencies,amplitudes,phases,numberOfFrequencies));
	return false;
}
int niscopew::GetFrequencyResponseBufSize(ViConstString channelName){
	/*
	Gets the number of elements returned by GetFrequencyResponse.
	*/
	ViInt32 numberOfFrequencies;
	if (sessionOpen) return (int)niScope_GetFrequencyResponse(session, channelName, 0, VI_NULL, VI_NULL, VI_NULL, &numberOfFrequencies);
	return 0;
}

bool niscopew::ConfigureEqualizationFilterCoefficients(ViConstString channel, ViInt32 numberOfCoefficients, ViReal64* coefficients){
	/*
	Configures the custom coefficients for the equalization FIR filter on the device.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureEqualizationFilterCoefficients(session,channel,numberOfCoefficients,coefficients));
	return false;
}

bool niscopew::SetAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32 value) {
	/*
	Sets the value of a ViInt32 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViInt32(session, channelList, attributeID, value));
	return false;
}

bool niscopew::SetAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64 value) {
	/*
	Sets the value of a ViInt64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViInt64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::SetAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64 value) {
	/*
	Sets the value of a ViReal64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViReal64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::SetAttributeViString(ViConstString channelList, ViAttr attributeID, ViConstString value) {
	/*
	Sets the value of a ViString attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViString(session, channelList, attributeID, value));
	return false;
}

bool niscopew::SetAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean value) {
	/*
	Sets the value of a ViBoolean attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViBoolean(session, channelList, attributeID, value));
	return false;
}

bool niscopew::SetAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession value) {
	/*
	Sets the value of a ViSession attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_SetAttributeViSession(session, channelList, attributeID, value));
	return false;
}

bool niscopew::GetAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32* value) {
	/*
	Gets the value of a ViInt32 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViInt32(session, channelList, attributeID, value));
	return false;
}

bool niscopew::GetAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64* value) {
	/*
	Gets the value of a ViInt64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViInt64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::GetAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64* value) {
	/*
	Gets the value of a ViReal64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViReal64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::GetAttributeViString(ViConstString channelList, ViAttr attributeID, ViInt32 bufSize, ViChar value[]) {
	/*
	Gets the value of a ViString attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViString(session, channelList, attributeID, bufSize, value));
	return false;
}

int niscopew::GetAttributeViStringBufSize(ViConstString channelList, ViAttr attributeID) {
	/*
	Gets the size of ViString attribute returned by GetAttributeViString.
	*/
	if (sessionOpen) return (int)niScope_GetAttributeViString(session, channelList, attributeID, 0, VI_NULL);
	return 0;
}

bool niscopew::GetAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean* value) {
	/*
	Gets the value of a ViBoolean attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViBoolean(session, channelList, attributeID, value));
	return false;
}

bool niscopew::GetAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession* value) {
	/*
	Gets the value of a ViSession attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetAttributeViSession(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViInt32(ViConstString channelList, ViAttr attributeID, ViInt32 value) {
	/*
	Verifies the validity of a value you specify for a ViInt32 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViInt32(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViInt64(ViConstString channelList, ViAttr attributeID, ViInt64 value) {
	/*
	Verifies the validity of a value you specify for a ViInt64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViInt64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViReal64(ViConstString channelList, ViAttr attributeID, ViReal64 value) {
	/*
	Verifies the validity of a value you specify for a ViReal64 attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViReal64(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViString(ViConstString channelList, ViAttr attributeID, ViConstString value) {
	/*
	Verifies the validity of a value you specify for a ViString attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViString(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViBoolean(ViConstString channelList, ViAttr attributeID, ViBoolean value) {
	/*
	Verifies the validity of a value you specify for a ViBoolean attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViBoolean(session, channelList, attributeID, value));
	return false;
}

bool niscopew::CheckAttributeViSession(ViConstString channelList, ViAttr attributeID, ViSession value) {
	/*
	Verifies the validity of a value you specify for a ViSession attribute.
	*/
	if (sessionOpen) return HandleStatus(niScope_CheckAttributeViSession(session, channelList, attributeID, value));
	return false;
}

bool niscopew::Abort(){
	/*
	Aborts an acquisition and returns the digitizer to the Idle state. Call this function if the digitizer times out waiting for a trigger.
	*/
	if (sessionOpen) return HandleStatus(niScope_Abort(session));
	return false;
}

bool niscopew::AcquisitionStatus(ViInt32* status){
	/*
	Returns status information about the acquisition to the status output parameter.
	*/
	if (sessionOpen) return HandleStatus(niScope_AcquisitionStatus(session,status));
	return false;
}

bool niscopew::Commit(){
	/*
	Commits to hardware all the parameter settings associated with the task. Use this function if you want a parameter change to be immediately reflected in the hardware. This function is supported for the NI 5122/5124 only.
	*/
	if (sessionOpen) return HandleStatus(niScope_Commit(session));
	return false;
}

bool niscopew::Fetch(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViReal64* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Returns the waveform from a previously initiated acquisition that the digitizer acquires for the specified channel. This function returns scaled voltage waveforms.
	*/
	if (sessionOpen) return HandleStatus(niScope_Fetch(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::FetchComplex(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, NIComplexNumber* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Retrieves data that the digitizer has acquired from a previously initiated acquisition and returns a one-dimensional array of complex, scaled waveforms.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchComplex(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::FetchComplexBinary16(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, NIComplexI16* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Retrieves data from single channels and records. Returns a one-dimensional array of complex binary 16-bit waveforms.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchComplexBinary16(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::FetchBinary8(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt8* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 8-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchBinary8(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::FetchBinary16(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt16* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 16-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchBinary16(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::FetchBinary32(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViInt32* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 32-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchBinary32(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::GetNormalizationCoefficients(ViConstString channelList, ViInt32 bufferSize, struct niScope_coefficientInfo coefficientInfo[], ViInt32* numberOfCoefficientSets){
	/*
	Returns coefficients that can be used to convert binary data to normalized and calibrated data.
    */
	if (sessionOpen) return HandleStatus(niScope_GetNormalizationCoefficients(session,channelList,bufferSize,coefficientInfo,numberOfCoefficientSets));
	return false;
}

int niscopew::GetNormalizationCoefficientsBufSize(ViConstString channelList){
	/*
	Returns buffer size for GetNormalizationCoefficients function.
    */
	ViInt32 numberOfCoefficientSets;
	if (sessionOpen) return niScope_GetNormalizationCoefficients(session,channelList,0,NULL,&numberOfCoefficientSets);
	return 0;
}


bool niscopew::GetScalingCoefficients(ViConstString channelList, ViInt32 bufferSize, struct niScope_coefficientInfo coefficientInfo[], ViInt32* numberOfCoefficientSets){
	/*
	Returns coefficients that can be used to scale binary data to volts.
    */
	if (sessionOpen) return HandleStatus(niScope_GetScalingCoefficients(session,channelList,bufferSize,coefficientInfo,numberOfCoefficientSets));
	return false;
}

int niscopew::GetScalingCoefficientsBufSize(ViConstString channelList){
	/*
	Returns buffer size for GetScalingCoefficients function.
    */
	ViInt32 numberOfCoefficientSets;
	if (sessionOpen) return niScope_GetScalingCoefficients(session,channelList,0,NULL,&numberOfCoefficientSets);
	return 0;
}

bool niscopew::InitiateAcquisition(){
	/*
	Initiates a waveform acquisition.
    */
	if (sessionOpen) return HandleStatus(niScope_InitiateAcquisition(session));
	return false;
}

bool niscopew::Read(ViConstString channelList, ViReal64 timeout, ViInt32 numSamples, ViReal64* wfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Initiates an acquisition, waits for it to complete, and retrieves the data. The process is similar to calling niScope_InitiateAcquisition, niScope_AcquisitionStatus, and niScope_Fetch. The only difference is that with niScope_Read, you enable all channels specified with channelList before the acquisition; in the other method, you enable the channels with niScope_ConfigureVertical.
    */
	if (sessionOpen) return HandleStatus(niScope_Read(session,channelList,timeout,numSamples,wfm,wfmInfo));
	return false;
}

bool niscopew::AddWaveformProcessing(ViConstString channelList, ViInt32 measFunction){
	/*
	Adds one measurement to the list of processing steps that are completed before the measurement.
	*/
	if (sessionOpen) return HandleStatus(niScope_AddWaveformProcessing(session,channelList,measFunction));
	return false;
}

bool niscopew::ClearWaveformMeasurementStats(ViConstString channelList, ViInt32 clearableMeasurementFunction){
	/*
	Clears the waveform stats on the channel and measurement you specify.
	*/
	if (sessionOpen) return HandleStatus(niScope_ClearWaveformMeasurementStats(session, channelList, clearableMeasurementFunction));
	return false;
}

bool niscopew::ClearWaveformProcessing(ViConstString channelList){
	/*
	Clears the list of processing steps assigned to the given channel.
	*/
	if (sessionOpen) return HandleStatus(niScope_ClearWaveformProcessing(session,channelList));
	return false;
}

bool niscopew::FetchArrayMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 arrayMeasFunction, ViInt32 measWfmSize, ViReal64* measWfm, struct niScope_wfmInfo* wfmInfo){
	/*
	Obtains a waveform from the digitizer and returns the specified measurement array.
	*/
	if (sessionOpen) return HandleStatus(niScope_FetchArrayMeasurement(session,channelList,timeout,arrayMeasFunction,measWfmSize,measWfm,wfmInfo));
	return false;
}

bool niscopew::FetchMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result){
	/*
	Fetches a waveform from the digitizer and performs the specified waveform measurement. Refer to Using Fetch Functions for more information.
    */
	if (sessionOpen) return HandleStatus(niScope_FetchMeasurement(session,channelList,timeout,scalarMeasFunction,result));
	return false;
}

bool niscopew::FetchMeasurementStats(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result, ViReal64* mean, ViReal64* stdev, ViReal64* min, ViReal64* max, ViInt32*numInStats){
	/*
	Obtains a waveform measurement and returns the measurement value. This function may return multiple statistical results depending on the number of channels, the acquisition type, and the number of records you specify.
	*/
	if (sessionOpen) return HandleStatus(niScope_FetchMeasurementStats(session,channelList,timeout,scalarMeasFunction,result,mean,stdev,min,max,numInStats));
	return false;
}

bool niscopew::ReadMeasurement(ViConstString channelList, ViReal64 timeout, ViInt32 scalarMeasFunction, ViReal64* result){
	/*
	Initiates an acquisition, waits for it to complete, and performs the specified waveform measurement for a single channel and record or for multiple channels and records.
    */
	if (sessionOpen) return HandleStatus(niScope_ReadMeasurement(session,channelList,timeout,scalarMeasFunction,result));
	return false;
}

bool niscopew::CalSelfCalibrate(ViConstString channelList, ViInt32 option){
	/*
	Self-calibrates the digitizer.
	*/
	if (sessionOpen) return HandleStatus(niScope_CalSelfCalibrate(session,channelList,option));
	return false;
}

bool niscopew::ResetDevice(){
	/*
	Performs a hard reset of the device.
	*/
	if (sessionOpen) return HandleStatus(niScope_ResetDevice(session));
	return false;
}

bool niscopew::Disable(){
	/*
	Aborts any current operation, opens data channel relays, and releases RTSI and PFI lines.
    */
	if (sessionOpen) return HandleStatus(niScope_Disable(session));
	return false;
}

bool niscopew::ProbeCompensationSignalStart(){
	/*
	Starts the 1 kHz square wave output on PFI 1 for probe compensation.
    */
	if (sessionOpen) return HandleStatus(niScope_ProbeCompensationSignalStart(session));
	return false;
}

bool niscopew::ProbeCompensationSignalStop(){
	/*
	Stops the 1 kHz square wave output on PFI 1 for probe compensation.
    */
	if (sessionOpen) return HandleStatus(niScope_ProbeCompensationSignalStop(session));
	return false;
}

bool niscopew::IsDeviceReady(ViRsrc resourceName, ViConstString channelList, ViBoolean* deviceReady){
	/*
	Returns the revision numbers of the instrument driver and instrument firmware.
    */
	return HandleStatus(niScope_IsDeviceReady(resourceName,channelList,deviceReady));
}

bool niscopew::Reset(){
	/*
	Resets the digitizer to its default state.
	*/
	if (sessionOpen) return HandleStatus(niScope_reset(session));
	return false;
}

bool niscopew::ResetWithDefaults(){
	/*
	Performs a software reset of the device, returning it to the default state and applying any initial default settings from the IVI Configuration Store.
    */
	if (sessionOpen) return HandleStatus(niScope_ResetWithDefaults(session));
	return false;
}

bool niscopew::RevisionQuery(ViChar driverRev[IVI_MAX_MESSAGE_BUF_SIZE], ViChar instrRev[IVI_MAX_MESSAGE_BUF_SIZE]){
	/*
	Returns the revision numbers of the instrument driver and instrument firmware.
    */
	if (sessionOpen) return HandleStatus(niScope_revision_query(session, driverRev, instrRev));
	return false;
}

bool niscopew::SelfTest(ViInt16* selfTestResult, ViChar selfTestMessage[IVI_MAX_MESSAGE_BUF_SIZE]){
	/*
	Runs the instrument self-test routine and returns the test result(s).
    */
	if (sessionOpen) return HandleStatus(niScope_self_test(session,selfTestResult,selfTestMessage));
	return false;
}

bool niscopew::GetStreamEndpointHandle(ViConstString streamName, ViUInt32 *writerHandle){
	/*
	Returns a writer endpoint that can be used with NI-P2P to configure a peer-to-peer stream with a digitizer endpoint.
    */
	if (sessionOpen) return HandleStatus(niScope_GetStreamEndpointHandle(session, streamName, writerHandle));
	return false;
}

bool niscopew::ErrorHandler(ViInt32 errorCode, ViChar errorSource[MAX_FUNCTION_NAME_SIZE], ViChar errorDescription[MAX_ERROR_DESCRIPTION]){
	/*
	Takes the error code returned by NI-SCOPE functions and returns the interpretation as a user-readable string.
    */
	if (sessionOpen) return HandleStatus(niScope_errorHandler(session,errorCode,errorSource,errorDescription));
	return false;
}

bool niscopew::GetError(ViStatus* errorCode, ViInt32 bufferSize, ViChar description[]){
	/*
	Reads an error code and message from the error queue. National Instruments digitizers do not contain an error queue. Errors are reported as they occur. Therefore, this function does not detect errors.
    */
	if (sessionOpen) return HandleStatus(niScope_GetError(session,errorCode,bufferSize,description));
	return false;
}

int niscopew::GetErrorBufSize(ViStatus* errorCode) {
	/*
	Returns buffer size for GetError function.
	*/
	if (sessionOpen) return niScope_GetError(session,errorCode,0,VI_NULL);
	return 0;
}

bool niscopew::GetErrorMessage(ViStatus errorCode, ViInt32 bufferSize, ViChar errorMessage[]){
	/*
	Returns the error code from an NI-SCOPE function as a user-readable string.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetErrorMessage(session,errorCode,bufferSize,errorMessage));
	return false;
}

int niscopew::GetErrorMessageBufSize(ViStatus errorCode) {
	/*
	Returns buffer size for GetErrorMessage function.
	*/
	if (sessionOpen) return niScope_GetErrorMessage(session,errorCode,0,VI_NULL);
	return 0;
}

bool niscopew::LockSession(ViBoolean* callerHasLock){
	/*
	Obtains a multithread lock on the instrument session.
	*/
	if (sessionOpen) return HandleStatus(niScope_LockSession(session,callerHasLock));
	return false;
}

bool niscopew::UnlockSession(ViBoolean* callerHasLock){
	/*
	Releases a lock that you acquired on an instrument session using niScope LockSession.
    */
	if (sessionOpen) return HandleStatus(niScope_UnlockSession(session,callerHasLock));
	return false;
}

bool niscopew::ClearError(){
	/*
	Clears the error information for the current execution thread and the IVI session you specify.
	*/
	if (sessionOpen) return HandleStatus(niScope_ClearError(session));
	return false;
}

bool niscopew::ClearInterchangeWarnings(){
	/*
	Clears the list of current interchange warnings.
    */
	if (sessionOpen) return HandleStatus(niScope_ClearInterchangeWarnings(session));
	return false;
}

bool niscopew::ConfigureAcquisitionRecord(ViReal64 timeperRecord, ViInt32 minNumPoints, ViReal64 acquisitionStartTime){
	/*
	Configures the most commonly configured attributes of the instrument acquisition subsystem.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureAcquisitionRecord(session,timeperRecord,minNumPoints,acquisitionStartTime));
	return false;
}

bool niscopew::ConfigureChannel(ViConstString channel, ViReal64 range, ViReal64 offset, ViInt32 coupling, ViReal64 probeAttenuation, ViBoolean enabled){
	/*
	Configures the most commonly configured attributes of the instrument's channel subsystem.
    */
	if (sessionOpen) return HandleStatus(niScope_ConfigureChannel(session,channel,range,offset,coupling,probeAttenuation,enabled));
	return false;
}

bool niscopew::ConfigureEdgeTriggerSource(ViConstString source, ViReal64 level, ViInt32 slope){
	/*
	Sets the edge triggering attributes.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureEdgeTriggerSource(session,source,level,slope));
	return false;
}

bool niscopew::ConfigureRefLevels(ViReal64 low, ViReal64 mid, ViReal64 high){
	/*
	Configures the reference levels for all channels of the digitizer.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureRefLevels(session,low,mid,high));
	return false;
}

bool niscopew::ConfigureTrigger(ViInt32 triggerType, ViReal64 holdoff){
	/*
	Configures the common attributes of the trigger subsystem.
    */
	if (sessionOpen) return HandleStatus(niScope_ConfigureTrigger(session,triggerType,holdoff));
	return false;
}

bool niscopew::ConfigureTriggerCoupling(ViInt32 coupling){
	/*
	Sets the trigger coupling attribute.
    */
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerCoupling(session,coupling));
	return false;
}

bool niscopew::ConfigureTriggerOutput(ViInt32 triggerEvent, ViConstString triggerOutput){
	/*
	Configures the digitizer to generate a signal pulse that other digitizers can detect when configured for digital triggering.
    */
	if (sessionOpen) return HandleStatus(niScope_ConfigureTriggerOutput(session,triggerEvent,triggerOutput));
	return false;
}

bool niscopew::ConfigureTVTriggerLineNumber(ViInt32 lineNumber){
	/*
	Configures the TV line upon which the instrument triggers.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTVTriggerLineNumber(session,lineNumber));
	return false;
}

bool niscopew::ConfigureTVTriggerSource(ViConstString source, ViInt32 signalFormat, ViInt32 evt, ViInt32 polarity){
	/*
	Configures the instrument for TV triggering.
	*/
	if (sessionOpen) return HandleStatus(niScope_ConfigureTVTriggerSource(session,source,signalFormat,evt,polarity));
	return false;
}

bool niscopew::ErrorQuery(ViInt32* errCode, ViChar errMessage[]){
	/*
	Reads an error code and message from the error queue. National Instruments digitizers do not contain an error queue. Errors are reported as they occur. Therefore, this function does not detect errors.
	*/
	if (sessionOpen) return HandleStatus(niScope_error_query(session,errCode,errMessage));
	return false;
}

bool niscopew::FetchWaveform(ViConstString channel, ViInt32 waveformSize, ViReal64 waveform[], ViInt32* actualPoints, ViReal64* initialX, ViReal64* xIncrement){
	/*
	Returns the waveform from a previously initiated acquisition that the digitizer acquires for the channel you specify.
	*/
	if (sessionOpen) return HandleStatus(niScope_FetchWaveform(session,channel,waveformSize,waveform,actualPoints,initialX,xIncrement));
	return false;
}

bool niscopew::FetchWaveformMeasurement(ViConstString channel, ViInt32 measFunction, ViReal64* measurement){
	/*
	Fetches a waveform measurement from a specific channel from a previously initiated waveform acquisition.
	*/
	if (sessionOpen) return HandleStatus(niScope_FetchWaveformMeasurement(session,channel,measFunction,measurement));
	return false;
}

bool niscopew::GetChannelName(ViInt32 index, ViInt32 bufferSize, ViChar channelString[]){
	/*
	Returns the channel string that is in the channel table at an index you specify. Not applicable to National Instruments digitizers.
    */
	if (sessionOpen) return HandleStatus(niScope_GetChannelName(session, index, bufferSize, channelString));
	return false;
}

bool niscopew::GetNextCoercionRecord(ViInt32 bufferSize, ViChar record[]){
	/*
	Returns the coercion information associated with the IVI session.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetNextCoercionRecord(session,bufferSize,record));
	return false;
}

bool niscopew::GetNextInterchangeWarning(ViInt32 bufferSize, ViChar interchangeWarning[]){
	/*
	Returns the interchangeability warnings associated with the IVI session.
	*/
	if (sessionOpen) return HandleStatus(niScope_GetNextInterchangeWarning(session,bufferSize,interchangeWarning));
	return false;
}

bool niscopew::IsInvalidWfmElement(ViReal64 elementValue, ViBoolean* isInvalid){
	/*
	Determines whether a value you pass from the waveform array is invalid.
	*/
	if (sessionOpen) return HandleStatus(niScope_IsInvalidWfmElement(session,elementValue,isInvalid));
	return false;
}

bool niscopew::ReadWaveform(ViConstString channel, ViInt32 waveformSize, ViInt32 maxtime, ViReal64 waveform[], ViInt32* actualPoints, ViReal64* initialX, ViReal64* xIncrement){
	/*
	Initiates an acquisition on the channels that you enable with ConfigureVertical.
	*/
	if (sessionOpen) return HandleStatus(niScope_ReadWaveform(session,channel,waveformSize,maxtime,waveform,actualPoints,initialX,xIncrement));
	return false;
}

bool niscopew::ReadWaveformMeasurement(ViConstString channel, ViInt32 measFunction, ViInt32 maxTime, ViReal64* measurement){
	/*
	Initiates a new waveform acquisition and returns a specified waveform measurement from a specific channel.
    */
	if (sessionOpen) return HandleStatus(niScope_ReadWaveformMeasurement(session,channel,measFunction,maxTime,measurement));
	return false;
}

bool niscopew::ResetInterchangeCheck(){
	/*
	 When developing a complex test system that consists of multiple test modules, it is generally a good idea to design the test modules so that they can run in any order.
	*/
	if (sessionOpen) return HandleStatus(niScope_ResetInterchangeCheck(session));
	return false;
}

bool niscopew::SendSWTrigger(){
	/*
	Sends a command to trigger the digitizer. Call this function after you call ConfigureTriggerSoftware.
    */
	if (sessionOpen) return HandleStatus(niScope_SendSWTrigger(session));
	return false;
}

