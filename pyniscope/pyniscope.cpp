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

#include "pyniscope.h"

pyniscope::pyniscope() {}

/*
pyniscope::pyniscope(std::string resourceName, bool IDQuery, bool resetDevice) {
	Initialize(resourceName, IDQuery, resetDevice);
}

pyniscope::pyniscope(std::string resourceName, bool IDQuery, bool resetDevice, std::string optionString) {
	Initialize(resourceName, IDQuery, resetDevice, optionString);
}
*/
pyniscope::~pyniscope() {
	/*
	Called upon deletion of the pyniscope object.
	*/
	Close();
}

char* pyniscope::ToChar(std::string strng) {
	/*
	Convert std::string to char array (for conversion from python string to c string).
	*/
	int stl = strlen(strng.c_str()) + 1;
	char* chr = new char[stl];
	memcpy(chr, strng.c_str(), stl);
	return chr;
}

list pyniscope::WfmInfoExtract(struct niScope_wfmInfo wfmInfo) {
	/*
	Convert struct niScope_wfmInfo to python list.
	*/
	list winfos;
	winfos.append(wfmInfo.absoluteInitialX);
	winfos.append(wfmInfo.relativeInitialX);
	winfos.append(wfmInfo.xIncrement);
	winfos.append(wfmInfo.actualSamples);
	winfos.append(wfmInfo.gain);
	winfos.append(wfmInfo.offset);
	return winfos;
}

bool pyniscope::Initialize(std::string resourceName, bool IDQuery, bool resetDevice) {
	/*
	Initialize IVI instrument driver session.
	*/
	return scope.Initialize(ToChar(resourceName), (ViBoolean)IDQuery, (ViBoolean)resetDevice);
}
/*
bool pyniscope::Initialize(std::string resourceName, bool IDQuery, bool resetDevice, std::string optionString) {
	return scope.Initialize(ToChar(resourceName), (ViBoolean)IDQuery, (ViBoolean)resetDevice, ToChar(optionString));
}
*/
bool pyniscope::Close() {
	/*
	Close IVI driver session and clean up.
	*/
	return scope.Close();
}

bool pyniscope::AutoSetup() {
	/*
	Automatically configures the instrument.
	*/
	return scope.AutoSetup();
}

bool pyniscope::ConfigureAcquisition(int acquisitionType){
	/*
	Configures how the digitizer acquires data and fills the waveform record.
	*/
	return scope.ConfigureAcquisition((ViInt32)acquisitionType);
}

bool pyniscope::ConfigureHorizontalTiming(double minSampleRate, int minNumPts, double refPosition, int numRecords, bool enforceRealtime){
	/*
	Configures the common properties of the horizontal subsystem for a multirecord acquisition in terms of minimum sample rate.
	*/
	return scope.ConfigureHorizontalTiming((ViReal64)minSampleRate, (ViInt32)minNumPts, (ViReal64)refPosition, (ViInt32)numRecords, (ViBoolean)enforceRealtime);
}

bool pyniscope::ConfigureChanCharacteristics(std::string channelList, double inputImpedance, double maxInputFrequency){
	/*
	Configures how the digitizer acquires data and fills the waveform record.
	*/
	return scope.ConfigureChanCharacteristics(ToChar(channelList), (ViReal64)inputImpedance, (ViReal64)maxInputFrequency);
}

bool pyniscope::ConfigureVertical(std::string channelList, double range, double offset, int coupling, double probeAttenuation, bool enabled){
	/*
	Configures the common properties of the horizontal subsystem for a multirecord acquisition in terms of minimum sample rate.
	*/
	return scope.ConfigureVertical(ToChar(channelList), (ViReal64)range, (ViReal64)offset, (ViInt32)coupling, (ViReal64)probeAttenuation, (ViBoolean)enabled);
}

int pyniscope::ActualMeasWfmSize(int arrayMeasFunction){
	/*
	Returns the total available size of an array measurement acquisition.
	*/
	ViInt32 measWfmSize;
	if (scope.ActualMeasWfmSize((ViInt32)arrayMeasFunction, &measWfmSize)) return (int)measWfmSize;
	return 0;
}

int pyniscope::ActualNumWfms(std::string channelList){
	/*
	Helps you to declare appropriately sized waveforms. NI-SCOPE handles the channel list parsing for you.
    */
	ViInt32 numWfms;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) return (int)numWfms;
	return 0;
}

int pyniscope::ActualRecordLength(){
	/*
	Returns the actual number of points the digitizer acquires for each channel.
	*/
	ViInt32 recordLength;
	if (scope.ActualRecordLength(&recordLength)) return (int)recordLength;
	return 0;
}

int pyniscope::SampleMode(){
	/*
	Returns the sample mode the digitizer is currently using.
    */
	ViInt32 sampleMode;
	if (scope.SampleMode(&sampleMode)) return (int)sampleMode;
	return 0;
}

double pyniscope::SampleRate(){
	/*
	Returns the effective sample rate, in samples per second, of the acquired waveform using the current configuration.
	*/
	ViReal64 sampleRate;
	if (scope.SampleRate(&sampleRate)) return (double)sampleRate;
	return 0.0;
}

bool pyniscope::ConfigureTriggerDigital(std::string triggerSource, int slope, double holdoff, double delay){
	/*
	Configures the common properties of a digital trigger.
	*/
	return scope.ConfigureTriggerDigital(ToChar(triggerSource), (ViInt32)slope, (ViReal64)holdoff, (ViReal64)delay);
}

bool pyniscope::ConfigureTriggerEdge(std::string triggerSource, double level, int slope, int triggerCoupling, double holdoff, double delay){
	/*
	Configures common properties for analog edge triggering.
	*/
	return scope.ConfigureTriggerEdge(ToChar(triggerSource), (ViReal64)level, (ViInt32)slope, (ViInt32)triggerCoupling, (ViReal64)holdoff, (ViReal64)delay);

}

bool pyniscope::ConfigureTriggerVideo(std::string triggerSource, bool enableDCRestore, int signalFormat, int evt, int lineNumber, int polarity, int triggerCoupling, double holdoff, double delay){
	/*
	Configures the common properties for video triggering, including the signal format, TV event, line number, polarity, and enable DC restore. A video trigger occurs when the digitizer finds a valid video signal sync.
	*/
	return scope.ConfigureTriggerVideo(ToChar(triggerSource), (ViBoolean)enableDCRestore, (ViInt32)signalFormat, (ViInt32)evt, (ViInt32)lineNumber, (ViInt32)polarity, (ViInt32)triggerCoupling, (ViReal64)holdoff, (ViReal64)delay);
}

bool pyniscope::ConfigureTriggerHysteresis(std::string triggerSource, double level, double hysteresis, int slope, int triggerCoupling, double holdoff, double delay){
	/*
	Configures common properties for analog hysteresis triggering.
	*/
	return scope.ConfigureTriggerHysteresis(ToChar(triggerSource), (ViReal64)level, (ViReal64)hysteresis, (ViInt32)slope, (ViInt32)triggerCoupling, (ViReal64)holdoff, (ViReal64)delay);
}

bool pyniscope::ConfigureTriggerImmediate(){
	/*
	Configures common properties for immediate triggering. Immediate triggering means the digitizer triggers itself.
	*/
	return scope.ConfigureTriggerImmediate();
}

bool pyniscope::ConfigureTriggerSoftware(double holdoff, double delay){
	/*
	Configures common properties for software triggering.
	*/
	return scope.ConfigureTriggerSoftware((ViReal64)holdoff, (ViReal64)delay);
}

bool pyniscope::ConfigureTriggerWindow(std::string triggerSource, double lowLevel, double highLevel, int windowMode, int triggerCoupling, double holdoff, double delay){
	/*
	Configures common properties for analog window triggering. A window trigger occurs when a signal enters or leaves a window you specify with the high level or low level parameters.
	*/
	return scope.ConfigureTriggerWindow(ToChar(triggerSource), (ViReal64)lowLevel, (ViReal64)highLevel, (ViInt32)windowMode, (ViInt32)triggerCoupling, (ViReal64)holdoff, (ViReal64)delay);
}

bool pyniscope::SendSoftwareTriggerEdge(int whichTrigger){
	/*
	Sends the selected trigger to the digitizer.
	*/
	return scope.SendSoftwareTriggerEdge((ViInt32)whichTrigger);
}

bool pyniscope::AdjustSampleClockRelativeDelay(double delay){
	/*
	Configures the relative sample clock delay. Each time this function is called, the sample clock is delayed by the specified amount of time.
    */
	return scope.AdjustSampleClockRelativeDelay((ViReal64)delay);
}

bool pyniscope::ConfigureClock(std::string inputClockSource, std::string outputClockSource, std::string clockSyncPulseSource, bool masterEnabled){
	/*
	Configures the attributes for synchronizing the digitizer to a reference or sending the digitizer's reference clock output to be used as a synchronizing clock for other digitizers.
	*/
	return scope.ConfigureClock(ToChar(inputClockSource), ToChar(outputClockSource), ToChar(clockSyncPulseSource), (ViBoolean)masterEnabled);
}

bool pyniscope::ExportSignal(int signal, std::string signalIdentifier, std::string outputTerminal){
	/*
	Configures the digitizer to generate a signal that other devices can detect when configured for digital triggering or sharing clocks.
	*/
	return scope.ExportSignal((ViInt32)signal, ToChar(signalIdentifier), ToChar(outputTerminal));
}

list pyniscope::GetFrequencyResponse(std::string channelName){
	/*
	Gets the frequency response of the digitizer for the current configurations of the channel attributes.
	*/
	list retval;
	ViInt32 numberOfFrequencies;
	ViInt32 bufferSize;
	bufferSize = scope.GetFrequencyResponseBufSize(ToChar(channelName));
	if (bufferSize > 0) {
		ViReal64* frequencies = new ViReal64[bufferSize];
		ViReal64* amplitudes = new ViReal64[bufferSize];
		ViReal64* phases = new ViReal64[bufferSize];
		if (scope.GetFrequencyResponse(ToChar(channelName), bufferSize, frequencies, amplitudes, phases, &numberOfFrequencies)) {
			list lFrequencies;
			list lAmplitudes;
			list lPhases;
			for (int i = 0; i < bufferSize; i++) {
				lFrequencies.append(frequencies[i]);
				lAmplitudes.append(amplitudes[i]);
				lPhases.append(phases[i]);
			}
			retval.append(lFrequencies);
			retval.append(lAmplitudes);
			retval.append(lPhases);
			retval.append(numberOfFrequencies);
		}
	}
	return retval;
}

bool pyniscope::ConfigureEqualizationFilterCoefficients(std::string channel, list coefficients){
	/*
	Configures the custom coefficients for the equalization FIR filter on the device.
	*/
	ViInt32 numberOfCoefficients = len(coefficients);
	ViReal64* data = new ViReal64[numberOfCoefficients];
	for (int i = 0; i < numberOfCoefficients; i++) {
		data[i] = (ViReal64)extract<double>(coefficients[i]);
	}
	return scope.ConfigureEqualizationFilterCoefficients(ToChar(channel), numberOfCoefficients, data);
}


bool pyniscope::SetAttributeViInt32(std::string channelList, int attributeID, int value){
	/*
	Sets the value of a ViInt32 attribute.
	*/
	return scope.SetAttributeViInt32(ToChar(channelList), (ViAttr)attributeID, (ViInt32)value);
}

bool pyniscope::SetAttributeViInt64(std::string channelList, int attributeID, int value){
	/*
	Sets the value of a ViInt64 attribute.
	*/
	return scope.SetAttributeViInt64(ToChar(channelList), (ViAttr)attributeID, (ViInt64)value);
}

bool pyniscope::SetAttributeViReal64(std::string channelList, int attributeID, double value){
	/*
	Sets the value of a ViReal64 attribute.
	*/
	return scope.SetAttributeViReal64(ToChar(channelList), (ViAttr)attributeID, (ViReal64)value);
}

bool pyniscope::SetAttributeViString(std::string channelList, int attributeID, std::string value){
	/*
	Sets the value of a ViString attribute.
	*/
	return scope.SetAttributeViString(ToChar(channelList), (ViAttr)attributeID, ToChar(value));
}

bool pyniscope::SetAttributeViBoolean(std::string channelList, int attributeID, bool value){
	/*
	Sets the value of a ViBoolean attribute.
	*/
	return scope.SetAttributeViBoolean(ToChar(channelList), (ViAttr)attributeID, (ViBoolean)value);
}

bool pyniscope::SetAttributeViSession(std::string channelList, int attributeID, int value){
	/*
	Sets the value of a ViSession attribute.
	*/
	return scope.SetAttributeViSession(ToChar(channelList), (ViAttr)attributeID, (ViInt32)value);
}

int pyniscope::GetAttributeViInt32(std::string channelList, int attributeID){
	/*
	Gets the value of a ViInt32 attribute.
	*/
	ViInt32 value;
	if (scope.GetAttributeViInt32(ToChar(channelList), (ViAttr)attributeID, &value)) return (int)value;
	return 0;
}

int pyniscope::GetAttributeViInt64(std::string channelList, int attributeID){
	/*
	Gets the value of a ViInt64 attribute.
	*/
	ViInt64 value;
	if (scope.GetAttributeViInt64(ToChar(channelList), (ViAttr)attributeID, &value)) return (int)value;
	return 0;
}

double pyniscope::GetAttributeViReal64(std::string channelList, int attributeID){
	/*
	Gets the value of a ViReal64 attribute.
	*/
	ViReal64 value;
	if (scope.GetAttributeViReal64(ToChar(channelList), (ViAttr)attributeID, &value)) return (double)value;
	return 0;
}

std::string pyniscope::GetAttributeViString(std::string channelList, int attributeID){
	/*
	Gets the value of a ViString attribute.
	*/
	ViInt32 bufSize;
	bufSize = scope.GetAttributeViStringBufSize(ToChar(channelList), (ViAttr)attributeID);
	if (bufSize > 0) {
		ViChar* value = new ViChar[bufSize];
		if (scope.GetAttributeViString(ToChar(channelList), (ViAttr)attributeID, bufSize, value)) return (std::string)value;
	}
	return "";
}

bool pyniscope::GetAttributeViBoolean(std::string channelList, int attributeID){
	/*
	Gets the value of a ViBoolean attribute.
	*/
	ViBoolean value;
	if (scope.GetAttributeViBoolean(ToChar(channelList), (ViAttr)attributeID, &value)) return (bool)value;
	return false;
}

int pyniscope::GetAttributeViSession(std::string channelList, int attributeID){
	/*
	Gets the value of a ViSession attribute.
	*/
	ViSession value;
	if (scope.GetAttributeViSession(ToChar(channelList), (ViAttr)attributeID, &value)) return (int)value;
	return 0;
}

bool pyniscope::CheckAttributeViInt32(std::string channelList, int attributeID, int value){
	/*
	Verifies the validity of a value you specify for a ViInt32 attribute.
	*/
	return scope.CheckAttributeViInt32(ToChar(channelList), (ViAttr)attributeID, (ViInt32)value);
}

bool pyniscope::CheckAttributeViInt64(std::string channelList, int attributeID, int value){
	/*
	Verifies the validity of a value you specify for a ViInt64 attribute.
	*/
	return scope.CheckAttributeViInt64(ToChar(channelList), (ViAttr)attributeID, (ViInt64)value);
}

bool pyniscope::CheckAttributeViReal64(std::string channelList, int attributeID, double value){
	/*
	Verifies the validity of a value you specify for a ViReal64 attribute.
	*/
	return scope.CheckAttributeViReal64(ToChar(channelList), (ViAttr)attributeID, (ViReal64)value);
}

bool pyniscope::CheckAttributeViString(std::string channelList, int attributeID, std::string value){
	/*
	Verifies the validity of a value you specify for a ViString attribute.
	*/
	return scope.CheckAttributeViString(ToChar(channelList), (ViAttr)attributeID, ToChar(value));
}

bool pyniscope::CheckAttributeViBoolean(std::string channelList, int attributeID, bool value){
	/*
	Verifies the validity of a value you specify for a ViBoolean attribute.
	*/
	return scope.CheckAttributeViBoolean(ToChar(channelList), (ViAttr)attributeID, (ViBoolean)value);
}

bool pyniscope::CheckAttributeViSession(std::string channelList, int attributeID, int value){
	/*
	Verifies the validity of a value you specify for a ViSession attribute.
	*/
	return scope.CheckAttributeViInt32(ToChar(channelList), (ViAttr)attributeID, (ViInt32)value);
}

bool pyniscope::Abort(){
	/*
	Aborts an acquisition and returns the digitizer to the Idle state. Call this function if the digitizer times out waiting for a trigger.
	*/
	return scope.Abort();
}

int pyniscope::AcquisitionStatus(){
	/*
	Returns status information about the acquisition to the status output parameter.
	*/
	ViInt32 status;
	if (scope.AcquisitionStatus(&status)) return status;
	return 0;
}

bool pyniscope::Commit(){
	/*
	Commits to hardware all the parameter settings associated with the task. Use this function if you want a parameter change to be immediately reflected in the hardware. This function is supported for the NI 5122/5124 only.
	*/
	return scope.Commit();
}

list pyniscope::Fetch(std::string channelList, double timeout, int numSamples){
	/*
	Returns the waveform from a previously initiated acquisition that the digitizer acquires for the specified channel. This function returns scaled voltage waveforms.
	*/
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		ViReal64* wfm = new ViReal64[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.Fetch(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfi.append(wfm[i*numSamples + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::FetchComplex(std::string channelList, double timeout, int numSamples){
	/*
	Retrieves data that the digitizer has acquired from a previously initiated acquisition and returns a one-dimensional array of complex, scaled waveforms.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		NIComplexNumber* wfm = new NIComplexNumber[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchComplex(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfr;
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfr.append(wfm[i*numSamples + j].real);
					wfi.append(wfm[i*numSamples + j].imaginary);
				}
				retval.append(wfi);
				retval.append(wfr);
			}
		}
	}
	return retval;
}

list pyniscope::FetchComplexBinary16(std::string channelList, double timeout, int numSamples){
	/*
	Retrieves data from single channels and records. Returns a one-dimensional array of complex binary 16-bit waveforms.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		NIComplexNumber* wfm = new NIComplexNumber[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchComplex(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfr;
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfr.append(wfm[i*numSamples + j].real);
					wfi.append(wfm[i*numSamples + j].imaginary);
				}
				retval.append(wfi);
				retval.append(wfr);
			}
		}
	}
	return retval;
}

list pyniscope::FetchBinary8(std::string channelList, double timeout, int numSamples){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 8-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		ViInt8* wfm = new ViInt8[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchBinary8(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfi.append(wfm[i*numSamples + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::FetchBinary16(std::string channelList, double timeout, int numSamples){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 16-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		ViInt16* wfm = new ViInt16[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchBinary16(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfi.append(wfm[i*numSamples + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::FetchBinary32(std::string channelList, double timeout, int numSamples){
	/*
	Retrieves data from a previously initiated acquisition and returns binary 32-bit waveforms. This function may return multiple waveforms depending on the number of channels, the acquisition type, and the number of records you specify.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		ViInt32* wfm = new ViInt32[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchBinary32(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfi.append(wfm[i*numSamples + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::GetNormalizationCoefficients(std::string channelList){
	/*
	Returns coefficients that can be used to convert binary data to normalized and calibrated data.
    */
	list retval;
	ViInt32 bufferSize;
	bufferSize = scope.GetNormalizationCoefficientsBufSize(ToChar(channelList));
	if (bufferSize>0) {
		struct niScope_coefficientInfo* coefficientInfo = new struct niScope_coefficientInfo[bufferSize];
		ViInt32 numberOfCoefficientSets;
		if (scope.GetNormalizationCoefficients(ToChar(channelList), (ViInt32)bufferSize, coefficientInfo, &numberOfCoefficientSets)) {
			for (int i=0; i<bufferSize; i++) {
				list cf;
				cf.append(coefficientInfo[i].offset);
				cf.append(coefficientInfo[i].gain);
				retval.append(cf);
			}
		}
	}
	return retval;
}

list pyniscope::GetScalingCoefficients(std::string channelList){
	/*
	Returns coefficients that can be used to scale binary data to volts.
    */
	list retval;
	ViInt32 bufferSize;
	bufferSize = scope.GetScalingCoefficientsBufSize(ToChar(channelList));
	bufferSize = 2;
	if (bufferSize>0) {
		struct niScope_coefficientInfo* coefficientInfo = new struct niScope_coefficientInfo[bufferSize];
		ViInt32 numberOfCoefficientSets;
		if (scope.GetScalingCoefficients(ToChar(channelList), (ViInt32)bufferSize, coefficientInfo, &numberOfCoefficientSets)) {
			for (int i=0; i<bufferSize; i++) {
				list cf;
				cf.append(coefficientInfo[i].offset);
				cf.append(coefficientInfo[i].gain);
				retval.append(cf);
			}
		}
	}
	return retval;
}

bool pyniscope::InitiateAcquisition(){
	/*
	Initiates a waveform acquisition.
    */
	return scope.InitiateAcquisition();
}

list pyniscope::Read(std::string channelList, double timeout, int numSamples){
	/*
	Initiates an acquisition, waits for it to complete, and retrieves the data. The process is similar to calling niScope_InitiateAcquisition, niScope_AcquisitionStatus, and niScope_Fetch. The only difference is that with niScope_Read, you enable all channels specified with channelList before the acquisition; in the other method, you enable the channels with niScope_ConfigureVertical.
    */
	ViInt32 numWfms;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms)) {
		ViReal64* wfm = new ViReal64[numWfms*numSamples];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.Read(ToChar(channelList), (ViReal64)timeout, (ViInt32)numSamples, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < numSamples; j++) {
					wfi.append(wfm[i*numSamples + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

bool pyniscope::AddWaveformProcessing(std::string channelList, int measFunction){
	/*
	Adds one measurement to the list of processing steps that are completed before the measurement.
	*/
	return scope.AddWaveformProcessing(ToChar(channelList), (ViInt32)measFunction);
}

bool pyniscope::ClearWaveformMeasurementStats(std::string channelList, int clearableMeasurementFunction){
	/*
	Clears the waveform stats on the channel and measurement you specify.
	*/
	return scope.ClearWaveformMeasurementStats(ToChar(channelList), (ViInt32)clearableMeasurementFunction);
}

bool pyniscope::ClearWaveformProcessing(std::string channelList){
	/*
	Clears the list of processing steps assigned to the given channel.
	*/
	return scope.ClearWaveformProcessing(ToChar(channelList));
}

list pyniscope::FetchArrayMeasurement(std::string channelList, double timeout, int arrayMeasFunction){
	/*
	Obtains a waveform from the digitizer and returns the specified measurement array.
	*/
	ViInt32 numWfms;
	ViInt32 measWfmSize;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms) && scope.ActualMeasWfmSize((ViInt32)arrayMeasFunction, &measWfmSize)) {
		ViReal64* wfm = new ViReal64[numWfms*measWfmSize];
		struct niScope_wfmInfo* wfmInfo = new niScope_wfmInfo[numWfms];
		if (scope.FetchArrayMeasurement(ToChar(channelList), (ViReal64)timeout, (ViInt32)arrayMeasFunction, measWfmSize, wfm, wfmInfo)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				retval.append(WfmInfoExtract(wfmInfo[i]));
				for (int j = 0; j < measWfmSize; j++) {
					wfi.append(wfm[i*measWfmSize + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::FetchMeasurement(std::string channelList, double timeout, int scalarMeasFunction){
	/*
	Fetches a waveform from the digitizer and performs the specified waveform measurement. Refer to Using Fetch Functions for more information.
    */
	ViInt32 numWfms;
	ViInt32 measWfmSize;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms) && scope.ActualMeasWfmSize((ViInt32)scalarMeasFunction, &measWfmSize)) {
		ViReal64* wfm = new ViReal64[numWfms*measWfmSize];
		if (scope.FetchMeasurement(ToChar(channelList), (ViReal64)timeout, (ViInt32)scalarMeasFunction, wfm)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				for (int j = 0; j < measWfmSize; j++) {
					wfi.append(wfm[i*measWfmSize + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}

list pyniscope::FetchMeasurementStats(std::string channelList, double timeout, int scalarMeasFunction){
	/*
	Obtains a waveform measurement and returns the measurement value. This function may return multiple statistical results depending on the number of channels, the acquisition type, and the number of records you specify.
	*/
	// TODO: implement
	list retval;
	return retval;
}

list pyniscope::ReadMeasurement(std::string channelList, double timeout, int scalarMeasFunction){
	/*
	Initiates an acquisition, waits for it to complete, and performs the specified waveform measurement for a single channel and record or for multiple channels and records.
    */
	ViInt32 numWfms;
	ViInt32 measWfmSize;
	list retval;
	if (scope.ActualNumWfms(ToChar(channelList), &numWfms) && scope.ActualMeasWfmSize((ViInt32)scalarMeasFunction, &measWfmSize)) {
		ViReal64* wfm = new ViReal64[numWfms*measWfmSize];
		if (scope.ReadMeasurement(ToChar(channelList), (ViReal64)timeout, (ViInt32)scalarMeasFunction, wfm)) {
			for (int i = 0; i < numWfms; i++) {
				list wfi;
				for (int j = 0; j < measWfmSize; j++) {
					wfi.append(wfm[i*measWfmSize + j]);
				}
				retval.append(wfi);
			}
		}
	}
	return retval;
}


bool pyniscope::CalSelfCalibrate(std::string channelList, int option){
	/*
	Self-calibrates the digitizer.
	*/
	return scope.CalSelfCalibrate(ToChar(channelList), (ViInt32)option);
}

bool pyniscope::ResetDevice(){
	/*
	Performs a hard reset of the device.
	*/
	return scope.ResetDevice();
}

bool pyniscope::Disable(){
	/*
	Aborts any current operation, opens data channel relays, and releases RTSI and PFI lines.
    */
	return scope.Disable();
}

bool pyniscope::ProbeCompensationSignalStart(){
	/*
	Starts the 1 kHz square wave output on PFI 1 for probe compensation.
    */
	return scope.ProbeCompensationSignalStart();
}

bool pyniscope::ProbeCompensationSignalStop(){
	/*
	Stops the 1 kHz square wave output on PFI 1 for probe compensation.
    */
	return scope.ProbeCompensationSignalStop();
}

bool pyniscope::IsDeviceReady(std::string resourceName, std::string channelList){
	/*
	Call this function to determine whether the device is ready for use or the device is still undergoing initialization.
    */
	ViBoolean retval;
	if (scope.IsDeviceReady(ToChar(resourceName), ToChar(channelList), &retval)) return (bool)retval;
	return false;
}

bool pyniscope::Reset(){
	/*
	Resets the digitizer to its default state.
	*/
	return scope.Reset();
}

bool pyniscope::ResetWithDefaults(){
	/*
	Performs a software reset of the device, returning it to the default state and applying any initial default settings from the IVI Configuration Store.
    */
	return scope.ResetWithDefaults();
}

list pyniscope::RevisionQuery(){
	/*
	Returns the revision numbers of the instrument driver and instrument firmware.
    */
	list retval;
	ViChar* driverRev = new ViChar[IVI_MAX_MESSAGE_BUF_SIZE];
	ViChar* instrRev = new ViChar[IVI_MAX_MESSAGE_BUF_SIZE];
	if (scope.RevisionQuery(driverRev, instrRev)) {
		retval.append((std::string)driverRev);
		retval.append((std::string)instrRev);
	}
	return retval;
}

list pyniscope::SelfTest(){
	/*
	Runs the instrument self-test routine and returns the test result(s).
    */
	list retval;
	ViInt16 selfTestResult;
	ViChar* selfTestMessage = new ViChar[IVI_MAX_MESSAGE_BUF_SIZE];
	if (scope.SelfTest(&selfTestResult, selfTestMessage)) {
		retval.append((int)selfTestResult);
		retval.append((std::string)selfTestMessage);
	}
	return retval;
}

int pyniscope::GetStreamEndpointHandle(std::string streamName){
	/*
	Returns a writer endpoint that can be used with NI-P2P to configure a peer-to-peer stream with a digitizer endpoint.
    */
	ViUInt32 writerHandle;
	if (scope.GetStreamEndpointHandle(ToChar(streamName), &writerHandle)) return (int)writerHandle;
	return 0;
}

list pyniscope::ErrorHandler(int errorCode){
	/*
	Takes the error code returned by NI-SCOPE functions and returns the interpretation as a user-readable string.
    */
	list retval;
	ViChar* errorSource = new ViChar[MAX_FUNCTION_NAME_SIZE];
	ViChar* errorDescription = new ViChar[MAX_ERROR_DESCRIPTION];
	if (scope.ErrorHandler(errorCode, errorSource, errorDescription)) {
		retval.append(errorSource);
		retval.append(errorDescription);
	}
	return retval;
}

std::string pyniscope::GetError(int errorCode){
	/*
	Reads an error code and message from the error queue. National Instruments digitizers do not contain an error queue. Errors are reported as they occur. Therefore, this function does not detect errors.
    */
	ViInt32 bufferSize;
	bufferSize = scope.GetErrorBufSize((ViStatus*)errorCode);
	if (bufferSize>0) {
		ViChar* description = new ViChar[bufferSize]; 
		if (scope.GetError((ViStatus*)errorCode, bufferSize, description)) return (std::string)description;
	}
	return "";
}

std::string pyniscope::GetErrorMessage(int errorCode){
	/*
	Returns the error code from an NI-SCOPE function as a user-readable string.
	*/
	ViInt32 bufferSize;
	bufferSize = scope.GetErrorMessageBufSize((ViStatus)errorCode);
	if (bufferSize>0) {
		ViChar* description = new ViChar[bufferSize]; 
		if (scope.GetErrorMessage((ViStatus)errorCode, bufferSize, description)) return (std::string)description;
	}
	return "";
}


bool pyniscope::LockSession(){
	/*
	Obtains a multithread lock on the instrument session.
	*/
	ViBoolean callerHasLock;
	return scope.LockSession(&callerHasLock);
}

bool pyniscope::UnlockSession(){
	/*
	Releases a lock that you acquired on an instrument session using niScope LockSession.
    */
	ViBoolean callerHasLock;
	return scope.UnlockSession(&callerHasLock);
}


bool pyniscope::ClearError(){
	/*
	Clears the error information for the current execution thread and the IVI session you specify.
	*/
	return scope.ClearError();
}

bool pyniscope::ClearInterchangeWarnings(){
	/*
	Clears the list of current interchange warnings.
    */
	return scope.ClearInterchangeWarnings();
}

bool pyniscope::ConfigureAcquisitionRecord(double timeperRecord, int minNumPoints, double acquisitionStartTime){
	/*
	Configures the most commonly configured attributes of the instrument acquisition subsystem.
	*/
	return scope.ConfigureAcquisitionRecord((ViReal64)timeperRecord, (ViInt32)minNumPoints, (ViReal64)acquisitionStartTime);
}

bool pyniscope::ConfigureChannel(std::string channel, double range, double offset, int coupling, double probeAttenuation, bool enabled){
	/*
	Configures the most commonly configured attributes of the instrument's channel subsystem.
    */
	return scope.ConfigureChannel(ToChar(channel), (ViReal64)range, (ViReal64)offset, (ViInt32)coupling, (ViReal64)probeAttenuation, (ViBoolean)enabled);
}

bool pyniscope::ConfigureEdgeTriggerSource(std::string source, double level, int slope){
	/*
	Sets the edge triggering attributes.
	*/
	return scope.ConfigureEdgeTriggerSource(ToChar(source), (ViReal64)level, (ViInt32)slope);
}

bool pyniscope::ConfigureRefLevels(double low, double mid, double high){
	/*
	Configures the reference levels for all channels of the digitizer.
	*/
	return scope.ConfigureRefLevels((ViReal64)low, (ViReal64)mid, (ViReal64)high);
}

bool pyniscope::ConfigureTrigger(int triggerType, double holdoff){
	/*
	Configures the common attributes of the trigger subsystem.
    */
	return scope.ConfigureTrigger((ViInt32)triggerType, (ViReal64)holdoff);
}

bool pyniscope::ConfigureTriggerCoupling(int coupling){
	/*
	Sets the trigger coupling attribute.
    */
	return scope.ConfigureTriggerCoupling((ViInt32)coupling);
}

bool pyniscope::ConfigureTriggerOutput(int triggerEvent, std::string triggerOutput){
	/*
	Configures the digitizer to generate a signal pulse that other digitizers can detect when configured for digital triggering.
    */
	return scope.ConfigureTriggerOutput((ViInt32)triggerEvent, ToChar(triggerOutput));
}

bool pyniscope::ConfigureTVTriggerLineNumber(int lineNumber){
	/*
	Configures the TV line upon which the instrument triggers.
	*/
	return scope.ConfigureTVTriggerLineNumber((ViInt32)lineNumber);
}

bool pyniscope::ConfigureTVTriggerSource(std::string source, int signalFormat, int evt, int polarity){
	/*
	Configures the instrument for TV triggering.
	*/
	return scope.ConfigureTVTriggerSource(ToChar(source), (ViInt32)signalFormat, (ViInt32)evt, (ViInt32)polarity);
}

list pyniscope::ErrorQuery(){
	/*
	Reads an error code and message from the error queue. National Instruments digitizers do not contain an error queue. Errors are reported as they occur. Therefore, this function does not detect errors.
	*/
	list retval;
	ViInt32 errCode;
	ViChar* errMessage = new ViChar[IVI_MAX_MESSAGE_BUF_SIZE];
	if (scope.ErrorQuery(&errCode, errMessage)) {
		retval.append((int)errCode);
		retval.append((std::string)errMessage);
	}
	return retval;
}

list pyniscope::FetchWaveform(std::string channel, int waveformSize){
	/*
	Returns the waveform from a previously initiated acquisition that the digitizer acquires for the channel you specify.
	*/
	// TODO: implement
	list retval;
	return retval;
}

list pyniscope::FetchWaveformMeasurement(std::string channel, int measFunction){
	/*
	Fetches a waveform measurement from a specific channel from a previously initiated waveform acquisition.
	*/
	// TODO: implement
	list retval;
	return retval;
}

std::string pyniscope::GetChannelName(int index){
	/*
	Returns the channel string that is in the channel table at an index you specify. Not applicable to National Instruments digitizers.
    */
	// TODO: implement
	return "";
}

std::string pyniscope::GetNextCoercionRecord(){
	/*
	Returns the coercion information associated with the IVI session.
	*/
	// TODO: implement
	return "";
}

std::string pyniscope::GetNextInterchangeWarning(){
	/*
	Returns the interchangeability warnings associated with the IVI session.
	*/
	// TODO: implement
	return "";
}

bool pyniscope::IsInvalidWfmElement(double elementValue){
	/*
	Determines whether a value you pass from the waveform array is invalid.
	*/
	ViBoolean isInvalid;
	if (scope.IsInvalidWfmElement((ViReal64)elementValue, &isInvalid)) return isInvalid;
	return true;
}

list pyniscope::ReadWaveform(std::string channel, int waveformSize, int maxtime){
	/*
	Initiates an acquisition on the channels that you enable with ConfigureVertical.
	*/
	// TODO: implement
	list retval;
	return retval;
}

list pyniscope::ReadWaveformMeasurement(std::string channel, int measFunction, int maxTime){
	/*
	Initiates a new waveform acquisition and returns a specified waveform measurement from a specific channel.
    */
	// TODO: implement
	list retval;
	return retval;
}

bool pyniscope::ResetInterchangeCheck(){
	/*
	 When developing a complex test system that consists of multiple test modules, it is generally a good idea to design the test modules so that they can run in any order.
	*/
	return scope.ResetInterchangeCheck();
}

bool pyniscope::SendSWTrigger(){
	/*
	Sends a command to trigger the digitizer. Call this function after you call ConfigureTriggerSoftware.
    */
	return scope.SendSWTrigger();
}
