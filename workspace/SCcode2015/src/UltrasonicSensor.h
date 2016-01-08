#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "WPILib.h"

class UltrasonicSensor {
public:
	UltrasonicSensor(uint32_t myPortNumber) {
		portNumber = myPortNumber;
		ultraInput = new AnalogInput(portNumber);
	}

	float GetRangeInInches(){
		float voltage = ultraInput->GetVoltage();
		float inches = voltage/(.009766);
		return inches;

	}
	float GetRangeInFeet() {
		float inches = GetRangeInInches();
		return inches/12;
	}
	float GetRangeInMM() {
		float inches = GetRangeInInches();
		return inches*25.4;
	}

private:
	uint32_t portNumber;
	AnalogInput* ultraInput;
};

#endif
