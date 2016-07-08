#ifndef __RAILV1_HPP__
#define __RAILV1_HPP__

#include <cassert>
#include <cstdint> // uint16_t
#include <cstring>
#include <array> // std::array
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/st7735r.h>  // LCD
#include <libsc/lcd_typewriter.h>// LCD WriteString
#include <libsc/lcd_console.h>
#include <libsc/mini_lcd.h>
#include <libbase/k60/adc.h> // sensor
#include <libsc/futaba_s3010.h> // servo
#include <libsc/joystick.h>//joystick
#include <libsc/simple_buzzer.h>//buzzer
#include <libsc/button.h>//button
#include <libbase/k60/gpio.h>
#include <libsc/led.h>//led
#include <libsc/dir_motor.h>
#include <libsc/ab_encoder.h>
#include <sstream>
#include <string>
#include <libutil/kalman_filter.h>
#include <libutil/pGrapher.h>
#include <libsc/ldc1000.h>


class Railcar{
public:

	bool Start = false;
	bool MotorEnable = false;
	//mapping
	float plusValue = 0;
	float crossValue = 1;
	//steering control
	float tKp = 0.065;
	float tKi = 0;
	float tKd = 0;
	float tError = 0;
	float tPreError = 0;
	float tKpl = 1.065;
	float tKil = 0;
	float tKdl = 0;
	float MaxReading = 0;//16600- 185000
	float MinReading = 0; //13000- 133000
	float mappingTemp = 0;

	float angle = 0;
	uint32_t frequency = 0;
	uint16_t data = 0;


	uint16_t f1 = 0;
	uint16_t f2 = 0;

	float turningMiddle = 1040;
	float turningMax = turningMiddle + 210;
	float turningMin = turningMiddle - 210;
	float leftTemp =0;
	float adcOut = 35;
	float leftMin = 0;
	float leftMax = 0;
	float rightMin = 0;
	float rightMax = 0;
	float leftRange = 0;
	float rightRange = 0;
	float noise = 400;

	//speed control
	float mKp = 1;
	float mKi = 0;
	float mKd = 0;
	float mError = 0;
	float mPreError = 0;
	float mPrePreError = 0;
	float targetSpeed = 60;
	int16_t runSpeed = 0;
	int32_t realSpeed = 0;
	float low = 5;
	float high = 35;

//	float lKp = 0;
//	float rKp = 0;
//	float rKd = 0;
//	float lKd = 0;
//
//	float leftError = 0;
//	float rightError = 0;
//	float leftPreError = 0;
//	float rightPreError = 0;


	float leftSensor = 0;
	float rightSensor = 0;
	float middleSensor = 0;
	float q=35;
	float r=100-q;





	libsc::Led* Led0;
	libsc::Led* Led1;
	libsc::Led* Led2;
	libsc::Led* Led3;
	libsc::AbEncoder* encoder;
	libsc::FutabaS3010* servo;
	libsc::DirMotor* motor;
	libsc::Button* button1;
	libsc::Button* button2;
	libsc::Joystick* joystick;

//	libsc::SimpleBuzzer* buzzer;

	libutil::KalmanFilter* Ldc1000Right;
	libutil::KalmanFilter* Ldc1000Left;


	libsc::Ldc1000 *Ldc1000_0;
	libsc::Ldc1000 *Ldc1000_1;

	libutil::KalmanFilter* middleSensorF;












	void printRoad();
	void steering();
	void speedControl();
	void config();
	void collectSensorData();
	void getThreshold();
	void checkReading();
	void filtering();
	void mapping();

};

#endif

