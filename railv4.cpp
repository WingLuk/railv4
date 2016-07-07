#include "railv1.h"

using namespace libsc;
using namespace libbase::k60;
using namespace libutil;

void Railcar::config(){


	Led::Config led_config;
	led_config.id = 1;
	led_config.is_active_low = true;
	Led1 = new Led(led_config);
	Led1->SetEnable(true);

	Led::Config led_config1;
	led_config1.id = 2;
	led_config1.is_active_low = true;
	Led2 = new Led(led_config1);
	Led2->SetEnable(false);



	FutabaS3010::Config servo_config;
	servo_config.id = 0;
	servo = new FutabaS3010(servo_config);
	servo->SetDegree(1020);

	DirMotor::Config motor_config;
	motor_config.id = 0;
	motor = new DirMotor(motor_config);
	motor->SetPower(0);
	motor->SetClockwise(false);

	Button::Config ButtonConfig;
	ButtonConfig.id=0;
	ButtonConfig.is_active_low=true;
	Button::Config ButtonConfig1;
	ButtonConfig1.id=1;
	ButtonConfig1.is_active_low=true;
	button1 = new Button(ButtonConfig);
	button2 = new Button(ButtonConfig1);


	Adc::Config adc_config;
	adc_config.adc = libbase::k60::Adc::Name::kAdc1Ad12;
	adcSensor = new Adc(adc_config);




//	joystick = new Joystick(GetJoystickConfig());


	//buzzer = new SimpleBuzzer(GetBuzzerConfig());

//
//	AbEncoder::Config AbEncoder1;
//	encoder = new AbEncoder(AbEncoder1);

	Ldc1000Left = new KalmanFilter(q,r,0.5,0.5);
	Ldc1000Right = new KalmanFilter(q,r,0.5,0.5);
	middleSensorF = new KalmanFilter(q,r,0.5,0.5);


	Ldc1000_0 = new Ldc1000({0});
	Ldc1000_1 = new Ldc1000({1});


}




void Railcar::collectSensorData(){
	Ldc1000_0->Update();
	Ldc1000_1->Update();
	f1 = Ldc1000_0->GetData();
	f2 = Ldc1000_1->GetData();


}



void Railcar::steering(){
//	rightRange = rightMax - rightMin;
//	leftRange = leftMax - leftMin;
//	rightError = rightSensor - rightMin;
//	leftError = leftSensor - leftMin;
//
//	if(leftError == rightError){
//
//	}
//	angle = rKp* rightError + rKd* (rightError - rightPreError);
//	angle = lKp* leftError + lKd* (leftError - leftPreError);

	tPreError = tError;
//	if(leftSensor < MinReading && rightSensor < MinReading && middleSensor > adcOut){
//		tError = tPreError;
//		angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));
//	}
//	if(abs(leftSensor - rightSensor) > noise){
		if(leftSensor > rightSensor){
			tError = leftSensor - rightSensor;
			angle = turningMiddle +(tKpl*tError + tKdl*(tError - tPreError));
		}
		else{

		tError = leftSensor - rightSensor;
		angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));

		}
//	}
//
//	else{
//		angle = turningMiddle;
//		tError = 0;}


	if(angle > turningMax)angle = turningMax;
	if(angle < turningMin )angle = turningMin;


			servo->SetDegree(angle);


	}



void Railcar::filtering(){
	leftTemp = Ldc1000Left-> Filter(f1);
	rightSensor = Ldc1000Right-> Filter(f2);
	leftSensor = crossValue* leftTemp + plusValue;

}

void Railcar::mapping(){
	rightRange = rightMax - rightMin;
	leftRange = leftMax - leftMin;
	crossValue = rightRange/leftRange;
	mappingTemp = crossValue * leftMin;
	plusValue = rightMin - mappingTemp;
	Start = true;
	Led1 ->Switch();

}


void Railcar::speedControl(){
	encoder->Update();
	realSpeed = encoder->GetCount();
	mPrePreError = mPreError;
	mPreError = mError;
	mError = targetSpeed - realSpeed;
	runSpeed += (int16_t)(mKp *( mError - mPreError)+ mKi * (mError));
	motor->SetPower(runSpeed);

}
//
void Railcar::checkReading(){
	if(rightSensor - leftSensor >noise){
		Led2->SetEnable(true);
	}
}


