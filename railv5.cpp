#include "railv1.h"

using namespace libsc;
using namespace libbase::k60;
using namespace libutil;

void Railcar::config(){
	Led0 = new Led({0, true});
	Led0->SetEnable(false);

	Led1 = new Led({1, true});
	Led1->SetEnable(false);

	Led2 = new Led({2,true});
	Led2->SetEnable(false);

	Led3 = new Led({3,true});
	Led3->SetEnable(false);





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





//	joystick = new Joystick(GetJoystickConfig());

	SimpleBuzzer::Config buzzerConfig;
	buzzerConfig.id = 0;
	buzzerConfig.is_active_low = true;
	buzzer = new SimpleBuzzer(buzzerConfig);

//
//	AbEncoder::Config AbEncoder1;
//	encoder = new AbEncoder(AbEncoder1);

	Ldc1000Left = new KalmanFilter(q,r,0.5,0.5);
	Ldc1000Right = new KalmanFilter(q,r,0.5,0.5);
	Ldc1000Middle = new KalmanFilter(q,r,0.5,0.5);


	Ldc1000_0 = new Ldc1000({1});
	Ldc1000_1 = new Ldc1000({0});
	Ldc1000_2 = new Ldc1000({2});


}




void Railcar::collectSensorData(){
//	pre2f1 = pref1;
//	pre2f2 = pref2;
//	pre2f3 = pref3;
	pref1 = f1;
	pref2 = f2;
	pref3 = f3;


	Ldc1000_0->Update();
	Ldc1000_1->Update();
	Ldc1000_2->Update();
	f1 = Ldc1000_0->GetData();
	f2 = Ldc1000_1->GetData();
	f3 = Ldc1000_2->GetData();
//	temp1= Ldc1000_0->GetData();
//	temp2 = Ldc1000_1->GetData();


}



void Railcar::steering(){


	tPreError = tError;



	tError = (leftSensor - rightSensor)/(leftSensor + rightSensor);
	angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));

//	if(middleSensor < MinReading){
//
//		if(leftSensor >MinReading && rightSensor <MinReading){
//			angle = turningMax;
//		}
//		else if(leftSensor < MinReading && rightSensor >MinReading){
//			angle = turningMin;
//		}
//		else{
//			tError = tPreError;
//			angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));
//		}
//
//	}
//	else{
//		tError = (leftSensor - rightSensor)/(leftSensor + rightSensor);
//		angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));
//	}
//
//






	if(angle > turningMax)angle = turningMax;
	if(angle < turningMin )angle = turningMin;


			servo->SetDegree(angle);


	}



void Railcar::filtering(){
//filter extreme
	if(f1 >= 30000 || f1 <= 5000){
		f1 = pref1;
	}
	if(f2 >= 30000 || f2 <= 5000){
		f2 = pref2;
	}
	if(f3 >= 30000 || f3 <= 5000){
		f3 = pref3;
	}
//
//	leftTemp = (float)((pre2f1 + pref1 + f1)/3);
//	rightMean = (float)((pre2f2 + pref2 + f2)/3);
//	leftSensor = crossValue* leftTemp + plusValue;
//	middleMean = (float)((pre2f3 + pref3 + f3)/3);
	leftTemp = Ldc1000Left-> Filter((float)f1);
	rightSensor = Ldc1000Right-> Filter((float)f2);
	leftSensor = crossValue* leftTemp + plusValue;
	middleSensor = Ldc1000Middle->Filter((float)f3);

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
void Railcar::steering2(){
	tPreError = tError;
//test if out track/large turn
	if(middleSensor <= (middleMin + 300)){
		flag1 = 1;
		Led1->SetEnable(flag1);



		//if all out
		if(abs(leftSensor - rightSensor) < 500){
//			flag4 = 1;
//			Led2->SetEnable(flag4);
			if(tPreError > 0 && tERight == 0){
				angle = turningMax;
				tELeft = 1;
				tError = 1;
			}
			else{
				if(tELeft == 1 && tERight == 0){
				angle = turningMax;
				tError = 1;
								}
				else{
				angle = turningMin;
				tERight = 1;
				tError = -1;
				}

			}
		}
		//else if large turn/test if needed
		else if(leftSensor>rightSensor){
			angle = turningMax;
			tERight = 0;
			tELeft = 0;
		}
		else{
			angle = turningMin;
			tERight = 0;
			tELeft = 0;
		}
	}
	else{
	flag1 = 0;
	tERight = 0;
	tELeft = 0;
	Led1->SetEnable(flag1);
	tError = (leftSensor - rightSensor)/(leftSensor + rightSensor);
	angle = turningMiddle +(tKp*tError + tKd*(tError - tPreError));}


	if(angle > turningMax)angle = turningMax;
	if(angle < turningMin )angle = turningMin;


			servo->SetDegree(angle);


}
void Railcar::steering3(){
	tPreError=tError;
	tPreDError=tDError;
	tError = (leftSensor - rightSensor)/(leftSensor + rightSensor);
	tDError=tError-tPreError;




	offset+=tKp*tDError+
			tKi*tError+
			tKd*(tDError-tPreDError);









	angle =  turningMiddle + offset;
	servo->SetDegree(angle);
}

//void Railcar::steering4(){
//	tPreError=tError;
//		tPreDError=tDError;
//		tError = (leftSensor - rightSensor)/(leftSensor + rightSensor);
//
//
//		if(middleSensor <= (middleMin + 400)){
//			flag1 = 1;
//			Led1->SetEnable(flag1);
//
//
//
//			//if all out
//			if(abs(leftSensor - rightSensor) < 500){
//
//				if(tPreError > 0 && tERight == 0){
////					angle = turningMax;
//					tELeft = 1;
//					tError = 1;
//				}
//				else{
//					if(tELeft == 1 && tERight == 0){
////					angle = turningMax;
//					tError = 1;
//									}
//					else{
////					angle = turningMin;
//					tERight = 1;
//					tError = -1;
//
//					}
//
//				}
//			}
//		}
//
//		tDError=tError-tPreError;
//
//
//
//		offset+=tKp*tDError+
//				tKi*tError+
//				tKd*(tDError-tPreDError);
//
//
//
//
//
//
//
//
//
//		angle =  turningMiddle + offset;
//		servo->SetDegree(angle);
//}

