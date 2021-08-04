#include "robot_move.h"

// Pin Definitions
// Pin Definitions
/*const int right_pwm_pin = 33; //pwm
const int right_output_pin1 = 29; //
const int right_output_pin2 = 31; //

const int left_pwm_pin = 32; //pwm
const int left_output_pin1 = 38; // 
const int left_output_pin2 = 40; // pwm
*/


bool end_this_program = false;


/*void delay(int s){
	this_thread::sleep_for(chrono::milliseconds(s));
}*/

void signalHandler (int s){
	end_this_program = true;
}

int Distance(int knowWidth, int knowHeight, int frame_width, int frame_height)
{
 // int focalLength = (864/2)*sqrt(3);
    int focalLength = (frame_width/2)*sqrt(3);

   int realheight = 175 - 45;
   //int imageheight = 480;

   //int objectheight = rec.height;
   int sensorheight = 270;
   //int Distance = 60*focalLength/knowWidth + 30;
   int Distance = (focalLength*realheight*frame_height)/(knowHeight*sensorheight) - 100;
  
  return Distance;
}
/*
// factory function for PWM
GPIO::PWM CreatePWMObject(int output_pin)
{
    // Pin Setup.	
    // Board pin-numbering scheme	
    GPIO::setmode(GPIO::BOARD);
	// set pin as an output pin with optional initial state of HIGH	
    GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);	

    GPIO::PWM p(output_pin, 50);

    return p;
}

static GPIO::PWM p_right = CreatePWMObject(right_pwm_pin);
static GPIO::PWM p_left = CreatePWMObject(left_pwm_pin);

void SetupMotor()
{
	GPIO::setup(right_output_pin1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(right_output_pin2, GPIO::OUT, GPIO::LOW);

	GPIO::setup(left_output_pin1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(left_output_pin2, GPIO::OUT, GPIO::LOW);
}

void MotorLeft(bool isForward, int Speed) // left
{
	p_left.start(Speed);

	if(isForward == true) // forward
		{
			GPIO::output(left_output_pin1, GPIO::HIGH);
			GPIO::output(left_output_pin2, GPIO::LOW);
		}
	else{ // backward
			GPIO::output(left_output_pin1, GPIO::LOW);
			GPIO::output(left_output_pin2, GPIO::HIGH);
		}
}

void MotorRight(bool isForward, int Speed) // right
{
	p_right.start(Speed);

	if(isForward == true) // forward
		{
			GPIO::output(right_output_pin1, GPIO::HIGH);
			GPIO::output(right_output_pin2, GPIO::LOW);
		}
	else{ // backward
			GPIO::output(right_output_pin1, GPIO::LOW);
			GPIO::output(right_output_pin2, GPIO::HIGH);
		}
}

void Move(int frameWidth,int frameHeight, int recX, int recWidth, int recHeight)
{

	int distance = Distance(recWidth, recHeight, frameWidth, frameHeight);

	int objX =  recX + recX/2; 
	int errorPan = objX - frameWidth/2;

	if(distance > 220)
	  {
		if(abs(errorPan) > 100)
		{
		if(errorPan > 0) //Taget in right of frame 
		  {
			p_right.stop();
			MotorLeft(FORWARD, MaxLeft - 50);
		  }
		else       //Taget in left of frame
		  {
			p_left.stop();
			MotorRight(FORWARD, MaxRight - 50);
		  }
		}
		else {
			MotorLeft(FORWARD, MaxLeft - 20);
			MotorRight(FORWARD, MaxRight - 20);
		   }
	  }
	else if(180 < distance && distance < 220)
	{
		p_left.stop();
		p_right.stop();
	}
	else
	{
        cout<<"Lui"<<endl;
		MotorLeft(BACKWARD, MaxLeft - 20);
		MotorRight(BACKWARD, MaxRight - 20);
	}
}

void Cleanup()
{
	p_right.stop();
	p_left.stop();
	GPIO::cleanup(29);
	GPIO::cleanup(31);
	GPIO::cleanup(32);
	GPIO::cleanup(33);
	GPIO::cleanup(38);
	GPIO::cleanup(40);
}
*/
