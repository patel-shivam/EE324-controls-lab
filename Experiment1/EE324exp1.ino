#define input_pin A1


int output_pin1 = 5;
int output_pin2 = 6;
//int o_1=5;
//int o_2=6;
int centre =200;
double feedback, control_signal; // sensed signal and control signal 
double calibrate; // setpoint for calibrating the initial position of the motor
double k_prop=40; // proportional controller, integral controller, differential controller
double k_int=0.01;
double k_diff=2;

float error = 0; // the error of current iteration
float prev_error = 0; // the error of the previous iteration for differential controller
float total_error = 0; //cumulative error of the controller for integral controller
float target;
float diff_error;


void setup() {
  Serial.begin(9600); // put your setup code here, to run once:
  pinMode(input_pin, INPUT);
  pinMode(output_pin1,OUTPUT);
  pinMode(output_pin2,OUTPUT);
  feedback=analogRead(input_pin);
  calibrate=feedback;
  target = feedback+512;
  error = 512;
}

void loop() {
  // put your main code here, to run repeatedly:
  feedback = analogRead(input_pin);
  // this takes the current value of the potentiometer output
  Serial.print(millis());
  Serial.print("\t");
  Serial.println(feedback);
  Serial.print("\t");
  //Serial.println(error);
  error = target - feedback;
  total_error += error;
  diff_error = error - prev_error;
  control_signal = (int)(k_int*total_error + k_diff*diff_error + k_prop*error);
  control_signal/=10;
  if (abs(control_signal)>255){  // clipping the output at 255 for the analog output pin
    control_signal=255;
  }

  if (error>0)
  {
    analogWrite(output_pin1,0);
    analogWrite(output_pin2,abs(control_signal));
  }

  else
  {
    analogWrite(output_pin2,0);
    analogWrite(output_pin1,abs(control_signal));
  }
  
  prev_error = error;
  
  

}
