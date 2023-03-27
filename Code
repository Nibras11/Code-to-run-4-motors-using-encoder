class SimplePID{
  private:
    float kp, kd, ki; // Parameters
    float eprev, eintegral;

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), eprev(0.0), eintegral(0.0){}

  void setParams(float kpIn, float kdIn, float kiIn){
    kp = kpIn; kd = kdIn; ki = kiIn;
  }
  
  void evalu(float v1Measured, float target, float deltaT, int &pwr, int &dir){
    float e = target - v1Measured;
  
    float dedt = (e-eprev)/(deltaT);
  
    eintegral = eintegral + e*deltaT;

    float u = kp*e + kd*dedt + ki*eintegral;
  
    pwr = (int) fabs(u);
    if( pwr > 255 ){
      pwr = 255;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
};
void readEncoder1();
void readEncoder2();
void readEncoder3();
#define NMOTORS 3

const int encA[] = {2,3,18};
const int encB[] = {13,9,10};
const int pwm[] = {5,6,4};
const int in1[] = {7,11,14};
const int in2[] = {8,12,15};

long prevT = 0;
int posPrev[] = {0,0};
float v1[] = {0,0};
volatile int posi[] = {0,0};
float v1Filt[] = {0,0};
float v1Prev[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(encA[k],INPUT);
    pinMode(encB[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    pid[k].setParams(0.6,0.055,0.0);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder1<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder2<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder3<0>,RISING);
   //attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder2<0>,RISING);
 // attachInterrupt(digitalPinToInterrupt(encB[1]),readEncoder2<1>,RISING);
   
  
  Serial.println("target pos");
}


void loop(){

  float target = 700.0;
 

  int pos[NMOTORS];
  noInterrupts(); 
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts();

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  for(int k=0;k<NMOTORS;k++)
  {
    v1[k] = (pos[k]-posPrev[k])/deltaT;
    posPrev[k] = pos[k];
    v1Filt[k] = 0.854*v1Filt[k] + 0.0728*v1[k] + 0.0728*v1Prev[k];
    v1Prev[k] = v1[k];
  }
  
  //for(int k = 0; k < NMOTORS; k++){
    int pwr1, pwr2,pw3,dir;
    // evaluate the control signal
    pid[0].evalu(v1Filt[0],target,deltaT,pwr1,dir);
    pid[1].evalu(v1Filt[1],target,deltaT,pwr2,dir);
    pid[2].evalu(v1Filt[2],target,deltaT,pwr2,dir);
    // signal the motor
    //setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
    setMotor(dir,pwr1,pwr2,pw3,pwm[0],pwm[1],pwm[2],in1[0],in2[0],in1[1],in2[1],in1[2],in2[2]);
  //}

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target);
    Serial.print(" ");
    Serial.print(v1Filt[k]);
    Serial.print(" ");
    delay(1);
  }
  Serial.println();
}


void setMotor(int dir, int pwmVal1,int pwmVal2,int pwmVal3,int pwm1,int pwm2,int pwm3, int in1, int in2,int in3,int in4,int in5,int in6){
  analogWrite(pwm1,pwmVal1); 
  analogWrite(pwm2,pwmVal2);
  analogWrite(pwm3,pwmVal3);// Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    digitalWrite(in5,HIGH);
    digitalWrite(in6,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    digitalWrite(in5,LOW);
    digitalWrite(in6,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);  
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    digitalWrite(in5,LOW);
    digitalWrite(in6,LOW);    
  }
}

template <int j>
void readEncoder1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encB[0]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  posi[j] = posi[j] + increment;

}
template <int m>
void readEncoder2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encB[1]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  posi[m] = posi[m] + increment;

}
template <int n>
void readEncoder3(){
  // Read encoder B when ENCA rises
  int b = digitalRead(encB[2]);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  posi[n] = posi[n] + increment;

}
