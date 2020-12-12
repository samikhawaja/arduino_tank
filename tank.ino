//www.elegoo.com
//2017.12.12

/************************
Exercise the motor using
the L293D chip
************************/

#define IR_ENABLE 1
#define SR_ENABLE 0

#if IR_ENABLE
#include "IRremote.h"

#define RECIEVER 8
#endif

#define RIGHTA 6
#define RIGHTB 5
#define RIGHTEN 7

#define LEFTA 2
#define LEFTB 3
#define LEFTEN 4

#if SR_ENABLE
int i;
#include "SR04.h"
#define TRIG_PIN 12
#define ECHO_PIN 11
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;
#endif

enum dir_t {
  DIR_UNKNOWN = 0,
  FORWARD = 1,
  BACKWARD = 2,
};

enum turn_t {
  TURN_UNKNOWN = 0,
  LEFT = 1,
  RIGHT = 2,
};

typedef struct {
  int a;
  int b;
  int en;

  int speed;
  bool is_enabled;
  enum dir_t dir;
} motor_t;

motor_t right {
  .a = RIGHTA,
  .b = RIGHTB,
  .en = RIGHTEN,
};

motor_t left {
  .a = LEFTA,
  .b = LEFTB,
  .en = LEFTEN,
};

#if IR_ENABLE

enum BUTTON_T {
  BUTTON_UNKNOWN = 0,
  POWER,
  UP,
  DOWN,
  VOLUP,
  VOLDOWN,  
  FAST_BACK,
  FAST_FORWARD,
};

typedef struct {  
  int pin;
  IRrecv irrecv;
  decode_results results;
  BUTTON_T last_button;
  int last_button_count;
} remote_t;

remote_t tvremote {
  .pin = RECIEVER,
  .irrecv = IRrecv(RECIEVER),
};

BUTTON_T remote_button_type(remote_t *remote) {
  BUTTON_T ret = BUTTON_UNKNOWN;
  switch(remote->results.value)

  {
  case 0xFFA25D: ret = POWER; Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); break;
  case 0xFF629D: ret = VOLUP; Serial.println("VOL+"); break;
  case 0xFF22DD: ret = FAST_BACK; Serial.println("FAST BACK");    break;
  case 0xFF02FD: Serial.println("PAUSE");    break;
  case 0xFFC23D: ret = FAST_FORWARD; Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: ret=DOWN;  Serial.println("DOWN"); break;
  case 0xFFA857: ret = VOLDOWN; Serial.println("VOL-");    break;
  case 0xFF906F: ret = UP;  Serial.println("UP"); break;
  case 0xFF9867: Serial.println("EQ");    break;
  case 0xFFB04F: Serial.println("ST/REPT");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: ret = remote->last_button;/* Serial.println(" REPEAT");*/break;  

  default: 
    //return -1;
    Serial.println(" other button   ");

  }// End Case

  return ret;
}

//int getRemote() {
//  int ret = -1;
//  if (irrecv.decode(&results)) // have we received an IR signal?
//
//  {
//    ret = check();//translateIR(); 
////    Serial.println("button %d\n", ret);
//    irrecv.resume(); // receive the next value
//  }
//
//  return ret;
//}

void remote_setup(remote_t *remote) {
  remote->irrecv.enableIRIn(); // Start the receiver
}

BUTTON_T remote_read_button(remote_t *remote) {
  if (remote->irrecv.decode(&remote->results)) {
    BUTTON_T ret = remote_button_type(remote);
    //delay(500); // Do not get immediate repeat
    remote->irrecv.resume();
    remote->last_button = ret;
    remote->last_button_count = 0;
  } else if (remote->last_button != BUTTON_UNKNOWN
        && remote->last_button_count++ > 2500) {
    Serial.println(" reseting last button   ");
    remote->last_button = BUTTON_UNKNOWN;
    remote->last_button_count = 0;
  }

  return remote->last_button;
}
#endif

void motor_set_speed(motor_t *motor, int speed) {
  motor->speed = speed;
  analogWrite(motor->en, motor->speed);
}

void motor_set_direction(motor_t *motor, enum dir_t dir) {
  motor->dir = dir;

  bool forward = (dir == FORWARD);
  digitalWrite(motor->a,forward?HIGH:LOW);
  digitalWrite(motor->b,forward?LOW:HIGH);
}

void setup_motor(motor_t *motor) {
  pinMode(motor->en,OUTPUT);
  pinMode(motor->a,OUTPUT);
  pinMode(motor->b,OUTPUT);
  motor_set_speed(motor, 0);
  motor_set_direction(motor, FORWARD);
}

void turn(turn_t towards) {
  motor_set_direction(&right, FORWARD);
  motor_set_direction(&left, FORWARD);
  switch (towards) {
    case LEFT:
      motor_set_speed(&left, 0);
      motor_set_speed(&right, 255);
    break;
    case RIGHT:
    motor_set_speed(&right, 0);
    motor_set_speed(&left, 255);
    break;
  }
}

void stopit() {
  motor_set_speed(&left, 0);
  motor_set_speed(&right, 0);
}

void set_direction(dir_t dir) {
  motor_set_direction(&right, dir);
  motor_set_direction(&left, dir);
  motor_set_speed(&right, 255);
  motor_set_speed(&left, 255);
}

void do_action() {
#if IR_ENABLE  
  //irrecv.enableIRIn(); // Start the receiver
  BUTTON_T buttonpress = remote_read_button(&tvremote);
  
  dir_t dir = DIR_UNKNOWN;
  turn_t towards = TURN_UNKNOWN;
  switch(buttonpress) {
    case VOLUP:
      dir = FORWARD;
    break;
    case VOLDOWN:
      dir = BACKWARD;
    break;
    case FAST_BACK:
      towards = LEFT;
    break;
    case FAST_FORWARD:
      towards = RIGHT;
    break;    
  }

  if (dir != DIR_UNKNOWN) {
    set_direction(dir);
//    Serial.println("direction");
  } else if (towards != TURN_UNKNOWN) {
    turn(towards);
//    Serial.println("turn");
  } else {
//    Serial.println("stop");
    stopit();
  }
  
#endif
}

int readvolt = 500;
void setup() {
  //---set pin direction
//  pinMode(RIGHTEN,OUTPUT);
//  pinMode(RIGHTA,OUTPUT);
//  pinMode(RIGHTB,OUTPUT);

  setup_motor(&right);
  setup_motor(&left);
  
  Serial.begin(9600);
#if IR_ENABLE  
  //irrecv.enableIRIn(); // Start the receiver
  remote_setup(&tvremote);
#endif
//  digitalWrite(RIGHTA,HIGH); //one way
//  digitalWrite(RIGHTB,LOW);
//  digitalWrite(ENABLE,LOW); // enable on
}
void loop() {
//  motor_set_speed(&right, 255);
//  motor_set_speed(&left, 255);
//  delay(2000);
////  set_direction(BACKWARD);
////  delay(4000);
////  set_direction(FORWARD);
//  turn(LEFT);
//  delay(2000);
//  turn(RIGHT);
//  delay(2000);

#if IR_ENABLE
  //int op = getRemote();
  //remote_read_button(&tvremote);
  do_action();
#endif

//  if (readvolt-- <= 0) {
//  // read the input on analog pin 0:
//    int lsensorValue = analogRead(A0);
//    int rsensorValue = analogRead(A1);
//    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
//    float lvoltage = lsensorValue * (5.0 / 1023.0);
//    float rvoltage = rsensorValue * (5.0 / 1023.0);
//    // print out the value you read:
//    Serial.print(lvoltage);
//    Serial.print("  ");
//    Serial.println(rvoltage);
//    readvolt = 10000;
//  }

//  motor_set_direction(motor, FORWARD);

  
  //---back and forth example 
  //analogWrite(ENABLE,255); //enable on    
//   Serial.println("forward");
//
//  digitalWrite(RIGHTA,HIGH);  //reverse
//  digitalWrite(RIGHTB,LOW);  
//  digitalWrite(RIGHTEN,HIGH); // enable on
//  delay(2000);
//  digitalWrite(ENABLE,LOW); // enable on
//  delay(1000);
//  Serial.println("now reverse");    
//  digitalWrite(DIRA,LOW);  //reverse
//  digitalWrite(DIRB,HIGH);
//  digitalWrite(ENABLE,HIGH); // enable on
//  delay(2000);    
//  digitalWrite(ENABLE,LOW); // enable on
//  delay(1000);    
//  digitalWrite(ENABLE,LOW); // disable
//  delay(2000);
//
//  Serial.println("fast Slow example");
//  //---fast/slow stop example
//  digitalWrite(ENABLE,HIGH); //enable on
//  digitalWrite(DIRA,HIGH); //one way
//  digitalWrite(DIRB,LOW);
//  delay(3000);
//  digitalWrite(ENABLE,LOW); //slow stop
//  delay(1000);
//  digitalWrite(ENABLE,HIGH); //enable on
//  digitalWrite(DIRA,LOW); //one way
//  digitalWrite(DIRB,HIGH);
//  delay(3000);
//  digitalWrite(DIRA,LOW); //fast stop
//  delay(2000);
//
//  Serial.println("PWM full then slow");
//  //---PWM example, full speed then slow
//  analogWrite(ENABLE,255); //enable on
//  digitalWrite(DIRA,HIGH); //one way
//  digitalWrite(DIRB,LOW);
//  delay(2000);
//  analogWrite(ENABLE,180); //half speed
//  delay(2000);
//  analogWrite(ENABLE,128); //half speed
//  delay(2000);
//  analogWrite(ENABLE,50); //half speed
//  delay(2000);
//  analogWrite(ENABLE,128); //half speed
//  delay(2000);
//  analogWrite(ENABLE,180); //half speed
//  delay(2000);
//  analogWrite(ENABLE,255); //half speed
//  delay(2000);
//  digitalWrite(ENABLE,LOW); //all done
//  delay(10000);
}
   
