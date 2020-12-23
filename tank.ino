//www.elegoo.com
//2017.12.12

/************************
Exercise the motor using
the L293D chip
************************/

#define IR_ENABLE 1
#define SR_ENABLE 1

#include "IRremote.h"
#define RECIEVER 7

#define LIGHT_INPUT A3

#define RIGHTA 12
#define RIGHTB 4
#define RIGHTEN 3

#define LEFTA 5
#define LEFTB 2
#define LEFTEN 6

#define SERVO_PIN 9

#if SR_ENABLE
//int i;
#include "SR04.h"
#define TRIG_PIN 10
#define ECHO_PIN 11
//SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
//long a;
#endif
#define MAX_DISTANCE 5000

unsigned long CIRCLE_TURN_AFTER = 2000;

#include <Servo.h>

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

typedef struct {
  SR04 sr04;
} sr_sensor_t;

typedef struct {
  int pin;
} light_sensor_t;

typedef struct {
  int servo_pin;
  sr_sensor_t eyes;
  light_sensor_t lsensor;
  turn_t orientation;
  int degree;
  Servo servo;
} head_t;

enum function_t {
  FUNCTION_UNKNOWN = 0,
  FUNCTION_REMOTE,
  FUNCTION_EYES,
  FUNCTION_EYES_CIRCLE,
  FUNCTION_FOLLOW_LIGHT,
};

typedef struct {
  head_t head;
  function_t function;
  turn_t turning;
  int turning_degree;
  unsigned long time_since_turn;
} robot_t;

robot_t robot {
  .head = {
    .servo_pin = SERVO_PIN,
    .eyes = {
      .sr04 = SR04(ECHO_PIN,TRIG_PIN),
    },
    .lsensor = {
      .pin = LIGHT_INPUT,
    },
    .orientation = TURN_UNKNOWN,
  },
  .function = FUNCTION_REMOTE,
};

enum BUTTON_T {
  BUTTON_UNKNOWN = 0,
  POWER,
  UP,
  DOWN,
  VOLUP,
  VOLDOWN,  
  FAST_BACK,
  FAST_FORWARD,
  NUM_1,
  NUM_2,
  NUM_3,
  NUM_4,
  NUM_5,
  NUM_6,
  NUM_7,
  NUM_8,
  NUM_9,
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
  case 0xFF30CF: ret = NUM_1; Serial.println("1");    break;
  case 0xFF18E7: ret = NUM_2; Serial.println("2");    break;
  case 0xFF7A85: ret = NUM_3; Serial.println("3");    break;
  case 0xFF10EF: ret = NUM_4; Serial.println("4");    break;
  case 0xFF38C7: ret = NUM_5; Serial.println("5");    break;
  case 0xFF5AA5: ret = NUM_6; Serial.println("6");    break;
  case 0xFF42BD: ret = NUM_7; Serial.println("7");    break;
  case 0xFF4AB5: ret = NUM_8; Serial.println("8");    break;
  case 0xFF52AD: ret = NUM_9; Serial.println("9");    break;
  case 0xFFFFFFFF: ret = remote->last_button;/* Serial.println(" REPEAT");*/break;  

//  default: 
    //return -1;
//    Serial.println(" other button   ");

  }// End Case

  return ret;
}

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
    //Serial.println(" reseting last button   ");
    remote->last_button = BUTTON_UNKNOWN;
    remote->last_button_count = 0;
  }

  return remote->last_button;
}

long sr_distance(sr_sensor_t *sensor) {
  return sensor->sr04.Distance();
}

int light_read(light_sensor_t *light) {
  int val = 0;
  int count = 50;
  while (--count > 0) {
    val = (val + analogRead(light->pin)) / 2;
  }

  return val;
}

void head_setup(head_t *head) {
  head->servo.attach(head->servo_pin);
}

void head_turn(head_t *head, turn_t towards, int degree) {
  bool dodelay = (head->orientation != towards) || (head->degree != degree);
  head->orientation = towards;
  head->degree = degree;
  int angle = 89;
  switch(towards) {
    case LEFT:
      angle = 90 + degree;
    break;
    case RIGHT:
      angle = 90 - degree;
    break;
    default:
      head->degree = 0;
  }

  if (dodelay) {
    head->servo.write(angle);
    delay(500);
  }
}

long head_distance(head_t *head, turn_t towards, int degree) {
  head_turn(head, towards, degree);  
  return sr_distance(&head->eyes);
}

bool head_find_light(head_t *head, turn_t *towards) {
  const int diff_thres = 80;
  head_turn(head, LEFT, 90);
  int left = light_read(&head->lsensor);
  head_turn(head, TURN_UNKNOWN, 0);
  int forward = light_read(&head->lsensor);
  head_turn(head, RIGHT, 90);
  int right = light_read(&head->lsensor);

  int diff = left - right;
  *towards = TURN_UNKNOWN;

//  Serial.print(left);
//  Serial.print(" ");
//  Serial.print(right);
//  Serial.print(" ");
//  Serial.print(forward);
//  Serial.print(" ");
//  Serial.print("diff: ");
//  Serial.print(diff);
//  Serial.print(" ");

  if (left < 5 && right < 5 && forward < 5)
    return false;

  int maxsides = left;
  if (diff < -15) {
    maxsides = right;
    *towards = RIGHT;
//    Serial.println("turn right");
  } else if (diff > 15) {
    *towards = LEFT;
//    Serial.printlnSerial.print(left);
//  Serial.print(" ");
//  Serial.print(right);
//  Serial.print(" ");
//  Serial.print("diff: ");
//  Serial.print(diff);
//  Serial.print(" ");("turn left");
  } else {
//    Serial.println("don't turn");
  }

  if ((forward - maxsides) > 50) {
    *towards = TURN_UNKNOWN;
  }

//  Serial.println("");
  return true;
  //return sr_distance(&head->eyes);
}

void motor_set_speed(motor_t *motor, int speed) {
  motor->speed = speed;
  analogWrite(motor->en, motor->speed);
}

void motor_set_direction(motor_t *motor, enum dir_t dir) {
  motor->dir = dir;

  bool forward = (dir == FORWARD);
  digitalWrite(motor->a,forward?HIGH:LOW);
  digitalWrite(motor->b,forward?LOW:HIGH);
//  Serial.println("go forward");
}

void setup_motor(motor_t *motor) {
  pinMode(motor->en,OUTPUT);
  pinMode(motor->a,OUTPUT);
  pinMode(motor->b,OUTPUT);
  motor_set_speed(motor, 0);
  motor_set_direction(motor, FORWARD);
}

void turn(turn_t towards) {
  const int turn_speed = 255;
  motor_set_direction(&right, FORWARD);
  motor_set_direction(&left, FORWARD);
  switch (towards) {
    case LEFT:
      motor_set_speed(&left, 0);
      motor_set_speed(&right, turn_speed);
//      Serial.println("go left");
    break;
    case RIGHT:
    motor_set_speed(&right, 0);
    motor_set_speed(&left, turn_speed);
//    Serial.println("go right");
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

void do_setup(robot_t *robot) {
  setup_motor(&right);
  setup_motor(&left);
  head_setup(&robot->head);
}

void do_remote_action(BUTTON_T buttonpress) {
  //irrecv.enableIRIn(); // Start the receiver  
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
}

bool is_turning_blocked(long dis) {
  return (dis <= 5 || dis > MAX_DISTANCE);
}

bool is_blocked(long dis) {
  return (dis <= 7 || dis > MAX_DISTANCE);
}

bool can_turn(long dis) {
  return (dis >=20 && dis <= MAX_DISTANCE);
}


void do_light_action(robot_t *robot) {
  stopit();
  turn_t towards;
  bool start = head_find_light(&robot->head, &towards);
  if (start) {
    if (towards != TURN_UNKNOWN) {
      turn(towards);
      delay(500);
    } else {
      set_direction(FORWARD);
      delay(1000);
    }
  } else {
    robot->function = FUNCTION_REMOTE;
//    delay(5000);
  }
}

void do_eyes_action(robot_t *robot) {
  long dis;
  bool force_turn = false;
  if (robot->turning != TURN_UNKNOWN) {
      turn_t other_side = (robot->turning == LEFT)?RIGHT:LEFT;
      dis = head_distance(&robot->head, other_side, robot->turning_degree);
      if (is_turning_blocked(dis)) {
        turn(robot->turning);
        return;
      }

      if (robot->turning_degree <= 75) {
        robot->turning_degree += 15;
        stopit();
        dis = head_distance(&robot->head, other_side, robot->turning_degree);
        return;
      }

      robot->turning_degree = 0;
      robot->turning = TURN_UNKNOWN;
      set_direction(FORWARD);
      robot->time_since_turn = millis();
      //Serial.println("Stop.. now go forwa rd");
  } else if (robot->function == FUNCTION_EYES_CIRCLE) {
    if ((robot->time_since_turn + CIRCLE_TURN_AFTER) <= millis()) {
      force_turn = true;
      robot->time_since_turn = millis();
    }
  }
  /* can go forward */
  dis = head_distance(&robot->head, TURN_UNKNOWN, 0);
  //Serial.print("forward: ");
  //Serial.println(dis);

  /* cannot go forward */
  if (is_blocked(dis) || force_turn) {
    stopit();
    dis = head_distance(&robot->head, LEFT, 90);
    //Serial.print("left: ");
    //Serial.println(dis);

    if (can_turn(dis)) {
      /* going left */
      robot->turning = LEFT;
      robot->turning_degree = 0;
      turn(LEFT);      
      robot->time_since_turn = millis();
      //Serial.println("Going Left");
    } else {
      dis = head_distance(&robot->head, RIGHT, 90);
      //Serial.print("Right: ");
      //Serial.println(dis);

      if (can_turn(dis)) {
        //Serial.println("Going Right");
        robot->turning = RIGHT;
        robot->turning_degree = 0;
        turn(RIGHT);
        robot->time_since_turn = millis();
      }
    }
//    stop();
  }
}

int readvolt = 500;
void setup() {
  Serial.begin(9600); 
  do_setup(&robot);
  remote_setup(&tvremote);
}

void loop() {
//  long dis = head_distance(&robot.head, TURN_UNKNOWN, 0);
//  dis = head_distance(&robot.head, LEFT, 90);
//  dis = head_distance(&robot.head, TURN_UNKNOWN, 0);
//  Serial.print("forward: ");
//  Serial.println(dis);
//  return;
//
// turn(LEFT);
//  stopit();
//  set_direction(FORWARD);
//  return;

  BUTTON_T buttonpress = remote_read_button(&tvremote);

  if (robot.function == FUNCTION_EYES_CIRCLE) {
    switch (buttonpress) {
      case NUM_1:
      case NUM_2:
      case NUM_3:
      case NUM_4:
      case NUM_5:
      case NUM_6:
      case NUM_7:
      case NUM_8:
      case NUM_9:
        CIRCLE_TURN_AFTER = (buttonpress - NUM_1) * 1000;
        buttonpress = 0;
    }
  }

  switch(buttonpress) {
    case POWER:
      stopit();
      robot.function = FUNCTION_REMOTE;
    break;
    case NUM_1:
      robot.turning = TURN_UNKNOWN;
      robot.function = FUNCTION_EYES;
      set_direction(FORWARD);
      break;
    case NUM_2:
      robot.turning = TURN_UNKNOWN;
      robot.time_since_turn = 0;
      robot.function = FUNCTION_EYES_CIRCLE;
      set_direction(FORWARD);
      break;
    case NUM_3:
      robot.turning = TURN_UNKNOWN;
      robot.function = FUNCTION_FOLLOW_LIGHT;
      break;
  }

  if (robot.function == FUNCTION_EYES
        || robot.function == FUNCTION_EYES_CIRCLE) {
    do_eyes_action(&robot);
  } else if (robot.function == FUNCTION_FOLLOW_LIGHT) {
    do_light_action(&robot);
  } else {
    do_remote_action(buttonpress);
  }
}
   
