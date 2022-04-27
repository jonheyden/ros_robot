#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Sabertooth.h>
#include <arduino-timer.h>


auto timer = timer_create_default(); // create a timer with default settings

/* assign a unique id to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


// connect to the two encoder outputs!
#define ENCODER_RIGHT_A   2
#define ENCODER_RIGHT_B   4
#define MOTOR_RIGHT_FWD 0
#define MOTOR_RIGHT_REV 1
#define MOTOR_RIGHT 1


#define ENCODER_LEFT_A   3
#define ENCODER_LEFT_B   5
#define MOTOR_LEFT_FWD 4
#define MOTOR_LEFT_REV 5
#define MOTOR_LEFT 0

#define DRIVE_LIMIT 126
#define MOTOR_ADRS 128

#define INPUT_SIZE 10
#define FREQ 10 // hz to print data to serial port
#define INTEGRAL_LIMIT 100

int leftCmd=0;
int leftCmdDestroy=0;
int rightCmd=0;
int rightCmdDestroy=0;
float Kp=2; // proportional gain
float Ki=1; // integral gain
float leftInt = 0;
float rightInt = 0;
float leftErr=0;
float rightErr=0;
float leftDrive = 0;
float rightDrive = 0;
int readIndex = 0;
char readBuffer[INPUT_SIZE];
int lastCommandTime = 0;
volatile int leftCount=0;
volatile int rightCount=0;
bool readComplete = false;  // whether the string is complete

void interrupt_right_A() {
  if(digitalRead(ENCODER_RIGHT_B)) {
    rightCount++;  
  } else {
    rightCount--;
  }
}
void interrupt_left_A() {
  if(digitalRead(ENCODER_LEFT_B)) {
    leftCount--;  
  } else {
    leftCount++;
  }
}
bool on_timer(void *);
//  connect pin d13 to the motor controller s1
SoftwareSerial motorSerial(12,13);
Sabertooth ST(128, motorSerial);
void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), interrupt_right_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), interrupt_left_A, RISING);
  motorSerial.begin(9600);

  ST.autobaud();
   /* enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();
  timer.every(100, on_timer);

}

int limit(int val, int max, int min) {
  if(val>max) val=max;
  else if(val<min) val=min;
  return val;
}

void loop() {
  timer.tick();
}


//This gets called every 1/10 of a second, every 100ms
bool on_timer(void *) {
  int l=leftCount;
  int r=rightCount;
  leftCount = rightCount = 0;
  if (readComplete) {

    // clear the string:
    char *sep = strchr(readBuffer, ',');
    if(sep != NULL) {
      leftCmd = atoi(readBuffer);
      leftCmdDestroy = leftCmd;
      rightCmd = atoi(++sep);
      rightCmdDestroy = rightCmd;
    }
    readIndex = 0;
    readComplete = false;
    lastCommandTime = 0;
  }
  if(lastCommandTime++ >= 11) {
    leftCmd=rightCmd=0;
  }

//Linear Regression for motors

  if (leftCmd != 0) {
     if (leftCmd > 0){
        leftDrive=(leftCmd * 3.625) + 30.83;
     } else {
        leftDrive=(leftCmd * 3.625) - 30.83;
     }
  } else {
     leftDrive = 0;
  }
  
  if (rightCmd != 0) {
     if (rightCmd > 0) {
        rightDrive=(rightCmd * 3.625) + 30.83;
     } else {
        rightDrive=(rightCmd * 3.625) - 30.83;
     }
  } else {
     rightDrive=0;
  }
  

  leftDrive = limit(leftDrive,127,-127);
  rightDrive = limit(rightDrive,127,-127);

  ST.motor(2, -leftDrive);
  ST.motor(1, rightDrive);
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  sensors_event_t acc;
  accel.getEvent(&acc);


  Serial.print(l);
  Serial.print(",");
  Serial.print(r);
  
  /* (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print(",");
  Serial.print(event.magnetic.x);
  Serial.print(",");
  Serial.print(event.magnetic.y);
  Serial.print(",");
  Serial.print(event.magnetic.z);
  Serial.print(",");
  Serial.print(acc.acceleration.x);
  Serial.print(",");
  Serial.print(acc.acceleration.y);
  Serial.print(",");
  Serial.print(acc.acceleration.z);
  Serial.print(",");
  Serial.print((int)leftCmd);
  Serial.print(",");
  Serial.print((int)rightCmd);
  Serial.print(",");
  Serial.print((int)leftDrive);
  Serial.print(",");
  Serial.print((int)rightDrive);
  Serial.println("");
  return true;
}

void serialEvent() {
  while (Serial.available()) {
    //make sure we don't overflow the buffer
    if(readIndex >= INPUT_SIZE) {
      readComplete = true;
      return;
    }
    // get the new byte:
    char ch;
    size_t num = Serial.readBytes(&ch,1);
    // if its a new line then that is the end of the string
    if (ch == '\n') {
      readBuffer[readIndex++] = 0;
      readComplete = true;
    }
    // otherwise add it to the buffer:
    readBuffer[readIndex++] = ch;
  }
}
