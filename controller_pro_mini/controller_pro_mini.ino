/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>

/**
 * @brief Arduino Driver v2.0 to control yaw angle position using DMP6 example
 */
 
/*Modification to DMP example: Gustavo Diaz*/

/*Requiered Libraries*/
#include "hdd_driver.h"
// #include <TimerThree.h>
// #include <TimerOne.h>

#include <Filters.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===               FRAME TEST PARAMS                          ===
// ================================================================

#define SET_SPEED 1
#define KEEP_ATTITUDE 2
#define SET_MODE 3
#define STOP 4
#define USE_CURRENT_SETPOINT 5
#define INCREASE_SETPOINT 6
#define DECREASE_SETPOINT 7
#define PRINT_SPEED 8
#define AUTOMATIC_MODE 9

float a;
float b;
uint8_t t;

// ================================================================
// ===               Actuators params                           ===
// ================================================================

/*Output pins for ESC control*/
#define ESC_PWM_PIN_OUT 5
#define ESC_DIR_PIN_OUT 4

#define ESC2_PWM_PIN_OUT 6
#define ESC2_DIR_PIN_OUT 7

#define ESC3_PWM_PIN_OUT 9
#define ESC3_DIR_PIN_OUT 8

#define OFSET 2.1

/*Device Control Handler*/
HddDriver hdd(ESC_PWM_PIN_OUT, ESC_DIR_PIN_OUT, 1300, 1850, &Serial);
HddDriver hdd2(ESC2_PWM_PIN_OUT, ESC2_DIR_PIN_OUT, 1000, 1500, &Serial);

int vel = 0;
int new_vel = 0;
float calc_vel = 0;
uint8_t mode = 0;

// ================================================================
// ===               Sensor PARAMS                              ===
// ================================================================

#define HALL1 2
#define HALL2 16
#define HALL3 17

unsigned long time_ref = 0;
volatile uint8_t steps = 0;
float speed_rpm = 0.0;
float filtered_speed = 0.0;

float speedFilterFrecuency = 1; //[Hz]
FilterOnePole lowpassFilter(LOWPASS, speedFilterFrecuency);

// ================================================================
// ===               PID PARAMS                                 ===
// ================================================================

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=5, Ki=1, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// ===               Comunication                               ===
// ================================================================
uint8_t Timeout = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    a = 0;
    b = 0;
    t = 0;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(115200);
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // while (!Serial2);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    /*while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready!"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // PID Initialization
    //initialize the variables we're linked to
    Input = 0;
    Setpoint = -100.0;
    myPID.SetOutputLimits(-700, 700);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    //Hall Encoder pins
    pinMode(HALL3, INPUT);
    attach_halls();

    //Init HDD
    hdd.init();
    hdd2.init();
    // Serial.println("HDD ready!");
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // Serial.print("quat\t");
            // Serial.print(q.w);
            // Serial.print("\t");
            // Serial.print(q.x);
            // Serial.print("\t");
            // Serial.print(q.y);
            // Serial.print("\t");
            // Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            // Serial.print("euler\t");
            // Serial.print(euler[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(euler[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            // Serial.print("areal\t");
            // Serial.print(aaReal.x);
            // Serial.print("\t");
            // Serial.print(aaReal.y);
            // Serial.print("\t");
            // Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            // Serial.print("aworld\t");
            // Serial.print(aaWorld.x);
            // Serial.print("\t");
            // Serial.print(aaWorld.y);
            // Serial.print("\t");
            // Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            // Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    // updateSetpoint();
    update_speed();
    filtered_speed = lowpassFilter.input(speed_rpm);

    Input = ypr[0] * 180/M_PI;
    myPID.Compute();

    float var1 = Input;
    float var2 = 0;
    float var3 = speed_rpm;
    float var4 = filtered_speed;

    main_behavior();
    send_data(var1, var2, var3, var4);
}

void sendFrame(uint8_t frame[], uint8_t sz)
{   
    for (int j=0;j<sz;j++) Serial.write(frame[j]);
}

uint8_t checksum(uint8_t *packet, uint8_t n)
{
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00FF;
}

void bytesEncode(float number, uint8_t encode_bytes[])
{
    uint16_t int_part = (uint16_t) abs(number);           //[0-65535]
    float decimal_part = (abs(number) - int_part)*100;   //[0-99]

    uint8_t NH = (int_part)>>8;                //Number High Byte
    uint8_t NL = (int_part) & 0x00FF;          //Number Low Byte
    uint8_t D = (int)decimal_part;             //Decimal part (7 bits)
    uint8_t SD = D;                                 //Sign and Decimal Byte
    if (number<=0) SD = D|0b10000000;               //Sign bit

    encode_bytes[0] = NH;
    encode_bytes[1] = NL;
    encode_bytes[2] = SD;
}

void encode(float data[], uint8_t packet[])
{
    uint8_t num1_bytes[3];
    uint8_t num2_bytes[3];
    uint8_t num3_bytes[3];
    uint8_t num4_bytes[3];
    bytesEncode(data[0], num1_bytes);
    bytesEncode(data[1], num2_bytes);
    bytesEncode(data[2], num3_bytes);
    bytesEncode(data[3], num4_bytes);

    packet[0] = num1_bytes[0];
    packet[1] = num1_bytes[1];
    packet[2] = num1_bytes[2];

    packet[3] = num2_bytes[0];
    packet[4] = num2_bytes[1];
    packet[5] = num2_bytes[2];

    packet[6] = num3_bytes[0];
    packet[7] = num3_bytes[1];
    packet[8] = num3_bytes[2];

    packet[9] = num4_bytes[0];
    packet[10] = num4_bytes[1];
    packet[11] = num4_bytes[2];

    packet[12] = checksum(packet, 13);
}

void printFrame(uint8_t frame[], uint8_t sz)
{
    // Serial.print("frame = [");
    for (int i = 0; i < sz-1; i++)
    {
        // Serial.print(frame[i]);
        // Serial.print(",");
    }
    // Serial.print(frame[sz-1]);
    // Serial.println("]");
}

void printCommand(float com[], uint8_t sz)
{
    // Serial.print("command = [");
    for (int i = 0; i < sz-1; i++)
    {
        // Serial.print(com[i]);
        // Serial.print(",");
    }
    // Serial.print(com[sz-1]);
    // Serial.println("]");
}

void send_data(float D1, float D2, float D3, float D4)
{
    float data[4] = {D1, D2, D3, D4};
    uint8_t frame_test[13];
    encode(data, frame_test);
    sendFrame(frame_test, 13);
    // printFrame(frame_test, 13);
}

void updateSetpoint(void)
{
    if(Serial.available() >= 1)
    {
        Setpoint = Serial.parseInt();
        // Serial.print("Setpoint = "); Serial.println(Setpoint);
    }
}

float getNumber(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    // Numer Elements
    uint16_t int_part = (byte1<<8)|byte2;
    uint8_t sign = byte3>>7;
    uint16_t decimal_part = byte3&(0b01111111);
    // Number
    float number;
    if (sign == 0) number = int_part+((float)decimal_part/100);
    else number = -(int_part+((float)decimal_part/100));
    return number;
}

float saturate(float data, int16_t MIN, int16_t MAX)
{
    if (data>MAX)
        return MAX;
    else if (data<MIN)
        return MIN;
    else
        return data;
}

void decode(uint8_t frame[], float data[])
{
    data[0] = saturate(getNumber(frame[0], frame[1], frame[2]), 0, 100);
    data[1] = saturate(getNumber(frame[3], frame[4], frame[5]), -800, 800);
    data[2] = saturate(getNumber(frame[6], frame[7], frame[8]), -360, 360);
    data[3] = saturate(getNumber(frame[9], frame[10], frame[11]), -360, 360);
}

void stop_read(void)
{
    // Serial.print("Stop Read");
    // Timer1.detachInterrupt();
    Timeout = 1;
}

uint8_t read(uint8_t frame[])
{
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t sz = 13;
    // Serial.print("r");
    // Timer1.initialize(30000);
    // Timer1.attachInterrupt(stop_read);
    while (k < 2*sz)// and !Timeout)
    // while (Serial.available() >= 1)
    {
        // Serial.print(">");
        if (Serial.available() >= 1)
        {
            uint8_t byte = Serial.read();
            frame[i] = byte;
            // Serial.println(byte);
            i+=1;
            if (i==sz)
            {
                uint8_t chksm = checksum(frame, sz);
                if (chksm == frame[sz-1] && chksm !=0)
                {
                    printFrame(frame, 13);
                    return 1; // packet received OK
                }
                else
                {
                    // Bad checksum
                    printFrame(frame, 13);
                    // Serial.println("Bad checksum");
                    for (uint8_t j = 0; j < sz-1; j++)
                    {
                        frame[j] = frame[j+1]; // Shift frame Left
                    }
                    frame[sz-1] = 0; //Clean last byte to receive other packet
                    i = sz-1;
                }
            }
            k+=1;
        }
    }
    // Frame not received Correctly
    // Serial.println("Frame Lost");
    for (uint8_t j = 0; j < sz; j++) frame[j] = 0; // Reset packet
    // Timeout = 0;
    while(Serial.available()) Serial.read();
    return 0;
}

void main_behavior()
{
  // Serial.println("M");
  /* test behavior */
  if (mode == 0)
  {
    hdd.rotate(vel);
    hdd2.rotate(vel);
  }
  else if (mode == 1)
  {
    hdd.rotate(calc_vel);
    hdd2.rotate(calc_vel);
  }

  //Calcs
  calc_vel = Output;

  if(Serial.available() >= 1)
  {
    /*new_vel = Serial.parseInt(); //Leer un entero por serial
    Serial.print("cmd: ");Serial.println(new_vel);*/
    uint8_t frame[13];
    float command[4];
    read(frame);
    decode(frame, command);
    // printFrame(frame, 13);
    // printCommand(command, 4);
    if(command[0]==SET_SPEED)
    {
      mode = 0;
      vel = command[1];
      // Serial.print("New speed set: ");
      // Serial.println(vel);
    }
    else if (command[0]==KEEP_ATTITUDE)
    {
      Setpoint = command[1];
      // Serial.println("change set point");
    }
    else if (command[0]==SET_MODE)
    {
      mode = command[1];
      // Serial.println("change mode");
    }
    else if (command[0]==STOP)
    {
      mode = 0;
      vel = 0;
      hdd.idle();
      hdd2.idle();
      // Serial.println("Motor Stoped");
    }
    else if (command[0]==PRINT_SPEED)
    {
      // Serial.print("Motor speed: ");
      // if (mode == 0) Serial.println(vel);
      // else if (mode == 1) Serial.println(calc_vel);
    }
    else if (command[0] == AUTOMATIC_MODE)
    {
      mode = 1;
      // Serial.print("Balance Mode");
      // Serial.print("Motor speed: "); Serial.println(vel);
    }
    else if (command[0] == USE_CURRENT_SETPOINT)
    {
      Setpoint = Input;
      // Serial.println("using current pos as setpoint");
    }
    else if (command[0] == INCREASE_SETPOINT)
    {
      Setpoint = Setpoint+10;
      // Serial.println("Increase setpoint");
    }
    else if (command[0] == DECREASE_SETPOINT)
    {
      Setpoint = Setpoint-10;
      // Serial.println("decrease setpoint");
    }
    // Serial.print(Input); Serial.print("\t\t\t"); Serial.println(Output);
  }
}

void attach_halls(void)
{
    // attachInterrupt(digitalPinToInterrupt(HALL1), step, FALLING);
    // attachInterrupt(digitalPinToInterrupt(HALL2), step, FALLING);
    attachInterrupt(digitalPinToInterrupt(HALL3), step, FALLING);
}

void dettach_halls(void)
{
    // detachInterrupt(digitalPinToInterrupt(HALL1));
    // detachInterrupt(digitalPinToInterrupt(HALL2));
    detachInterrupt(digitalPinToInterrupt(HALL3));
}

void update_speed(void)
{
    if (steps)
    {
        // Serial.print("steps:");Serial.println(steps);
        dettach_halls();
        speed_rpm = 30000.0*steps/(millis()-time_ref);
        // Serial.print("speed_rpm:");Serial.println(speed_rpm);

        time_ref = millis();
        steps = 0;

        attach_halls();
    }
}

void step(void)
{
    steps++;
}