/**
 * @brief Arduino Driver v1.0 to control and HDD brushless motor with an ESC
 */
 
/*Author: Gustavo Diaz*/

/*Requiered Libraries*/
#include "hdd_driver.h"
#include "Mpu6050.h"

/*Output pins for ESC control*/
#define ESC_PWM_PIN_OUT 2
#define ESC_DIR_PIN_OUT 6

#define OFSET_PITCH 0.86
#define OFSET_ROLL -0.27

// SoftwareSerial Bluetooth(7,8);    // RX, TX Arduino PINs, conected to Bluetooth

/*Bluetooth Comunication Interface*/
// char command = 's';
// comInterface com_interface(&Bluetooth, &command, &Serial);

/*Device Control Handler*/
HddDriver hdd(ESC_PWM_PIN_OUT, ESC_DIR_PIN_OUT, &Serial);
Mpu6050 mpu;

int vel = 0;
int new_vel = 0;
float calc_vel = 0;
uint8_t mode = 0;

void setup()
{
	/*Init Serial port*/
	Serial.begin(115200);
  // Bluetooth.begin(115200);

  //Init HDD
  hdd.init();
  Serial.println("HDD ready!");

  //Init MPU
  mpu.init();
  Serial.println("MPU ready!");
}

void loop()
{
  /* test behavior */
  if (mode == 0) hdd.rotate(vel);
  else if (mode == 1) hdd.rotate(calc_vel);
  mpu.updateAngle();

  //Calcs
  calc_vel = (mpu.angle_[1]+OFSET_PITCH)*5;

  if(Serial.available() >= 1)
  {
    new_vel = Serial.parseInt(); //Leer un entero por serial
    if(new_vel != 0 and new_vel != -1 and new_vel != -2 and new_vel != -3)
    {
      mode = 0;
      vel = new_vel;
      Serial.print("New speed set: ");
      Serial.println(vel);
      Serial.print(mpu.angle_[0]+OFSET_ROLL); Serial.print(";"); Serial.println(mpu.angle_[1]+OFSET_PITCH);
    }
    else if (new_vel == -1)
    {
      mode = 0;
      vel = 0;
      hdd.idle();
      Serial.println("Motor Stoped");
      Serial.print(mpu.angle_[0]+OFSET_ROLL); Serial.print(";"); Serial.println(mpu.angle_[1]+OFSET_PITCH);
    }
    else if (new_vel == -2)
    {
      Serial.print("Motor speed: ");
      if (mode == 0) Serial.println(vel);
      else if (mode == 1) Serial.println(calc_vel);
      Serial.print(mpu.angle_[0]+OFSET_ROLL); Serial.print(";"); Serial.println(mpu.angle_[1]+OFSET_PITCH);
    }
    else if (new_vel == -3)
    {
      mode = 1;
      Serial.print("Balance Mode");
      Serial.print("Motor speed: "); Serial.println(vel);
      Serial.print(mpu.angle_[0]+OFSET_ROLL); Serial.print(";"); Serial.println(mpu.angle_[1]+OFSET_PITCH);
    }
  }
}