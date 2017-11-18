/**
 * @brief HDD motor Device Library
 */
 
/*Author: Gustavo Diaz*/

#include "hdd_driver.h"

//-------------------------- Define Math functions -----------------------------------------
float HddDriver::torque2pwm(float value, float toLow, float toHigh)
{
  float fromLow = 0;
  float fromHigh = MAX_TORQUE;
  float new_range = toHigh-toLow;
  float old_range = fromHigh - fromLow;
  if (value < -fromHigh || value > fromHigh)
  {
    // debug_port_->println("Warning value out of range");
    value = fromHigh;
  }
  if (value<0) value = -value;
  float new_value = (new_range/old_range)*(value-fromLow)+toLow;
  return new_value;
}

//-------------------------- Define Handlers functions -----------------------------------------------
float HddDriver::motor_dynamic(float torque)
{
    int vel = torque; //TO DO: REPLACE FOR PARTICULAR DYNAMICS: w = Te/(Js+B)
    return vel;
}

void HddDriver::init_esc()
{
    esc_.writeMicroseconds(1000);   //1000[us] = 1[ms]
    delay(5000);           //necesary to init the ESC
}

int HddDriver::voltage_to_pwm(float volage)
{
    float pwm = 58.8536*volage*volage-205.0240*volage+1618.0829;
    int pwm_ms = (int) pwm;
    return pwm_ms;
}

uint8_t HddDriver::direction_handler(float volage)
{
    if (volage>=0) return 1;
    else return 0;
}

//-------------------------- Public High Level Abstract Methods --------------------------
void HddDriver::init(void)
{
    /*Assign ouput pins*/
    // pinMode(esc_pwm_pin_, OUTPUT);
    // pinMode(esc_dir_pin_, OUTPUT);

    esc_.attach(esc_pwm_pin_);
    init_esc();

    /*initialize state variables*/
    state_vars_.hdd_angle = 0;       //[rad/s]
    state_vars_.hdd_vel = 0;         //[rad/s]

    // initialize output variables
    output_vars_.output_voltage = 0;     //[V]
    output_vars_.output_pwm = 1000;      //[us]
    output_vars_.output_dir = 0;         //[0:right, 1:left]
}

void HddDriver::idle()
{
    esc_.writeMicroseconds(1000);
}

void HddDriver::rotate(float volage)
{
    output_vars_.output_voltage = volage;
    output_vars_.output_pwm = voltage_to_pwm(output_vars_.output_voltage);
    output_vars_.output_dir = direction_handler(output_vars_.output_voltage);
    /*Update outputs*/
    // debug_port_->print("output_voltage: ");
    // debug_port_->println(output_vars_.output_voltage);
    // debug_port_->print("output_dir: ");
    // debug_port_->println(output_vars_.output_dir);
    if (output_vars_.output_dir == 1) digitalWrite(esc_dir_pin_, LOW);
    else digitalWrite(esc_dir_pin_, HIGH);
    if (output_vars_.output_pwm>=min_pwm_ && output_vars_.output_pwm<=max_pwm_) esc_.writeMicroseconds(output_vars_.output_pwm);
    // esc_.writeMicroseconds((int) velocity);// ---------------------------------------------------------------------------------TEST!!!!!!!!!!!!!!!!!!!!!!!!!-------------------------------------
}

/*void HddDriver::printVariables(void)
{
    debug_port_->print(state_vars_.hdd_angle*RAD_A_DEG);
    debug_port_->print(";");
    debug_port_->print(state_vars_.hdd_vel*RAD_A_DEG);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_voltage);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_pwm);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_dir);
    debug_port_->println(";");
}*/