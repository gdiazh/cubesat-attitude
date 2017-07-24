/**
 * @brief HDD motor Device Library
 */

/*Author: Gustavo Diaz*/

/*Requiered Libraries*/
#include <Arduino.h>
#include <Servo.h>

/*System behavior*/
#define SET_POINT 0
#define JUMPING_VEL 1550
#define JUMPING_DELAY 4000

// Motor parameters
#define Km 0.0251
#define K_ESC 100
#define MAX_TORQUE 600

typedef struct StateVariables{
    float hdd_angle;
    float hdd_vel;
} StateVariables;

typedef struct OutputVariables{
    float output_vel;
    float output_pwm;
    int output_dir;
} OutputVariables;

/**
 * @class HddDriver
 * @brief Class for manage HDD motor with an ESC.
 */

class HddDriver
{
    /*Members*/
    uint8_t esc_pwm_pin_;
    uint8_t esc_dir_pin_;

    Servo esc_;

    StateVariables state_vars_;
    OutputVariables output_vars_;

    HardwareSerial *debug_port_;

public:
    /*Public Members*/

    /*constructor de base (null)*/
    HddDriver() {}

    // constructror parametrizado
    HddDriver(uint8_t esc_pwm_pin, uint8_t esc_dir_pin, HardwareSerial *debug_port):
        esc_pwm_pin_(esc_pwm_pin),
        esc_dir_pin_(esc_dir_pin),
        debug_port_(debug_port)
    {
        // Config pins
        pinMode(esc_pwm_pin_, OUTPUT);
        pinMode(esc_dir_pin_, OUTPUT);
        // esc_.attach(esc_pwm_pin_);
    }

    // methods
    void init(void);
    void idle();
    void init_esc();
    void rotate(float vel);
    // void printVariables(void);

private:
    // methods
    
    // Declaration for Math functions
    float torque2pwm(float value, float toLow, float toHigh);

    // Declaration of Output Handlers
    float motor_dynamic(float torque);
    int pwm_modulation(float vel_control);
    uint8_t direction_handler(float vel_control);
};