#include "Arduino.h"
#include "pins_arduino.h"
#include "VescUart.h"

#include <stdlib.h>
#include <stdint.h>

#define CTRL_PIN_FWD_BWD    A1
#define CTRL_PIN_LEFT_RIGHT A0
#define CTRL_INVERT_FWD_BWD
// #define CTRL_INVERT_LEFT_RIGHT
#define CTRL_MEDIAN_SAMPLES 10
static int ctrl_cnt = 0;
static float arr_fb[CTRL_MEDIAN_SAMPLES] = {0};
static float arr_lr[CTRL_MEDIAN_SAMPLES] = {0};

#define CALIB_MIDPT_SAMPLES 100
static float mid_fb = 0.0f;
static float mid_lr = 0.0f;

#define CENTRED_VOLTAGE_ADC

VescUart vesc_left;
VescUart vesc_right;

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

float median(const float * arr, const int length)
{
  float cpy[length];
  memcpy(cpy, arr, length*sizeof(arr[0]));
  qsort(cpy, length, sizeof(arr[0]), sort_desc);
  return cpy[(int)(length/2)];
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  float arr_mid_fb[CALIB_MIDPT_SAMPLES] = {0};
  float arr_mid_lr[CALIB_MIDPT_SAMPLES] = {0};
  for(int i = 0; i < CALIB_MIDPT_SAMPLES; i++)
  {
    arr_mid_fb[i] = (float)analogRead(CTRL_PIN_FWD_BWD);
    arr_mid_lr[i] = (float)analogRead(CTRL_PIN_LEFT_RIGHT);
  }
  mid_fb = median(arr_mid_fb, CALIB_MIDPT_SAMPLES);
  mid_lr = median(arr_mid_lr, CALIB_MIDPT_SAMPLES);
  Serial1.begin(115200);
  Serial2.begin(115200);
  vesc_right.setSerialPort(&Serial1);
  vesc_left.setSerialPort(&Serial2);
}

void loop()
{
  float ai_fb = (float)analogRead(CTRL_PIN_FWD_BWD);
  float ai_lr = (float)analogRead(CTRL_PIN_LEFT_RIGHT);

  ai_fb -= mid_fb;
  ai_lr -= mid_lr;

  if(ctrl_cnt < CTRL_MEDIAN_SAMPLES)
  {
    arr_fb[ctrl_cnt] = ai_fb;
    arr_lr[ctrl_cnt] = ai_lr;
    ctrl_cnt++;
    return;
  }

  ai_fb = median(arr_fb, CTRL_MEDIAN_SAMPLES);
  ai_lr = median(arr_lr, CTRL_MEDIAN_SAMPLES);
  ctrl_cnt = 0;

  #ifdef CTRL_INVERT_FWD_BWD
    ai_fb *= -1;
  #endif
  #ifdef CTRL_INVERT_LEFT_RIGHT
    ai_lr *= -1;
  #endif

  // debug_print_analog_in(ai_fb, ai_lr);

  static float v_fb = 0, v_lr = 0, magnitude = 0, angle = 0;
  vectorize_analog_input(&magnitude, &angle, &v_fb, &v_lr, ai_fb, ai_lr);
  // debug_print_analog_in(v_fb, v_lr);
  float leftMotorSet = 0, rightMotorSet = 0;
  mix_motors(&leftMotorSet, &rightMotorSet, magnitude, angle, v_fb, v_lr);
  static float leftMotorDamped = 0, rightMotorDamped = 0;
#if 1
  float step = (magnitude > 0.2) ? 20 : 50;
  utils_step_towards(&leftMotorDamped, leftMotorSet, step);
  utils_step_towards(&rightMotorDamped, rightMotorSet, step);
#else
  leftMotorDamped = leftMotorSet;
  rightMotorDamped = rightMotorSet;
#endif

  #define DUTY_MAX_PERCENT 25
  float duty_left = (leftMotorDamped/128.0f)*(DUTY_MAX_PERCENT/100.0f);
  float duty_right = (rightMotorDamped/128.0f)*(DUTY_MAX_PERCENT/100.0f);

  vesc_right.setDuty(duty_left);
  vesc_left.setDuty(duty_right);

  Serial.print("M_L:");
  Serial.print(leftMotorDamped/127.0f);
  Serial.print(",M_R:");
  Serial.println(rightMotorDamped/127.0f);
}

void vectorize_analog_input(float *magnitude, float *angle, float *ptr_v_fb, float *ptr_v_lr,
                            float ai_fb, float ai_lr)
{
  float v_fb = ai_fb/512;
  float v_lr = ai_lr/512;
  v_fb = constrain(v_fb, -1.0, 1.0);
  v_lr = constrain(v_lr, -1.0, 1.0);

  *ptr_v_fb = v_fb;
  *ptr_v_lr = v_lr;

  *magnitude = sqrtf(sq(v_fb)+sq(v_lr));
  *magnitude = constrain(*magnitude, 0.0, 1.0);
  *angle = atan(v_lr/v_fb);
}

void mix_motors(float *ptr_leftMotor, float *ptr_rightMotor,
                float magnitude, float angle, float v_fb, float v_lr)
{

  float ratio_ml_mr = abs(angle)/HALF_PI;
  if (magnitude < 0.1f)
  {
    *ptr_leftMotor = 0;
    *ptr_rightMotor = 0;
    return;
  }
  // debug_print_ctrl(magnitude, ratio_ml_mr);

  float leftMotor = magnitude*127;
  float rightMotor = leftMotor;

  #define ALT_LIM_HIGH 0.8
  #define ALT_LIM_LOW (1.0-ALT_LIM_HIGH)

  if (ratio_ml_mr > ALT_LIM_LOW)
  {

    if(v_lr < 0)
    {
      ratio_ml_mr = 1.0 - ratio_ml_mr;
    }
  
    if ((ratio_ml_mr < ALT_LIM_HIGH) && (ratio_ml_mr > ALT_LIM_LOW))
    {
      leftMotor *= ratio_ml_mr;
      rightMotor *= (1.0f - ratio_ml_mr);
    }
    else
    {
      if(ratio_ml_mr >= ALT_LIM_HIGH)
      {
        leftMotor *= 0.5;
        rightMotor = -1*leftMotor;
      }
      if(ratio_ml_mr <= ALT_LIM_LOW)
      {
        rightMotor *= 0.5;
        leftMotor = -1*rightMotor;
      }
    }
  }

  if(v_fb < -0.1)
  {
    // reverse options
    leftMotor *= -0.7f;
    rightMotor *= -0.7f;
  }

  *ptr_leftMotor = leftMotor;
  *ptr_rightMotor = rightMotor;
}

void utils_step_towards(float *value, float goal, float step)
{
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

void debug_print_ctrl(float mag, float angle)
{
  Serial.println("Min:-1.6,Max:1.6");
  Serial.print("mag:");
  Serial.print(mag);
  Serial.print(",angle:");
  Serial.println(angle);
}

void debug_print_analog_in(float fb, float lr)
{
  // Serial.println("Min:-512,Max:512");
  Serial.print("fb:");
  Serial.print(fb);
  Serial.print(",lr:");
  Serial.println(lr);
}