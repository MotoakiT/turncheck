#ifndef ETRC22_DEVICE_IO_H_
#define ETRC22_DEVICE_IO_H_

#include "ev3api.h"
#include "time.h"

class MotorIo {
 public:
  MotorIo();
  ~MotorIo();
  void Update();
  void SetWheelsPower(int8_t power_l, int8_t power_r);
  void StopWheels(bool brake);
  void Rotate();
  void TestRun();
  void SaveRunTime();

  int32_t counts_l_;
  int32_t counts_r_;
  int8_t power_l_;
  int8_t power_r_;

 private:
  void ResetCounts();
  struct timespec now_time;
  int curr_index = 0;
  unsigned long secs_st[100000] = {};
  unsigned long secs_ed[100000] = {};
  int32_t angle_l[100000]={};
  int32_t angle_r[100000]={};
};

class SensorIo {
 public:
  SensorIo();
  ~SensorIo();
  void Update();
  void SaveRunTime();

  bool touch_sensor_pressed_;
  bool back_button_pressed_;
  rgb_raw_t color_rgb_raw_;

 private:
  struct timespec now_time;
  int curr_index = 0;
  unsigned long secs_st[100000] = {};
  unsigned long secs_ed[100000] = {};
  int32_t angle_l[100000]={};
  int32_t angle_r[100000]={};
};

class Camera {
 public:
  Camera();
  ~Camera();
  void Update();
};

#endif  // ETRC22_DEVICE_IO_H_