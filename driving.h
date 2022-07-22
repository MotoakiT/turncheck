#ifndef ETRC22_DRIVING_H_
#define ETRC22_DRIVING_H_

#include "info_type.h"
#include "etrc_info.h"
#include "utils.h"
///高橋
///

class WheelsControl {
 public:
  WheelsControl(MotorIo* motor_io,SensorIo* sensor_io_);
  void Exec(int8_t target_power_l, int8_t target_power_r);
/// 高橋
 public:
  int32_t counts_r_;
  int32_t counts_l_;  
  rgb_raw_t color_rgb_raw_;

 private:
  MotorIo* motor_io_;
  SensorIo* sensor_io_;
};

class BasicDriver {
 public:
  BasicDriver(WheelsControl* wheels_control);
  ~BasicDriver();
  void SetParam(Move move_type, int8_t base_power);
  void Run();
  void Stop();
  ///高橋
  void SaveBasePower();
  ///

 private:
  ///高橋
  int now_angle[2][100000] = {};
  int color_index[3][100000] = {};
  int basepower_index = 0;
  float const_pid[2][3] = {};
  ///
  WheelsControl* wheels_control_;
  WheelsControlColor* wheels_control_color_;
  Move move_type_;
  int8_t base_power_;
};

class LineTracer {
 public:
  LineTracer(WheelsControl* wheels_control, Luminous* luminous);
  ~LineTracer();
  void SetParam(Move move_type, int8_t base_power, Gain gain);
  void Run();
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Luminous* luminous_;
  Move move_type_;
  int8_t base_power_;
  const int8_t line_trace_threshold = 40;
  PidControl* pid_control_;
};

class EndCondition {
 public:
  EndCondition(Luminous* luminous, Localize* localize);
  void SetParam(End end_type, Color end_color, float end_threshold);
  bool IsSatisfied();

 private:
  Luminous* luminous_;
  Localize* localize_;
  End end_type_;
  Color end_color_;
  float end_threshold_;
  bool end_state_;
  float ref_distance_;
  float ref_theta_;
};

class DrivingManager {
 public:
  DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition);
  void Update();
  void SetDriveParam(DrivingParam param);
  bool is_satisfied = false;

 private:
  void SetMoveParam(DrivingParam& param);
  void SetEndParam(DrivingParam& param);
  void Drive(DrivingParam& param);
  DrivingParam curr_param;
  BasicDriver* basic_driver_;
  LineTracer* line_tracer_;
  EndCondition* end_condition_;
};

#endif  // ETRC22_DRIVING_H_
