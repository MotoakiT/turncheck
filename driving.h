#ifndef ETRC22_DRIVING_H_
#define ETRC22_DRIVING_H_

#include "info_type.h"
#include "etrc_info.h"
#include "utils.h"
///高橋
///

class WheelsControl {
 public:
  WheelsControl(MotorIo* motor_io);
  void Exec(int8_t target_power_l, int8_t target_power_r);
/// 高橋
 public:
  int32_t counts_r_;
  int32_t counts_l_;  

 private:
  MotorIo* motor_io_;
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
  float error_now[2][100000] = {};
  int power_index[2][100000] = {};
  float now_apt_r_[100000] = {};
  float power_r_[100000] = {};
  int now_angle_r_[100000] = {};
  int now_angle_l_[100000] = {};
  int angle_least_squares[2][5] = {};
  float time_index[5] = {0.0,0.04,0.08,0.12,0.16};
  int basepower_index = 0;
  float Sx = 0;
  float Sxy[2] = {};
  float y_abe_l = 0;
  float y_abe_r = 0;
  float x_abe = 0;


  float Kp[2] = {};
  float Ki[2] = {};
  float Kd[2] = {};

  float error_interal[2] = {}; 
  float error_differential[2] = {};
  float delta_t_pid = 0.04;
  
  float motor_power_pid[2] = {};
  float target_value_speed = 0.0;
  float now_speed_l[100000] = {};
  float now_speed_r[100000] = {};
  ///
  WheelsControl* wheels_control_;
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
