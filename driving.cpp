#include "driving.h"
#include "math.h"

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io),sensor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  ///高橋
  counts_r_ = motor_io_->counts_r_;
  counts_l_ = motor_io_->counts_l_;
  color_rgb_raw_ = sensor_io->color_rgb_raw_;
  ///
  int8_t curr_power_l = motor_io_->power_l_;
  if (target_power_l > curr_power_l) {
    curr_power_l += 1;
  } else if (target_power_l < curr_power_l) {
    curr_power_l -= 1;
  }

  int8_t curr_power_r = motor_io_->power_r_;
  if (target_power_r > curr_power_r) {
    curr_power_r += 1;
  } else if (target_power_r < curr_power_r) {
    curr_power_r -= 1;
  }

  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(curr_power_l, curr_power_r);
  }
}

BasicDriver::BasicDriver(WheelsControl* wheels_control)
    : wheels_control_(wheels_control),
      move_type_(kInvalidMove), base_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void BasicDriver::Run() {
  int8_t power_l;
  int8_t power_r;
  int32_t counts_r_ = wheels_control_ -> counts_r_;
  int32_t counts_l_ = wheels_control_ -> counts_l_;
  rgb_raw_t color_rgb_raw_ = wheels_control_->color_rgb_raw_;
  now_angle[0][basepower_index] = counts_l_;
  now_angle[1][basepower_index] = counts_r_;
  color_index[0][basepower_index] = color_rgb_raw_[0];
  color_index[1][basepower_index] = color_rgb_raw_[1];
  color_index[2][basepower_index] = color_rgb_raw_[2];

  int run = 40
  int intarval = 20;
  if(basepower_index % (run + intarval) <= 40){
  power_l = -base_power_;
  power_r = base_power_;
  }else{
  power_l = 0;
  power_r = 0;
  }

  wheels_control_->Exec(power_l, power_r);
  basepower_index += 1;
}

void BasicDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

///高橋
void BasicDriver::SaveBasePower(){
  char str [256];
  FILE* fp = fopen("error_motoaki.csv", "w");
  sprintf(str, "counts_r, counts_l, r, g, b\n");
  fprintf(fp, str);
  for (int i = 0; i < basepower_index;  i++) {
    sprintf(str, "%d, %d, %d, %d\n",now_angle[0][i],now_angle[1][i],color_index[0][i],color_index[1][i],color_index[2][i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
///

LineTracer::LineTracer(WheelsControl* wheels_control, Luminous* luminous)
    : wheels_control_(wheels_control), luminous_(luminous),
      move_type_(kInvalidMove), base_power_(0) {
  pid_control_ = new PidControl();
}

LineTracer::~LineTracer() {
  delete pid_control_;
}

void LineTracer::SetParam(Move move_type, int8_t base_power, Gain gain) {
  move_type_ = move_type;
  base_power_ = base_power;
  pid_control_->SetGain(gain.kp, gain.ki, gain.kd);
}

void LineTracer::Run() {
  float curr_hsv = luminous_->hsv_.v;
  float mv = pid_control_->CalcMv(line_trace_threshold, curr_hsv);

  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }

  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

void LineTracer::Stop() {
  wheels_control_->Exec(0, 0);
}

EndCondition::EndCondition(Luminous* luminous, Localize* localize)
    : luminous_(luminous), localize_(localize),
      end_type_(kInvalidEnd), end_color_(kInvalidColor), end_threshold_(0),
      end_state_(false), ref_distance_(0), ref_theta_(0) {
}

void EndCondition::SetParam(End end_type, Color end_color, float end_threshold) {
  end_type_ = end_type;
  end_color_ = end_color;
  end_threshold_ = end_threshold;
  end_state_ = false;

  // if (end_type_ == kDistanceEnd) {
  //   ref_distance_ = localize_->distance_;
  // } else if (end_type_ == kThetaEnd) {
  //   ref_theta_ = localize_->pose_.theta;
  // }
}

bool EndCondition::IsSatisfied() {
  switch (end_type_) {
    case kColorEnd:
      if (end_color_ == luminous_->color_)
        end_state_ = true;
      break;

    // case kDistanceEnd:
    //   if (end_threshold_ > 0 && localize_->distance_ - ref_distance_ > end_threshold_)
    //     end_state_ = true;
    //   else if (end_threshold_ < 0 && localize_->distance_ - ref_distance_ < end_threshold_)
    //     end_state_ = true;
    //   break;

    // case kThetaEnd:
    //   if (end_threshold_ > 0 && localize_->pose_.theta - ref_theta_ > end_threshold_)
    //     end_state_ = true;
    //   else if (end_threshold_ < 0 && localize_->pose_.theta - ref_theta_ < end_threshold_)
    //     end_state_ = true;
    //   break;

    default:
      break;
  }

  return end_state_;
}

DrivingManager::DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
}

void DrivingManager::Update() {
  if (is_satisfied) {
    return;
  }

  Drive(curr_param);

  if (end_condition_->IsSatisfied()) {
    is_satisfied = true;
  }
}

void DrivingManager::SetDriveParam(DrivingParam param) {
  is_satisfied = false;
  curr_param = param;
  SetMoveParam(curr_param);
  SetEndParam(curr_param);
}

void DrivingManager::SetMoveParam(DrivingParam& param) {
  Move move_type = param.move_type;
  int8_t base_power = param.base_power;
  Gain gain = param.gain;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->SetParam(move_type, base_power, gain);
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->SetParam(move_type, base_power);
      break;

    default:
      break;
  }
}

void DrivingManager::SetEndParam(DrivingParam& param) {
  End end_type = param.end_type;
  Color end_color = param.end_color;
  float end_threshold = param.end_threshold;

  end_condition_->SetParam(end_type, end_color, end_threshold);
}

void DrivingManager::Drive(DrivingParam& param) {
  Move move_type = param.move_type;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->Run();
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->Run();
      break;

    case kStopWheels:
      basic_driver_->Stop();
      break;

    default:
      break;
  }
}
