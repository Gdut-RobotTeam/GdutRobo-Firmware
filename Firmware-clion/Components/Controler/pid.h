//
// Created by luohx on 2020/9/29.
//

#ifndef FIRMWARE_CLION_COMPONENTS_CONTROLER_PID_H_
#define FIRMWARE_CLION_COMPONENTS_CONTROLER_PID_H_

typedef enum {
  PID_POSITION = 0,
  PID_DELTA
} PID_MODE;

typedef struct {
  PID_MODE mode;

  float kp_;
  float ki_;
  float kd_;

  float max_out_;
  float max_iout_; // max integral output

  float set_;
  float ref_;

  float out_;
  float Pout_;
  float Iout_;
  float Dout_;
  float Dbuf_[3];//diffential
  float error_[3];
} pid_type_def;

extern void PID_init(pid_type_def *pid, PID_MODE mode, const float PID[3], float max_out, float max_iout);
extern float PID_cal(pid_type_def *pid, float ref, float set);
extern void PID_clear(pid_type_def *pid);

#endif //FIRMWARE_CLION_COMPONENTS_CONTROLER_PID_H_
