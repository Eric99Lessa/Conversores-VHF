#ifndef OBSERVER_H
#define OBSERVER_H

void kf_update_states(const float x_pred[], const float x_dot_pred[], float p11[], float p12[], float p22[],
                      const float z[], const float R[], float x_val[], float x_dot[], const int count);
void kf_predict_states(float x_pred[], float x_dot_pred[], const float x_val[], const float x_dot[],
                       float p11[], float p12[], float p22[], const float q_val[], const float q_dot[], const float Ts, const int count);

#endif