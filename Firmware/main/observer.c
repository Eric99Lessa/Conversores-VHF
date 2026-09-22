#include "observer.h"
#include "config.h"

static void kf_update_scalar(float *x_val, float *x_dot, float *p11, float *p12, float *p22, const float x_pred, const float x_dot_pred, const float z, const float R)
{
    float innov = z - x_pred;
    float S = *p11 + R;  // innovation covariance
    float k1 = *p11 / S; // Kalman gain
    float k2 = *p12 / S; // Kalman gain

    *x_val = x_pred + k1 * innov;
    *x_dot = x_dot_pred + k2 * innov;

    // covariance update
    *p22 -= k2 * (*p12);
    *p12 -= k1 * (*p12);
    *p11 -= k1 * (*p11);
}

static void kf_predict_scalar(float *x_val_p, float *x_dot_p, float *p11, float *p12, float *p22,
                              const float x_val, const float x_dot, const float q_val, const float q_dot, const float Ts)
{
    // State propagation f(x)
    *x_dot_p = x_dot;
    *x_val_p = x_val + Ts * x_dot;

    // Covariance prediction
    *p11 += 2 * Ts * (*p12) + (*p22) * Ts * Ts + q_val;
    *p12 += Ts * (*p22);
    *p22 += q_dot;
}

void kf_update_states(const float x_pred[], const float x_dot_pred[], float p11[], float p12[], float p22[], const float z[], const float R[], float x_val[], float x_dot[], const int count)
{
    for (int i = 0; i < count; i++)
    {
        kf_update_scalar(&x_val[i], &x_dot[i], &p11[i], &p12[i], &p22[i], x_pred[i], x_dot_pred[i], z[i], R[i]);
    }
}

void kf_predict_states(float x_pred[], float x_dot_pred[], const float x_val[], const float x_dot[], float p11[], float p12[], float p22[],
                       const float q_val[], const float q_dot[], const float Ts, const int count)
{
    for (int i = 0; i < count; i++)
    {
        kf_predict_scalar(&x_pred[i], &x_dot_pred[i], &p11[i], &p12[i], &p22[i], x_val[i], x_dot[i], q_val[i], q_dot[i], Ts);
    }
}