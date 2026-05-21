#ifndef KALMAN_HPP
#define KALMAN_HPP

/**
 * Filtro de Kalman escalar 1D.
 * Mantido apenas para uso interno do Sensor (fusão de roll/pitch/yaw).
 * O EKF do veículo foi migrado para AckermanEKF (EKF.hpp).
 */

#include <math.h>

class Kalman {
public:
    float x;   // estado estimado
    float P;   // covariância do erro
    float Q;   // ruído do processo
    float R;   // ruído da medição

    Kalman(float Q, float R, float P = 1.0f, float x = 0.0f)
        : Q(Q), R(R), P(P), x(x) {}

    void update(float measurement) {
        P += Q;
        float K = P / (P + R);
        x += K * (measurement - x);
        P *= (1.0f - K);
    }
};

#endif // KALMAN_HPP
