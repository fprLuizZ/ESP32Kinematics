#include "ESP32Kinematics.h"
#include <math.h>

#define SERVO_MIN 0
#define SERVO_MAX 180
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

/* ================================
   Construtor
================================ */

RobotArm::RobotArm(const Config &config)
    : _links(config.links),
      _offsets(config.offsets),
      _directions(config.directions),
      _pins(config.pins)
{
    _servos[0].attach(_pins[0], 800, 2300);
    _servos[1].attach(_pins[1], 800, 2300);
    _servos[2].attach(_pins[2], 800, 2300);
    _servos[3].attach(_pins[3], 500, 2400);

    for (int i = 0; i < 4; i++)
        _lastAngles[i] = 0.0f;
}

/* ================================
   Interface Pública
================================ */

bool RobotArm::SetPose(std::array<float, 4> target)
{
    // target = {x, y, z, phi}
    return InverseKinematics(target);
}

void RobotArm::TestarReal(std::array<float, 4> angulos)
{
    for (int i = 0; i < 4; i++)
        _servos[i].write(angulos[i]);
}

void RobotArm::TestarIdeal(std::array<float, 4> angulos)
{
    UpdateServos(angulos);
}

void RobotArm::SetLastAngles(std::array<float, 4> angles)
{
    _lastAngles = angles;
}

std::array<float, 4> RobotArm::GetLastAngles()
{
    return _lastAngles;
}

//void setOffset(int joint, float offset) { 
    //_offsets[joint] = offset; 
//}

//void setDirection(int joint, int dir) { 
    //_directions[joint] = dir; 
//}

/* ================================
   Conversão matemática → servo
================================ */

float RobotArm::thetaToServo(int joint, float theta_deg)
{
    return _offsets[joint] + _directions[joint] * theta_deg;
}

/* ================================
   Escrita nos servos
================================ */
bool RobotArm::UpdateServos(std::array<float, 4> thetas_deg)
{
    std::array<float, 4> servoAngles;

    for (int i = 0; i < 4; i++)
    {
        servoAngles[i] = thetaToServo(i, thetas_deg[i]);

        if (servoAngles[i] < SERVO_MIN || servoAngles[i] > SERVO_MAX)
        {
            Serial.println("Erro: ângulo resulta em servo fora dos limites");
            return false;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        _servos[i].write(servoAngles[i]);
        _lastAngles[i] = thetas_deg[i] * DEG_TO_RAD; // armazena em rad
    }

    return true;
}

/* ================================
   Cinemática Inversa (InverseKinematics)
================================ */

bool RobotArm::InverseKinematics(std::array<float, 4> target)
{
    const float PHI_STEP = 1.0;
    const float PHI_MAX_DELTA = 45;

    float L1 = _links[0];
    float L2 = _links[1];
    float L3 = _links[2];

    float x = target[0];
    float y = target[1];
    float z = target[2];
    float phi_deg = target[3];

    for (float delta = 0; delta <= PHI_MAX_DELTA; delta += PHI_STEP)
    {

        for (int sinal = -1; sinal <= 1; sinal += 2)
        {

            float phi_test = phi_deg + sinal * delta;
            float phi = phi_test * DEG_TO_RAD;

            float theta0 = atan2(y, x);
            float r = sqrt(x * x + y * y);

            if (r < 1e-6)
                theta0 = _lastAngles[0];

            float xw = r - L3 * cos(phi);
            float yw = z - L3 * sin(phi);
            float d = sqrt(xw * xw + yw * yw);

            if (d > L1 + L2 || d < fabs(L1 - L2))
                continue;

            float cos_theta2 =
                (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

            if (cos_theta2 > 1.0)
                cos_theta2 = 1.0;
            if (cos_theta2 < -1.0)
                cos_theta2 = -1.0;

            float theta2_pos = acos(cos_theta2);
            float theta2_neg = -theta2_pos;

            float psi = atan2(yw, xw);

            float sol[2][4];
            int n_sol = 0;

            for (int i = 0; i < 2; i++)
            {

                float theta2 = (i == 0) ? theta2_pos : theta2_neg;

                float alpha =
                    atan2(L2 * sin(theta2),
                          L1 + L2 * cos(theta2));

                float theta1 = psi - alpha;
                float theta3 = phi - theta1 - theta2;

                sol[n_sol][0] = theta0;
                sol[n_sol][1] = theta1;
                sol[n_sol][2] = theta2;
                sol[n_sol][3] = theta3;

                n_sol++;
            }

            int melhor = -1;
            float menor_custo = 1e9;

            for (int i = 0; i < n_sol; i++)
            {

                float s0 = thetaToServo(0, sol[i][0] * RAD_TO_DEG);
                float s1 = thetaToServo(1, sol[i][1] * RAD_TO_DEG);
                float s2 = thetaToServo(2, sol[i][2] * RAD_TO_DEG);
                float s3 = thetaToServo(3, sol[i][3] * RAD_TO_DEG);

                if (s0 < SERVO_MIN || s0 > SERVO_MAX ||
                    s1 < SERVO_MIN || s1 > SERVO_MAX ||
                    s2 < SERVO_MIN || s2 > SERVO_MAX ||
                    s3 < SERVO_MIN || s3 > SERVO_MAX)
                    continue;

                float custo =
                    fabs(sol[i][0] - _lastAngles[0]) +
                    fabs(sol[i][1] - _lastAngles[1]) +
                    fabs(sol[i][2] - _lastAngles[2]) +
                    fabs(sol[i][3] - _lastAngles[3]);

                if (custo < menor_custo)
                {
                    menor_custo = custo;
                    melhor = i;
                }
            }

            if (melhor != -1)
            {

                std::array<float, 4> thetas_deg;

                for (int j = 0; j < 4; j++)
                    thetas_deg[j] = sol[melhor][j] * RAD_TO_DEG;

                return UpdateServos(thetas_deg);
            }

            if (delta == 0)
                break;
        }
    }

    Serial.println("Nenhuma solução encontrada.");
    return false;
}