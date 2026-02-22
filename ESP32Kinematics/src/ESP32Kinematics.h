/*
    ESP32Kinematics.h
    Autor: Luiz Felipe P. Rodrigues
    18 de Março de 2026

    Biblioteca dedicada ao controle 
    de um braço robótico com 4 Graus de Liberdade (GDL):

    Base   (t0) - rotação do braço
    Ombro  (t1) - início do Elo 1
    Cotovelo (t2) - início do Elo 2
    Pulso  (t3) - início do Elo 3
*/

#ifndef ESP32_KINEMATICS_H
#define ESP32_KINEMATICS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <array>


class RobotArm {
public:
    // Configuração geral
    struct Config {
    std::array<float, 3> links;       // {L1, L2, L3}
    std::array<float, 4> offsets;     // {t0, t1, t2, t3} (graus)
    std::array<float, 4> directions;  // {t0, t1, t2, t3} (1 normal, -1 invertido)
    std::array<uint8_t, 4> pins;      // {t0, t1, t2, t3}
};
    // Construtor
    RobotArm(const Config& config);

    // Move o efetuador final para (x, y, z, phi)
    bool SetPose(std::array<float, 4> target);

    // Move usando ângulos matemáticos (graus)
    void TestarIdeal(std::array<float, 4> angulos);

    // Move usando ângulos reais do servo (graus)
    void TestarReal(std::array<float, 4> angulos);

    void SetLastAngles(std::array<float, 4> angles);

    std::array<float, 4> GetLastAngles();

    //void setOffset(int joint, float offset);
    //void setDirection(int joint, int dir);

private:
    //Escreve os angulos em cada servo
    bool UpdateServos(std::array<float, 4> thetas);
    //Converte angulo matemático para real
    float thetaToServo(int joint, float theta_deg);

    bool InverseKinematics(std::array<float, 4> target);

    std::array<float, 3> _links;
    std::array<float, 4> _offsets;
    std::array<float, 4> _directions;
    std::array<uint8_t, 4> _pins;
    std::array<Servo, 4> _servos;
    std::array<float, 4> _lastAngles; // ângulos matemáticos (graus)
};

#endif