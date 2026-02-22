#include "ESP32Kinematics.h"
#include <array>

RobotArm::Config config {
    {160.0, 140.0, 105.0},
    {90, 10, 121, 90},
    {1, 1, 1, 1},
    {16, 17, 18, 19}
};

RobotArm robo(config);

std::array<float, 4> angulos = {0, 90, -90, 0};

void setup() {
    robo.TestarIdeal(angulos);
    Serial.begin(115200);
    Serial.println("Digite: x y z phi");
}

void loop() {

    if (Serial.available()) {

        float x = Serial.parseFloat();
        float y = Serial.parseFloat();
        float z = Serial.parseFloat();
        float phi = Serial.parseFloat();

        // Limpa buffer restante
        while (Serial.available()) Serial.read();

        std::array<float, 4> target = {x, y, z, phi};

        bool ok = robo.SetPose(target);

        if (ok)
            Serial.println("Movimento executado.");
        else
            Serial.println("Falha na cinemática.");
    }
}