#include "Robot.h"
#include <SoftwareSerial.h>
#include <Servo.h>

class Kit_3_en_1 : public Robot
{
private:
    /* data */
    const uint8_t B_TX = 8;
    const uint8_t B_RX = 9;
    SoftwareSerial Bluetooth; // RX, TX
    char Estado;
    int Menu;
    int Velocidad_Max = 230;
    int Velocidad_Med = 130;

    const uint8_t Echo = 11;
    const uint8_t Trigger = 10;
    const uint8_t Servo_pin = A4;
    uint8_t Angulo_min = 0;
    uint8_t Angulo_max = 180;
    uint8_t velocidad_giro = 50;
    Servo servo_1;
    float distancia;

    const uint8_t S1 = A0;
    const uint8_t S2 = A1;
    const uint8_t S3 = A2;
    const uint8_t S4 = A3;
    const uint8_t S5 = 12;

    bool sensor[5];
    float Error = 0, P = 0, I = 0, D = 0, PID = 0;
    float Error_Anterior = 0, Anteriror_I = 0;

public:
    void init();
    void modo_3_en_1();
    char Leer_BT();
    void modo_bluetooth();

    void modo_evasor(int Distancia, uint8_t velocidad);
    float obtener_distancia();
    void set_velocidad_giro(uint8_t v_giro);

    void leer_sensores();
    void print_sensores();
    void modo_seguidor(float Kp, float Ki, float Kd, float Velocidad);
    void frenos();

    Kit_3_en_1(/* args */) : Robot::Robot(), Bluetooth(B_TX, B_RX){};
    ~Kit_3_en_1();
};
