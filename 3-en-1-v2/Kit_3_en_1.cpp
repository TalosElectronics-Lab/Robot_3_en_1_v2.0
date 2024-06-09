#include "Kit_3_en_1.h"
void Kit_3_en_1::init()
{
    Motores_init();
    Bluetooth.begin(9600);
    Serial.begin(9600);

    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);
    servo_1.attach(Servo_pin);
    servo_1.write(90);

    pinMode(S1, INPUT_PULLUP);
    pinMode(S2, INPUT_PULLUP);
    pinMode(S3, INPUT_PULLUP);
    pinMode(S4, INPUT_PULLUP);
    pinMode(S5, INPUT_PULLUP);
}
void Kit_3_en_1::modo_3_en_1()
{

    if (Bluetooth.available() > 0)
    {
        Estado = Bluetooth.read();
        // Serial.println(Estado);
    }
    if (Estado == 'a')
    {
        Menu = 30;
    }
    else if (Estado == 'b')
    {
        Menu = 10;
    }
    else if (Estado == 'c')
    {
        Menu = 20;
    }

    switch (Menu)
    {
    case 10:
        if (Bluetooth.available() > 0)
        {
            Estado = Bluetooth.read();
            // Serial.println(Estado);
        }
        if (Estado == '1')
        {
            // Arriba_Izquierda
            Motores_mv(Velocidad_Med, Velocidad_Max);
        }
        else if (Estado == '2')
        {
            // Derecho
            Motores_mv(Velocidad_Max, Velocidad_Max);
        }
        else if (Estado == '3')
        {
            // Arriba_Derecha
            Motores_mv(Velocidad_Max, Velocidad_Med);
        }
        else if (Estado == '4')
        {
            // Girar a la izquierda
            Motores_mv(-Velocidad_Max, Velocidad_Max);
        }
        else if (Estado == '5')
        {
            // Serial.println("Logo talos");
        }
        else if (Estado == '6')
        {
            // Girar a la derecha
            Motores_mv(Velocidad_Max, -Velocidad_Max);
        }
        else if (Estado == '7')
        {
            // Abajo Izquierda
            Motores_mv(-Velocidad_Med, -Velocidad_Max);
        }
        else if (Estado == '8')
        {
            // Reversa
            Motores_mv(-Velocidad_Max, -Velocidad_Max);
        }
        else if (Estado == '9')
        {
            // Abajo Derecha
            Motores_mv(-Velocidad_Max, -Velocidad_Med);
        }
        else if (Estado == 'w')
        {
            Motores_mv(0, 0);
        }
        break;

    case 20:
        modo_evasor(25, 100); // 25cm=Distancia del obstaculo
        break;

    case 30:
        modo_seguidor(8, .2, 5, 50);
        break;
    }
}
Kit_3_en_1::~Kit_3_en_1()
{
}

char Kit_3_en_1::Leer_BT()
{
    if (Bluetooth.available() > 0)
    {
        Estado = Bluetooth.read();
        // Serial.println(Estado);
    }
    return Estado;
}
void Kit_3_en_1::modo_evasor(int Distancia, uint8_t velocidad)
{
    float Distancia_izq, Distancia_der;

    Motores_mv(velocidad, velocidad);
    if (obtener_distancia() <= Distancia)
    {
        Motores_mv(-50, -50);
        delay(250);
        Motores_mv(0, 0);

        // Barrido de servo de derecha a izquierda
        for (size_t i = 90; i < Angulo_max; i++)
        {
            servo_1.write(i);
            delay(5);
        }

        delay(300);
        Distancia_izq = obtener_distancia();

        for (size_t i = Angulo_max; i > Angulo_min; i--)
        {
            servo_1.write(i);
            delay(5);
        }

        delay(300);
        Distancia_der = obtener_distancia();

        for (size_t i = Angulo_min; i < 90; i++)
        {
            servo_1.write(i);
            delay(5);
        }
        delay(250);

        if (Distancia_izq < Distancia_der)
        {
            // Obstaculo detectado en el lado izquierdo, debe girar a la derecha
            Motores_mv(velocidad_giro, -velocidad_giro);
            delay(500);
            Motores_mv(velocidad, velocidad);
        }
        else
        {
            // Obstaculo detectado en el lado derecho por lo que debe girar a la izquierda
            Motores_mv(-velocidad_giro, velocidad_giro);
            delay(500);
            Motores_mv(velocidad, velocidad);
        }
    }
}

float Kit_3_en_1::obtener_distancia()
{
    long lduration;
    long ldistance;

    // Se manda un pulso por el pin Trigger
    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);

    // Se mide cuanto tiempo tardo en us
    // Ecuación d = l_distancia/2 * velocidad cm/us
    lduration = pulseIn(Echo, HIGH);
    ldistance = (lduration / 2) / 29.1;
    return ldistance;
}
void Kit_3_en_1::frenos()
{
    leer_sensores();
    if (sensor[4] == 1)
    {
        while (sensor[3] == 0)
        {
            leer_sensores();
            Motores_mv(50, -50);
        }
    }
    else if (sensor[0] == 1)
    {
        while (sensor[1] == 0)
        {
            leer_sensores();
            Motores_mv(-50, 50);
        }
    }
}
void Kit_3_en_1::modo_seguidor(float Kp, float Ki, float Kd, float Velocidad)
{
    leer_sensores();
    int velocidad_recta;
    P = Error;
    I = I + P;
    D = Error - Error_Anterior;
    Error_Anterior = Error;
    if ((P * I) < 0)
        I = 0; // corrige el overshooting - integral windup

    PID = (Kp * P) + (Ki * I) + (Kd * D);

    // Limitamos la velocidad
    if (PID > Velocidad)
    {
        PID = Velocidad;
    }
    else if (PID < -Velocidad)
    {
        PID = -Velocidad;
    }

    velocidad_recta = 50 + (Velocidad + 20 - 50) * exp(-1 * abs(Kp * P));

    if (PID < 0)
    {
        Motores_mv(velocidad_recta + PID, velocidad_recta);
    }
    else
    {
        Motores_mv(velocidad_recta, velocidad_recta - PID);
    }
    /*
    Serial.print(Error);
    Serial.print("\t");
    Serial.print(PID);
    Serial.print("\t");
    Serial.println(velocidad_recta + PID);
    */

    frenos();
}

void Kit_3_en_1::leer_sensores()
{
    sensor[0] = digitalRead(S1);
    sensor[1] = digitalRead(S2);
    sensor[2] = digitalRead(S3);
    sensor[3] = digitalRead(S4);
    sensor[4] = digitalRead(S5);

    // Detectar la desviacion ("Error") del seguidor de linea
    if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
        Error = Error;
    }
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
        Error = 2.5;
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
        Error = 1.5;
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
        Error = 0;

    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
        Error = -1.5;
    else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
        Error = -2.5;
}
void Kit_3_en_1::print_sensores()
{
    for (size_t i = 0; i < 5; i++)
    {
        Serial.print(sensor[i]);
        Serial.print("\t");
        /* code */
    }
    Serial.println("");
}