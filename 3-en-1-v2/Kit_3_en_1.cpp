#include "Kit_3_en_1.h"
char traslateD1(int d1Value)
{
    char Estado;
    if (d1Value == 1)
    {
        Estado = '1';
    }
    if (d1Value == 2)
    {
        Estado = '2';
    }
    if (d1Value == 3)
    {
        Estado = '3';
    }
    if (d1Value == 4)
    {
        Estado = '4';
    }
    if (d1Value == 5)
    {
        Estado = '5';
    }
    if (d1Value == 6)
    {
        Estado = '6';
    }
    if (d1Value == 7)
    {
        Estado = '7';
    }
    if (d1Value == 8)
    {
        Estado = '8';
    }
    if (d1Value == 9)
    {
        Estado = '9';
    }

    return Estado;
}
int parseD1Value(String json)
{
    if (json == 's')
    {
        return 'w';
    }
    int d1Value = 0;
    int pos = json.indexOf("\"D1\":"); // Encontrar la posición donde comienza la clave "D1"
    if (pos != -1)
    {                                       // Si se encuentra la clave "D1"
        pos += 6;                           // Avanzar 6 posiciones para omitir la clave "D1": "
        String d1Str = json.substring(pos); // Obtener la subcadena que contiene el valor de D1
        d1Value = d1Str.toInt();            // Convertir la subcadena a un entero
    }
    return d1Value;
}

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
char Kit_3_en_1::modo_bluetooth()
{
    int Estado;
    int Velocidad_Max = 255;
    int Velocidad_Med = 180;
    Estado = Leer_BT();

    if (Estado == '6')
    {
        // Arriba_Izquierda
        Motores_mv(Velocidad_Med, Velocidad_Max);
    }
    else if (Estado == '3')
    {
        // Derecho
        Motores_mv(Velocidad_Max, Velocidad_Max);
    }
    else if (Estado == '8')
    {
        // Arriba_Derecha
        Motores_mv(Velocidad_Max, Velocidad_Med);
    }
    else if (Estado == '1')
    {
        // Girar a la izquierda
        Motores_mv(Velocidad_Max, -Velocidad_Max);
    }
    else if (Estado == '2')
    {
        // Girar a la derecha
        Motores_mv(-Velocidad_Max, Velocidad_Max);
    }
    else if (Estado == '7')
    {
        // Abajo Izquierda
        Motores_mv(Velocidad_Med, Velocidad_Max);
    }
    else if (Estado == '4')
    {
        // Reversa
        Motores_mv(Velocidad_Max, Velocidad_Max);
    }
    else if (Estado == '9')
    {
        // Abajo Derecha
        Motores_mv(Velocidad_Max, Velocidad_Med);
    }
    else if (Estado == 'w')
    {
        Motores_mv(0, 0);
    }
    return Estado;
}

Kit_3_en_1::~Kit_3_en_1()
{
}

char Kit_3_en_1::Leer_BT()
{
    if (Bluetooth.available())
    {
        char _command = Bluetooth.read();

        if (_command == '1')
        {
            return 'a';
        }
        if (_command == 's')
        {
            return 'w';
        }
        if (_command == '2')
        {
            return 'c';
        }
        if (_command == '{')
        {
            // Si hay datos disponibles para leer desde el módulo Bluetooth
            String data = Bluetooth.readStringUntil('}'); // Leer los datos hasta que se reciba un salto de línea (\n)
            int d1Value = parseD1Value(data);
            return traslateD1(d1Value);
        }
    }
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