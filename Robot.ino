#include "I2Cdev.h" //https://www.luisllamas.es/arduino-orientacion-imu-mpu-6050/
#include "MPU6050.h"
#include "Wire.h"

// SDA - Pin A4
// SCL - Pin A5

#define PIN_ACTIVATION 7

#define TRIGGER_PIN 11
#define ECHO_PIN 10

#define MOTOR1_PIN1 2
#define MOTOR1_PIN2 3
#define MOTOR2_PIN1 4
#define MOTOR2_PIN2 5

#define LINE_FRONT_PIN A0
#define LINE_BACK_PIN A1
#define LINE_LEFT_PIN A2
#define LINE_RIGHT_PIN A3

#define WHITE_LINE_VALUE 600 //Valor por el cual por encima consideramos que es blanco
#define BLACK_LINE_VALUE 100 //Valor por el cual por debajo consideramos que es negro

const int mpuAddress = 0x68; // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);

int ax, ay, az; // Aceleracion
int gx, gy, gz; // Giroscopio (Aceleracion de g)

long t;		// timepo que demora en llegar el eco
long distance; // distancia en centimetros

bool lineFront = false; //TRUE si es blanco
bool lineBack = false; 	//TRUE si es blanco
bool lineLeft = false; 	//TRUE si es blanco
bool lineRight = false; //TRUE si es blanco

int speed = 100; //Velocidad del robot en %

void setup()
{
	Serial.begin(9600);
	Wire.begin();
	mpu.initialize();
	Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));

	pinMode(TRIGGER_PIN, OUTPUT);   // pin como salida
	pinMode(ECHO_PIN, INPUT);	   // pin como entrada
	digitalWrite(TRIGGER_PIN, LOW); // Inicializamos el pin con 0
}

void get_distance()
{
	digitalWrite(TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIGGER_PIN, LOW);
	t = pulseIn(ECHO_PIN, HIGH);
	distance = (t / 2) / 29.1;
}

void get_sensor_data()
{
	// Leer las aceleraciones y velocidades angulares
	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);
}

void get_lines()
{
	lineFront = analogRead(LINE_FRONT_PIN) > WHITE_LINE_VALUE;
	lineBack = analogRead(LINE_BACK_PIN) > WHITE_LINE_VALUE;
	lineLeft = analogRead(LINE_LEFT_PIN) > WHITE_LINE_VALUE;
	lineRight = analogRead(LINE_RIGHT_PIN) > WHITE_LINE_VALUE;
}

void MotorHorario(int _motor,int _speed)
{
	//CONVENCION: MOTOR IZQUIERDO = 1, MOTOR DERECHO = 2
	switch (_motor)
	{
		case 1:
			digitalWrite (MOTOR1_PIN1, (int)(_speed*255)));
			digitalWrite (MOTOR1_PIN2, LOW);
		break;

		case 2:
			digitalWrite (MOTOR2_PIN1, (int)(_speed*255));
			digitalWrite (MOTOR2_PIN2, LOW);
		break;
	}
}

void MotorAntihorario(int _motor, int _speed)
{
	//CONVENCION: MOTOR IZQUIERDO = 1, MOTOR DERECHO = 2
	switch (_motor)
	{
		case 1:
			digitalWrite (MOTOR1_PIN1, LOW);
			digitalWrite (MOTOR1_PIN2, (int)(_speed*255));
		break;

		case 2:
			digitalWrite (MOTOR2_PIN1, LOW);
			digitalWrite (MOTOR2_PIN2, (int)(_speed*255));
		break;
	}
}

void MotorStop(int _motor)
{
	//CONVENCION: MOTOR IZQUIERDO = 1, MOTOR DERECHO = 2
	switch (_motor)
	{
		case 1:
			digitalWrite (MOTOR1_PIN1, LOW);
			digitalWrite (MOTOR1_PIN2, LOW);
		break;

		case 2:
			digitalWrite (MOTOR2_PIN1, LOW);
			digitalWrite (MOTOR2_PIN2, LOW);
		break;
	}
}

void loop()
{
	get_sensor_data();
	get_distance();
	get_lines();

	//HACER DECISIONES
}