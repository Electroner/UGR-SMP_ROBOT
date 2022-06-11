#include "I2Cdev.h" //https://www.luisllamas.es/arduino-orientacion-imu-mpu-6050/
#include "MPU6050.h"
#include "Wire.h"

// SDA - Pin A4
// SCL - Pin A5

#define PIN_ACTIVATION 7

#define PIN_TRIGGER 11
#define PIN_ECHO 10

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

void setup()
{
	Serial.begin(9600);
	Wire.begin();
	mpu.initialize();
	Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));

	pinMode(Trigger, OUTPUT);   // pin como salida
	pinMode(Echo, INPUT);	   // pin como entrada
	digitalWrite(Trigger, LOW); // Inicializamos el pin con 0
}

void get_distance()
{
	digitalWrite(Trigger, HIGH);
	delayMicroseconds(10);
	digitalWrite(Trigger, LOW);
	t = pulseIn(Echo, HIGH);
	distance = (t / 2) / 29.1;
}

void get_sensor_data()
{
	// Leer las aceleraciones y velocidades angulares
	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);
}

void loop()
{
	get_sensor_data();
	get_distance();

	//HACER DECISIONES
}