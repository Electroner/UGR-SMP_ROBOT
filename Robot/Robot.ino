#include <MPU6050.h>
#include <Wire.h>

// SDA - Pin A4
// SCL - Pin A5

#define PIN_ACTIVATION 8

#define TRIGGER_PIN 10
#define ECHO_PIN 11

#define MOTOR1_PIN1 3
#define MOTOR1_PIN2 5
#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 9

#define LINE_FRONT_PIN A0
#define LINE_BACK_PIN A1
#define LINE_LEFT_PIN A2
#define LINE_RIGHT_PIN A3

#define WHITE_LINE_VALUE1 400	// Valor por el cual por encima consideramos que es blanco
#define BLACK_LINE_VALUE1 100	// Valor por el cual por debajo consideramos que es negro

#define WHITE_LINE_VALUE2 300
#define BLACK_LINE_VALUE2 100


#define DISTANCE_THRESHOLD 60

const int mpuAddress = 0x68;
MPU6050 mpu(mpuAddress);

// ACELEROMETRO:
//(-x)  : Adelante
//(y) : Derecha
//(z) : Arriba

int ax, ay, az; // Aceleracion
int gx, gy, gz; // Giroscopio (Aceleracion de g)

long distance; // distancia en centimetros

bool lineFront = false; // TRUE si es blanco
bool lineBack = false;	// TRUE si es blanco
bool lineLeft = false;	// TRUE si es blanco
bool lineRight = false; // TRUE si es blanco

int speed = 100; // Velocidad del robot en %

const float speed_factor = 0.66;

void setup()
{
	Serial.begin(9600);
	Wire.begin();
	mpu.initialize();
	Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));

	pinMode(PIN_ACTIVATION, INPUT_PULLUP);

	pinMode(TRIGGER_PIN, OUTPUT); // pin como salida
	pinMode(ECHO_PIN, INPUT);	  // pin como entrada

	pinMode(MOTOR1_PIN1, OUTPUT);
	pinMode(MOTOR1_PIN2, OUTPUT);
	pinMode(MOTOR2_PIN1, OUTPUT);
	pinMode(MOTOR2_PIN2, OUTPUT);

	pinMode(LINE_FRONT_PIN, INPUT);
	pinMode(LINE_BACK_PIN, INPUT);
	pinMode(LINE_LEFT_PIN, INPUT);
	pinMode(LINE_RIGHT_PIN, INPUT);

	digitalWrite(TRIGGER_PIN, LOW); // Inicializamos el pin con 0

	digitalWrite(MOTOR1_PIN1, LOW);
	digitalWrite(MOTOR1_PIN2, LOW);
	digitalWrite(MOTOR2_PIN1, LOW);
	digitalWrite(MOTOR2_PIN2, LOW);
}

void get_distance()
{
	long t; // timepo que demora en llegar el eco
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
	lineFront = analogRead(LINE_FRONT_PIN)>= BLACK_LINE_VALUE1;
	lineBack = analogRead(LINE_BACK_PIN) >= BLACK_LINE_VALUE2;
}

void MotorHorario(int _motor, int _speed)
{
	// CONVENCION: MOTOR DERECHO = 1, MOTOR IZQUIERDO = 2
	switch (_motor)
	{
	case 1:
		analogWrite(MOTOR1_PIN1, (int)(((_speed / 100.0) * 255.0)*speed_factor));
		analogWrite(MOTOR1_PIN2, LOW);
		break;

	case 2:
		analogWrite(MOTOR2_PIN1, (int)((_speed / 100.0) * 255.0));
		analogWrite(MOTOR2_PIN2, LOW);
		break;
	}
}

void MotorAntihorario(int _motor, int _speed)
{
	// CONVENCION: MOTOR DERECHO = 1, MOTOR IZQUIERDO = 2
	switch (_motor)
	{
	case 1:
		analogWrite(MOTOR1_PIN1, LOW);
		analogWrite(MOTOR1_PIN2, (int)(((_speed / 100.0) * 255.0)*speed_factor));
		break;

	case 2:
		analogWrite(MOTOR2_PIN1, LOW);
		analogWrite(MOTOR2_PIN2, (int)((_speed / 100.0) * 255.0));
		break;
	}
}

void MotorStop(int _motor)
{
	// CONVENCION: MOTOR DERECHO = 1, MOTOR IZQUIERDO = 2
	switch (_motor)
	{
	case 1:
		analogWrite(MOTOR1_PIN1, LOW);
		analogWrite(MOTOR1_PIN2, LOW);
		break;

	case 2:
		analogWrite(MOTOR2_PIN1, LOW);
		analogWrite(MOTOR2_PIN2, LOW);
		break;
	}
}

int readActivation()
{
	return !digitalRead(PIN_ACTIVATION);
}

// FUNCIONES DE CONTROL
/*
  get_distance()    //Obtiene la distancia en cm
  get_sensor_data()   //Obtiene los datos de las aceleraciones y velocidades angulares
  get_lines()     //Obtiene los datos de las lineas
  readActivation()  //Obtiene el estado de la activacion
  MotorHorario()    //Mueve el motor en sentido horario
  MotorAntihorario()  //Mueve el motor en sentido antihorario
  MotorStop()     //Detiene el motor
*/

bool firsttime = true;

int lastTime;

void loop()
{
	//get_sensor_data();
	get_distance();
	get_lines();

	if (readActivation())
	{
    if(firsttime){
      delay(3000);
      firsttime = false;
    }else{
    if(lineFront){
        MotorHorario(1, speed);
        MotorHorario(2, speed);
    }else{
        MotorStop(1);
        MotorStop(2);
    }
    }
		/*
		if(firsttime){
			delay(3000);
			firsttime = false;
		}
		else
		{
			// Comprobar que el enemigo esta delante y no nos salimos de la linea
			if (distance <= DISTANCE_THRESHOLD && lineFront)
			{
				//Moverse hacia adelante
				MotorHorario(1, speed);
				MotorHorario(2, speed);
			}
			if(distance > DISTANCE_THRESHOLD && lineBack)
			{
				//Moverse hacia atras
				MotorAntihorario(1, speed);
				MotorAntihorario(2, speed);
			}
			if(distance <= DISTANCE_THRESHOLD && !lineFront){
				//Parar
				MotorStop(1);
				MotorStop(2);
				//Hemos Ganado
			}
			if(distance > DISTANCE_THRESHOLD && !lineBack){
				//Parar
				MotorStop(1);
				MotorStop(2);
			}

			// Comprobar que el enemigo esta a la izquierda y no nos salimos de la linea
			if (distance > DISTANCE_THRESHOLD && lineFront && !lineBack)
			{
				//Moverse hacia la izquierda
				MotorHorario(1, speed);
				MotorAntihorario(2, speed);
			}
		}
		*/
	}
	else
	{
    MotorStop(1);
    MotorStop(2);
    Serial.print("FRONT: ");
    Serial.print(analogRead(A0));
    Serial.print("  BACK: ");
    Serial.print(analogRead(A1));
		Serial.print("  CM: ");
		Serial.println(distance);
		//Calcular el tiempo que tarda en ejecutarse el programa
		Serial.print("  Tiempo: ");
		lastTime = millis();
		Serial.println(millis() - lastTime);
	}
}
