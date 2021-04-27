#include <Arduino.h>
#include <IMU.h>
#include <Motor.h>
#include <Controller.h>
#include <PID_v1.h>


/*
Main file for le super robot

Serial Communication
--------------------

    Output
    ------
    - Message: '&'
    - Error message: '!'
    - Data: '%'
        Data format: '%time, angle, speed_motor_left, speed_motor_left, pwm'

    Input
    -----
    - All commands must start w/ '#'
    - Possible commands:
        - Start, start, on
        - Stop, stop, off
        - Kp
        - Ki
        - Kd
*/

#define DEBUG 0

// Definiton of pins for motors and encoders
#define MOTOR_L_PIN1 17 
#define MOTOR_L_PIN2 16
#define MOTOR_R_PIN1 23
#define MOTOR_R_PIN2 22
#define ENC_L_PIN1 6
#define ENC_L_PIN2 5
#define ENC_R_PIN1 7
#define ENC_R_PIN2 8

#define MAX_VELOCITY 500
#define MAX_ANGLE 50 // Past this angle the motors stop [deg]
#define CAL_TIME 10000 // Time required for to calibrate IMU [ms]
#define IMU_STABILIZE_DELAY 12000 // Time required by IMU  to stabilise [ms]

#define COM_INTERVAL 100 // When to read serial port [ms]
#define PRINT_INTERVAL 50 // When to print data [ms]

#define ANGLE_TIMER 1 // angle delay [ms]
#define CONTROLLER_TIMER 10 // controller delay [ms]
#define VELOCITY_TIMER 5 // velocity delay [ms]

// Prototypes
void printVariables();
void computeAngleMain();
void calibrate();
void computeSpeedTimer();
void computeController();
void readSerial();
void computeVelocity();

// Motors
Motor motorL(MOTOR_L_PIN1, MOTOR_L_PIN2);
Motor motorR(MOTOR_R_PIN1, MOTOR_R_PIN2);

// Controller
Controller controller;
double pwm;
bool fallen = 0;

// Angle
float ypr[3];
float angle, prevAngle = 0;
float printVel, angularVel = 0;
float offset = 0;
double sampleSize = 0; 
double angleSum = 0;
unsigned long calCurrentTime = 0, calInterval = 0;

// Timers
IntervalTimer angleTimer, controllerTimer;
unsigned long currentTime = 0;
unsigned long lastPrintTime = 0 ;
unsigned long lastComTime = 0;
unsigned long lastAngleTime = 0;
unsigned long lastControllerTime = 0;
unsigned long lastVelocityTime = 0;

String commande; // Data read from serial port
bool toStabilize = true; //Robot will try to stabilize when true

void setup() {
    Serial.begin(9600);

    // Pinmodes
    pinMode(MOTOR_L_PIN1,OUTPUT);
    pinMode(MOTOR_L_PIN2,OUTPUT);
    pinMode(MOTOR_R_PIN1,OUTPUT);
    pinMode(MOTOR_R_PIN2,OUTPUT);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // led is turned on while setup

    // Wait for input
    // Serial.println(F("\n&Send any character to begin: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again
    
    // IMU Setup
    Serial.println("&IMU Setup");
    IMUSetup();

    // IMU timer
    angleTimer.begin(computeAngleMain, 100000); 
    angleTimer.priority(200);
    unsigned long delayStartTime = millis();
    while(delayStartTime - millis() < IMU_STABILIZE_DELAY){
        printVariables();
        delay(50);
    }

    // Calibration
    Serial.println("&IMU calibration");
    // calibrate IMU using the angle
    calibrate(); 
    // calibrate IMU using raw values
    // calibrateRaw(); 
    Serial.println("&IMU Calibration Done");
    delay(500);
    angleTimer.end();

    // Controller timer
    // controllerTimer.begin(computeControllerTimer, 10000);
    
    Serial.println("&Setup done");
    digitalWrite(13,LOW); // led is turned off when setup is done
}

void loop() {

    currentTime = millis();
    // Calculer la vitesse et la position des moteurs
    // motorL.computeSpeed();
    // motorR.computeSpeed();

    // Compute angle
    if(currentTime-lastAngleTime >= ANGLE_TIMER){
        computeAngleMain();
        lastAngleTime = currentTime;
    }
    
    // Compute angular velocty
    if(currentTime-lastVelocityTime >= VELOCITY_TIMER){
        computeVelocity();
        lastVelocityTime = currentTime;
    }

    // Call controller
    if(currentTime-lastControllerTime >= CONTROLLER_TIMER){
        computeController();
        lastControllerTime = currentTime;
    }

    // Read Serial Input
    if(currentTime - lastComTime > COM_INTERVAL){
        lastComTime = currentTime;
	    readSerial();
    }
    
    // Print variables 
    if(currentTime - lastPrintTime > PRINT_INTERVAL && toStabilize){
           lastPrintTime = currentTime;
           printVariables();
    }

}


// Timers handlers
void computeAngleMain(){
    computeAngle(ypr);
    angle = (ypr[2] * 180/M_PI) - offset; // [deg]
}

void computeVelocity(){
    angularVel = (angle - prevAngle)/((currentTime-lastVelocityTime)/1000.0);
    printVel = (angularVel > MAX_VELOCITY) ? MAX_VELOCITY : angularVel;
    printVel = (angularVel < -MAX_VELOCITY) ? -MAX_VELOCITY : printVel;
    prevAngle = angle;
}

void computeController(){
    if (toStabilize){
        // while robot is in range, stabilize
        if(angle > -MAX_ANGLE && angle < MAX_ANGLE && !fallen){
            pwm = controller.computeOutput(angle, angularVel);
            fallen = false;
        } 
        else if(angle >= -5 && angle <= 5 && fallen) {
            fallen = false;
            Serial.println("&ROBOT READY TO GO");
        } 
        else if((angle <= -MAX_ANGLE || angle >= MAX_ANGLE) && !fallen){
            fallen = true;
            pwm = 0;
            Serial.println("&!!!!!ROBOT DOWN!!!!!");
        } 
        else pwm = 0;

        motorR.move(pwm);
        motorL.move(pwm);

    }
    else {
        pwm = 0;
        motorR.move(pwm);
        motorL.move(pwm);
    }
}

// Calibrating function for the IMU
void calibrate(){
    Serial.println("calibration fct");
    unsigned long calStartTime = millis();

    while((millis() - calStartTime) < CAL_TIME){
        angleSum += angle;
        sampleSize++;
        // Serial.print("angle = ");
        // Serial.print(angle);
        // Serial.print("\t angleSum = ");
        // Serial.print(angleSum);
        // Serial.print("\t sampleSize = ");
        // Serial.println(sampleSize);
        delay(25);
        printVariables();
    }

    Serial.println("out of loop");

    offset =  angleSum / sampleSize;

    Serial.print("&IMU CALIBRATION DONE. Your offset is : ");
    Serial.println(offset);
}

// Serial Communication
void printVariables(){
    // double printAngle;
    // printAngle = (angle > MAX_ANGLE) ? MAX_ANGLE : angle;
    // printAngle = (angle < -MAX_ANGLE) ? -MAX_ANGLE : printAngle;

    if (!DEBUG){
        Serial.print("%");
        Serial.print(millis()/1000.0);
        Serial.print(",");

        Serial.print(angle);
        Serial.print(",");

        // Serial.print(motorL.getSpeed());
        // Serial.print(",");

        // Serial.print(motorR.getSpeed());
        // Serial.print(",");

        Serial.print(pwm);
        Serial.print(",");

        Serial.print(controller.getPAction());
        Serial.print(",");

        Serial.print(printVel);
        Serial.print(",");

        Serial.println(controller.getDAction());
    }
    else{
        Serial.print("Time: ");
        Serial.print(millis()/1000.0);
        Serial.print(",");

        Serial.print("\t Angle: ");
        Serial.print(angle);
        Serial.print(",");

        // Serial.print("\t Speed L: ");
        // Serial.print(motorL.getSpeed());
        // Serial.print(",");

        // Serial.print("\t Speed R: ");
        // Serial.print(motorR.getSpeed());
        // Serial.print(",");

        Serial.print("\t fallen : ");
        Serial.print(fallen);

        Serial.print("\t PWM: ");
        Serial.println(pwm);
    }


}

void readSerial(){
    // Read serial port
    while (Serial.available()) {
		char c = Serial.read();  //gets one byte from serial buffer
		commande += c; //makes the string readString
		delay(2);  //slow looping to allow buffer to fill with next character
	}

    // Analyze input
	if (commande.length() > 0) {

        // Input is a command
		if (commande[0] == '#') {
			commande = commande.substring(1);
			int len = commande.length();
			commande.remove(len-1, 1);

			Serial.print("&Commande received: ");
			Serial.println(commande);

            // Start/Stop
            if(commande.startsWith("Start") || commande.startsWith("start") || commande.startsWith("on")){
                toStabilize = true;
                controller.initialize();
            }

            else if(commande.startsWith("Stop") || commande.startsWith("stop") || commande.startsWith("off")){
                toStabilize = false;
                pwm = 0;
            }

            // Controller gains
            else if(commande.startsWith("Kp")){
                double newKp = atof(commande.substring(3).c_str());
                controller.setKp(newKp);
                Serial.print("&New Kp: ");
                Serial.println(controller.getKp(), 5);
            }

            else if(commande.startsWith("Ki")){
                double newKi = atof(commande.substring(3).c_str());
                controller.setKi(newKi);
                Serial.print("&New Ki: ");
                Serial.println(controller.getKi(), 5);
            }

            else if(commande.startsWith("Kd")){
                double newKd = atof(commande.substring(3).c_str());
                controller.setKd(newKd);
                Serial.print("&New Kd: ");
                Serial.println(controller.getKd(), 7);
            }
            
            // Unknown
            else{
                Serial.print("&Unknown command: ");
                Serial.println(commande);
            }

        }
        else
            Serial.println("!Input invalid");

		commande = ""; //empty for next input
    }
}

