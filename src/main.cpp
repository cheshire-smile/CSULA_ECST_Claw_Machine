//This code was created as part of the SMART internship at CSULA ECST maker space Summer of 2024
//Author: Cosme A. Lavalle


#include <Arduino.h>
#include <DRV8834.h>
#include <Servo.h>
#include "MultiDriver.h"
#include "SyncDriver.h"
#include <STM32FreeRTOS.h>
#include <task.h>


// Define pin connections for Nucleo-64 F401RE
#define X_STEP_PIN          D6
#define X_DIR_PIN           D5
#define X_SLEEP_PIN         D13
#define Y_STEP_PIN          D9
#define Y_DIR_PIN           D8
#define Y_SLEEP_PIN         D13

#define WINCH_IN1           PB13 // define motor pin numbers
#define WINCH_IN2           PB14
#define CLAW_PWM_PIN        PC6
#define LIMIT_X1            PB15
#define LIMIT_X2            PB1
#define LIMIT_Y1            PB2
#define LIMIT_Y2            PB12
#define LIMIT_WINCH         PC5

#define JOYSTICK_UP_PIN     D2
#define JOYSTICK_DOWN_PIN   D3
#define JOYSTICK_LEFT_PIN   D10
#define JOYSTICK_RIGHT_PIN  D11
#define DROP_BUTTON_PIN     D12

#define MICROSTEPS  1
#define MOTOR_STEPS 200
#define MOTOR_RPM   600

#define WINCH_SPEED 150
#define WINCH_DROP_TIME 1500

#define RIGGING_VALUE 5 //0 to 10, the higher the number the more rigged the claw is. 

bool error=false; //Global error state, if this is true, all other task are shut down until ou reboot.
const long limit_X1_timeout = 5000; //If the Limit swith isn't activated in this amount of time, the machine will enter error state.
const long limit_X2_timeout = 5000;
const long limit_Y1_timeout = 5000;
const long limit_Y2_timeout = 5000;
long limit_Winch_timeout = WINCH_DROP_TIME+2000;

DRV8834 stepperX(MOTOR_STEPS, X_DIR_PIN, X_STEP_PIN);
DRV8834 stepperY(MOTOR_STEPS, Y_DIR_PIN, Y_STEP_PIN);
SyncDriver controller(stepperX, stepperY);


Servo myservo;

TaskHandle_t TaskHandleJoystick;
TaskHandle_t TaskHandleDropButton;
TaskHandle_t TaskHandleError;

bool dropActivated = false;
typedef enum STATES {
    S_IDLE,
    S_OPEN,
    S_CLOSE,
    S_TEST,
    S_RIGGED

} STATES;

STATES clawState = S_OPEN;

//fuctions
void setupPins();
void bootUpSequence();
void dropSequence();
void homeGantry();
void chuteGantry();
void homeWinch();
void claw(STATES state);
//tasks
void TaskJoystick(void *pvParameters);
void TaskDropButton(void *pvParameters);
void TaskError(void *pvParameters);


void setup() {
    Serial.begin(115200);
    setupPins();
    stepperX.begin(MOTOR_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_RPM, MICROSTEPS);
    myservo.attach(CLAW_PWM_PIN);

    xTaskCreate(
        TaskJoystick,     // Task function
        "JoystickTask",   // Name of task
        1000,             // Stack size (bytes)
        NULL,             // Parameter to pass
        2,                // Task priority
        &TaskHandleJoystick // Task handle
    );

    xTaskCreate(
        TaskDropButton,   // Task function
        "DropButtonTask", // Name of task
        1000,             // Stack size (bytes)
        NULL,             // Parameter to pass
        1,                // Task priority
        &TaskHandleDropButton // Task handle
    );

        xTaskCreate(
        TaskError,   // Task function
        "ErrorTask", // Name of task
        1000,             // Stack size (bytes)
        NULL,             // Parameter to pass
        3,                // Task priority
        &TaskHandleError // Task handle
    );

    vTaskStartScheduler();
}

void loop() {
    // Nothing here, as tasks are managed by FreeRTOS
}

//--------------Tasks---------------


// Task to handle joystick input and move steppers
void TaskJoystick(void *pvParameters) {
    
    bootUpSequence();
    unsigned long previousMillis = 0;  // Stores last time the position was printed
    const long interval = 10000;  // Interval for position output (1000 ms = 1 second

    for (;;) {
        bool xMotorActive = false;
        bool yMotorActive = false;
        int xSteps = 50;
        int ySteps = 50;
        bool upState = 1;
        bool downState = 1;
        bool leftState = 1;
        bool rightState= 1;
        bool x1LimitState=1;
        bool x2LimitState=1;
        bool y1LimitState=1;
        bool y2LimitState=1;
        bool winchLimitState=1;

        //read the joystick and limit switches
        upState=digitalRead(JOYSTICK_UP_PIN);
        downState=digitalRead(JOYSTICK_DOWN_PIN);
        leftState=digitalRead(JOYSTICK_LEFT_PIN);
        rightState=digitalRead(JOYSTICK_RIGHT_PIN);
        x1LimitState=(digitalRead(LIMIT_X1));
        x2LimitState=(digitalRead(LIMIT_X2));
        y1LimitState=(digitalRead(LIMIT_Y1));
        y2LimitState=(digitalRead(LIMIT_Y2));
        winchLimitState=(digitalRead(LIMIT_WINCH));

        unsigned long currentMillis = millis();

        // Print joystick and limit states every 10 seconds for debugging
        if (currentMillis - previousMillis >= interval) {

            Serial.print("Up: ");
            Serial.print(upState);
            Serial.print("           Down: ");
            Serial.print(downState);
            Serial.print("           Left: ");
            Serial.print(leftState);
            Serial.print("          Right: ");
            Serial.println(rightState);
            Serial.print("Y Limit rear: ");
            Serial.print(y1LimitState);
            Serial.print(" Y limit front: ");
            Serial.print(y2LimitState);
            Serial.print(" X Limit Left: ");
            Serial.print(x1LimitState);
            Serial.print(" X limit right: ");
            Serial.print(x2LimitState);
            Serial.print(" Winch Limit: ");
            Serial.println(winchLimitState);

            previousMillis = currentMillis;

        }

        //if nobody is playing with the machine, put the motors to sleep.
        if(upState&&downState&&leftState&&rightState==1&&dropActivated==false){
            digitalWrite(X_SLEEP_PIN, LOW);
        } else {

            //figure out which way to spin the motors
            if(downState==0) ySteps=ySteps*-1;
            if(rightState==0) xSteps=xSteps*-1;
            if(downState&&upState) ySteps=0;
            if(leftState&&rightState) xSteps=0;

            //Check the limit switches and direction
            if(upState&&y2LimitState==0)    {ySteps=0;}
            if(downState&&y1LimitState==0)  {ySteps=-0;}
            if(leftState&&x2LimitState==0)  {xSteps=0;}
            if(rightState&&x1LimitState==0) {xSteps=-0;}

            //Move the motor if the drop sequence isn't active
            if (!dropActivated) {
                digitalWrite(X_SLEEP_PIN, HIGH);
                controller.move(xSteps, ySteps);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to debounce switches
    }
}

// Task to handle the drop button
void TaskDropButton(void *pvParameters) {
    //Test the claw on boot up
    claw(clawState=S_TEST);

    int speed = 255;

    for (;;) {
        if (digitalRead(DROP_BUTTON_PIN) == LOW) {

            dropSequence();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to debounce switches
        
    }
}

// Task to handle Errors
void TaskError(void *pvParameters) {
    for (;;) {
        if (error==true) {
            Serial.println("!!!!!!!!!!!!!Error State detected suspending all tasks!!!!!!!!!!!!");
            Serial.println("!!!!!Reboot required to clear error!!!!!!!!");
            vTaskSuspendAll();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to debounce switches
        
    }
}



//--------------Functions---------------
// Function to initialize pins
void setupPins() {
    pinMode(X_SLEEP_PIN, OUTPUT);
    pinMode(Y_SLEEP_PIN, OUTPUT);
    pinMode(CLAW_PWM_PIN, OUTPUT);

    pinMode(JOYSTICK_UP_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_DOWN_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_LEFT_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_RIGHT_PIN, INPUT_PULLUP);
    pinMode(DROP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LIMIT_X1, INPUT_PULLUP);
    pinMode(LIMIT_X2, INPUT_PULLUP);
    pinMode(LIMIT_Y1, INPUT_PULLUP);
    pinMode(LIMIT_Y2, INPUT_PULLUP);
    pinMode(LIMIT_WINCH, INPUT_PULLUP);

    digitalWrite(X_SLEEP_PIN, LOW);
    digitalWrite(Y_SLEEP_PIN, LOW);

    stepperX.begin(MOTOR_RPM);
    stepperY.begin(MOTOR_RPM);
}

// Boot-up sequence function
void bootUpSequence() {

    //change the winch timout to 10s just for bootup because spool might be completely unwound
    long held_value=limit_Winch_timeout;
    limit_Winch_timeout=10000;

    homeWinch();

    //put the winch timout limit back;
    limit_Winch_timeout=held_value;

    homeGantry();
    claw(clawState=S_OPEN);
}

void dropSequence() {

    dropActivated = true;

    //lower the winch
    analogWrite(WINCH_IN1, 0);
    analogWrite(WINCH_IN2, 0);
    analogWrite(WINCH_IN1, 255);
    vTaskDelay(pdMS_TO_TICKS(500));
    analogWrite(WINCH_IN1, WINCH_SPEED);
    vTaskDelay(pdMS_TO_TICKS(WINCH_DROP_TIME));
    analogWrite(WINCH_IN1, 0);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before next step

    //Close the claw
    bool rigged=(random(11)<(RIGGING_VALUE)) ? true : false; // are we rigging the claw this time?
    if (rigged) claw(clawState=S_RIGGED);
    else claw(clawState=S_CLOSE);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before next step


    //raise the winch
    homeWinch();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before next step


    //move the gantry to the chute
    chuteGantry();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait before next step
    

    //open the claw
    claw(clawState=S_OPEN);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait before next step


    //Return to home position
    homeGantry();


    vTaskDelay(pdMS_TO_TICKS(3000));// wait before starting again.

    dropActivated = false;
}


void homeGantry() {
    // Wake up motors
    digitalWrite(X_SLEEP_PIN, HIGH);
    digitalWrite(Y_SLEEP_PIN, HIGH);

    // Move to the left x limit
    while (digitalRead(LIMIT_X1)){
        stepperX.move(MOTOR_STEPS/4);
    }
    // Move to the back y limit
    while (digitalRead(LIMIT_Y1)) {
        stepperY.move(MOTOR_STEPS/4);
    }

    // Put motors to sleep
    digitalWrite(X_SLEEP_PIN, LOW);
    digitalWrite(Y_SLEEP_PIN, LOW);
}

void homeWinch(){

    unsigned long start_time = millis();
    unsigned long current_time = start_time;  // Stores last time the position was printed

    //raise the winch until it hit's the limit switch
    analogWrite(WINCH_IN1, 0);
    analogWrite(WINCH_IN2, 0);

    //Raise the claw until it hits the limit switch

    if(digitalRead(LIMIT_WINCH)==1){
        analogWrite(WINCH_IN2, WINCH_SPEED);
    }

    while (digitalRead(LIMIT_WINCH))
    {
            
        if (current_time - start_time >= limit_Winch_timeout) {

            Serial.println("!!!!!Error Winch limit switch timout!!!!!!!!");
            Serial.print(current_time);
            Serial.print("-");
            Serial.print(start_time);
            Serial.print("=");
            Serial.println(current_time-start_time);
            Serial.print("Limit is: ");
            Serial.println(limit_Winch_timeout);
            error=true;
            break;
        }
        current_time=millis();
        delay(100);

    }

    analogWrite(WINCH_IN1, 0);
    analogWrite(WINCH_IN2, 0);

}

void chuteGantry() {
    // Wake up motors
    digitalWrite(X_SLEEP_PIN, HIGH);
    digitalWrite(Y_SLEEP_PIN, HIGH);

    // Move to the left x limit
    while (digitalRead(LIMIT_X1)){
        stepperX.move(MOTOR_STEPS/4);
    }
    // Move to the front y limit
    while (digitalRead(LIMIT_Y2)) {
        stepperY.move(MOTOR_STEPS/-4);
    }

    // Put motors to sleep
    digitalWrite(X_SLEEP_PIN, LOW);
    digitalWrite(Y_SLEEP_PIN, LOW);

}


void claw(STATES state){
    switch (state)
    {
    case S_OPEN:
        myservo.write(10);
        break;
    case S_CLOSE:
        myservo.write(90);
        break;
    case S_TEST:
        myservo.write(10);
        vTaskDelay(pdMS_TO_TICKS(1000));
        myservo.write(90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        myservo.write(10);
        vTaskDelay(pdMS_TO_TICKS(1000));
        myservo.write(90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        myservo.write(10);
        break;
    case S_RIGGED:
        myservo.write(70);
        break;    
    default:
        break;
    }
}
