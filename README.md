# BAE305-SP25-Lab9
# Laboratory Report for Lab 9 of BAE305 Spring 2025
# Lab 9 - Take Control: The PID Feedback Controller

* By: Abby Phillips, Audrey Suit, and Elizabeth Frericks
* Submitted: April 9th, 2025


# Summary  



# Materials

* Computer running Arduino IDE and Chrome Browser
* SparkFun Inventor's kit: RedBoard, ultrasonic sensor, two motors, and motor driver

# Assembly Procedures 

## Part 1: PID Use

1. First, we opened Arduino IDE and isntalled the PID_V2 Library by opening library manager and searching for PID_V2. We installed the latest library by Brett Beauregard.
2. Next, we made the following modifications to our robot sketch:

a. We included the library in the code using this function:
```ruby
#include <PID_V2>
```

b. Next, we defined the setpoint, measurement, output, kp, ki, and kd varoables as type double using the lines of code below:
```ruby
// PID Variables
double setpoint = 20.0;  // Desired distance from the object (cm)
double measurement = 0.0;
double output = 0.0;

// PID tuning parameters (adjust these for best performance)
double Kp = 10.0, Ki =0.0, Kd = 0.0;
```

c. Next, we added this line of code before the setup function to create the PID instance: 
```ruby
PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);
```

d. Then, we initialized the PID within the setup function as shown below: 
```ruby
 myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetMode(AUTOMATIC);
```

e. Next, we added the following line within the loop section of the code to compute the output of the PID according to the tuning and inputs and write the output to its pointer: 

```ruby
myPID.Compute();
```

## Part 2: Keep Your Distance 

1. Next, we wrote a function to tell the robot to move back or forward depending on the measured distance. We implemented a function that obtained the distance output from the PID library (from 0 to 255) and told the robot to move forward or backward with a fast or slow speed depending on the sensed distance. The following function was used to move the robot based on the PID output:
```ruby
void moveRobot(double pidOutput) {
    int speed = map(abs(pidOutput), 0, 200, 70, 225); // Scale speed

    if (pidOutput > 0) {
        // Move forward
        rightMotor(speed);
        leftMotor(-speed);
    } else {
        // Move backward
        rightMotor(-speed);
        leftMotor(speed);
    }
}
```

2. After step 1, the robot was oscillating about the setpoint. We modified Kp, Ki, and Kd to mimimize the error in the system.

## Part 3: Wall Follower

We were unable to complete part 3 of the lab because it was impossible to attach the sensor in the way that this step required. To be able to sense the wall, our robot had to be facing the wall, which would make it impossible to move sideways along the wall. 

# Test Equipment

* Computer running Arduino IDE
* Assembled robot 

# Test Procedures

## Part 1: PID Use

To test the modifications made to our Arduino sketch, we wrote the setpoint, measurement, and output values to the serial port. We read the measurement displayed in the serial moniter (the sistance from the sensor) to ensure that it was reaching the setpoint. We ensured that the output (the motors) responded when the measurement was off from  the setpoint, and we made sure that the behavior was stable. Physically, we used various objects to ensure that the distance the sensor was reading was correct and the speed of the robot wa changing based on this distance. 

## Part 2: Keep Your Distance

To test our modifications to our sketch in this step, we used various objects (an ipad with a black case was most successful) to ensure that the robot was moving forward and backward to keep the set distance between itself and the object. Initially, the robot was oscillating heavily around the setpoint, so we altered the Kp, Ki, and Kd values to mimimize the error in the function, which mimimized oscillation. 


# Test Results

```ruby
#include <PID_v2.h>

// Motor control pins
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMA = 11;

const int BIN1 = 8;
const int BIN2 = 9;
const int PWMB = 10;

// Ultrasonic sensor pins
const int trigPin = 6;
const int echoPin = 7;

// PID Variables
double setpoint = 20.0;  // Desired distance from the object (cm)
double measurement = 0.0;
double output = 0.0;

// PID tuning parameters (adjust these for best performance)
double Kp = 10.0, Ki =0.0, Kd = 0.0;

// Create PID instance
PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(9600);

    // Initialize motor control pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Initialize ultrasonic sensor pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Initialize PID
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    // Measure distance
    measurement = getDistance();

    // Compute PID output
    myPID.Compute();

    // Move the robot based on PID output
    moveRobot(output);

    // Print values for debugging and tuning
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" cm, Measurement: ");
    Serial.print(measurement);
    Serial.print(" cm, Output: ");
    Serial.println(output);

    delay(100);
}

// Function to get distance from ultrasonic sensor
double getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.0343 / 2;  // Convert to cm
}

// Function to move the robot based on PID output
void moveRobot(double pidOutput) {
    int speed = map(abs(pidOutput), 0, 200, 70, 225); // Scale speed

    if (pidOutput > 0) {
        // Move forward
        rightMotor(speed);
        leftMotor(-speed);
    } else {
        // Move backward
        rightMotor(-speed);
        leftMotor(speed);
    }
}

// Function to control the right motor
void rightMotor(int motorSpeed) {
    if (motorSpeed > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else if (motorSpeed < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, abs(motorSpeed));
}

// Function to control the left motor
void leftMotor(int motorSpeed) {
    if (motorSpeed > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else if (motorSpeed < 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, abs(motorSpeed));
}


```
<p align="left"><em> Program 1: The above program demonstrates the use of a proportional-integral-derivative (PID) controller to maintain the disrtance from the robot to an object at 20.0 cm. It does this by communicating with an ultrasonic sensor which reads measurements continuously. This is then turned into a corrective output mapped into motor speed and direction. The code was fine tuned using kp, ki, and kd. Kp reacts to the current error, Ki accounts for past errors, and Kd predicts future errors. Our Kp is set to 10 and our Ki and Kd are set to zero meaning they are inactive. We can adjust the Ki and Kd to fine tune the movement further and prevent overshooting oscillations.   </em></p>



# Discussion



# Conclusion


