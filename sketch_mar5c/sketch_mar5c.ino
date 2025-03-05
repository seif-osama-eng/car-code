// STM32F103C8T6 (Blue Pill) - Car Control Code with Dual L298 Motor Drivers, IR Sensors, and PWM Speed Control
// Control 4 wheels (each wheel controlled independently)

// Left Front Motor
#define IN1 PA8
#define IN2 PA9

// Left Rear Motor
#define IN3 PB10
#define IN4 PA11

// PWM Control Pins
#define ENA1 PB6
#define ENB1 PB7

// IR Sensor connections
#define IR_RIGHT1 PB0
#define IR_RIGHT2 PC13
#define IR_LEFT1  PB13
#define IR_LEFT2  PB12
#define IR_RIGHT0 PB14
#define IR_LEFT0 PB1


int motorSpeedLeft = 200;  // Speed range 0-255
int motorSpeedRight = 200; // Speed range 0-255

void setup() {
  // Set motor control pins as output

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set PWM pins as output
  pinMode(ENA1, OUTPUT);
  pinMode(ENB1, OUTPUT);

  // Set sensor pins as input
  pinMode(IR_RIGHT1, INPUT);
  pinMode(IR_RIGHT2, INPUT);
  pinMode(IR_LEFT1, INPUT);
  pinMode(IR_LEFT2, INPUT);
  pinMode(IR_RIGHT0, INPUT);
  pinMode(IR_LEFT0, INPUT);

  // Initially stop the car
  stopMotors();
}

void loop() {
  bool right1 = digitalRead(IR_RIGHT1);
  bool right2 = digitalRead(IR_RIGHT2);
  bool left1 = digitalRead(IR_LEFT1);
  bool left2 = digitalRead(IR_LEFT2);
  bool right0 = digitalRead(IR_RIGHT0);
  bool left0 = digitalRead(IR_LEFT0);
  

 if (right1 == LOW && right2 == LOW && left1 == LOW && left2 == LOW && right0 == LOW && left0 == LOW) {
    movebackward(); 
    }
   else if ( left1 == LOW && left2 == HIGH && right1==LOW && right2==LOW && right0 == LOW && left0 == LOW) {
    sharpRight();
  }else if ( left1 == HIGH && left2 == HIGH && right1==LOW && right2==LOW && right0 == LOW && left0 == LOW) {
    sharpRight();
  } else if ( left1 == HIGH && left2 == HIGH && right1==LOW && right2==LOW && right0 == LOW && left0 == HIGH) {
    verySharpRight();
   } else if ( left1 == HIGH && left2 == LOW && right1==LOW && right2==LOW && right0 == LOW && left0 == HIGH) {
    verySharpRight();
   }else if ( left1 == LOW && left2 == LOW && right1==HIGH && right2==HIGH && right0 == HIGH && left0 == LOW) {
    verySharpLeft();
  } else if (right1 == LOW && right2 == HIGH && left1 == LOW && left2 == LOW && right0 == LOW && left0 == LOW) {
    sharpLeft();
    }else if (right1 == HIGH && right2 == HIGH && left1 == LOW && left2 == LOW && right0 == LOW && left0 == LOW ) {
    sharpLeft();
    }else if (right1 == HIGH && right2 == LOW && left1 == LOW && left2 == LOW && right0 == LOW && left0 == HIGH) {
    verySharpLeft();
    }
   else if (right1 == HIGH && right2 == HIGH && left1 == HIGH && left2 == HIGH && right0 == HIGH && left0 == HIGH  ) {
    stopMotors();
  
  }/*else if (right1 == HIGH && right2 == LOW && left1 == HIGH && left2 == LOW && center==HIGH) {
    movebackward();
    } else if (right1 == HIGH && right2 == HIGH && left1 == HIGH && left2 == LOW && center==HIGH) {
    movebackward();
    } 
    else if(left1 == HIGH && left2 == HIGH && right1==HIGH && right2==HIGH && center==HIGH)  {

movebackward();
  }*/
}

void moveForward() {
  analogWrite(ENA1, motorSpeedRight);
  analogWrite(ENB1, motorSpeedLeft);
 
 digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void sharpLeft() {
  analogWrite(ENA1, motorSpeedRight );
  analogWrite(ENB1, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void verySharpLeft() {
  analogWrite(ENA1, motorSpeedRight);
  analogWrite(ENB1, 100);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(400);
}

void sharpRight() {
  analogWrite(ENA1, 0);
  analogWrite(ENB1, motorSpeedRight );

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void verySharpRight() {
  analogWrite(ENA1, 100);
  analogWrite(ENB1, motorSpeedRight);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(400);
}

void stopMotors() {
  analogWrite(ENA1, 0);
  analogWrite(ENB1, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}
void movebackward() {
  analogWrite(ENA1, motorSpeedLeft);

  analogWrite(ENB1, motorSpeedRight);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}
