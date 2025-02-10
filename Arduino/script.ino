volatile long pos1 = 0;
volatile long pos2 = 0;

const int encoder1PinA = 2;
const int encoder1PinB = 4;
const int encoder2PinA = 3;
const int encoder2PinB = 5;

const int motor1Pin1 = 9;
const int motor1Pin2 = 8;
const int motor2Pin1 = 10;
const int motor2Pin2 = 7;

const long MAX_TICKS = 32000;
const int THRESHOLD = 100;

volatile int a1 = 0, a2 = 0;
int motorSpeed = 150; 

void handleEncoder1() {
  // Read both A and B channels
  bool a = digitalRead(encoder1PinA);
  bool b = digitalRead(encoder1PinB);

  if (b == a) {  // If B matches A, it's one direction
    pos1++;
  } else {  // Otherwise, it's the other direction
    pos1--;
  }

  if (pos1 > MAX_TICKS - THRESHOLD) {
    a1++;
    pos1 = 0;
  } else if (pos1 < -MAX_TICKS + THRESHOLD) {
    a1--;
    pos1 = 0;
  }
}

void handleEncoder2() {
  // Read both A and B channels
  bool a = digitalRead(encoder2PinA);
  bool b = digitalRead(encoder2PinB);

  if (b == a) {
    pos2++;
  } else {
    pos2--;
  }

  if (pos2 > MAX_TICKS - THRESHOLD) {
    a2++;
    pos2 = 0;
  } else if (pos2 < -MAX_TICKS + THRESHOLD) {
    a2--;
    pos2 = 0;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, CHANGE);
}

// pwmleft pwmright dirleft dirright

void loop() {
  if (Serial.available() > 0) {
    String rpmData = Serial.readStringUntil('\n');
    int spaceIndex = rpmData.indexOf(' ');
    if (spaceIndex > 0) {
      int rpmLeft = rpmData.substring(0, spaceIndex).toInt();
      int rpmRight = rpmData.substring(spaceIndex + 1).toInt();

      // Example conversion from RPM to PWM
      int pwmLeft = map(rpmLeft, 0, 300, 0, 255);
      int pwmRight = map(rpmRight, 0, 300, 0, 255);

      // Drive motors forward using PWM
      analogWrite(motor1Pin1, pwmLeft);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, pwmRight);
      analogWrite(motor2Pin2, 0);
    }
  }
    Serial.print(a1 * MAX_TICKS + pos1);
  Serial.print(" ");
  Serial.println(a2 * MAX_TICKS + pos2);
  delay(100);
}

void controlMotors(char command) {
  switch (command) {
    case 'F': 
      analogWrite(motor1Pin1, motorSpeed);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, motorSpeed);
      analogWrite(motor2Pin2, 0);
      Serial.println("Moving Forward");
      break;
    case 'B': 
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, motorSpeed);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, motorSpeed);
      Serial.println("Moving Backward");
      break;
    case 'L':
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, motorSpeed);
      analogWrite(motor2Pin1, motorSpeed);
      analogWrite(motor2Pin2, 0);
      Serial.println("Turning Left");
      break;
    case 'R': 
      analogWrite(motor1Pin1, motorSpeed);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, motorSpeed);
      Serial.println("Turning Right");
      break;
    case 'S': 
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, 0);
      Serial.println("Stopping");
      break;
  }
}