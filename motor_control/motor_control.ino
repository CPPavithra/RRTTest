

const int AIN1 = 9;
const int AIN2 = 8;
const int PWMA = 10;
const int BIN1 = 7;
const int BIN2 = 6;
const int PWMB = 5;

const int ACTUATOR_IN1 = 4;
const int ACTUATOR_IN2 = 3;
const int PWMACT = 2;

void setup() {
    Serial.begin(9600);
    
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    pinMode(ACTUATOR_IN1, OUTPUT);
    pinMode(ACTUATOR_IN2, OUTPUT);
    pinMode(PWMACT, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        String coords = Serial.readStringUntil('\n');
        int commaIndex = coords.indexOf(',');
        if (commaIndex > 0) {
            String xStr = coords.substring(0, commaIndex);
            String yStr = coords.substring(commaIndex + 1);
            float x = xStr.toFloat();
            float y = yStr.toFloat();
            
            move_rover_to_target(x, y);
            
            if (reached_target(x, y)) {
                collect_soil();
            }
        }
    }
}

void move_rover_to_target(float targetX, float targetY) {
    while (true) {
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;
        
        float threshold = 0.1;

        if (abs(deltaX) < threshold && abs(deltaY) < threshold) {
            stop_motors();
            break;
        }

        if (abs(deltaX) > abs(deltaY)) {
            if (deltaX > 0) {
                forward();
            } else {
                backward();
            }
        } else {
            if (deltaY > 0) {
                right();
            } else {
                left();
            }
        }
    }
}

void forward(){   
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 200); 
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 200);  
    delay(1000);  
}

void backward(){    
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 200);  
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 200); 
    delay(1000); 
}

void left(){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 200);  
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 200);  
    delay(1000);  
}

void right(){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 200); 
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 200); 
    delay(1000);  
}

void stop_motors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

bool reached_target(float currentX, float currentY) {
    return true;
}

void collect_soil() {
    extend();
    delay(3000); 
    retract_actuator();
    delay(3000);
}

void extend() {
    digitalWrite(ACTUATOR_IN1, HIGH);
    digitalWrite(ACTUATOR_IN2, LOW);
    analogWrite(PWMACT, 255);  
}

void pullback() {
    digitalWrite(ACTUATOR_IN1, LOW);
    digitalWrite(ACTUATOR_IN2, HIGH);
    analogWrite(PWMACT, 255);  
}
