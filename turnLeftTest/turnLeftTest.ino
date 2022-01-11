#define ER 10  // Enable Pin for right motor
#define EL 11  // Enable Pin for left motor
 
#define I1R 8  // Control pin 1 for motor 1
#define I2R 9  // Control pin 2 for motor 1
#define I1L 12  // Control pin 1 for motor 2
#define I2L 13  // Control pin 2 for motor 2

// Define echo and trigger pins for sensors

//Left Sensor
#define echoPinL 2 
#define trigPinL 3 

//Front Sensor
#define echoPinF 4 
#define trigPinF 5

long last_distance;

//Initialising PID constants
float Kp = 3; //~0.7 - 1 seems to work on smoother surfaces
float Ki = 0.002;
float Kd = 1.3;

//Initialising PID variables
float integral = 0;
float derivative = 0;
float last_error = 0;
float error = 0;

//Nominated velocity and distance from wall the car aims to keep
int vNomR = 150;//150; 
int vNomL = 150;
int dNom = 50; 
int VR, VL;

bool turnLeftBegun = false;

float threshold = 20;
 
void setup() {

    Serial.begin(9600);
 
    pinMode(ER, OUTPUT);
    pinMode(EL, OUTPUT);
    
    pinMode(I1R, OUTPUT);
    pinMode(I2R, OUTPUT);
    pinMode(I1L, OUTPUT);
    pinMode(I2L, OUTPUT);
    
    pinMode(trigPinL, OUTPUT);
    pinMode(trigPinF, OUTPUT);
    
    pinMode(echoPinL, INPUT);
    pinMode(echoPinF, INPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
      turnLeft();
    
}

void turnLeft(){

    turnLeftBegun = true;

    //Set speeds of motors
    analogWrite(ER, 250); 
    analogWrite(EL, 0);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);

    //Set speeds of motors
    analogWrite(ER, 125); 
    analogWrite(EL, 175);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);

    //Set speeds of motors
    analogWrite(ER, 0); 
    analogWrite(EL, 0);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);

    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);

    turnLeftBegun = false;
}

