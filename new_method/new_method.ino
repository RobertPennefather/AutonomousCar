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

long last_distanceL, last_distanceF, last_distanceR;

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
int vNomR = 200; 
int vNomL = 200;
int dNom = 30; 
int VR, VL;

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

    //Get distances from each sensor
    long distanceL = getDistance(trigPinL, echoPinL);
    long distanceF = getDistance(trigPinF, echoPinF);
    delay(50);

    digitalWrite(ER, LOW);
    digitalWrite(EL, LOW);
    delay(20);

    //if(distanceL < 20) distanceL = 0;
    error = distanceL - dNom;

    VR = vNomR + Kp*error;
    VL = vNomL - Kp*error;
    
    //The front sensor takes control if about to crash and nothing is close to left sensor
    if (distanceF < 50 || distanceL < 40 || last_distanceF < 50 || last_distanceL < 40){
      VR = 0;
      VL = 250;
    }

    //Set min and max for each velocity (0-250)
    if(VR > 250) VR = 250;
    if(VR < 0) VR = 0;
    if(VL > 250) VL = 250;
    if(VL < 0) VL = 0;

    Serial.print("Left: ");
    Serial.print(distanceL);
    Serial.print("\tFront: ");
    Serial.print(distanceF);
    Serial.print("\tR Speed: ");
    Serial.print(VR);
    Serial.print("\tL Speed: ");
    Serial.print(VL);
    Serial.println("");

    //Set speeds of motors
    analogWrite(ER, VR); 
    analogWrite(EL, VL);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);

    last_distanceL = distanceL;
    last_distanceF = distanceF;
    
    delay(100);
}

long getDistance (int trigPin, int echoPin){

    //The following trigPin/echoPin cycle is used to determine the distance of the nearest object by bouncing soundwaves off of it.
    digitalWrite(trigPin, LOW);  
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);  
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);  
    long duration = pulseIn(echoPin, HIGH);

    //Calculate the distance (in cm) based on the speed of sound.
    long distance = duration/58.2; 

    return distance;
}

void turnLeft(){

    //Set speeds of motors
    analogWrite(ER, 250); 
    analogWrite(EL, 0);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);
    delay(200);

    //Set speeds of motors
    analogWrite(ER, 125); 
    analogWrite(EL, 175);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);
    delay(3000);
}

