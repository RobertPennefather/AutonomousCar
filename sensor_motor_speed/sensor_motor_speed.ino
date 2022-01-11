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

//Right Sensor
#define echoPinR 6 
#define trigPinR 7 

long last_distance;

//Initialising PID constants
float Kp = 0.3; //~0.7 - 1 seems to work on smoother surfaces
float Ki = 0.002;
float Kd = 1.3;

//Initialising PID variables
float integral = 0;
float derivative = 0;
float last_error = 0;
float error = 0;

//Nominated velocity and distance from wall the car aims to keep
int vNomR = 200;//150; 
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
    pinMode(trigPinR, OUTPUT);
    
    pinMode(echoPinL, INPUT);
    pinMode(echoPinF, INPUT);
    pinMode(echoPinR, INPUT);
}
 
void loop() {

    //Get distances from each sensor
    long distanceL = getDistance(trigPinL, echoPinL);
    long distanceF = getDistance(trigPinF, echoPinF);
    delay(50);
    //long distanceR = getDistance(trigPinR, echoPinR);

    digitalWrite(ER, LOW);
    digitalWrite(EL, LOW);
    delay(20);

    //Follow wall, move forward if less than 50cm away from wall 
    //If distance is moving further away also readjust more
    float front_error = 50 - distanceF;
    if(front_error < 0) front_error = abs(front_error);
    else front_error = 0;
    
    error = distanceL - dNom + front_error;
    
    //PID calculations
    if(abs(error) < threshold){
      integral = integral + error;
    }
    else{
      integral=0;
    }

    derivative = error - last_error;
    last_error = error;
    last_distance = distanceL;

    //Right motor set to move faster as we want to be turning anticlockwise into wall/around corners when distance is too large
    VR = vNomR + Kp*error + derivative*Kd + integral * Ki;  
    VL = vNomL - Kp*error - derivative*Kd - integral * Ki;

    //Set min and max for each velocity (0-250)
    if(VR > 250) VR = 250;
    if(VR < 0) VR = 0;
    if(VL > 250) VL = 250;
    if(VL < 0) VL = 0;

    //Set speeds of motors
    analogWrite(ER, VR); 
    analogWrite(EL, VL);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);
    
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

