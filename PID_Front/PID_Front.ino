#define ER 10  // Enable Pin for motor 1
#define EL 11  // Enable Pin for motor 2
 
#define I1 8  // Control pin 1 for motor 1
#define I2 9  // Control pin 2 for motor 1
#define I3 12  // Control pin 1 for motor 2
#define I4 13  // Control pin 2 for motor 2

// Define echo and trigger pins for sensors

//Left Sensor
#define echoPinL 2 
#define trigPinL 3 

//Front Sensor
#define echoPinF 4 
#define trigPinF 5

int maximumRange = 350; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

//int leftmotorpmw = 220; 
//int rightmotorpmw = 160;

//initialising PID stuff
float Kp = 0.8; //~0.7 - 1 seems to work on smoother surfaces
float Ki = 0.002;
float Kd = 1.3;

int vNomR = 150; 
int vNomL = 200;
int dNom = 50; 
int v1, v2;

float integral = 0;
float derivative = 0;
float last_error = 0;
float error = 0;

float threshold = 20;
 
void setup() {

    Serial.begin (9600);
 
    pinMode(ER, OUTPUT);
    pinMode(EL, OUTPUT);
 
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

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

    //low high high low makes motors go forward
    //high low low high makes motors go backward

    // following wall, move forward if less than 50cm away from wall 

    error = distanceL - dNom;

    if(abs(error) < threshold){
      integral = integral + error;
    }
    else{
      integral=0;
    }

    derivative = error - last_error;
    last_error = error;

    v1 = vNomR + Kp*error + derivative*Kd + integral * Ki;  
    v2 = vNomL - Kp*error - derivative*Kd - integral * Ki;  

    //The front sensor takes control if about to crash and nothing is close to left sensor
    if (distanceF < 50){
      //v1 = 0;
      //v2 = 250;
    }

    if(v1 > 250) v1 = 250;
    if(v2 > 250) v2 = 250;
    if(v1 < 0) v1 = 0;
    if(v2 < 0) v2 = 0;

    Serial.print("Left: ");
    Serial.print(distanceL);
    Serial.print("\tFront: ");
    Serial.print(distanceF);
    Serial.print("\tR Speed: ");
    Serial.print(v1);
    Serial.print("\tL Speed: ");
    Serial.print(v2);
    Serial.println("");

    
    analogWrite(ER, v1); // Run slow speed (range 0 - 255, where 255 is max speed)
    analogWrite(EL, v2); 
    
    digitalWrite(I1, LOW); //high low high low sets direction -> forwards? 
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    
    delay(100);
}

long getDistance (int trigPin, int echoPin){

    //The following trigPinL/echoPinL cycle is used to determine the distance of the nearest object by bouncing soundwaves off of it.
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

