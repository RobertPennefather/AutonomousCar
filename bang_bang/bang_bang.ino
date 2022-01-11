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

int dist_min = 20;
int dist_max = 25;
int corner_threshold = 100;
int front_threshold = 50;

int vel_low = 50;
int vel_high = 200;
int VR, VL;
 
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
    
    if ((distanceF < front_threshold) && (last_distanceF < front_threshold)){
      VR = 0;
      VL = 0;
    }
    else{
      
      //Turn left
      if ((distanceL > corner_threshold) && (last_distanceL > corner_threshold)){
        VR = vel_low;
        VL = vel_low;
      }
  
      //Go towards middle
      else if(distanceL < dist_min){
        VR = vel_low;
        VL = vel_high;
      }
      else if (distanceL > dist_max){
        VR = vel_high;
        VL = vel_low;
      }
  
      //Within wanted range from wall
      else {
  
        //If distance is changing correct it
        if(last_distanceL > distanceL){
          VR = vel_low;
          VL = vel_high;
        }
        else if (last_distanceL < distanceL){
          VR = vel_high;
          VL = vel_low;
        }
  
        //Continue straight
        else {
          VR = vel_high;
          VL = vel_high;
        }
      }
    }

    //Set speeds of motors
    analogWrite(ER, VR); 
    analogWrite(EL, VL);

    //low high high low makes motors go forward
    digitalWrite(I1R, LOW); 
    digitalWrite(I2R, HIGH);
    digitalWrite(I1L, HIGH);
    digitalWrite(I2L, LOW);

    //Print readings
    Serial.print("Left: ");
    Serial.print(distanceL);
    Serial.print("\tFront: ");
    Serial.print(distanceF);
    Serial.print("\tR Speed: ");
    Serial.print(VR);
    Serial.print("\tL Speed: ");
    Serial.print(VL);
    Serial.println("");

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

