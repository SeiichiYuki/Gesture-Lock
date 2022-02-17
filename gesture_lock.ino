#include "Wire.h"

//mode
int mode = 0;
bool unlock = false;

//mpu6050
const int MPU6050_addr=0x68;
unsigned long now, lastTime = 0;
float dt;                                   //differential time

int16_t ax, ay, az, gx, gy, gz;             //original data of mpu6050
float aax=0, aay=0,aaz=0, agx=0, agy=-0, agz=0;    //angle variables
float angle[3] = {0};                       //angles of 3 axes
long axo = 0, ayo = 0, azo = 0;             //offsets of accelerometer
long gxo = 0, gyo = 0, gzo = 0;             //offsets of gyroscope

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //ratio of acceleration
float GyroRatio = 131.0;                    //ratio of gyroscope

uint8_t n_sample = 8;                       //sampling number of accelerometer filter
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //sampling array
long aax_sum, aay_sum,aaz_sum;                      //sampling sum

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //array for covariance calculation
float Px=1, Rx, Kx, Sx, Vx, Qx;             //Kalman variable of x axis
float Py=1, Ry, Ky, Sy, Vy, Qy;             //Kalman variable of y axis
float Pz=1, Rz, Kz, Sz, Vz, Qz;             //Kalman variable of z axis

//gesture & password

int password[4] = {1, 2, 3, 4};
long timeUnlck = 0; //the time when it is unlocked
int ges[4] = {0, 0, 0, 0};

//led & button
const int buttonPin = PIN_SW0;    // the number of the pushbutton pin
const int ledPin    = PIN_LED_13; // the number of the LED pin



void setup() {
  
  //I2C
    Wire.begin();
    Serial.begin(115200);
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

   //Kalman filter
    unsigned short times = 200;             //times of sampling
    for(int i=0;i<times;i++)
    {
        getdata(); //read origibal data
        axo += ax; ayo += ay; azo += az;      //samping sum
        gxo += gx; gyo += gy; gzo += gz;
    }
    
    axo /= times; ayo /= times; azo /= times; //calculate offsets of accelerometer
    gxo /= times; gyo /= times; gzo /= times; //calculate offsets of gyroscope

    // Set button input pin
    pinMode(buttonPin, INPUT_PULLUP);
    // Set LED output pins
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);
}
/*---------------------------------------------------------------------------------------------------------------------*/
//main code
void loop() {
  
  int b = checkButton();// Get button status
  
  //button clicked
  if (b == 1) {
    Serial.print("b=");Serial.print(b);Serial.println(".");
    //if locked, proceed to unlocking part
    if(unlock == false){
      Serial.println("unlocking");
      //get unlocking gesture
      getmotion();
      
      //print gesture
      for (int j = 0; j < 4; j++){
        Serial.print(ges[j]);Serial.print(","); 
      }
      //compare with password
      if(password[0]==ges[0] && password[1]==ges[1] && password[2]==ges[2] && password[3]==ges[3]){
        unlock = true;
        timeUnlck = millis();
        Serial.print("unlocked!!!");Serial.println(".");
        //led blink once
        digitalWrite(ledPin, LOW);    
        delay(500);
        digitalWrite(ledPin, HIGH);
      }
      else{
        unlock = false;
        Serial.print("fail to unlock");Serial.println(".");
      }
    }
    //if already unlocked, print
    else if(unlock == true){
      unlock = true;
      timeUnlck = millis();
      Serial.print("unlocked");Serial.println(".");
    }
  }//b=1
    
  //button pressed and hold
  else if (b == 2){
    Serial.print("b=");Serial.print(b);
    //if unlocked, proceed to record a key(passord)
    if(unlock == true){
      //get new gesture
      getmotion();
      //if motion valid
      if(ges[0]!=0 && ges[1]!=0 && ges[2]!=0 && ges[3]!=0){
        password[0]=ges[0];
        password[1]=ges[1];
        password[2]=ges[2];
        password[3]=ges[3];
        unlock = false;
      }
      //print password
      for (int j = 0; j < 4; j++){
        Serial.print(password[j]);Serial.print(","); 
      }
      Serial.print("password updated");Serial.println(".");
      
      digitalWrite(ledPin, LOW);  
      delay(2000);   
      digitalWrite(ledPin, HIGH); 
      timeUnlck = millis();
    }
    //if still locked
    else if(unlock == false){
       Serial.print("unlock to update password");Serial.println(".");
    }
  }//b=2
  
 b = 0; //reset b

 //15s stay unlocked without button pressing, back to locked
 if ((millis()-timeUnlck) > 30000){
   unlock = false;
 }

 
}//loop

/*-------------------------------------------------------------------------------------------------------------------------*/
//get gesture of mpu6050
 int* getmotion(){
  
   int gesture = 0; int k = -1;
   long t = 0; //time when a motion occurs
   t = millis();

   while(k < 3){
     
     //Serial.print("enter gestures");Serial.println(".");
     
     while((millis() - t) < 3000){
       Kalman(); //get angle
       if((agx>-50 && agx<50 ) && (agy>-50 && agy<50)){
          t = millis();
          Serial.print("middle");Serial.println(".");
          break;
       }
     }

     while((millis() - t) < 3000){
        Kalman(); //get angle
        
        //left
        if((agx>-90 && agx<-70 ) && (agy>-50 && agy<50)){
           gesture = 1;
           Serial.print("left");Serial.println(".");
           t = millis();
           break;
        }
      
        //right
        else if((agx>70 && agx<90 ) && (agy>-50 && agy<50)){
           gesture = 2;
           Serial.print("right");Serial.println(".");
           t = millis();
           break;
        }

        //forward
        else if((agy>60 && agy<90 ) && (agx>-50 && agx<50)){
           gesture = 3;
           Serial.print("forward");Serial.println(".");
           t = millis();
           break;
        }
        //backward
        else if((agy>-90 && agy<-65 ) && (agx>-50 && agx<50)){
           gesture = 4;
           Serial.print("backward");Serial.println(".");
           t = millis();
           break;
        }
        
     }//tilt

     while((millis() - t) < 1000){
        Kalman(); //get angle
        if((agx>-50 && agx<50 ) && (agy>-50 && agy<50)){
          k++;
          ges[k] = gesture;
          Serial.print("gesture");Serial.print(k);Serial.print("=");Serial.print(gesture);Serial.println(".");
          t = millis();
          break;
        }
     }

     if((millis() - t) > 10000){
        k = k + 5;
        ges[0] = 0;
        ges[1] = 0;
        ges[2] = 0;
        ges[3] = 0;
      }
   }//k<3
    
    k = -1;
 }

/*----------------------------------------------------------------------------------------------------------------------------*/
//button state
int checkButton(){
  int count = 0;
  while(digitalRead(buttonPin)==LOW){
    delay(10);
    count++;
   }
  if (count > 100){
    return 2; 
  }  
  else if (count > 2 && count < 100){
    return 1; 
  }
  else if (count < 2){
    return 0;
  }
}

/*-----------------------------------------------------------------------------------------------------------------------------*/
//get original data of mpu6050
void getdata()
{    
     Wire.beginTransmission(MPU6050_addr); 
     Wire.write(0x3B);
     Wire.endTransmission(false);
     Wire.requestFrom(MPU6050_addr,14,true);
     ax=Wire.read()<<8|Wire.read();
     ay=Wire.read()<<8|Wire.read();
     az=Wire.read()<<8|Wire.read();
     Wire.read()<<8|Wire.read();
     gx=Wire.read()<<8|Wire.read();
     gy=Wire.read()<<8|Wire.read();
     gz=Wire.read()<<8|Wire.read();
}
/*-------------------------------------------------------------------------------------------------------------------------------*/
//Kalman Filter, get stable data of accelerometer's angles
void Kalman(){
  unsigned long now = millis();            
    dt = (now - lastTime) / 1000.0;           //diferential time
    lastTime = now;                           //last time for sampling
    getdata(); 
    
    float accx = ax / AcceRatio;              
    float accy = ay / AcceRatio;             
    float accz = az / AcceRatio;              
    //angles
    aax = atan(accy / accz) * (-180) / pi;    
    aay = atan(accx / accz) * 180 / pi;       
    aaz = atan(accz / accy) * 180 / pi;     

    aax_sum = 0;      
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //adjust angles to 0-90°
    aays[n_sample-1] = aay;                        //use the ratio from experiment
    aay_sum += aay * n_sample;                     //in this case: 9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;
    
    //angular accelerations
    float gyrox = - (gx-gxo) / GyroRatio * dt; 
    float gyroy = - (gy-gyo) / GyroRatio * dt; 
    float gyroz = - (gz-gzo) / GyroRatio * dt; 
    agx += gyrox;                             
    agy += gyroy;                             
    agz += gyroz;
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    //average of acceleration
    for(int i=1;i<10;i++)
    {                 //测量值平均值运算
        a_x[i-1] = a_x[i];
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                       
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;
    
    //variance
    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    }
    
    Rx = Rx / 9;                    
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                 
    Kx = Px / (Px + Rx);            //Kalman gain
    agx = agx + Kx * (aax - agx);             
    Px = (1 - Kx) * Px;                      

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    /* kalman end */
    
}
