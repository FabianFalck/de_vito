/*
This code is part of the research project
"DE VITO: A Dual-arm, High Degree-of-freedom, Lightweight, Inexpensive, Passive Upper-limb Exoskeleton for Robot Teleoperation"
This code was written, edited and documented by:
- Kawin Larppichet (Imperial College London, Robot Intelligence Lab)
- Fabian Falck (Imperial College London, Robot Intelligence Lab)
For correspondence on this project, please open an Issue on Github.
Further details can be found at http://www.imperial.ac.uk/robot-intelligence/robots/de_vito/.


General remarks on this script:
-This code is designed to maximize the reading speed of the exoskeleton. It can read up to 600Hz (10 times increase from 60 Hz).
-The data is composed of 14 angles from potentiometers, 2 analog sticks, 4 buttons from Nintendo Nunchuks, and 1 IMU.
-All information is packed into packages of 23 bytes.
*/

#include <Wire.h>
#include <Nunchuk.h>
#include <I2Cdev.h>
#include <MPU6050.h>
int ch=0;
int channel=0;
const int ADC_channel = 14;
const int total_byte=23;
int angles[ADC_channel];
byte data[total_byte];
int hbyte,lbyte;
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int l_c,l_z,l_ax,l_ay;
int r_c,r_z,r_ax,r_ay;
const int debug=0;
#define TCAADDR 0x70

ISR(ADC_vect)           
{
  lbyte=ADCL;
  hbyte=ADCH;
  angles[channel]=(hbyte<<2)|(lbyte>>6);
  if (++channel >= ADC_channel){
    channel=0;                                 //Looping the ADC reading to the first channel
  }
  if (channel<7){
    ADMUX = (1<<ADLAR) | (1<<REFS0) | channel; //Select ADC Channel
    ADCSRB &= ~(1<<MUX5);
  }
  else {
    ADMUX = (1<<ADLAR) | (1<<REFS0) | (channel-7); //Select ADC Channel
    ADCSRB |= (1<<MUX5);
  }
  ADCSRA |= (1 << ADSC);    // Start A2D Conversions 
} 

void setup() {
  ADCSRA &=B11111111;
  
  Serial.begin(2000000);    //Please make sure that the python script defines that same reading speed 
  Wire.begin();
  Wire.setClock(800000);    //Set up the I2C speed

  tcaselect(0);             //select the channel for the i2c mux
  nunchuk_init();           //initiate the first Nunchuk
  delay(1);
  
  tcaselect(1);             //select the channel for the i2c mux
  nunchuk_init();           //initiate the second Nunchuk
  delay(1);
  
  tcaselect(2);             //select the channel for the i2c mux
  mpu.initialize();         //initiate the IMU sensor
  delay(1);
  
  for(int i=0;i<total_byte;i++){
    data[i]=0;
  }
  for(int i=0;i<ADC_channel;i++){
    angles[i]=0;
  }
  
  ADMUX |= (1 << REFS0);     // Set ADC reference to AVCC
  ADMUX |= (1 << ADLAR);     
  ADCSRA |= (1 << ADEN);     // Enable ADC
  ADCSRA |= (1 << ADIE);     // Enable ADC Interrupt
  sei();                     // Enable Global Interrupts
  ADCSRA |= (1 << ADSC);     // Start A2D Conversions  
}

void loop() {
  
    tcaselect(0);
    nunchuk_read();
    r_c=nunchuk_buttonC();
    r_z=nunchuk_buttonZ();
    r_ax=nunchuk_joystickX();
    r_ay=nunchuk_joystickY();
    
    tcaselect(1);
    nunchuk_read();
    l_c=nunchuk_buttonC();
    l_z=nunchuk_buttonZ();
    l_ax=nunchuk_joystickX();
    l_ay=nunchuk_joystickY();

    tcaselect(2);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //Saturating the IMU yaw motion
    
      if((gx/131)+128>255){
        gx=16637;
      }
      else if((gx/131)+128<0){
        gx=-16899;
      }
    
    //// packing poteniometers data
    
    //pack all of the lowbyte of integer variable
    
      for(int i=0;i<ADC_channel;i++){
        data[i]=lowByte(angles[i]);
      }
      
    // clean some bytes of data since the next algorithm will only use and , or logic. We need emthy byte.
      data[14]=B0;
      data[15]=B0;
      data[16]=B0;
      data[17]=B0;
      
    //packing highbytes to index 14 15 16 17
    
      for(int i=0;i<ADC_channel;i++){
        data[i/4+ADC_channel]+=highByte(angles[i])<<((2*i)%8);
      }
      
    // packing buttons data on x from Bxxxx???? of index 17
    
      int buttons=(r_c<<3)+(r_z<<2)+(l_c<<1)+(l_z);
      data[17]+=(buttons<<4);
      
    // packing analog buttons from nunchuk
      data[18]=(r_ax+127);
      data[19]=(r_ay+128);
      data[20]=(l_ax+127);
      data[21]=(l_ay+128);
      
    // imu data into byte
      data[22]=(gx/131+128);
  
      Serial.println("s");          //Begin the transmission with the "S\n" token
      Serial.write(data,23);        //Write the whole package of data
}

void tcaselect(uint8_t i) {
  if (i > 7)return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
