//Software for Arduino ProMini 3.3 V 8 Mhz
//Used in developed controller for seed variable rate aplication.
//Arduino was used to read square wave signal generated by optical switch and cut-off disk
//And calculate planter speed and angular velocity of eletric motor of seed metering device
//The planter speed and angular velocity it is send by BeagleBone Black each t_c seconds.
//Autor : Andre Coelho , Daniel Queiroz, Sarvio Valente and Franscisco Pinto
// 
//define digital pin number
int pin_w = 12; //optical switch of wheel
int pin_s = 10; //optical switch of seed metering devices
//
//Declarate variables
//
//_w refer to wheel (planter speed)
//_s refer to seed metering device (angular velocity of electric motor)
int pulse_w=0; //number of pulse
int pulse_s=0; //number of pulse
int st_w = 0; //digital status (0 or 1)
int ls_st_w = 0; //last digital status (old)
int st_s = 0; //digital status
int ls_st_s = 0; //last digital status
double vw; //planter speed
double vs; //angular velocity of motor
unsigned long t_c = 0; //Time interval to send datas
//Time for calculate and send datas without delay
unsigned long previousMillis; 
unsigned long currentMillis;
//
//
void setup() {
  //Configure pinMode and Start Serial Comunication
  pinMode(pin_w, INPUT);
  pinMode(pin_s, INPUT);
  Serial.begin(9600);
}
//
void loop() {
  // If BeagleBone Black send the time interval
  if (Serial.available()){
    t_c=Serial.parseInt();//Read the data and convert to int
  }
  //Read actual level (high or low) of square wave
  st_w = digitalRead(pin_w);
  st_s = digitalRead(pin_s);
  //If have a rising edge or falling edge in square wave, increment pulse variabless
  if (st_w == 0 && ls_st_w == 1)  pulse_w++; 
  if (st_w == 1 && ls_st_w == 0) pulse_w++; 
  if (st_s == 0 && ls_st_s == 1) pulse_s++;
  if (st_s == 1 && ls_st_s == 0) pulse_s++;
  //update the actual level
  ls_st_w = st_w;
  ls_st_s = st_s;
  //update actual time
  currentMillis = millis();
  //Each time interval t_c:
  if (currentMillis - previousMillis > t_c && t_c>0) {
      //
	  previousMillis = currentMillis; //update last time
	  //Calculate the planter speed (in m/s)
	  //the disk have 80 cut-off, therefore in a cycle there are 80 rising edge and 80 falling edge
	  //2.0 it is the perimeter of ground wheel of planters
      vw=2.0*(pulse_w/160.0)/(t_c/1000.0);
	  //Calculate of angular speed of electric motor (in rad/s)
      vs=(2.0*3.14159*pulse_s/160.)/(t_c/1000.0);
	  //Send to BeagleBone Black the datas as a message in format : @,vw,vs,t_c\n
      Serial.print('@');
      Serial.print(',');
      Serial.print(vw);
      Serial.print(',');
      Serial.print(vs);
      Serial.print(',');
      Serial.println(t_c);
	  //Reset number of pulses
      pulse_s=0;
      pulse_w=0;
   }
} //End of void loop function