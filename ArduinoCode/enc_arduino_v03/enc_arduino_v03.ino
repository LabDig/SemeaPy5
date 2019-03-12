int pin_w = 12;
int pin_s = 10;
int pulse_w=0;
int pulse_s=0;
int st_w = 0;
int ls_st_w = 0;
int st_s = 0;
int ls_st_s = 0;
double vw;
double vs;
unsigned long t_c = 0;
unsigned long previousMillis;
unsigned long currentMillis;


void setup() {

  pinMode(pin_w, INPUT);
  pinMode(pin_s, INPUT);
  Serial.begin(9600);

}

void loop() {

  if (Serial.available()) {
    t_c = Serial.parseInt();
  }
  st_w = digitalRead(pin_w);
  st_s = digitalRead(pin_s);

  if (st_w == 0 && ls_st_w == 1)  pulse_w++; //up border
  if (st_w == 1 && ls_st_w == 0) pulse_w++; //for low spped
  if (st_s == 0 && ls_st_s == 1) pulse_s++;//up border
  if (st_s == 1 && ls_st_s == 0) pulse_s++;
  //atualiza variaveis
  ls_st_w = st_w;
  ls_st_s = st_s;

  currentMillis = millis();
  if (currentMillis - previousMillis > t_c && t_c>0) {
    previousMillis = currentMillis;

 
      vw=2.0*(pulse_w/160.0)/(t_c/1000.0);
      vs=(pulse_s/8.0)/(t_c/1000.0);
      Serial.print(vw);
      Serial.print(',');
      Serial.println(t_c);
      pulse_s=0;
      pulse_w=0;
 
  }
}





