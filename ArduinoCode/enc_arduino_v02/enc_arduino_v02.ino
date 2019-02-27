int pin_w = 12;
int pin_s = 10;

int st_w = 0;
int ls_st_w = 0;
int st_s = 0;
int ls_st_s = 0;
unsigned long last_ts;
unsigned long last_tw;
double vw;
double vs;
unsigned long tz_w;
unsigned long tz_s;
unsigned long t_c = 250;
unsigned long previousMillis;
unsigned long currentMillis;
char rec;

void setup() {

  pinMode(pin_w, INPUT);
  pinMode(pin_s, INPUT);
  Serial.begin(9600);

}

void loop() {

  if (Serial.available()) {
    rec = Serial.read();
  }
  delay(5);//5 ms
  st_w = digitalRead(pin_w);
  st_s = digitalRead(pin_s);

  if (st_w == 0 && ls_st_w == 1) tz_w = millis(); //up border
  if (st_w == 1 && ls_st_w == 0) tz_w = millis(); //for low spped
  if (st_s == 0 && ls_st_s == 1) tz_s = millis(); //up border
  if (st_s == 1 && ls_st_s == 0) tz_s = millis();
  if ((millis() - tz_s) < last_ts && last_ts < 600) vs = (1.0/8.0)/(last_ts/1000.0);
  if ((millis() - tz_w) < last_tw && last_tw < 600) vw = (2.02/8.0)/(last_tw/1000.0); //significa que passou pela borda
  if (millis()-tz_w>600) vw=0;
  if (millis()-tz_s>700) vs=0;
  //atualiza variaveis
  last_ts = millis() - tz_s;
  last_tw = millis() - tz_w;
  ls_st_w = st_w;
  ls_st_s = st_s;

  currentMillis = millis();
  if (currentMillis - previousMillis > t_c) {
    previousMillis = currentMillis;
    if (rec == '1') {
      Serial.print(vw);
      Serial.print(',');
      Serial.println(vs);
    }
  }
}




