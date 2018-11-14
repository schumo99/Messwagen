//Messwagen mit Bluetooth-Uebertragung basierend auf der Idee und dem Sketch von bubikopf

//#define DEBUG

#define LEDPIN 13
#define HALLPIN 2
#define HALL_ON_OFF_PIN 3


#define MEASURE_LOOP        10   // Anzahl der Umdrehungen zur Ermittlung der Durchschnittsgeschwindigkeit
#define SPEED_CALC_TIME     1000 //in ms: Zeitraum zur Ermittlung der Durchschnittsgeschwindigkeit
#define DEBOUNCE            5    //in ms
#define SHOW_INTERVAL       250  //in ms
#define MIN_SHOW_INTERVAL   5000 //in ms


unsigned long cnt_millis;
volatile unsigned int cnt;
volatile byte measureCnt;
unsigned int show_cnt;
unsigned int show_rpm;
unsigned long showTime;
unsigned long debounceTime;
volatile unsigned int rpm = 0;
unsigned int maxrpm = 0;
unsigned long curTime;
char inbyte = 0;
volatile unsigned long cnt_time[MEASURE_LOOP + 1];


void doCount()
{
  if (digitalRead(HALLPIN) == LOW)
  {
    cnt_millis = millis();
    if (cnt_millis - debounceTime > DEBOUNCE) {
      debounceTime = cnt_millis;
      cnt++;
      measureCnt = (measureCnt + 1) % MEASURE_LOOP;
      byte CntIndex = 1;
      unsigned long diff_time = cnt_millis - cnt_time[measureCnt];
      while (diff_time > SPEED_CALC_TIME && CntIndex < MEASURE_LOOP) {
        diff_time = cnt_millis - cnt_time[(measureCnt + CntIndex) % MEASURE_LOOP];
        CntIndex++;
      }
      if (diff_time > SPEED_CALC_TIME) {
        rpm = 60000L * (2) / (cnt_millis - cnt_time[(measureCnt - 2 + MEASURE_LOOP) % MEASURE_LOOP]);
      } else {
        rpm = 60000L * (MEASURE_LOOP + 1 - CntIndex) / diff_time;
      }
      cnt_time[measureCnt] = cnt_millis;
    }
  }
}

void setup()
{
  Serial.begin(9600);
#ifdef DEBUG
  Serial.println("BT Messwagen");
#endif

  pinMode(HALL_ON_OFF_PIN, OUTPUT);
  digitalWrite(HALL_ON_OFF_PIN, HIGH);
  pinMode(HALLPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(HALLPIN, HIGH);
  for (int x = 0; x < MEASURE_LOOP; x++) {
    cnt_time[x] = x;
  }
  char buffervar[15];
  sprintf(buffervar, "#%05i-%05i~", cnt, rpm);
  Serial.print(buffervar);
  delay(1000);
  Serial.print(buffervar);
  attachInterrupt(digitalPinToInterrupt(HALLPIN), doCount, FALLING);  // hall pin on interrupt 0 = pin 2
}

void loop()
{
  curTime = millis();
  if ((curTime - cnt_time[measureCnt]) > SHOW_INTERVAL * 2) {
    if ((curTime - cnt_time[measureCnt]) > SPEED_CALC_TIME) {
      rpm = 0;
    } else {
      rpm = 60000L * (1) / (cnt_time[measureCnt] - cnt_time[(measureCnt + MEASURE_LOOP - 1) % MEASURE_LOOP]);
    }
  }
  if (rpm > maxrpm) maxrpm = rpm;
  if (curTime - showTime > SHOW_INTERVAL) {
    if (cnt != show_cnt || rpm != show_rpm || (curTime - showTime) > MIN_SHOW_INTERVAL) {
      show_cnt = cnt;
      show_rpm = rpm;
      showTime = curTime;
      char buffervar[15];
      sprintf(buffervar, "#%05i+%05i~", cnt, rpm);
      Serial.print(buffervar);
#ifdef DEBUG
      Serial.print(" ");
      Serial.print(digitalRead(HALLPIN));
      Serial.print(" ");
      Serial.print(curTime);
      Serial.print(" ");
      Serial.print(cnt);
      Serial.print(" ");
      Serial.print(measureCnt);
      Serial.print(" ");
      Serial.print(rpm);
      Serial.print(" ");
      Serial.println(maxrpm);
#endif
    }
  }

  if (Serial.available() > 0)
  {
    inbyte = Serial.read();
    if (inbyte == '0')
    {
      //LED off
      digitalWrite(LEDPIN, LOW);
      digitalWrite(HALLPIN, LOW);
    }
    if (inbyte == '1')
    {
      //LED on
      digitalWrite(LEDPIN, HIGH);
      digitalWrite(HALLPIN, HIGH);
    }
  }
}
