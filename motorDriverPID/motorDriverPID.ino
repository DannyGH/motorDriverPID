volatile int RPM = 0;
volatile unsigned int lastMillis = 0;
void updateRPM()
{
  if(!lastMillis)
  {
    lastMillis = millis();
    RPM = 0;
    return;
  }
  int diff = millis() - lastMillist;
  lastMillis = millis();
  if(diff > 0)
  {
    RPM = 60000 / diff;
  }
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(0, updateRPM, RISING);
}

void loop() {
  unsigned int curMillis = millis();
  if(lastMillis && (curMillis - lastMillis) > 1000)
  {
    lastmillis = 0;
    RPM = 0;
  }
}
