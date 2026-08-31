/*

First, we generate a calibration curve for each p-bit.
Validation uses the calibration curve in the reverse direction. For each p-bit, 
we choose a target Bernoulli parameter p, use the stored calibration curve to 
find the corresponding analog control voltage, apply this voltage through the DAC,
and then sample the p-bit many times. The measured probability P(1) is compared
with the requested value of p. If the difference is within a predefined tolerance TOL, 
the p-bit passes validation.

*/

#include <Arduino.h>

const uint8_t N=8, qPin[N]={2,3,4,5,6,7,8,9};
const float VMIN=0.0f, VMAX=3.3f, TOL=0.02f;
const uint8_t STEPS=50;
const uint32_t SAMPLES=3000;
const uint16_t SAMPLE_US=20;

enum Mode { CALIBRATION, VALIDATION, MONITOR };
Mode mode=CALIBRATION;

float calibP[N][STEPS+1];
const float testP[N]={0.1,0.2,0.3,0.4,0.6,0.7,0.8,0.9};

void setDACVoltage(uint8_t ch,float v) {
  // TODO: depends on which DAC we will use. 12 or 16 bit resolution will be good.
}

float stepV(uint8_t s) { return VMIN+(VMAX-VMIN)*s/STEPS; }

float measureP1(uint8_t ch) {
  uint32_t ones=0;
  for(uint32_t i=0;i<SAMPLES;i++) {
    ones+=digitalRead(qPin[ch])==HIGH;
    delayMicroseconds(SAMPLE_US);
  }
  return (float)ones/SAMPLES;
}

float voltageForP(uint8_t ch,float p) {
  p=constrain(p,0.0f,1.0f);
  int best=0; float errBest=fabs(calibP[ch][0]-p);

  for(uint8_t s=1;s<=STEPS;s++) {
    float p0=calibP[ch][s-1], p1=calibP[ch][s];
    float e=fabs(p1-p);
    if(e<errBest) { errBest=e; best=s; }

    if((p-p0)*(p-p1)<=0 && fabs(p1-p0)>1e-6f)
      return stepV(s-1)+(p-p0)*(stepV(s)-stepV(s-1))/(p1-p0);
  }
  return stepV(best);
}

void calibrate() {
  Serial.println("CALIBRATION");

  for(uint8_t s=0;s<=STEPS;s++) {
    float v=stepV(s);
    for(uint8_t ch=0;ch<N;ch++) setDACVoltage(ch,v);
    delay(5);

    for(uint8_t ch=0;ch<N;ch++) {
      calibP[ch][s]=measureP1(ch);
      Serial.print(ch); Serial.print(',');
      Serial.print(v,3); Serial.print(',');
      Serial.println(calibP[ch][s],4);
    }
  }
  mode=VALIDATION;
}

void validate() {
  Serial.println("VALIDATION");

  for(uint8_t ch=0;ch<N;ch++)
    setDACVoltage(ch,voltageForP(ch,testP[ch]));

  delay(20);

  for(uint8_t ch=0;ch<N;ch++) {
    float p=measureP1(ch), e=p-testP[ch];
    Serial.print(ch); Serial.print(',');
    Serial.print(testP[ch],2); Serial.print(',');
    Serial.print(p,3); Serial.print(',');
    Serial.println(fabs(e)<=TOL ? "PASS":"FAIL");
  }
  mode=MONITOR;
}

void monitor() {
  Serial.print("Q=");
  for(uint8_t ch=0;ch<N;ch++) Serial.print(digitalRead(qPin[ch]));
  Serial.println();
  delay(100);
}

void setup() {
  Serial.begin(115200);
  for(uint8_t ch=0;ch<N;ch++) pinMode(qPin[ch],INPUT);
  delay(500);
}

void loop() {
  if(mode==CALIBRATION) calibrate();
  else if(mode==VALIDATION) validate();
  else monitor();
}