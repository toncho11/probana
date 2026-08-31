/*
Calibrates the physical p-bits, runs a hard-coded 8-p-bit Ising chain,
then checks measured probabilities and correlations against theory.

Expected:
  P(mi=1) ~= 0.5
  <mi*m(i+1)> ~= tanh(1) ~= 0.7616
*/

#include <Arduino.h>
#include <math.h>

const uint8_t N=8, qPin[N]={2,3,4,5,6,7,8,9}, STEPS=50;
const float VMIN=0.0f, VMAX=3.3f, P_TOL=0.05f, C_TOL=0.08f;
const uint32_t CAL_SAMPLES=2000, TEST_SAMPLES=20000, BURN_IN=10000;
const uint16_t SAMPLE_US=20, SETTLE_US=100;
const uint8_t THIN=N;

float calibP[N][STEPS+1], h[N]={0};
int8_t m[N];

float J[N][N]={
 {0,1,0,0,0,0,0,0},{1,0,1,0,0,0,0,0},
 {0,1,0,1,0,0,0,0},{0,0,1,0,1,0,0,0},
 {0,0,0,1,0,1,0,0},{0,0,0,0,1,0,1,0},
 {0,0,0,0,0,1,0,1},{0,0,0,0,0,0,1,0}
};

void setDACVoltage(uint8_t ch,float v) {
  // TODO: selected 12/16-bit DAC
}

float stepV(uint8_t s) { return VMIN+(VMAX-VMIN)*s/STEPS; }

float measureP1(uint8_t ch) {
  uint32_t ones=0;
  for(uint32_t n=0;n<CAL_SAMPLES;n++) {
    ones+=digitalRead(qPin[ch])==HIGH;
    delayMicroseconds(SAMPLE_US);
  }
  return (float)ones/CAL_SAMPLES;
}

void calibrate() {
  Serial.println("CALIBRATION");
  for(uint8_t s=0;s<=STEPS;s++) {
    float v=stepV(s);
    for(uint8_t i=0;i<N;i++) setDACVoltage(i,v);
    delay(5);
    for(uint8_t i=0;i<N;i++) calibP[i][s]=measureP1(i);
  }
}

float voltageForP(uint8_t ch,float p) {
  p=constrain(p,0.0f,1.0f);
  int best=0; float bestE=fabs(calibP[ch][0]-p);

  for(uint8_t s=1;s<=STEPS;s++) {
    float p0=calibP[ch][s-1], p1=calibP[ch][s], e=fabs(p1-p);
    if(e<bestE) { bestE=e; best=s; }
    if((p-p0)*(p-p1)<=0 && fabs(p1-p0)>1e-6f)
      return stepV(s-1)+(p-p0)*(stepV(s)-stepV(s-1))/(p1-p0);
  }
  return stepV(best);
}

void updatePbit(uint8_t i) {
  float I=h[i];
  for(uint8_t j=0;j<N;j++) I+=m[j]*J[j][i];

  float p=1.0f/(1.0f+expf(-2.0f*I));
  setDACVoltage(i,voltageForP(i,p));
  delayMicroseconds(SETTLE_US);
  m[i]=digitalRead(qPin[i]) ? 1:-1;
}

void runTest() {
  uint32_t ones[N]={0};
  int32_t corr[N-1]={0};

  Serial.println("TEST");

  for(uint32_t n=0;n<BURN_IN;n++) updatePbit(random(N));

  for(uint32_t s=0;s<TEST_SAMPLES;s++) {
    for(uint8_t k=0;k<THIN;k++) updatePbit(random(N));

    for(uint8_t i=0;i<N;i++) if(m[i]>0) ones[i]++;
    for(uint8_t i=0;i<N-1;i++) corr[i]+=m[i]*m[i+1];
  }

  bool pass=true;
  Serial.println("PBIT  P(1)   EXPECTED");
  for(uint8_t i=0;i<N;i++) {
    float p=(float)ones[i]/TEST_SAMPLES;
    Serial.print(i); Serial.print("     ");
    Serial.print(p,3); Serial.println("   0.500");
    if(fabs(p-0.5f)>P_TOL) pass=false;
  }

  const float expected=tanhf(1.0f);
  Serial.println("\nPAIR  CORR   EXPECTED");
  for(uint8_t i=0;i<N-1;i++) {
    float c=(float)corr[i]/TEST_SAMPLES;
    Serial.print(i); Serial.print("-"); Serial.print(i+1);
    Serial.print("   "); Serial.print(c,3);
    Serial.print("   "); Serial.println(expected,3);
    if(fabs(c-expected)>C_TOL) pass=false;
  }

  Serial.println(pass ? "\nRESULT: PASS" : "\nRESULT: FAIL");
}

void setup() {
  Serial.begin(115200);
  for(uint8_t i=0;i<N;i++) pinMode(qPin[i],INPUT);
  delay(500);

  calibrate();
  for(uint8_t i=0;i<N;i++) m[i]=digitalRead(qPin[i]) ? 1:-1;
  randomSeed(analogRead(A0));

  runTest();
}

void loop() {}