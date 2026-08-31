/*
Probana main firmware. It accepts and executes a probabilistic circuit.
Performs initial calibration, then waits for commands from the PC.

Commands:
PING
CLEAR <n>
H <i> <value>
J <from> <to> <value>
COMMIT
RUN <samples> <burn_in> <thin>

J and h are uploaded once. During RUN, coupling updates, calibration
correction and physical p-bit sampling are performed locally.

Use probana_backend in p-kit.
*/

#include <Arduino.h>
#include <math.h>

const uint8_t N=8, qPin[N]={2,3,4,5,6,7,8,9}, STEPS=50;
const float VMIN=0.0f, VMAX=3.3f;
const uint32_t CAL_SAMPLES=2000;
const uint16_t SAMPLE_US=20, SETTLE_US=100;

float calibP[N][STEPS+1], J[N][N], h[N];
int8_t m[N];
uint8_t activeN=N;
bool committed=false;

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

void initState() {
  for(uint8_t i=0;i<activeN;i++) setDACVoltage(i,voltageForP(i,0.5f));
  delay(2);
  for(uint8_t i=0;i<activeN;i++) m[i]=digitalRead(qPin[i]) ? 1:-1;
}

void updatePbit(uint8_t i) {
  float I=h[i];
  for(uint8_t j=0;j<activeN;j++) I+=m[j]*J[j][i];

  float p=1.0f/(1.0f+expf(-2.0f*I));
  setDACVoltage(i,voltageForP(i,p));
  delayMicroseconds(SETTLE_US);
  m[i]=digitalRead(qPin[i]) ? 1:-1;
}

void runCircuit(uint32_t samples,uint32_t burn,uint16_t thin) {
  for(uint32_t n=0;n<burn;n++) updatePbit(random(activeN));

  Serial.println("OK");
  for(uint32_t s=0;s<samples;s++) {
    for(uint16_t k=0;k<thin;k++) updatePbit(random(activeN));

    Serial.print("S ");
    for(uint8_t i=0;i<activeN;i++) Serial.print(m[i]>0 ? '1':'0');
    Serial.println();
  }
  Serial.println("DONE");
}

void handleCommand(char *line) {
  char *cmd=strtok(line," ");
  if(!cmd) return;

  if(!strcmp(cmd,"PING")) {
    Serial.print("OK PROBANA 1 "); Serial.print(N); Serial.println(" WORKING");
  }
  else if(!strcmp(cmd,"CLEAR")) {
    char *a=strtok(NULL," "); int n=a ? atoi(a):-1;
    if(n<1 || n>N) { Serial.println("ERR INVALID_N"); return; }
    activeN=n; memset(J,0,sizeof(J)); memset(h,0,sizeof(h));
    committed=false; Serial.println("OK");
  }
  else if(!strcmp(cmd,"H")) {
    char *a=strtok(NULL," "), *b=strtok(NULL," ");
    if(!a || !b) { Serial.println("ERR H"); return; }
    int i=atoi(a);
    if(i<0 || i>=activeN) { Serial.println("ERR INDEX"); return; }
    h[i]=atof(b); committed=false; Serial.println("OK");
  }
  else if(!strcmp(cmd,"J")) {
    char *a=strtok(NULL," "), *b=strtok(NULL," "), *c=strtok(NULL," ");
    if(!a || !b || !c) { Serial.println("ERR J"); return; }
    int from=atoi(a), to=atoi(b);
    if(from<0 || from>=activeN || to<0 || to>=activeN) {
      Serial.println("ERR INDEX"); return;
    }
    J[from][to]=atof(c); committed=false; Serial.println("OK");
  }
  else if(!strcmp(cmd,"COMMIT")) {
    initState(); committed=true; Serial.println("OK");
  }
  else if(!strcmp(cmd,"RUN")) {
    char *a=strtok(NULL," "), *b=strtok(NULL," "), *c=strtok(NULL," ");
    if(!a || !b || !c || !committed) { Serial.println("ERR RUN"); return; }

    uint32_t samples=strtoul(a,NULL,10), burn=strtoul(b,NULL,10);
    uint16_t thin=atoi(c); if(!thin) thin=1;
    runCircuit(samples,burn,thin);
  }
  else Serial.println("ERR UNKNOWN_COMMAND");
}

void setup() {
  Serial.begin(115200);
  for(uint8_t i=0;i<N;i++) pinMode(qPin[i],INPUT);
  delay(500);

  Serial.println("CALIBRATION");
  calibrate();

  randomSeed(micros() ^ analogRead(A0));
  Serial.print("READY PROBANA 1 "); Serial.print(N); Serial.println(" WORKING");
}

void loop() {
  static char buf[96];
  static uint8_t pos=0;

  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\n') {
      buf[pos]=0; handleCommand(buf); pos=0;
    } else if(c!='\r' && pos<sizeof(buf)-1) buf[pos++]=c;
  }
}