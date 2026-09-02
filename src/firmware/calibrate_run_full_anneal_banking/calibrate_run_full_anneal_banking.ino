/* 
Probana v2: 128 virtual p-bits on 8 physical p-bits, banking and full annealing.
The idea of banking comes from old retro computers, but optimisation/compilation
must be done before that.
Requires probana_compile_backend.py.  
Required commands: PING, VCLEAR, VBANK, H, J, VCOMMIT, ANNEAL, RUN.
*/

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t N=8, qPin[N]={2,3,4,5,6,7,8,9}, CAL_STEPS=50;
const uint16_t MAX_VIRTUAL=128, MAX_BANKS=128, MAX_EDGES=4096, MAX_ANNEAL=8192, INVALID=0xffff;
const uint8_t ANNEAL_CHUNK=8;
const float VMIN=0.0f, VMAX=3.3f;
const uint32_t CAL_SAMPLES=2000;
const uint16_t SAMPLE_US=20, SETTLE_US=100;

enum UpdateMode { SNAPSHOT, SEQUENTIAL };
enum AnnealMode { A_CONSTANT, A_LINEAR, A_TABLE };

float calibP[N][CAL_STEPS+1], h[MAX_VIRTUAL];
int8_t stateA[MAX_VIRTUAL], stateB[MAX_VIRTUAL], *currentState=stateA, *nextState=stateB;
uint16_t bankPtr[MAX_BANKS+1], nodes[MAX_VIRTUAL], nodePos[MAX_VIRTUAL], nodeBank[MAX_VIRTUAL];
uint16_t edgePtr[MAX_VIRTUAL+1], edgeSrc[MAX_EDGES];
float edgeWeight[MAX_EDGES], annealTable[MAX_ANNEAL];

uint16_t nVirtual=0, nBanks=0, banksReceived=0, nodesReceived=0;
uint16_t expectedEdges=0, edgesReceived=0, edgePosition=0;
bool configured=false, committed=false, tableLoading=false;
UpdateMode updateMode=SNAPSHOT;
AnnealMode annealMode=A_CONSTANT;
float annealStart=1.0f, annealEnd=1.0f;
uint32_t annealSteps=1;
uint16_t tableCount=0, tableLoaded=0;

void setDACVoltage(uint8_t channel,float voltage) {
  (void)channel; (void)voltage;
  // TODO: selected 8-channel 12/16-bit DAC driver.
}

float stepV(uint8_t step) { return VMIN+(VMAX-VMIN)*step/CAL_STEPS; }

float measureP1(uint8_t channel) {
  uint32_t ones=0;
  for(uint32_t i=0;i<CAL_SAMPLES;i++) {
    ones+=digitalRead(qPin[channel])==HIGH;
    delayMicroseconds(SAMPLE_US);
  }
  return (float)ones/CAL_SAMPLES;
}

void calibrate() {
  for(uint8_t step=0;step<=CAL_STEPS;step++) {
    float voltage=stepV(step);
    for(uint8_t lane=0;lane<N;lane++) setDACVoltage(lane,voltage);
    delay(5);
    for(uint8_t lane=0;lane<N;lane++) calibP[lane][step]=measureP1(lane);
  }
}

float voltageForP(uint8_t lane,float p) {
  p=constrain(p,0.0f,1.0f);
  uint8_t best=0;
  float bestError=fabsf(calibP[lane][0]-p);
  for(uint8_t step=1;step<=CAL_STEPS;step++) {
    float p0=calibP[lane][step-1], p1=calibP[lane][step], error=fabsf(p1-p);
    if(error<bestError) { bestError=error; best=step; }
    if((p-p0)*(p-p1)<=0.0f && fabsf(p1-p0)>1.0e-6f)
      return stepV(step-1)+(p-p0)*(stepV(step)-stepV(step-1))/(p1-p0);
  }
  return stepV(best);
}

float probability(float field) {
  if(field>=10.0f) return 1.0f;
  if(field<=-10.0f) return 0.0f;
  return 1.0f/(1.0f+expf(-2.0f*field));
}

bool parseUInt(char *token,uint32_t &value) {
  if(!token || !*token || *token=='-') return false;
  char *end;
  value=strtoul(token,&end,10);
  return end!=token && *end==0;
}

bool parseFloat(char *token,float &value) {
  if(!token || !*token) return false;
  char *end;
  value=strtof(token,&end);
  return end!=token && *end==0 && isfinite(value);
}

void resetPlan() {
  memset(h,0,sizeof(h)); memset(stateA,0,sizeof(stateA)); memset(stateB,0,sizeof(stateB));
  memset(bankPtr,0,sizeof(bankPtr)); memset(nodes,0,sizeof(nodes)); memset(edgePtr,0,sizeof(edgePtr));
  for(uint16_t i=0;i<MAX_VIRTUAL;i++) nodePos[i]=nodeBank[i]=INVALID;
  currentState=stateA; nextState=stateB;
  nVirtual=nBanks=banksReceived=nodesReceived=expectedEdges=edgesReceived=edgePosition=0;
  configured=committed=false;
}

bool beginPlan(uint16_t virtualCount,uint16_t bankCount,uint16_t edgeCount,UpdateMode mode) {
  if(!virtualCount || virtualCount>MAX_VIRTUAL || !bankCount || bankCount>MAX_BANKS ||
     bankCount>virtualCount || edgeCount>MAX_EDGES) return false;
  resetPlan();
  nVirtual=virtualCount; nBanks=bankCount; expectedEdges=edgeCount; updateMode=mode;
  configured=true; bankPtr[0]=edgePtr[0]=0;
  return true;
}

bool addBank(uint16_t bank,uint16_t count,char **tokens) {
  if(!configured || bank!=banksReceived || bank>=nBanks || !count || count>N ||
     nodesReceived+count>nVirtual) return false;
  uint16_t parsed[N];
  for(uint16_t lane=0;lane<count;lane++) {
    uint32_t node;
    if(!parseUInt(tokens[lane],node) || node>=nVirtual || nodePos[node]!=INVALID) return false;
    for(uint16_t previous=0;previous<lane;previous++)
      if(parsed[previous]==node) return false;
    parsed[lane]=(uint16_t)node;
  }
  bankPtr[bank]=nodesReceived;
  for(uint16_t lane=0;lane<count;lane++) {
    uint16_t node=parsed[lane];
    nodes[nodesReceived]=node;
    nodePos[node]=nodesReceived;
    nodeBank[node]=bank;
    nodesReceived++;
  }
  bankPtr[bank+1]=nodesReceived;
  banksReceived++;
  committed=false;
  return true;
}

bool addEdge(uint16_t source,uint16_t destination,float weight) {
  if(!configured || banksReceived!=nBanks || source>=nVirtual || destination>=nVirtual ||
     edgesReceived>=expectedEdges || edgesReceived>=MAX_EDGES) return false;
  uint16_t position=nodePos[destination];
  if(position==INVALID || position<edgePosition) return false;
  while(edgePosition<position) edgePtr[++edgePosition]=edgesReceived;
  edgeSrc[edgesReceived]=source;
  edgeWeight[edgesReceived]=weight;
  edgesReceived++;
  committed=false;
  return true;
}

bool finishPlan() {
  if(!configured || banksReceived!=nBanks || nodesReceived!=nVirtual ||
     edgesReceived!=expectedEdges) return false;
  for(uint16_t node=0;node<nVirtual;node++)
    if(nodePos[node]==INVALID) return false;
  while(edgePosition<nVirtual) edgePtr[++edgePosition]=edgesReceived;

  if(updateMode==SEQUENTIAL) {
    for(uint16_t position=0;position<nVirtual;position++) {
      uint16_t destination=nodes[position], bank=nodeBank[destination];
      for(uint16_t edge=edgePtr[position];edge<edgePtr[position+1];edge++)
        if(edgeSrc[edge]!=destination && nodeBank[edgeSrc[edge]]==bank)
          return false;
    }
  }
  return true;
}

void initState() {
  for(uint8_t lane=0;lane<N;lane++)
    setDACVoltage(lane,voltageForP(lane,0.5f));
  delay(2);

  for(uint16_t bank=0;bank<nBanks;bank++) {
    uint16_t start=bankPtr[bank], count=bankPtr[bank+1]-start;
    for(uint16_t lane=0;lane<count;lane++)
      currentState[nodes[start+lane]]=digitalRead(qPin[lane])==HIGH ? 1:-1;
    delayMicroseconds(SAMPLE_US);
  }
  memcpy(nextState,currentState,nVirtual*sizeof(int8_t));
}

float fieldAt(uint16_t position,const int8_t *state,float scale) {
  uint16_t node=nodes[position];
  float field=h[node];
  for(uint16_t edge=edgePtr[position];edge<edgePtr[position+1];edge++)
    field+=edgeWeight[edge]*state[edgeSrc[edge]];
  return field*scale;
}

void applyBank(uint16_t bank,const int8_t *readState,float scale) {
  uint16_t start=bankPtr[bank], count=bankPtr[bank+1]-start;
  for(uint16_t lane=0;lane<count;lane++) {
    float p=probability(fieldAt(start+lane,readState,scale));
    setDACVoltage((uint8_t)lane,voltageForP((uint8_t)lane,p));
  }
}

void readBank(uint16_t bank,int8_t *writeState) {
  uint16_t start=bankPtr[bank], count=bankPtr[bank+1]-start;
  for(uint16_t lane=0;lane<count;lane++)
    writeState[nodes[start+lane]]=digitalRead(qPin[lane])==HIGH ? 1:-1;
}

void sweep(float scale) {
  if(updateMode==SNAPSHOT) {
    for(uint16_t bank=0;bank<nBanks;bank++) {
      applyBank(bank,currentState,scale);
      delayMicroseconds(SETTLE_US);
      readBank(bank,nextState);
    }
    int8_t *old=currentState;
    currentState=nextState;
    nextState=old;
  } else {
    for(uint16_t bank=0;bank<nBanks;bank++) {
      applyBank(bank,currentState,scale);
      delayMicroseconds(SETTLE_US);
      readBank(bank,currentState);
    }
  }
}

float annealScale(uint32_t logicalSweep,uint32_t sample) {
  if(annealMode==A_CONSTANT) return annealStart;
  if(annealMode==A_TABLE) return annealTable[sample];
  if(annealSteps<=1) return annealStart;
  float x=(float)logicalSweep/(annealSteps-1);
  if(x>1.0f) x=1.0f;
  return annealStart+x*(annealEnd-annealStart);
}

void printState() {
  Serial.print("S ");
  for(uint16_t node=0;node<nVirtual;node++)
    Serial.print(currentState[node]>0 ? '1':'0');
  Serial.println();
}

void runCircuit(uint32_t samples,uint32_t burn,uint16_t thin) {
  Serial.println("OK");
  uint32_t logicalSweep=0;

  for(uint32_t i=0;i<burn;i++,logicalSweep++)
    sweep(annealScale(logicalSweep,0));

  for(uint32_t sample=0;sample<samples;sample++) {
    for(uint16_t i=0;i<thin;i++,logicalSweep++)
      sweep(annealScale(logicalSweep,sample));
    printState();
  }
  Serial.println("DONE");
}

void handleVClear() {
  uint32_t nv,np,nb,ne;
  char *a=strtok(NULL," "), *b=strtok(NULL," "), *c=strtok(NULL," ");
  char *d=strtok(NULL," "), *mode=strtok(NULL," ");

  if(!parseUInt(a,nv) || !parseUInt(b,np) || !parseUInt(c,nb) ||
     !parseUInt(d,ne) || np!=N || nv>MAX_VIRTUAL ||
     nb>MAX_BANKS || ne>MAX_EDGES || !mode) {
    Serial.println("ERR VCLEAR");
    return;
  }

  UpdateMode update;
  if(!strcmp(mode,"SNAPSHOT")) update=SNAPSHOT;
  else if(!strcmp(mode,"SEQUENTIAL")) update=SEQUENTIAL;
  else {
    Serial.println("ERR UPDATE_MODE");
    return;
  }

  if(!beginPlan((uint16_t)nv,(uint16_t)nb,(uint16_t)ne,update)) {
    Serial.println("ERR CAPACITY");
    return;
  }
  Serial.println("OK");
}

void handleVBank() {
  uint32_t bank,count;
  if(!parseUInt(strtok(NULL," "),bank) ||
     !parseUInt(strtok(NULL," "),count) ||
     bank>=nBanks || !count || count>N) {
    Serial.println("ERR VBANK");
    return;
  }

  char *tokens[N];
  for(uint32_t lane=0;lane<count;lane++)
    if(!(tokens[lane]=strtok(NULL," "))) {
      Serial.println("ERR VBANK");
      return;
    }

  if(!addBank((uint16_t)bank,(uint16_t)count,tokens)) {
    Serial.println("ERR VBANK");
    return;
  }
  Serial.println("OK");
}

void handleH() {
  uint32_t node;
  float value;
  if(!configured || !parseUInt(strtok(NULL," "),node) ||
     node>=nVirtual || !parseFloat(strtok(NULL," "),value)) {
    Serial.println("ERR H");
    return;
  }
  h[node]=value;
  committed=false;
  Serial.println("OK");
}

void handleJ() {
  uint32_t source,destination;
  float weight;
  if(!parseUInt(strtok(NULL," "),source) ||
     !parseUInt(strtok(NULL," "),destination) ||
     source>=nVirtual || destination>=nVirtual ||
     !parseFloat(strtok(NULL," "),weight) ||
     !addEdge((uint16_t)source,(uint16_t)destination,weight)) {
    Serial.println("ERR J_ORDER_OR_COUNT");
    return;
  }
  Serial.println("OK");
}

void handleAnneal() {
  char *type=strtok(NULL," ");
  if(!type) {
    Serial.println("ERR ANNEAL");
    return;
  }

  if(!strcmp(type,"CONSTANT")) {
    float scale;
    if(!parseFloat(strtok(NULL," "),scale)) {
      Serial.println("ERR ANNEAL");
      return;
    }
    annealMode=A_CONSTANT;
    annealStart=annealEnd=scale;
    annealSteps=1;
    tableLoading=false;
    Serial.println("OK");
    return;
  }

  if(!strcmp(type,"LINEAR")) {
    float start,end;
    uint32_t steps;
    if(!parseFloat(strtok(NULL," "),start) ||
       !parseFloat(strtok(NULL," "),end) ||
       !parseUInt(strtok(NULL," "),steps) || !steps) {
      Serial.println("ERR ANNEAL");
      return;
    }
    annealMode=A_LINEAR;
    annealStart=start;
    annealEnd=end;
    annealSteps=steps;
    tableLoading=false;
    Serial.println("OK");
    return;
  }

  if(!strcmp(type,"TABLE")) {
    uint32_t count;
    if(!parseUInt(strtok(NULL," "),count) ||
       !count || count>MAX_ANNEAL) {
      Serial.println("ERR ANNEAL_TABLE_SIZE");
      return;
    }
    tableCount=count;
    tableLoaded=0;
    tableLoading=true;
    Serial.println("OK");
    return;
  }

  if(!strcmp(type,"VALUES")) {
    uint32_t offset,count;
    if(!tableLoading ||
       !parseUInt(strtok(NULL," "),offset) ||
       !parseUInt(strtok(NULL," "),count) ||
       !count || count>ANNEAL_CHUNK ||
       offset!=tableLoaded || offset+count>tableCount) {
      Serial.println("ERR ANNEAL_VALUES");
      return;
    }

    float values[ANNEAL_CHUNK];
    for(uint32_t i=0;i<count;i++)
      if(!parseFloat(strtok(NULL," "),values[i])) {
        Serial.println("ERR ANNEAL_VALUES");
        return;
      }

    for(uint32_t i=0;i<count;i++)
      annealTable[offset+i]=values[i];

    tableLoaded+=count;
    Serial.println("OK");
    return;
  }

  if(!strcmp(type,"COMMIT")) {
    if(!tableLoading || tableLoaded!=tableCount) {
      Serial.println("ERR ANNEAL_COMMIT");
      return;
    }
    annealMode=A_TABLE;
    tableLoading=false;
    Serial.println("OK");
    return;
  }

  Serial.println("ERR ANNEAL");
}

void handleRun() {
  uint32_t samples,burn,thin;
  if(!committed || tableLoading ||
     !parseUInt(strtok(NULL," "),samples) || !samples ||
     !parseUInt(strtok(NULL," "),burn) ||
     !parseUInt(strtok(NULL," "),thin) ||
     !thin || thin>0xffff ||
     (annealMode==A_TABLE && samples>tableCount)) {
    Serial.println("ERR RUN");
    return;
  }
  runCircuit(samples,burn,(uint16_t)thin);
}

void handleCommand(char *line) {
  char *command=strtok(line," ");
  if(!command) return;

  if(!strcmp(command,"PING")) {
    Serial.print("OK PROBANA 2 ");
    Serial.print(N);
    Serial.println(" WORKING");
  }
  else if(!strcmp(command,"VCLEAR")) handleVClear();
  else if(!strcmp(command,"VBANK")) handleVBank();
  else if(!strcmp(command,"H")) handleH();
  else if(!strcmp(command,"J")) handleJ();
  else if(!strcmp(command,"VCOMMIT")) {
    if(!finishPlan())
      Serial.println(updateMode==SEQUENTIAL ?
                     "ERR BANK_CONFLICT":"ERR VCOMMIT");
    else {
      initState();
      committed=true;
      Serial.println("OK");
    }
  }
  else if(!strcmp(command,"ANNEAL")) handleAnneal();
  else if(!strcmp(command,"RUN")) handleRun();
  else Serial.println("ERR UNKNOWN_COMMAND");
}

void setup() {
  Serial.begin(115200);
  for(uint8_t lane=0;lane<N;lane++)
    pinMode(qPin[lane],INPUT);

  delay(500);
  resetPlan();
  Serial.println("CALIBRATION");
  calibrate();

  Serial.print("READY PROBANA 2 ");
  Serial.print(N);
  Serial.println(" WORKING");
}

void loop() {
  static char buffer[192];
  static uint16_t position=0;
  static bool overflow=false;

  while(Serial.available()) {
    char c=(char)Serial.read();

    if(c=='\n') {
      if(overflow) Serial.println("ERR LINE_TOO_LONG");
      else {
        buffer[position]=0;
        handleCommand(buffer);
      }
      position=0;
      overflow=false;
    }
    else if(c!='\r') {
      if(position<sizeof(buffer)-1)
        buffer[position++]=c;
      else
        overflow=true;
    }
  }
}