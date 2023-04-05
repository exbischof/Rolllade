#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <EEPROM.h>

// Hier ändern
const char* ssid = "SSIS";
const char* password = "PASSWORD";
const char* mqtt_server = "192.168.10.24";
const char* address = "test";
const char* all = "all";

#define T1 2
#define TEILER 25
#define REVERSE_TIME 1.0
#define END_TIME 0.5
#define MAX_RUN_TIME 60.0
#define SENSOR_TIME 0.03
#define TEST_WAIT 2

#define F1 (1000.0/(float)T1)
#define F2 (F1/(float)TEILER)

#define BLOCKING_TICKS ((unsigned int)(F2*REVERSE_TIME))
#define MAX_RUN ((unsigned int)(F2*MAX_RUN_TIME))
#define END_WAIT ((unsigned int)(F2*END_TIME))
#define MAX_LEVEL ((unsigned int)(F1*SENSOR_TIME))
#define TEST_WAIT_TICS ((unsigned int)(F2*TEST_WAIT))

#define REEDCONTACT D2
#define LED_RED D3
#define LED_GREEN  D2
#define LED_ON  LOW
#define LED_OFF  HIGH
#define SUP  D6
#define SDOWN  D7
#define MUP  D4
#define MDOWN D5
#define RELAIS_ONOFF D1
#define RELAIS_DIR D0
#define RELAIS_OFF LOW
#define RELAIS_ON HIGH
#define DIR_UP RELAIS_ON
#define DIR_DOWN RELAIS_OFF
#define SENSOR_ACTIVE LOW
#define REEDCONTACT_CLOSED LOW
#define REEDCONTACT_OPEN (!REEDCONTACT_CLOSED)

#define EEPROM_SIZE 16
#define CALI_ADD 0
#define MOTOR_ADD 4
#define SWITCH_ADD 6
#define REED_ADD 8

typedef enum {
  eControlStop,
  eControlGoUp,
  eControlGoDown,
  eControlGotoUp,
  eControlGotoDown,
  eControlRefUp,
  eControlRefDown,
  eCaliUp,
  eCaliDown,
} typeControlMode;

typedef enum {
  eRelaisStop,
  eRelaisUp,
  eRelaisDown
} typeRelaisStat;

typedef enum {
  eRelaisModeStop,
  eRelaisModeWaitStop,
  eRelaisModeUp,
  eRelaisModeWaitUp,
  eRelaisModeDown,
  eRelaisModeWaitDown
} typeRelaisMode;

typedef enum {
  eOff,
  eBlink,
  eSupPort,
  eSup,
  eSdownPort,
  eSdown,
  eMdownPort,
  eMdown,
  eMupPort,
  eMup,
  eRelaisOnOff,
  eRelaisDir,
  eInt
} typeLedStat;

typedef enum {
  eMotorStop,
  eMotorUp,
  eMotorDown
} typeMotorMode;

WiFiClient espClient;
PubSubClient client(espClient);
Ticker timer;
long ticks;
long endTime;
long setpoint;
unsigned int caliValue;

int counter=0;
int maxRunCounter=MAX_RUN;

boolean bIsUp;
boolean bIsDown;

int endCounter=END_WAIT;
int actPosition;
int tempCali;

boolean lastUpSwitch=!SENSOR_ACTIVE;
boolean lastDownSwitch=!SENSOR_ACTIVE;
boolean bIntStat;

typeControlMode tcontrol;

typeMotorMode tMotorStat=eMotorStop;
typeMotorMode tMotorMode=eMotorStop;
int blockingCounter;

typeRelaisStat tRelaisStat;
typeRelaisMode tRelaisMode;

int levelSup=MAX_LEVEL;
int levelSdown=MAX_LEVEL;
int levelMup=MAX_LEVEL;
int levelMdown=MAX_LEVEL;
boolean bSupStat=!SENSOR_ACTIVE;
boolean bSdownStat=!SENSOR_ACTIVE;
boolean bMupStat=!SENSOR_ACTIVE;
boolean bMdownStat=!SENSOR_ACTIVE;
unsigned short testResult;
unsigned int testCount;

boolean invertMotor;
boolean invertSwitch;

unsigned int blinkCounter;
typeLedStat tGreenStat;
typeLedStat tRedStat;

// ---- Hauptfunktionen ----

void setup() {
  testResult=0;
  testCount=TEST_WAIT_TICS;
  
  EEPROM.begin(EEPROM_SIZE);
  
  Serial.begin(115200);

  delay(1000);
  setupLed();
  setupSensors();
  setupRelais();
  setupMotor();
  setupEndPosition();
  setupControl();
  setupSwitch();
  
  timer.attach_ms(T1,processRollade);
  
  setupWifi();
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.print("invertMotor: ");
  Serial.println(invertMotor);
  Serial.print("invertSwitch: ");
  Serial.println(invertSwitch);
}

void processRollade() {
  bIntStat=true;
  digitalWrite(LED_GREEN,getLedStat(tGreenStat));
  digitalWrite(LED_RED,getLedStat(tRedStat));
  processSensors();
  ++counter;
  if(counter>=TEILER) {
    counter=0;
    processSwitch();
    processRelais();
    processMotor();
    processEndPosition();
    processControl();
    processBlink();
    processTest();
  
  }  
  bIntStat=false;
  digitalWrite(LED_GREEN,getLedStat(tGreenStat));
  digitalWrite(LED_RED,getLedStat(tRedStat));
}

void invMot() {
  MotorStop();
  invertMotor=EEPROM.read(MOTOR_ADD);
  EEPROM.write(MOTOR_ADD,!invertMotor);
  EEPROM.commit();
  reset();
}

void invSwitch() {
  invertSwitch=EEPROM.read(SWITCH_ADD);
  EEPROM.write(SWITCH_ADD,!invertSwitch);
  EEPROM.commit();
}

void reset() {
  wdt_enable(WDTO_2S);
  for(;;);
}

void clreeprom() {
  for(int i=0;i<EEPROM_SIZE;++i) {
    EEPROM.write(i,255);
  }
  EEPROM.commit();
  reset();
}

void ping() {
      client.publish(address, "Hello");
}

// ---- Funksteuerung ----

void setupWifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
   delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  char buffer[20];
  unsigned int temp;
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();
  if((strcmp(topic,address)==0||strcmp(topic,all)==0)&&length<20) {
    int i;
    for(i=0;i<length;++i) {
      buffer[i]=payload[i];
    }
    buffer[i]=0;
    if(strcmp(buffer,"calibrate")==0) {
      calibrate();
    } else if(strcmp(buffer,"up")==0) {
      goUp();
    } else if(strncmp(buffer,"up",2)==0) {
      temp=atoi(buffer+2);
     gotoRefUp(temp);
    } else if(strcmp(buffer,"down")==0) {
      goDown();
    } else if(strncmp(buffer,"down",4)==0) {
      temp=atoi(buffer+4);
      gotoRefDown(temp);
    } else if(strcmp(buffer,"stop")==0) {
     MotorStop();
    } else if(strcmp(buffer,"invertmotor")==0) {
     invMot();
    } else if(strcmp(buffer,"invertswitch")==0) {
     invSwitch();
    } else if(strncmp(buffer,"reed",4)==0) {
       temp=atoi(buffer+4);
       reed(temp);
    } else if(strcmp(buffer,"clreeprom")==0) {
     clreeprom();
    } else if(strcmp(buffer,"reset")==0) {
      reset();
    } else if(strcmp(buffer,"ping")==0) {
      ping();
    } else if(strcmp(buffer,"test")==0) {
      test();
    } else if(strncmp(buffer,"goto",4)==0) {
       temp=atoi(buffer+4);
        gotoPosition(temp);
    } else if(strncmp(buffer,"red",3)==0) {
        tRedStat=(typeLedStat)atoi(buffer+3);
       digitalWrite(LED_RED,getLedStat(tRedStat));
    } else if(strncmp(buffer,"green",5)==0) {
        tGreenStat=(typeLedStat)atoi(buffer+5);
      digitalWrite(LED_GREEN,getLedStat(tGreenStat));
    }  else {
      Serial.println("unkown command");
    }
  }
}

void reconnect() {
  long counter;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(address);
      client.subscribe(all);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ---- Schaltersteuerung -----

void setupSwitch() {
  lastUpSwitch=!SENSOR_ACTIVE;
  lastDownSwitch=!SENSOR_ACTIVE;
}

void processSwitch() {
  boolean upSwitch=getSensorSup();
  boolean downSwitch=getSensorSdown();
  if((upSwitch==!SENSOR_ACTIVE&&lastUpSwitch==SENSOR_ACTIVE)||(downSwitch==!SENSOR_ACTIVE&&lastDownSwitch==SENSOR_ACTIVE)) {
    MotorStop();
  } else if(upSwitch==SENSOR_ACTIVE&&!lastUpSwitch==SENSOR_ACTIVE) {
    goUp();
  } else if(downSwitch==SENSOR_ACTIVE&&!lastDownSwitch==SENSOR_ACTIVE) {
    goDown();
  }
  lastUpSwitch=upSwitch;
  lastDownSwitch=downSwitch;
}

// ---- Steuerung ----

void setupControl() {
  tcontrol=eControlStop;
  setMotorStat(eMotorStop);
  actPosition=-1;
  
  EEPROM.get(CALI_ADD,caliValue);
  if(caliValue==0xffffffff) {
    caliValue=0;
  }
  Serial.println();
  Serial.print("Kalibriewert: ");
  Serial.println(caliValue);
}

void calibrate() {
  tcontrol=eCaliDown;
  setMotorStat(eMotorDown);
}

void MotorStop() {
  tcontrol=eControlStop;
  setMotorStat(eMotorStop);
}

void goUp() {
  tcontrol=eControlGoUp;
  setMotorStat(eMotorUp);
}

void gotoRefUp(unsigned int set) {
  if(set<=0) {
    tcontrol=eControlGoUp;
    setMotorStat(eMotorUp);
  } else if(getReedcontact()==REEDCONTACT_CLOSED) {
    if(set>=100) {
      tcontrol=eControlGoDown;
      setMotorStat(eMotorDown);
    } else {
      setpoint=(long)(((float)caliValue)*((float)set)/100.0); 
      if(isUp()) {
        tcontrol=eControlGotoDown;
        setMotorStat(eMotorDown);
       
      } else {
         tcontrol=eControlRefUp;
         setMotorStat(eMotorUp);
      }
    } 
  }
}

void goDown() {
  if(getReedcontact()==REEDCONTACT_CLOSED){
    tcontrol=eControlGoDown;
    setMotorStat(eMotorDown);
  }
}

void gotoRefDown(unsigned int set) {
  if(set<=0) {
    tcontrol=eControlGoUp;
    setMotorStat(eMotorUp);
  } else if(set>=100) {
      if(getReedcontact()==REEDCONTACT_CLOSED){
        tcontrol=eControlGoDown;
        setMotorStat(eMotorDown);
      }
  } else {
    setpoint=(long)(((float)caliValue)*((float)set)/100.0); 
    if(isDown()) {
      tcontrol=eControlGotoUp;
      setMotorStat(eMotorUp);
    } else if(getReedcontact()==REEDCONTACT_CLOSED){
      tcontrol=eControlRefDown;
      setMotorStat(eMotorDown);
    }
  }
}

void gotoPosition(unsigned int set) {
  if(set<=0) {
    tcontrol=eControlGoUp;
    setMotorStat(eMotorUp);
  } else if(set>=100 && getReedcontact()==REEDCONTACT_CLOSED) {
    tcontrol=eControlGoDown;
    setMotorStat(eMotorDown);   
  } else if(actPosition>=0) {
    setpoint=(long)(((float)caliValue)*((float)set)/100.0); 
    if(actPosition<setpoint) {
      if(getReedcontact()==REEDCONTACT_CLOSED) {
        tcontrol=eControlGotoDown;
        setMotorStat(eMotorDown);
      }
    } else if(actPosition>setpoint) {
      tcontrol=eControlGotoUp;
      setMotorStat(eMotorUp);
    } else {
      tcontrol=eControlStop;
      setMotorStat(eMotorStop);
    }
  } else {
    gotoRefUp(set);
  }
}

void processControl() {
  typeMotorMode stat=getMotorStat();
  if(stat==eMotorStop) {
    maxRunCounter=MAX_RUN;
  } else {
    if(maxRunCounter>0) {
         --maxRunCounter;
    } else {
        tcontrol=eControlStop;
        setMotorStat(eMotorStop);
      }
   }

   if(caliValue>0) {
     if(isUp()) {
      actPosition=0;
     } else if(isDown()) {
       actPosition=caliValue;
     }
     
    if(actPosition>=0) {
      if(stat==eMotorUp) {
        if(actPosition>0) {
          --actPosition;
        }
      } else if(stat==eMotorDown) {
        if(actPosition<caliValue) {
          ++actPosition;
        }
      }
    }
  }
  switch(tcontrol) {
    case eControlStop:
    case eControlGoUp:
    case eControlGoDown:
    default:
    break;
    case eCaliDown:
    if(isDown()) {
      tcontrol=eCaliUp;
      setMotorStat(eMotorUp);
      tempCali=0;
    }
    break;
    case eCaliUp:
    if(isUp()) {
      tcontrol=eControlStop;
      setMotorStat(eMotorStop);
      Serial.println();
      Serial.print("Ticks rauf: ");
      caliValue=tempCali;
      Serial.println(caliValue);
      EEPROM.put(CALI_ADD,caliValue);
      EEPROM.commit();
   } else {
    ++tempCali;
   }
    break;
    case eControlRefUp:
    if(isUp()) {
      tcontrol=eControlGotoDown;
      setMotorStat(eMotorDown);
    }
    break;
    case eControlRefDown:
    if(isDown()) {;
      tcontrol=eControlGotoUp;
      setMotorStat(eMotorUp);
    }
    break;
    case eControlGotoUp:
    if(actPosition<=setpoint) {
      tcontrol=eControlStop;
      setMotorStat(eMotorStop);
    }
    break;
    case eControlGotoDown:
    if(actPosition>=setpoint) {
      tcontrol=eControlStop;
      setMotorStat(eMotorStop);
    }
    break;
  }
}

// ---- Endpositionserkennung ----

void setupEndPosition() {
  endCounter=END_WAIT;
  bIsDown=false;
  bIsUp=false;
}
boolean isDown() {
  return bIsDown;
}
boolean isUp() {
  return bIsUp;
}

void processEndPosition() {
  switch(getMotorStat()) {
    case eMotorStop:
    default:
    endCounter=END_WAIT;
    break;
    case eMotorUp:
    bIsDown=false;
    if(endCounter>0) {
      --endCounter;
    } else {
      if(getSensorMdown()!=SENSOR_ACTIVE) {
        bIsUp=true;
      }
    }
    break;
    case eMotorDown:
    bIsUp=false;
    if(endCounter>0) {
      --endCounter;
    } else {
      if(getSensorMup()!=SENSOR_ACTIVE) {
        bIsDown=true;
      }
    }
    break;
  }
}

// ---- Motorsteuerung ----

void setupMotor() {
  tMotorMode=eMotorStop;
  setRelaisMode(eRelaisStop);
  blockingCounter=0;
}

void setMotorStat(typeMotorMode tStat) {
  tMotorStat=tStat;
  if(tStat==eMotorStop) {
    setRelaisMode(eRelaisStop);
  }
}

typeMotorMode getMotorStat() {
  if(digitalRead(RELAIS_ONOFF)==RELAIS_OFF) {
    return eMotorStop;
  } else if(digitalRead(RELAIS_DIR)==(invertMotor?DIR_DOWN:DIR_UP)) {
    return eMotorUp;
  } else {
    return eMotorDown;
  }
}

void processMotor() {
  if(getMotorStat()==eMotorStop) {
    if(blockingCounter>0) {
      --blockingCounter;
    }
  } else {
    blockingCounter=BLOCKING_TICKS;
  }
  switch(tMotorMode) {
    case eMotorStop:
    switch(tMotorStat) {
      case eMotorStop:
      break;
      case eMotorUp:
      if(blockingCounter==0) {
        setRelaisMode(eRelaisUp);
        tMotorMode=eMotorUp;
      }
      break;
      case eMotorDown:
      if(blockingCounter==0) {
        setRelaisMode(eRelaisDown);
        tMotorMode=eMotorDown;
      }
      break;
    }
    break;
    case eMotorUp:
    switch(tMotorStat) {
      case eMotorStop:
      setRelaisMode(eRelaisStop);
      tMotorMode=eMotorStop;
      break;
      case eMotorUp:
      break;
      case eMotorDown:
      setRelaisMode(eRelaisStop);    
      tMotorMode=eMotorStop;
      break;
    }
    break;
    case eMotorDown:
    switch(tMotorStat) {
      case eMotorStop:
      setRelaisMode(eRelaisStop);
      tMotorMode=eMotorStop;
      break;
      case eMotorDown:
      break;
      case eMotorUp:
      setRelaisMode(eRelaisStop);    
      tMotorMode=eMotorStop;
      break;
    }
    break;
  }
}

// ---- Realsisteuerung ----

void setupRelais() {
  invertMotor=!EEPROM.read(MOTOR_ADD);
  
  digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
  digitalWrite(RELAIS_DIR,RELAIS_OFF),
  pinMode(RELAIS_ONOFF,OUTPUT);
  pinMode(RELAIS_DIR,OUTPUT);
  tRelaisStat=eRelaisStop;
  tRelaisMode=eRelaisModeStop;
}

void setRelaisMode(typeRelaisStat stat) {
  tRelaisStat=stat;
  if(stat==eRelaisStop) {
    digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
  }
}

void processRelais() {
  switch(tRelaisMode) {
    case eRelaisModeStop:
      switch(tRelaisStat) {
        case eRelaisUp:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_DOWN:DIR_UP);
        digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
        tRelaisMode=eRelaisModeWaitUp;
        break;
        case eRelaisDown:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_UP:DIR_DOWN);
        digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
        tRelaisMode=eRelaisModeWaitDown;
        break;
        case eRelaisStop:
        default:
        break;
      }
      break;
   case eRelaisModeUp:
      switch(tRelaisStat) {
        case eRelaisStop:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_DOWN:DIR_UP);
        digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
        tRelaisMode=eRelaisModeWaitStop;
        break;
        default:
        break;
      }
      break;
   case eRelaisModeDown:
      switch(tRelaisStat) {
        case eRelaisStop:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_UP:DIR_DOWN);
        digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
        tRelaisMode=eRelaisModeWaitStop;
        break;
        default:
        break;
      }
      break;
   case eRelaisModeWaitStop:
       digitalWrite(RELAIS_DIR,RELAIS_OFF);
       digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
       tRelaisMode=eRelaisModeStop;
  break;
   case eRelaisModeWaitUp:
     switch(tRelaisStat) {
      case eRelaisStop:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_DOWN:DIR_UP);
        digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
        tRelaisMode=eRelaisModeWaitStop;
        break;
        default:
        digitalWrite(RELAIS_DIR,invertMotor?DIR_DOWN:DIR_UP);
        digitalWrite(RELAIS_ONOFF,RELAIS_ON);
        tRelaisMode=eRelaisModeUp;
        break;
     }
     break;
     case eRelaisModeWaitDown:
       switch(tRelaisStat) {
        case eRelaisStop:
          digitalWrite(RELAIS_DIR,invertMotor?DIR_UP:DIR_DOWN);
          digitalWrite(RELAIS_ONOFF,RELAIS_OFF);
          tRelaisMode=eRelaisModeWaitStop;
          break;
          default:
          digitalWrite(RELAIS_DIR,invertMotor?DIR_UP:DIR_DOWN);
          digitalWrite(RELAIS_ONOFF,RELAIS_ON);
          tRelaisMode=eRelaisModeDown;
          break;
       }
       break;
    }
}

// ---- Test ----

void processTest() {
  if(testCount>0) {
    --testCount;
  } else {
    if(getSensorSup()) {
      testResult|=0x01;
    } else {
      testResult|=0x10;
    }
    if(getSensorSdown()) {
      testResult|=0x02;
    } else {
      testResult|=0x20;
    }
    if(getSensorMup()) {
      testResult|=0x04;
    } else {
      testResult|=0x40;
    }
    if(getSensorMdown()) {
      testResult|=0x08;
    } else {
      testResult|=0x80;
    }
    #ifdef REEDCONTACT
    if(digitalRead(REEDCONTACT)) {
      testResult|=0x100;
    } else {
      testResult|=0x200;
    }
    #endif
  }
}

void test() {
  unsigned short testMask;
  #ifdef REEDCONTACT
  testMask=0x03ff;
  #else
  testMask=0x00ff;
  #endif
  if((testResult & testMask)==testMask) {
      client.publish(address, "passed");
  } else {
      client.publish(address, "failed");
  }
}

// ---- Sensorauswerung ----

void setupSensors() {
  invertSwitch=!EEPROM.read(SWITCH_ADD);
  
  pinMode(SUP,INPUT);
  pinMode(SDOWN,INPUT);
  pinMode(MUP,INPUT);
  pinMode(MDOWN,INPUT);

  levelSup=MAX_LEVEL;
  levelSdown=MAX_LEVEL;
  levelMup=MAX_LEVEL;
  levelMdown=MAX_LEVEL;
  bSupStat=!SENSOR_ACTIVE;
  bSdownStat=!SENSOR_ACTIVE;
  bMupStat=!SENSOR_ACTIVE;
  bMdownStat=!SENSOR_ACTIVE;
}

void processSensors() {
  if(digitalRead(SUP)==SENSOR_ACTIVE) {
    levelSup=MAX_LEVEL;
    bSupStat=SENSOR_ACTIVE;
  } else {
    if(levelSup>0) {
      --levelSup;
    } else {
      bSupStat=!SENSOR_ACTIVE;
    }
  }
  if(digitalRead(SDOWN)==SENSOR_ACTIVE) {
    levelSdown=MAX_LEVEL;
    bSdownStat=SENSOR_ACTIVE;
  } else {
    if(levelSdown>0) {
      --levelSdown;
    } else {
      bSdownStat=!SENSOR_ACTIVE;
    }
  }
  if(digitalRead(MUP)==SENSOR_ACTIVE) {
    levelMup=MAX_LEVEL;
    bMupStat=SENSOR_ACTIVE;
  } else {
    if(levelMup>0) {
      --levelMup;
    } else {
      bMupStat=!SENSOR_ACTIVE;
    }
  }
  if(digitalRead(MDOWN)==SENSOR_ACTIVE) {
    levelMdown=MAX_LEVEL;
    bMdownStat=SENSOR_ACTIVE;
  } else {
    if(levelMdown>0) {
      --levelMdown;
    } else {
      bMdownStat=!SENSOR_ACTIVE;
    }
  }
}  

void reed(boolean status) {
  EEPROM.write(REED_ADD,status);
  EEPROM.commit();
}
boolean getReedcontact() {
  #ifdef REEDCONTACT
  if(EEPROM.read(REED_ADD)) {
   return digitalRead(REEDCONTACT)?REEDCONTACT_OPEN:REEDCONTACT_CLOSED;
  } else {
    return REEDCONTACT_CLOSED;
  }
  #else
  return REEDCONTACT_CLOSED;
  #endif
}
boolean getSensorSup() {
  return invertSwitch?bSdownStat:bSupStat;
}

boolean getSensorSdown() {
  return invertSwitch?bSupStat:bSdownStat;
}

boolean getSensorMup() {
   return invertMotor?bMdownStat:bMupStat;
}

boolean getSensorMdown() {
  return invertMotor?bMupStat:bMdownStat;
}

// ---- LED Steuerung ----

void setupLed() {
  digitalWrite(LED_GREEN,getLedStat(tGreenStat));
  digitalWrite(LED_RED,getLedStat(tRedStat));
  pinMode(LED_RED,OUTPUT);
  #ifdef REEDCONTACT
  pinMode(REEDCONTACT,INPUT);
  #else
  pinMode(LED_GREEN,OUTPUT);
  #endif
  blinkCounter=0;
  tGreenStat=eOff;
  tRedStat=eOff; 
  bIntStat=false;
}

void processBlink() {
  ++blinkCounter;
  if(blinkCounter>=10) {
    blinkCounter=0;
  }
}

boolean getLedStat(typeLedStat stat) {
  switch(stat) {
    case eOff:
    default:
    return LED_OFF;
    case eBlink:
    return blinkCounter>5;
    case eSupPort:
    return (digitalRead(SUP)==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eSup:
    return (getSensorSup()==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eSdownPort:
    return (digitalRead(SDOWN)==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eSdown:
    return (getSensorSdown()==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eMupPort:
    return (digitalRead(MUP)==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eMup:
    return (getSensorMup()==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eMdownPort:
    return (digitalRead(MDOWN)==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eMdown:
    return (getSensorMup()==SENSOR_ACTIVE)?LED_ON:LED_OFF;
    case eRelaisOnOff:
    return (digitalRead(RELAIS_ONOFF)==RELAIS_ON)?LED_ON:LED_OFF;
    case eRelaisDir:
    return (digitalRead(RELAIS_DIR)==RELAIS_ON)?LED_ON:LED_OFF;
    case eInt:
    return bIntStat?LED_ON:LED_OFF;
  }
}

// ---- Hauptschleife ----
 
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  digitalWrite(LED_GREEN,getLedStat(tGreenStat));
  digitalWrite(LED_RED,getLedStat(tRedStat));

}
