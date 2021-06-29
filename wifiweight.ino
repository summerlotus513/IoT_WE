#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <SPI.h>
#include "SoftwareSerial.h"

#define GapValue 1
#define uchar unsigned char
#define uint unsigned int
#define BUZZER 6
#define MAX_LEN 16
#define PCD_IDLE 0x00
#define PCD_AUTHENT 0x0E
#define PCD_RECEIVE 0x08
#define PCD_TRANSMIT 0x04
#define PCD_TRANSCEIVE 0x0C
#define PCD_RESETPHASE 0x0F
#define PCD_CALCCRC 0x03
#define PICC_REQIDL 0x26
#define PICC_REQALL 0x52
#define PICC_ANTICOLL 0x93
#define PICC_SElECTTAG 0x93
#define PICC_AUTHENT1A 0x60
#define PICC_AUTHENT1B 0x61
#define PICC_READ 0x30
#define PICC_WRITE 0xA0
#define PICC_DECREMENT 0xC0
#define PICC_INCREMENT 0xC1
#define PICC_RESTORE 0xC2
#define PICC_TRANSFER 0xB0
#define PICC_HALT 0x50
#define MI_OK 0
#define MI_NOTAGERR 1
#define MI_ERR 2
#define Reserved00 0x00
#define CommandReg 0x01
#define CommIEnReg 0x02
#define DivlEnReg 0x03
#define CommIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define Reserved01 0x0F
#define Reserved10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define Reserved11 0x1A
#define Reserved12 0x1B
#define MifareReg 0x1C
#define Reserved13 0x1D
#define Reserved14 0x1E
#define SerialSpeedReg 0x1F
#define Reserved20 0x20
#define CRCResultRegM 0x21
#define CRCResultRegL 0x22
#define Reserved21 0x23
#define ModWidthReg 0x24
#define Reserved22 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F
#define Reserved30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define Reserved31 0x3C
#define Reserved32 0x3D
#define Reserved33 0x3E
#define Reserved34 0x3F

uchar serNum[5];

#define WIFI_AP "CSJ"
#define WIFI_PASSWORD "csj888888"
#define TOKEN "zhouyq"

/* func */ 
extern unsigned long HX711_Read(void);
extern long Get_Weight();

/* 变量定义 */
int HX711_SCK =2;   ///     out
int HX711_DT= 3;    ///     in
long HX711_Buffer = 0;
long Weight_Maopi = 0, Weight_Shiwu = 0;
float Weight = 0;

const int chipSelectPin = 10;
const int NRSTPD = 5;

char thingsboardServer[] = "47.100.91.97";

WiFiEspClient espClient;

PubSubClient client(espClient);

SoftwareSerial soft(4, 7);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

void setup() {
  pinMode(HX711_SCK, OUTPUT);
  pinMode(HX711_DT, INPUT);
  
  Serial.begin(9600);
  SPI.begin();
  pinMode(chipSelectPin,OUTPUT);
  digitalWrite(chipSelectPin, LOW);
  pinMode(NRSTPD,OUTPUT);
  
  yeap_Init();
  Serial.print("Welcome to use!\n");
  delay(3000);
  
  Weight_Maopi = HX711_Read();
  
  pinMode(BUZZER,OUTPUT);
 
  digitalWrite(BUZZER,HIGH);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  lastSend = 0;
}

void loop() {
  status = WiFi.status();
  if (status != WL_CONNECTED) {
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(WIFI_AP);

      status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      delay(500);
    }
    Serial.println("Connected to AP");
  }

  if (!client.connected()) {
    reconnect();
  }
  
  uchar status;
  uchar str[MAX_LEN];

  status = yeap_Request(PICC_REQIDL, str);
  
  status = yeap_Anticoll(str);

  String idcard = " ";
  String weight = " ";

  if (status == MI_OK) {
      String payload = "{";
      memcpy(serNum, str, 5);
      idcard += ShowCardID(serNum);

      Weight = Get_Weight();

      uchar* id = serNum;
      if( id[0]==0x4B && id[1]==0xE6 && id[2]==0xD1 && id[3]==0x3B ) {
          Serial.println("Hello Mary!");
      }
      else if(id[0]==0x3B && id[1]==0xE6 && id[2]==0xD1 && id[3]==0x3B) {
          Serial.println("Hello Greg!");
      }
      else{
          Serial.println("Hello unkown guy!");
      }
      
      Serial.print(float(Weight/1000),3);
      String weight0 = String(float(Weight/1000));
      Serial.println( weight0 );
      weight += String(float(Weight/1000));
      payload += "\"id\":";
      payload += idcard;
      payload += ",";
      payload += "\"weight\":";
      payload += weight;
      buzzer_Di();
      payload += "}";
      char attributes[100];
      payload.toCharArray( attributes, 100 );
      client.publish( "zhouyq", attributes );
      Serial.println( attributes );
  }
  

  if (millis() - lastSend > 50000) {

    String payloadheart = "{heart}";
    char attributes[100];
    payloadheart.toCharArray( attributes, 100 );
    client.publish( "zhouyq", attributes );
    Serial.println( attributes );
    lastSend = millis();
  }
  
  client.loop();
}

void InitWiFi() {

  soft.begin(9600);

  WiFi.init(&soft);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");

    while (true);
  }

  Serial.println("Connecting to AP ...");

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial.println("Connected to AP");
}

void reconnect() {

    while (!client.connected()) {
        Serial.print("Connecting to Thingsboard node ...");

        if ( client.connect("zhouyq", TOKEN, NULL) ) {
            Serial.println( "[DONE]" );
            buzzer_Di();
            delay( 500 );
            buzzer_Di();
        }
        else {
            Serial.print( "[FAILED] [ rc = " );
            Serial.print( client.state() );
            Serial.println( " : retrying in 5 seconds]" );

            delay( 5000 );
        }
    }
}

void buzzer_Di() {
    digitalWrite(BUZZER,LOW);
    delay(100);
    digitalWrite(BUZZER,HIGH);
}

long Get_Weight() {
   HX711_Buffer = HX711_Read();
   Weight_Shiwu = HX711_Buffer;
   Weight_Shiwu = Weight_Shiwu - Weight_Maopi;
   Weight_Shiwu = (long)((float)Weight_Shiwu*1.5/GapValue);
   return Weight_Shiwu; 
}

unsigned long HX711_Read(void) {
   unsigned long count;
   unsigned char i;     
  
   digitalWrite(HX711_DT, HIGH);
   delayMicroseconds(1);
   digitalWrite(HX711_SCK, LOW);
   delayMicroseconds(1);
   count=0; 
   while(digitalRead(HX711_DT));
   for(i=0;i<24;i++)
   { 
       digitalWrite(HX711_SCK, HIGH);
                                    
       delayMicroseconds(1);
       count=count<<1;
       digitalWrite(HX711_SCK, LOW);
       delayMicroseconds(1);
       if(digitalRead(HX711_DT))
          count++; 
    }
   
   digitalWrite(HX711_SCK, HIGH);
   count = 0x800000;
   delayMicroseconds(1);
   digitalWrite(HX711_SCK, LOW);
   delayMicroseconds(1);
   return(count);
}

String ShowCardID(uchar *id) {
    int IDlen=4;
    String idnum = "";
    for(int i=0; i<IDlen; i++){
        Serial.print(0x0F & (id[i]>>4), HEX);
        idnum += String(0x0F & (id[i]>>4));
        Serial.print(0x0F & id[i],HEX);
        idnum += String(0x0F & id[i]);
    }
    idnum += "";
    Serial.println("");

    return idnum;
}

void ShowCardType(uchar* type) {
    Serial.print("Card type: ");
    if(type[0]==0x04&&type[1]==0x00)
        Serial.println("MFOne-S50");
    else if(type[0]==0x02&&type[1]==0x00)
        Serial.println("MFOne-S70");
    else if(type[0]==0x44&&type[1]==0x00)
        Serial.println("MF-UltraLight");
    else if(type[0]==0x08&&type[1]==0x00)
        Serial.println("MF-Pro");
    else if(type[0]==0x44&&type[1]==0x03)
        Serial.println("MF Desire");
    else
        Serial.println("Unknown");
}

void Write_rfiiiii(uchar addr, uchar val) {
    digitalWrite(chipSelectPin, LOW);
    
    SPI.transfer((addr<<1)&0x7E);
    SPI.transfer(val);
    
    digitalWrite(chipSelectPin, HIGH);
}

uchar Read_rfiiiii(uchar addr) {
    uchar val;

    digitalWrite(chipSelectPin, LOW);

    SPI.transfer(((addr<<1)&0x7E) | 0x80);
    val =SPI.transfer(0x00);
    
    digitalWrite(chipSelectPin, HIGH);
    
    return val;
}

void SetBitMask(uchar reg, uchar mask) {
    uchar tmp;
    tmp = Read_rfiiiii(reg);
    Write_rfiiiii(reg, tmp | mask);
}

void ClearBitMask(uchar reg, uchar mask) {
    uchar tmp;
    tmp = Read_rfiiiii(reg);
    Write_rfiiiii(reg, tmp & (~mask));
}

void AntennaOn(void) {
    uchar temp;

    temp = Read_rfiiiii(TxControlReg);
    if (!(temp & 0x03)) {
        SetBitMask(TxControlReg, 0x03);
    }
}

void AntennaOff(void) {
    ClearBitMask(TxControlReg, 0x03);
}

void yeap_Reset(void) {
    Write_rfiiiii(CommandReg, PCD_RESETPHASE);
}

void yeap_Init(void) {
    digitalWrite(NRSTPD,HIGH);

    yeap_Reset();
         
    Write_rfiiiii(TModeReg, 0x8D);
    Write_rfiiiii(TPrescalerReg, 0x3E);
    Write_rfiiiii(TReloadRegL, 30);
    Write_rfiiiii(TReloadRegH, 0);
    
    Write_rfiiiii(TxAutoReg, 0x40);
    Write_rfiiiii(ModeReg, 0x3D);

    AntennaOn();
}

uchar yeap_Request(uchar reqMode, uchar *TagType) {
    uchar status;
    uint backBits;

    Write_rfiiiii(BitFramingReg, 0x07);
    
    TagType[0] = reqMode;
    status = yeap_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }
   
    return status;
}

uchar yeap_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen) {
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command) {
        case PCD_AUTHENT: {
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        }
        case PCD_TRANSCEIVE: {
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        }
        default:
            break;
    }
   
    Write_rfiiiii(CommIEnReg, irqEn|0x80);
    ClearBitMask(CommIrqReg, 0x80);
    SetBitMask(FIFOLevelReg, 0x80);
    
    Write_rfiiiii(CommandReg, PCD_IDLE);
    
    for (i=0; i<sendLen; i++) {
        Write_rfiiiii(FIFODataReg, sendData[i]);
    }

    Write_rfiiiii(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        SetBitMask(BitFramingReg, 0x80);
    }
    
    i = 2000;
    do {
        n = Read_rfiiiii(CommIrqReg);
        i--;
    }while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);
    
    if (i != 0) {
        if(!(Read_rfiiiii(ErrorReg) & 0x1B)) {
            status = MI_OK;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }
            
            if (command == PCD_TRANSCEIVE) {
                n = Read_rfiiiii(FIFOLevelReg);
                lastBits = Read_rfiiiii(ControlReg) & 0x07;
                if (lastBits) {
                    *backLen = (n-1)*8 + lastBits;
                }
                else {
                    *backLen = n*8;
                }
                
                if (n == 0) {
                    n = 1;
                }
                if (n > MAX_LEN) {
                    n = MAX_LEN;
                }
                
                for (i=0; i<n; i++) {
                    backData[i] = Read_rfiiiii(FIFODataReg);
                }
            }
        }
        else {
            status = MI_ERR;
        }     
    }

    return status;
}

uchar yeap_Anticoll(uchar *serNum) {
    uchar status;
    uchar i;
    uchar serNumCheck=0;
    uint unLen;
    
    Write_rfiiiii(BitFramingReg, 0x00);
 
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = yeap_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK) {
        for (i=0; i<4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = MI_ERR;
        }
    }

    return status;
}

void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData) {
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);
    SetBitMask(FIFOLevelReg, 0x80);
    for (i=0; i<len; i++) {
        Write_rfiiiii(FIFODataReg, *(pIndata+i));
    }
    Write_rfiiiii(CommandReg, PCD_CALCCRC);

    i = 0xFF;
    do {
        n = Read_rfiiiii(DivIrqReg);
        i--;
    }while ((i!=0) && !(n&0x04));

    pOutData[0] = Read_rfiiiii(CRCResultRegL);
    pOutData[1] = Read_rfiiiii(CRCResultRegM);
}

uchar yeap_Write(uchar blockAddr, uchar *writeData) {
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[18];
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = yeap_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
        status = MI_ERR;
    }
        
    if (status == MI_OK) {
        for (i=0; i<16; i++) {//Write 16 bytes data into FIFO
            buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = yeap_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
            status = MI_ERR;
        }
    }
    
    return status;
}

void yeap_Halt(void) {
    uchar status;
    uint unLen;
    uchar buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
 
    status = yeap_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}
