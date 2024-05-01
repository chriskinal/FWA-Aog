/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles GNSS reading and writing.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef GNSS_H
#define GNSS_H

class GNSS{
public:
  GNSS(){}
	GNSS(uint8_t _port, uint32_t _baudRate=115200){
   #if MICRO_VERSION == 1
    uint8_t rxP=5;
    uint8_t txP=17;
    if(_port > 3) _port = 2;

		if(_port == 1) serial = &Serial1;
		else if(_port == 2) {
      serial = &Serial2;
      //pin definition required for esp32
      rxP=5;
      txP=17;
    }
    //else if(_port == 3) serial = &Serial3;
		baudRate = _baudRate;
    serial->begin(baudRate, SERIAL_8N1, rxP, txP);//begin(baudRate); //pin definition required for esp32
   #endif
   #if MICRO_VERSION == 2
    if(_port > 8){//means it is not a Serial but a pin # for defining Teensy RX
      serial = &Serial5;//Rx on pin 21
      //serial->setRX(_port);//IMU SDA (18 in AIO-2.5/4)  SCL not needed. Not possible on 18, not a xbar pin.!!!
    }
		else if(_port == 1) serial = &Serial1;//Rx on pin 0
		else if(_port == 2) serial = &Serial2;//Rx on pin 7
		else if(_port == 3) serial = &Serial3;//Rx on pin 15
		else if(_port == 4) serial = &Serial4;//Rx on pin 16
		else if(_port == 5) serial = &Serial5;//Rx on pin 21
		else if(_port == 6) serial = &Serial6;//Rx on pin 25
		else if(_port == 7) serial = &Serial7;//Rx on pin 28
		else if(_port == 8) serial = &Serial8;//Rx on pin 34

		baudRate = _baudRate;
    serial->begin(baudRate);
    serial->addMemoryForRead(rxBuffer, 512);
    serial->addMemoryForWrite(txBuffer, 512);

   #endif
    gga[0] = '\0';
	  vtg[0] = '\0';
    delay(100);

    Serial.printf("GNSS initialised on serial: %d, @ %lu\n", _port, baudRate);
	}
	
  void read(){
    while(serial->available() != 0){
      char c = serial->read();
      if(c=='\n'){//reads a 'new line' character
        if(msgBuffer[bufferCounter-3]!='*') return;//identify that it is completed, the message has a checksum to compare
        msgBuffer[bufferCounter]='\0';//ends the string in the correct position
        getNmea();//checks the string and updates the valid nmea messages
      }
      
      bufferCounter = (c=='$')? 0 : bufferCounter+1;//updates the count of characters in buffer (rests when is a new line)
      msgBuffer[bufferCounter] = c;//stores the char in the buffer
    }
	}

  String getPanda(){
    char str[110];
    str[0]='\0';

    strcat(str,pandaGga);
    strcat(str,",");
    strcat(str,pandaVtg);

    isUsed = true;

    return str;
  }

  String forward(){
    char str[150];
    strcpy(str, gga);
    strcat(str, "\n\r");
    strcat(str, vtg);
    strcat(str, "\n\r");

    return str;
  }

  void sendNtrip(uint8_t NTRIPData[], uint16_t size) {
    serial->write(NTRIPData, size);
  }

private:
	uint32_t baudRate = 115200;
	uint8_t bufferCounter=0;//NMEA has a maximum of 82 characters, limiterC is to detect the end of the message
	char rxBuffer[512];
	char txBuffer[512];
	char msgBuffer[512];
	HardwareSerial* serial;
	char gga[90];
	char vtg[50];
	char pandaGga[90];
	char pandaVtg[20];
  bool isNmeaValid = false;
  bool isUsed = false;
  bool debug = false;

  void getNmea(){
    if(debug){ Serial.print("Raw message: "); Serial.println(msgBuffer);}
    if(checksum()){
      isNmeaValid = true;
      char type[4];
      int offset = 3;
      for(int i=0; i<3; i++) type[i]= msgBuffer[i+offset];
      type[3]='\0';
      if(debug){ Serial.print("NMEA Type: "); Serial.println(type);}

      if( strcmp(type,"GGA") == 0 ){ strcpy(gga, msgBuffer); getPandaGGA();}
      else if(strcmp(type,"VTG")==0){strcpy(vtg, msgBuffer); getPandaVTG();}
      else isNmeaValid = false;
    }else isNmeaValid = false;
    if(isNmeaValid) isUsed = false;
    if(debug) Serial.print("NMEA read done.\n");
  }

  bool checksum(){
    uint16_t length = bufferCounter-3;
    if(msgBuffer[length]!='*')return false;
    int16_t checksum = 0;
    for (uint8_t i = 1; i < length; i++) checksum ^= msgBuffer[i];
    const char hexValue[] = {msgBuffer[length+1], msgBuffer[length+2], '\0'};
    /*
      Serial.println(hexValue);
      Serial.println(strtol(hexValue,0,16));
      Serial.println(checksum);
    */
    return checksum == strtol(hexValue,0,16);
  }

  void getPandaGGA(){
    int i=0, field=0, headerOffset=0;
    uint8_t lastIndex=0;
    pandaGga[0] = '\0';

    while(gga[i++]!='*'){
      if(gga[i] == ','){
        if(field==0) headerOffset = i;
        field++;
        if(field==1 || field==10) lastIndex = i-headerOffset;
      } 
      if((field>0 && field<10) || (field==13)) pandaGga[lastIndex++] = gga[i];
      if(field==14){
        pandaGga[lastIndex] = '\0';
        break;
      }
    }
  }

  void getPandaVTG(){
    int i=0;
    int j=0;
    uint8_t lastIndex=0;
    pandaVtg[0] = '\0';

    while(vtg[i++]!='*'){
      if(vtg[i] != ','){ 
        if(j>0) pandaVtg[i-lastIndex] = vtg[i];
      }else{
        if(j>0){ 
          pandaVtg[i-lastIndex] = '\0';
          if(debug) Serial.printf("Field: %d, value: /%s\n", j, pandaVtg);
          if(j==5) break;//exit as it has been collected the field we are interested at
        }
        lastIndex = i+1;
        j++;
      }
    }
  }
};
#endif