/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, Feb 11th, 2024.Based on the implementation
  written by MechanicTony located on https://github.com/MechanicTony/AOG_CAN_Teensy4.1/blob/main/Autosteer_AOGv5_Teensy4.1UDP_SteerReadyCAN/CAN_All_Brands.ino


  This library handles the reading of analog signal on ESP32 
  with internal pin.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef CANMANAGER_H
#define CANMANAGER_H

#if MICRO_VERSION == 1
  #include <ACAN_ESP32.h>
  #define V_Bus ACAN_ESP32::can   //Steering Valve Bus
  #define ISO_Bus ACAN_ESP32::can //Only one can available for ESP32
  #define K_Bus ACAN_ESP32::can   //Only one can available for ESP32
#endif
#if MICRO_VERSION == 2
  #include <ACAN_T4.h>
  #define V_Bus ACAN_T4::can1   //Steering Valve Bus
  #define ISO_Bus ACAN_T4::can2 //ISO Bus
  #define K_Bus ACAN_T4::can3   //Tractor / Control Bus
#endif

class CANManager{
public:
  CANManager(){}
  CANManager(JsonDB* _db, uint8_t _brand, uint8_t _mode, bool _debug=false){
		brand = _brand;
    mode = _mode;
		db = _db;
    debug = _debug;

   #if MICRO_VERSION == 2
    
    uint32_t errorCode1 = 0, errorCode2 = 0, errorCode3 = 0;

    //V_Bus is CAN-3 and is the Steering BUS
    ACAN_T4_Settings settings (250000);
    settings.mTransmitBufferSize = 256;
    if(brand == 0){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CAC1E13), // Filter #0 //Claas Curve Data & Valve State Message
        ACANPrimaryFilter (kData, kExtended, 0x18EF1CD2), // Filter #1 //Claas Engage Message
        ACANPrimaryFilter (kData, kExtended, 0x1CFFE6D2)  // Filter #2 //Claas Work Message (CEBIS Screen MR Models)
        };
      CANBUS_ModuleID = 0x1E;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 3);
    }else if(brand == 1){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CAC1C13), // Filter #0 //Valtra Curve Data & Valve State Message
        ACANPrimaryFilter (kData, kExtended, 0x18EF1C32), // Filter #1 //Valtra Engage Message
        ACANPrimaryFilter (kData, kExtended, 0x18EF1CFC), // Filter #2 //Mccormick Engage Message
        ACANPrimaryFilter (kData, kExtended, 0x18EF1C00)  // Filter #3 //MF Engage Message
        };
      CANBUS_ModuleID = 0x1C;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 3);
    }else if(brand == 2){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CACAA08), // Filter #0 //CaseIH Curve Data & Valve State Message
        ACANPrimaryFilter (kData, kExtended, 0x0CEFAA08), // Filter #1 //User defined
        ACANPrimaryFilter (kData, kExtended, 0x0CEF08AA), // Filter #2 //User defined
        ACANPrimaryFilter (kData, kExtended, 0x0000FF00)  // Filter #3 //User defined
        };
      CANBUS_ModuleID = 0xAA;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 4);
    }else if(brand == 3){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CEF2CF0) // Filter #0 //Fendt Curve Data & Valve State Message
        };
      CANBUS_ModuleID = 0x2C;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 1);
    }else if(brand == 4){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CACAB13), // Filter #0 //JCB Curve Data & Valve State Message
        ACANPrimaryFilter (kData, kExtended, 0x18EFAB27) // Filter #1 //JCB engage message
        };
      CANBUS_ModuleID = 0xAB;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 2);
    }else if(brand == 5){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CEF2CF0) // Filter #0 //FendtONE Curve Data & Valve State Message
        };
      CANBUS_ModuleID = 0x2C;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 1);
    }else if(brand == 6){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CACF013) // Filter #0 //Lindner Curve Data & Valve State Message
        };
      CANBUS_ModuleID = 0xF0;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 1);
    }else if(brand == 7){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x0CAC1C13), // Filter #0 //AgOpenGPS Curve Data & Valve State Message
        ACANPrimaryFilter (kData, kExtended, 0x19EF1C13) // Filter #1 //AgOpenGPS error message
        };
      CANBUS_ModuleID = 0x1C;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 2);
    }else if(brand == 8){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x18EF1CF0) // Filter #0 //Cat MTxxx Curve data, valve state and engage messages
        };
        CANBUS_ModuleID = 0x1C;
      errorCode1 = V_Bus.begin(settings, primaryFilters, 1);
    }
    // Claim V_Bus Address 
    if(brand >= 0 && brand <= 8){
      CANMessage msg;
      if(brand == 0) msg.id = 0x18EEFF1E;       //Claas
      else if(brand == 1) msg.id = 0x18EEFF1C;  //Massey, Valtra, ETC
      else if(brand == 2) msg.id = 0x18EEFFAA;  //Case, Hew Holland
      else if(brand == 3) msg.id = 0x18EEFF2C;  //Fendt
      else if(brand == 4) msg.id = 0x18EEFFAB;  //JCB
      else if(brand == 5) msg.id = 0x18EEFF2C;  //FendtONE
      else if(brand == 6) msg.id = 0x18EEFFF0;  //Linder
      else if(brand == 7) msg.id = 0x18EEFF1C;  //AgOpenGPS
      else if(brand == 8) msg.id = 0x18EEFF1C;  //Cat MTxxx
      msg.ext = true;
      msg.len = 8;
      msg.data[0] = 0x00;
      msg.data[1] = 0x00;
      msg.data[2] = 0xC0;
      msg.data[3] = 0x0C;
      msg.data[4] = 0x00;
      msg.data[5] = 0x17;
      msg.data[6] = 0x02;
      msg.data[7] = 0x20;
      V_Bus.tryToSend(msg);
    }
    delay(100);

    //ISO_Bus is CAN-2 
    errorCode2 = ISO_Bus.begin(settings);
    if(brand >= 0 && brand <= 7){
      CANMessage msg;
      if (brand == 0) msg.id = 0x18EEFF1E;       //Claas
      else if(brand == 1) msg.id = 0x18EEFF1C;  //Massey, Valtra, ETC
      else if(brand == 2) msg.id = 0x18EEFFAA;  //Case, Hew Holland
      else if(brand == 3) msg.id = 0x18EEFF2C;  //Fendt
      else if(brand == 4) msg.id = 0x18EEFFAB;  //JCB
      else if(brand == 5) msg.id = 0x18EEFF2C;  //FendtOne
      else if(brand == 6) msg.id = 0x18EEFFF0;  //Linder
      else if(brand == 7) msg.id = 0x18EEFF1C;  //AgOpenGPS
      msg.ext = true;
      msg.len = 8;
      msg.data[0] = 0x00;
      msg.data[1] = 0x00;
      msg.data[2] = 0xC0;
      msg.data[3] = 0x0C;
      msg.data[4] = 0x00;
      msg.data[5] = 0x17;
      msg.data[6] = 0x02;
      msg.data[7] = 0x20;
      ISO_Bus.tryToSend(msg);
    }
    delay (500); 

    //K_Bus is CAN-1 and is the Main Tractor Bus
    //Put filters into here to let them through (All blocked by above line)
    if(brand == 3){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kStandard, 0x613) // Filter #0 //Fendt Arm Rest Buttons
        };
      errorCode3 = K_Bus.begin(settings, primaryFilters, 1);
    }else if(brand == 5){
      ACAN_T4_Settings settings (500000);
      settings.mTransmitBufferSize = 256;
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0xCFFD899) // Filter #0 //FendtOne Engage
        };
      errorCode3 = K_Bus.begin(settings, primaryFilters, 1);
    }else if(brand == 2){
      const ACANPrimaryFilter primaryFilters [] = {
        ACANPrimaryFilter (kData, kExtended, 0x14FF7706), // Filter #0 //CaseIH Engage Message
        ACANPrimaryFilter (kData, kExtended, 0x18FE4523) // Filter #1 //CaseIH Rear Hitch Infomation
        };
      errorCode3 = K_Bus.begin(settings, primaryFilters, 2);
    }
    delay(300);

    if (errorCode1 == 0 && errorCode2 == 0 && errorCode3 == 0) {
      Serial.println ("CAN Configuration OK!");
    } else {
      Serial.printf("CAN Configuration error K_Bus:0x%X, ISO_Bus:0x%X, V_Bus:0x%X\n", errorCode1, errorCode2, errorCode3) ;
    }

    Serial.printf("CAN Manager initialised on Brand: %s, Mode:%d\n", getBrandName().c_str(), mode);
   #endif
	}

  uint8_t mode = 0; //Variable to set CAN mode. 0: none, 1: V+ISO+K !Keya, 2: V+ISO+Keya !K, 3: V+Keya+K, 4: V+ISO !K!Keya, 5:V+K !Keya!ISO, 6: V+Keya !K!ISO, 7: V !K!ISO!Keya, 8:Keya !V!K!ISO
  uint8_t brand = 0;//Variable to set CAN configuration
  double was = 0.5; //normalised was value, range:[0.0 - 1.0]

	void receive(){
   #if MICRO_VERSION == 2
    VBusReceive();
    ISOReceive();
    KReceive();
   #endif 
	}

  String getBrandName(){
    return brandName[brand];
  }

  //Fendt K-Bus Buttons
  void pressGo(){
    CANMessage msg;
    msg.id = 0X613;
    msg.len = 8;
    msg.data[0] = 0x15;
    msg.data[1] = 0x20;
    msg.data[2] = 0x06;
    msg.data[3] = 0xCA;
    msg.data[4] = 0x80;
    msg.data[5] = 0x01;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    K_Bus.tryToSend(msg);
    goDown = true;
    Serial.println("Press Go");
  }

  void liftGo(){
    CANMessage msg;
    msg.id = 0X613;
    msg.len = 8;
    msg.data[0] = 0x15;
    msg.data[1] = 0x20;
    msg.data[2] = 0x06;
    msg.data[3] = 0xCA;
    msg.data[4] = 0x00;
    msg.data[5] = 0x02;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    K_Bus.tryToSend(msg);
    goDown = false;
  }

  void pressEnd(){
    CANMessage msg;
    msg.id = 0X613;
    msg.len = 8;
    msg.data[0] = 0x15;
    msg.data[1] = 0x21;
    msg.data[2] = 0x06;
    msg.data[3] = 0xCA;
    msg.data[4] = 0x80;
    msg.data[5] = 0x03;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    K_Bus.tryToSend(msg);
    endDown = true;
    Serial.println("Press End");
  }

  void liftEnd(){
    CANMessage msg;
    msg.id = 0X613;
    msg.len = 8;
    msg.data[0] = 0x15;
    msg.data[1] = 0x21;
    msg.data[2] = 0x06;
    msg.data[3] = 0xCA;
    msg.data[4] = 0x00;
    msg.data[5] = 0x04;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    K_Bus.tryToSend(msg);
    endDown = false;
  }

  // CLAAS CSM buttons Start
  void pressCSM1(){                                     
    CANMessage msg;
    msg.id = 0x14204146;
    msg.ext = true;
    msg.len = 8;
    msg.data[0] = 0xF1;
    msg.data[1] = 0xFC;
    msg.data[2] = 0xFF;
    msg.data[3] = 0xFF;
    msg.data[4] = 0xFF;
    msg.data[5] = 0xFF;
    msg.data[6] = 0xFF;
    msg.data[7] = 0x67;
    K_Bus.tryToSend(msg);
    goDown = true;
    Serial.println("Press CSM1");
  }

  void pressCSM2(){
    CANMessage msg;
    msg.id = 0x14204146;
    msg.ext = true;
    msg.len = 8;
    msg.data[0] = 0xF4;
    msg.data[1] = 0xFC;
    msg.data[2] = 0xFF;
    msg.data[3] = 0xFF;
    msg.data[4] = 0xFF;
    msg.data[5] = 0xFF;
    msg.data[6] = 0xFF;
    msg.data[7] = 0x3F;
    K_Bus.tryToSend(msg);
    endDown = true;
    Serial.println("Press CSM2");
  }

private:
	JsonDB* db;
  uint8_t CANBUS_ModuleID = 0x1C; //Used for the Module CAN ID
  String brandName[8] = {"Class","Valtra/MF","CaseIH/NH","Fendt","JCB","FendtOne","Lindner","AgOpenGPS-Remote"};
  /*
    0 = Claas (1E/30 Navigation Controller, 13/19 Steering Controller) - See Claas Notes on Service Tool Page
    1 = Valtra, Massey Fergerson (Standard Danfoss ISO 1C/28 Navigation Controller, 13/19 Steering Controller)
    2 = CaseIH, New Holland (AA/170 Navagation Controller, 08/08 Steering Controller)
    3 = Fendt (2C/44 Navigation Controller, F0/240 Steering Controller)
    4 = JCB (AB/171 Navigation Controller, 13/19 Steering Controller)
    5 = FendtOne - Same as Fendt but 500kbs K-Bus.
    6 = Lindner (F0/240 Navigation Controller, 13/19 Steering Controller)
    7 = AgOpenGPS - Remote CAN/PWM module (1C/28 Navigation Controller, 13/19 Steering Controller)
  */
  bool debug = false;
  bool intendToSteer = false;
  // V_Bus: was, steering valve status, engage (0,1,4,8),  workswitch (0,1)
  uint8_t steeringValveReady = 0;               //Variable for Steering Valve State from CAN
  bool engageCAN = false;                       //Variable for Engage from CAN
  bool workCAN = false;                         //Variable for Workswitch from CAN
  uint8_t engageLED = 24;                       //Option for LED, to see if Engage message is recived.
  uint32_t time, relayTime;                      //Time to keep "Button Pressed" from CAN Message
  // brand 7 ¿¿¿¿why and how calculate????, 3&5 FendtEstCurve value range?
  int16_t pwmDisplay = 0;
  uint8_t pressureReading;
  uint8_t currentReading;
  // brand 8 ¿¿¿¿why and how calculate???? Andres monedero o  agedu inversiones SL
  bool reverse_MT=false;

  bool goDown = false, endDown = false;

  //ISO_bus: work msg and fendt3 engage
  uint8_t rearHitch = 250;     //Variable for hitch height from ISOBUS & CaseIH-Kbus (0-250 *0.4 = 0-100%)

  //K_bus: fendt3&5 engage, CaseIH engage & rearHitch. All buttons defined in public method

  void VBusReceive(){
    CANMessage msg;
    if (V_Bus.receive(msg)){
      if(brand == 0){
        //**Current Wheel Angle & Valve State**
        if(msg.id == 0x0CAC1E13){        
          uint16_t estCurve = ((msg.data[1] << 8) + msg.data[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
          was = estCurve/64256;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]); 
        }

        //**Engage Message**
        if (msg.id == 0x18EF1CD2){
          if ((msg.data[1])== 0 && (msg.data[2])== 0){   //Ryan Stage5 Models?
            engageCAN = bitRead(msg.data[0],2);
            time = millis();
            digitalWrite(engageLED,HIGH); 
            relayTime = ((millis() + 1000));
            //*****Turn safety valve ON**********
            if (engageCAN) digitalWrite(db->conf.driver_pin[1], 1);       
          }
          if ((msg.data[0]) == 39 && (msg.data[2]) == 241){   //Ryan MR Models?
            engageCAN = bitRead(msg.data[1],0);
            time = millis();
            digitalWrite(engageLED,HIGH); 
            relayTime = ((millis() + 1000));
            //*****Turn safety valve ON**********
            if (engageCAN) digitalWrite(db->conf.driver_pin[1], 1);       
          }
          if ((msg.data[1])== 0 && (msg.data[2])== 125){ //Tony Non MR Models? Ryan Mod to bit read engage bit
            engageCAN = bitRead(msg.data[0],2);
            time = millis();
            digitalWrite(engageLED,HIGH); 
            relayTime = ((millis() + 1000));
            //*****Turn saftey valve ON**********
            if (engageCAN == 1) digitalWrite(db->conf.driver_pin[1], 1);       
          }
        } 

        //**Work Message**
        if (msg.id == 0x1CFFE6D2){
          if ((msg.data[0])== 144){
            workCAN = bitRead(msg.data[6],0);
          }
        }
      }else if(brand == 1){
        //**Current Wheel Angle & Valve State**
        if (msg.id == 0x0CAC1C13){        
          uint16_t estCurve = ((msg.data[1] << 8) + msg.data[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
          was = estCurve/64256;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]); 
        } 

        //**Engage Message**
        if (msg.id == 0x18EF1C32){
          if ((msg.data[0])== 15 && (msg.data[1])== 96 && (msg.data[2])== 1){   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = true;
            relayTime = ((millis() + 1000));
          }
        }else if (msg.id == 0x18EF1CFC){//Mccormick engage message
          if ((msg.data[0])== 15 && (msg.data[1])== 96 && (msg.data[3])== 255){   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = true;
            relayTime = ((millis() + 1000));
          }
        } 
        if (msg.id == 0x18EF1C00){//MF engage message
          if ((msg.data[0])== 15 && (msg.data[1])== 96 && (msg.data[2])== 1){
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = true;
            relayTime = ((millis() + 1000));
          }
        } 
      }else if(brand == 2){
        //**Current Wheel Angle & Valve State**
        if (msg.id == 0x0CACAA08){        
          uint16_t estCurve = ((msg.data[1] << 8) + msg.data[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
          was = estCurve/64256;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]); 
        } 
      }else if(brand == 3){
        //**Current Wheel Angle**
        if(msg.len == 8 && msg.data[0] == 5 && msg.data[1] == 10){
          //FendtEstCurve = (((int8_t)msg.data[4] << 8) + msg.data[5]);
          uint16_t estCurve = (((int8_t)msg.data[4] << 8) + msg.data[5]) + 32128;
          was = estCurve/64256;//normalise to range:[0-1] 
        }
        //**Cutout CAN Message** 
        if (msg.len == 3 && msg.data[2] == 0) steeringValveReady = 80;      // Fendt Stopped Steering So CAN Not Ready
      }else if(brand == 4){
        //**Current Wheel Angle & Valve State**
        if (msg.id == 0x0CACAB13){        
          uint16_t estCurve = ((msg.data[1] << 8) + msg.data[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
          was = estCurve/64256;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]); 
        }
        //**Engage Message**
        if (msg.id == 0x18EFAB27){
          if ((msg.data[0])== 15 && (msg.data[1])== 96 && (msg.data[2])== 1){
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
        }    
      }else if(brand == 5){
        //**Current Wheel Angle**
        if (msg.len == 8 && msg.data[0] == 5 && msg.data[1] == 10){
          //FendtEstCurve = (((int8_t)msg.data[4] << 8) + msg.data[5]);
          uint16_t estCurve = (((int8_t)msg.data[4] << 8) + msg.data[5]) + 32128;
          was = estCurve/64256;//normalise to range:[0-1] 
        }
        //**Cutout CAN Message** 
        if (msg.len == 3 && msg.data[2] == 0) steeringValveReady = 80;      // Fendt Stopped Steering So CAN Not Ready
      }else if(brand == 6){
        //**Current Wheel Angle & Valve State**
        if (msg.id == 0x0CACF013){        
          uint16_t estCurve = ((msg.data[1] << 8) + msg.data[0]);  // CAN Buf[1]*256 + CAN Buf[0] = CAN Est Curve 
          was = estCurve/64256;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]); 
        } 
      }else if(brand == 7){
        //**Current Wheel Angle & Valve State**
        if (msg.id == 0x0CAC1C13){        
          was = float(int16_t(msg.data[1] << 8| msg.data[0])) / 100; 
          was = was/360;//normalise to range:[0-1] 
          steeringValveReady = (msg.data[2]);
          pwmDisplay = (msg.data[3]);
          pressureReading = (msg.data[4]);
          currentReading = (msg.data[5]);
        }
      }else if(brand == 8){
        if (msg.id == 0x18EF1CF0){
          if ((msg.data[0]) == 0xF0 && (msg.data[1]) == 0x20){//MT Curve & Status
            uint16_t estCurve = ((msg.data[2] << 8) + msg.data[3]);
            was = estCurve/64256;//normalise to range:[0-1] 
            //if (gpsSpeed < 1.0) estCurve = 32128;
            byte tempByteA = msg.data[4];
            byte tempByteB = msg.data[5];
            if (tempByteA == 5){
              steeringValveReady = 16;
            }else{
              steeringValveReady = 80;
            }
            byte tempGearByte = tempByteB << 4;
            if (tempGearByte == 32) reverse_MT = 1;
            else reverse_MT = 0;
          }
          if ((msg.data[0]) == 0x0F && (msg.data[1]) == 0x60){   //MT Engage
            if (msg.data[2] == 0x01) {
              digitalWrite(engageLED, HIGH);
              engageCAN = 1;
              relayTime = ((millis() + 1000));
            }
          }
        }
      }//End Brand == 8

      if (debug){
        Serial.print(time);
        Serial.print(", V-Bus"); 
        Serial.print(", ID: 0x"); Serial.print(msg.id, HEX );
        Serial.print(", EXT: "); Serial.print(msg.ext);
        Serial.print(", LEN: "); Serial.print(msg.len);
        Serial.print(", DATA: ");
        for ( uint8_t i = 0; i < 8; i++ ) {
          Serial.print(msg.data[i]); Serial.print(", ");
        }
        Serial.println("");
      }//End Show Data
    }//End if message 
  }

  void ISOReceive(){
    CANMessage msg;
    if (ISO_Bus.receive(msg)){ 
      time = millis();
      //Put code here to sort a message out from ISO-Bus if needed 
      unsigned long PGN=0;
      byte priority=0;
      byte srcaddr=0;
      byte destaddr=0;

      j1939_decode(msg.id, &PGN, &priority, &srcaddr, &destaddr);

      //**Work Message**
      if (PGN == 65093){ //Rear hitch data
        rearHitch = (msg.data[0]); 
        if(brand != 7) pressureReading = rearHitch;
        if (db->steerC.PressureSensor == 1 && rearHitch < db->steerC.PulseCountMax && brand != 7) workCAN = 1; 
        else workCAN = 0; 
      }
  
      if(brand == 3){
        if (msg.id == 0x18EF2CF0){   //**Fendt Engage Message**  
          if ((msg.data[0])== 0x0F && (msg.data[1])== 0x60 && (msg.data[2])== 0x01){   
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
        }
      }

      if (debug){
        Serial.print(time);
        Serial.print(", ISO-Bus"); 
        Serial.print(", ID: 0x"); Serial.print(msg.id, HEX );
        Serial.print(", PGN: "); Serial.print(PGN);
        Serial.print(", Priority: "); Serial.print(priority);
        Serial.print(", SA: "); Serial.print(srcaddr);
        Serial.print(", DA: "); Serial.print(destaddr);
        Serial.print(", EXT: "); Serial.print(msg.ext);
        Serial.print(", LEN: "); Serial.print(msg.len);
        Serial.print(", DATA: ");
        for ( uint8_t i = 0; i < 8; i++ ) {
          Serial.print(msg.data[i]); Serial.print(", ");
        }
  
        if (PGN == 44032) Serial.print("= Curvature Data");
        else if(PGN == 65093) Serial.print("= Rear Hitch Data");
        else if (PGN == 65096) Serial.print("= Wheel Speed, Direction, Distance");
        else if (PGN == 65267) Serial.print("= GPS Vechile Pos");
        else if (PGN == 65256) Serial.print("= GPS Vechile Heading/Speed");
        else if (PGN == 65254) Serial.print("= GPS Time");
        else if (PGN == 129029) Serial.print("= GPS Info (GGA)");

        Serial.println("");
      }//End Show Data
    }
  }

  void KReceive(){
    CANMessage msg;
    if (K_Bus.receive(msg)) { 
      //Put code here to sort a message out from K-Bus if needed 
  
      if(brand == 3){
        if (msg.data[0]==0x15 && msg.data[2]==0x06 && msg.data[3]==0xCA){
          if(msg.data[1]==0x8A && msg.data[4]==0x80) steeringValveReady = 80;      // Fendt Auto Steer Active Pressed So CAN Not Ready
      
          if (msg.data[1]==0x88 && msg.data[4]==0x80){ // Fendt Auto Steer Go   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
        }                                                             
      }else if(brand == 5){
        if (msg.id == 0xCFFD899){   //**FendtOne Engage Message**  
          if ((msg.data[3])== 0xF6){   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
        }
      }else if(brand == 2){//CaseIH info from /buched Emmanuel
        if (msg.id == 0x14FF7706){   //**case IH Engage Message**  
          if ((msg.data[0])== 130 && (msg.data[1])== 1){   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
          if ((msg.data[0])== 178 && (msg.data[1])== 4){   
            time = millis();
            digitalWrite(engageLED,HIGH); 
            engageCAN = 1;
            relayTime = ((millis() + 1000));
          }
        }else if(msg.id == 0x18FE4523){
          rearHitch = (msg.data[0]); 
          pressureReading = rearHitch;
          if (db->steerC.PressureSensor == 1 && rearHitch < db->steerC.PulseCountMax) workCAN = 1; 
          else workCAN = 0; 
        }
      }

      if(debug){
        Serial.print(time);
        Serial.print(", K-Bus"); 
        Serial.print(", ID: 0x"); Serial.print(msg.id, HEX );
        Serial.print(", EXT: "); Serial.print(msg.ext);
        Serial.print(", LEN: "); Serial.print(msg.len);
        Serial.print(", DATA: ");
        for ( uint8_t i = 0; i < 8; i++ ) {
          Serial.print(msg.data[i]); Serial.print(", ");
        }
        Serial.println("");
      }//End Show Data
    }
  }

  void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte* dest_addr){
    /* decode j1939 fields from 29-bit CAN id */
    *src_addr = 255;
    *dest_addr = 255;

    *priority = (int)((ID & 0x1C000000) >> 26);	//Bits 27,28,29

    *PGN = ID & 0x01FFFF00;	//Tony Note: Changed this from 0x00FFFF00 to decode PGN 129029, it now gets the 17th bit of the 18 bit PGN (Bits 9-25, Bit 26 is not used)
    *PGN = *PGN >> 8;

    ID = ID & 0x000000FF;	//Bits 1-8
    *src_addr = (int)ID;

    /* decode dest_addr if a peer to peer message */
    if ((*PGN > 0 && *PGN <= 0xEFFF) || (*PGN > 0x10000 && *PGN <= 0x1EFFF)) {
      *dest_addr = (int)(*PGN & 0xFF);
      *PGN = *PGN & 0x01FF00;
    }
  }

};
#endif
