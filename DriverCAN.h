/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, Feb 11th, 2024.

  This library handles the driving through CAN network using OEM equipment.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DRIVERCAN_H
#define DRIVERCAN_H

#include "Driver.h"
#include "CANManager.h"

class DriverCAN: public Driver {
public:
  DriverCAN() {}
  DriverCAN(CANManager* _canManager) {
    canM = _canManager;
    Serial.printf("Initialised CANBUS Driver on Brand: %s\n", canM->getBrandName());
    k = 32128;
    value = 0;
  }
	
  uint16_t pwm(){
    return static_cast<uint16_t>((value+1)*k);
  }

	void drive(float pwm){
    sendCan(pwm);
    value = pwm;
	}
	
	void disengage(){
    sendCan(0, false);
		value = 0;
	}

private:
  CANManager* canM;
  bool debug = false;

  void sendCan(float pwm, bool intendToSteer=true){
    //brand7: steerAngleSetPoint??
    uint16_t setCurve = (pwm+1)*k;
    CANMessage msg;
    if (canM->brand == 0){
        msg.id = 0x0CAD131E;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = lowByte(setCurve);
        msg.data[1] = highByte(setCurve);
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 0;
        msg.data[4] = 0;
        msg.data[5] = 0;
        msg.data[6] = 0;
        msg.data[7] = 0;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 1){
        msg.id = 0x0CAD131C;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = lowByte(setCurve);
        msg.data[1] = highByte(setCurve);
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 255;
        msg.data[4] = 255;
        msg.data[5] = 255;
        msg.data[6] = 255;
        msg.data[7] = 255;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 2){
        msg.id = 0x0CAD08AA;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = lowByte(setCurve);
        msg.data[1] = highByte(setCurve);
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 255;
        msg.data[4] = 255;
        msg.data[5] = 255;
        msg.data[6] = 255;
        msg.data[7] = 255;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 3){
        uint16_t FendtSetCurve = setCurve - 32128;
        msg.id = 0x0CEFF02C;
        msg.ext = true;
        msg.len = 6;
        msg.data[0] = 5;
        msg.data[1] = 9;
        msg.data[3] = 10;
        if (intendToSteer == 1){  
          msg.data[2] = 3;
          msg.data[4] = highByte(FendtSetCurve);
          msg.data[5] = lowByte(FendtSetCurve);
        }else{
          msg.data[2] = 2;
          msg.data[4] = 0;
          msg.data[5] = 0; 
        }
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 4){
        msg.id = 0x0CAD13AB;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = lowByte(setCurve);
        msg.data[1] = highByte(setCurve);
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 255;
        msg.data[4] = 255;
        msg.data[5] = 255;
        msg.data[6] = 255;
        msg.data[7] = 255;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 5){
        uint16_t FendtSetCurve = setCurve - 32128;
        msg.id = 0x0CEFF02C;
        msg.ext = true;
        msg.len = 6;
        msg.data[0] = 5;
        msg.data[1] = 9;
        msg.data[3] = 10;
        if (intendToSteer == 1){  
          msg.data[2] = 3;
          msg.data[4] = highByte(FendtSetCurve);
          msg.data[5] = lowByte(FendtSetCurve);
        }
        else{
          msg.data[2] = 2;
          msg.data[4] = 0;
          msg.data[5] = 0; 
        }
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 6){
        msg.id = 0x0CAD13F0;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = lowByte(setCurve);
        msg.data[1] = highByte(setCurve);
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 255;
        msg.data[4] = 255;
        msg.data[5] = 255;
        msg.data[6] = 255;
        msg.data[7] = 255;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 7){
        msg.id = 0x0CAD131C;
        msg.ext = true;
        msg.len = 8;
        int16_t sp = (int16_t)(value*100);// old value was steerAngleSetPoint
        msg.data[0] = (uint8_t)sp;
        msg.data[1] = sp >> 8;
        msg.data[2] = (intendToSteer)? 253 : 252;
        msg.data[3] = 0;
        msg.data[4] = 0;
        msg.data[5] = 0;
        msg.data[6] = 0;
        msg.data[7] = 0;
        V_Bus.tryToSend(msg);
    }else if(canM->brand == 8){
        msg.id = 0x1CEFF01C;
        msg.ext = true;
        msg.len = 8;
        msg.data[0] = 0xF0;
        msg.data[1] = 0x1F;
        msg.data[2] = highByte(setCurve);
        msg.data[3] = lowByte(setCurve);
        msg.data[4] = (intendToSteer)? 253 : 252;
        msg.data[5] = 255;
        msg.data[6] = 255;
        msg.data[7] = 255;
        V_Bus.tryToSend(msg);
    }
  }
};
#endif
