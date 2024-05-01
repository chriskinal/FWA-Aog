/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library handles the driving of BNO08X IMU.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef IMUCLASSIC_H
#define IMUCLASSIC_H

#include "Imu.h"
#include "BNO08x_AOG.h"

class ImuClassic: public Imu{
public:
	ImuClassic(JsonDB* _db, uint16_t tickRate=11000, uint8_t _axisOrientation=1){
		db = _db;
    axisOrientation = _axisOrientation;
    acceleration = Vector3(0,0,0);
    rotation = Vector3(0,0,0);
		getOffset();
		isOn = true;

		addresses_size = sizeof(addresses)/sizeof(addresses[0]);
		loop_time = 1000000/tickRate; //Best results seem to be 90-95ms - tickRate:11000-10500
		Wire.begin();
		
		for(int16_t i = 0; i < addresses_size; i++){
			uint8_t address = addresses[i];
			Wire.beginTransmission(address);
			error = Wire.endTransmission();
			if (error == 0){
				// Initialize BNO080 lib
				if (bno08x.begin(address, Wire)){
					Wire.setClock(400000); //Increase I2C data rate to 400kHz
          delay(300);
					// Use gameRotationVector
					bno08x.enableGameRotationVector(loop_time); //Send data update every REPORT_INTERVAL in ms for BNO085
          bno08x.enableAccelerometer(loop_time);
          // Break out of loop
          break;
				}else error = 2; //BNO080 not detected at given I2C address.
			}else error = 4; //BNO08X not Connected or Found
		}
	}

	bool parse(){
		uint32_t now = millis();
		if (isOn && ((now - last_update) >= loop_time)){
			// Load up data from gyro loop ready
			if (bno08x.dataAvailable() == true){
        double r = bno08x.getRoll();
        double p = bno08x.getPitch();
        double y = bno08x.getYaw();
        if(isFirstValue){//get the startup yaw offset value
          if(millis()>5000){//add some delay, as the very first imu value is not accurate
            yaw0 = y;
            isFirstValue = false;
          }
        }
        y = fmod(y-yaw0+c2pi,c2pi);//makes 0 at starting heading & adjust range to [0-2pi]
        // (roll,pitch,yaw) order 1: XYZ, 2: YXZ, 3:ZXY, 4:ZYX, 5:YZX , 6:XZY 
        if(axisOrientation==1){
	  			rotation = Vector3(r, p, y);
  				acceleration = Vector3(bno08x.getAccelX(),  bno08x.getAccelY(), bno08x.getAccelZ());
        }else if(axisOrientation==2){
	  			rotation = Vector3(y, r, p);
  				acceleration = Vector3(bno08x.getAccelY(), bno08x.getAccelX(), bno08x.getAccelZ());
        }else if(axisOrientation==3){
	  			rotation = Vector3(y,r,p);
  				acceleration = Vector3(bno08x.getAccelZ(),  bno08x.getAccelX(), bno08x.getAccelY());
        }else if(axisOrientation==4){
	  			rotation = Vector3(y,p,r);
  				acceleration = Vector3(bno08x.getAccelZ(),  bno08x.getAccelY(), bno08x.getAccelX());
        }else if(axisOrientation==5){
	  			rotation = Vector3(p,y,r);
  				acceleration = Vector3(bno08x.getAccelY(), bno08x.getAccelZ(), bno08x.getAccelX());
        }else if(axisOrientation==6){
	  			rotation = Vector3(r, y, p);
  				acceleration = Vector3(bno08x.getAccelX(), bno08x.getAccelZ(),  bno08x.getAccelY());
        }

				isUsed = false;
			}
			
			// save time to check for 10 msec
			last_update = now;
		}
		return true;
	}
	
private:
	BNO080 bno08x;
	uint8_t addresses[2] = {0x4A,0x4B};
	uint8_t error = 0;
	int16_t addresses_size;
	uint16_t loop_time = 90; //Report interval in ms,how offen should we get data from IMU (The Imu.loop_time will just grab this data without reading IMU)
	uint32_t last_update;
	bool isOn = false;
};
#endif
