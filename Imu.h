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
#ifndef IMU_H
#define IMU_H

#include "JsonDB.h"

typedef unsigned char uint8_t;
typedef signed short int int16_t;

class Vector3{
public:
	Vector3(double _x=0.0, double _y=0.0, double _z=0.0){
		x=_x;
		y=_y;
		z=_z;
	}

	double x=0, y=0, z=0;
};
	

class Imu{
public:
	Imu():rotation(0,0,0), acceleration(0,0,0){}
	
	Vector3 rotation, acceleration;

  virtual bool parse()=0;

	void setOn(bool value=true){
		isOn=value;
	}

	void setOffset(){
		pitch_offset = rotation.z+(pitch_offset?pitch_offset:0);
		yaw_offset = rotation.y+(yaw_offset?yaw_offset:0);
		roll_offset = rotation.x+(roll_offset?roll_offset:0);
		
		//store
    db->get("/imuOffset.json", [&](JsonDocument& doc){
			doc["pitch_offset"] = pitch_offset;
			doc["yaw_offset"] = yaw_offset;
			doc["roll_offset"] = roll_offset;
		}, 2);
	}
	
	void getOffset(){
    db->get("/imuOffset.json", [&](JsonDocument& doc){
        JsonObject root = doc.to<JsonObject>();
	  		pitch_offset = root.containsKey("pitch_offset")? doc["pitch_offset"].as<float>() : 0;
		  	yaw_offset = root.containsKey("yaw_offset")? doc["yaw_offset"].as<float>() : 0;
			  roll_offset = root.containsKey("roll_offset")? doc["roll_offset"].as<float>() : 0;
		  });
	}

	String getPanda(){
    uint32_t now = millis();
    uint32_t timeLapse = now - pTime;
    pTime = now;
    String panda="";
    panda+=addValue(rotation.z);//yaw
    panda+=addValue(rotation.x);//roll
    panda+=addValue(rotation.y);//pitch
    panda+=addValue(((rotation.z-pYaw)/timeLapse)*1000, true);//YawRate
    pYaw = rotation.z;
    

    return panda;
  }

	Vector3 getOffsetV(){
		return Vector3(pitch_offset, yaw_offset, roll_offset);
	}

  bool used(){
    if(isUsed) return isUsed;
    isUsed = true;
		return !isUsed;
	}

  bool isActive(){
		return isOn;
	}

protected:
	double pitch_offset, yaw_offset, roll_offset, yaw0=0;
	const double pi = 3.14159265, c2pi = 6.2831853, conv = 3.14159265/18000 /*rad*/, g = 0.00980665/*to transform from mg to m/s2*/;
  uint8_t axisOrientation = 1;// (roll,pitch,yaw) order 1: XYZ, 2: YXZ, 3:ZXY, 4:ZYX, 5:YZX , 6:XZY 
	bool isOn = true, isUsed = true, isFirstValue = true;
  double pYaw = 0;// to calculate YawRate
  uint32_t pTime=0;// to calculate YawRate
	JsonDB* db;
  
  String addValue(double v, bool end=false){
    char n[7];
    dtostrf(v/conv/10,6,0,n);
    String str = String(n)+(end?"":",");
    str.trim();

    return str;
  }
};
#endif
