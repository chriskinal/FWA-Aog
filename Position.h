/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library takes care of the position related components (GNSS, IMU, WAS).

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef POSITION_H
#define POSITION_H

#if MICRO_VERSION == 1
 #include <AsyncUDP_WT32_ETH01.h>
#endif
#if MICRO_VERSION == 2
 #include <AsyncUDP_Teensy41.h>
#endif

#include "JsonDB.h"
#include "GNSS.h"
#include "IMU.h"
#include "ImuRvc.h"
#include "ImuClassic.h"
#include "ImuVoid.h"
#include "Sensor.h"
#include "SensorInternalReader.h"
#include "SensorADS1115Reader.h"
#include "CANManager.h"
#include "SensorCAN.h"

class Position{
public:
  Position(){}
	Position(JsonDB* _db, AsyncUDP* udpService, CANManager* canM, bool sensorsDebug=false):gnss(_db->conf.gnss_port, _db->conf.gnss_baudRate){
		udp = udpService;
    db = _db;
    debugSensors = sensorsDebug;

    // Create imu, interact with sensor ######################################################################################################
    (_db->conf.imu_type == 1)? imu = new ImuRvc(_db, _db->conf.imu_port, _db->conf.imu_axisOrder) : (_db->conf.imu_type == 2)? imu = new ImuClassic(_db, _db->conf.imu_tickRate, _db->conf.imu_axisOrder): imu = new ImuVoid(_db);

    // Create and initialize the object to read the WAS sensor ###############################################################################
    (_db->conf.was_type == 1)? was = new SensorInternalReader(_db, _db->conf.was_pin, _db->conf.was_resolution) : (_db->conf.was_type == 2)? was = new SensorADS1115Reader(_db, _db->conf.was_pin) : was = new SensorCAN(_db, canM);
    delay(100);

    // time configuration variables
    previousTime = millis();
    reportPeriodMs = 1000000/_db->conf.reportTickRate;
    reportKPeriodMs = reportPeriodMs/5;
  }

  GNSS gnss;
  Imu* imu;
  Sensor* was;

	bool report(){
	 	gnss.read();

		// set timer to run periodically
    uint32_t now = millis();

		//check for internal was update, 5 times faster as per kalman filter
 		if((db->conf.was_type == 1) && (now - previousKTime > reportKPeriodMs)){
      previousKTime = now;
      was->update();
    }

		if(now - previousTime < reportPeriodMs) return false;
		previousTime = now;
    
		//actual code to run periodically
    if(db->conf.was_type != 1) was->update(); //update if was is not internal reader    

    //char nmea[150];
    String nmea;
    uint8_t strSize=0;
    if(imu->isActive()){//check if there is imu to build PANDA sentences or forward NMEA
      imu->parse();

      // Build the new PANDA sentence ################################################################
      nmea = "$PANDA"+gnss.getPanda()+","+imu->getPanda();
      // Calculate checksum
      char cks[6];
      cks[0]='\0';
      int16_t sum = 0;
      strSize = nmea.length();//strlen(nmea);
      for (uint8_t inx = 1; inx < strSize; inx++) sum ^= nmea[inx];  // Build checksum
      sprintf(cks, "*%02X\r\n", sum);//add the checksum in hex
      nmea += cks;
    }else{//Forward gnss stream
      nmea = gnss.forward();
      strSize = nmea.length()-7;
    }

    if(debugSensors){
      Serial.printf("Was value: %.4f, was angle: %.4f\n", was->value, was->angle);
      Serial.print(nmea);
      //Serial.print("Sending upd packet... (");Serial.print(db->conf.server_ip);Serial.printf(":%d)\n",db->conf.server_destination_port);
	  }

		//send position to udp server #################################################################
    udp->writeTo((uint8_t*)nmea.c_str(), strSize+7, db->conf.server_ip, db->conf.server_destination_port);

    return true;
	}

private:
	AsyncUDP* udp;
 	JsonDB* db;
	uint32_t previousTime;
	uint32_t previousKTime;
	uint16_t reportPeriodMs;
	uint16_t reportKPeriodMs;
  bool debugSensors=false;
};
#endif
