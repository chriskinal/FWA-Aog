/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, Feb 11th, 2024.

  This library handles the reading of analog signal on ESP32 
  with internal pin.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SENSORCAN_H
#define SENSORCAN_H

#include "CANManager.h"


class SensorCAN: public Sensor {
public:
  SensorCAN(JsonDB* _db, CANManager* _canManager){
		db = _db;
    canM = _canManager;
    value = 0.5;
    Serial.printf("CAN sensor reader initialised on Brand: %s\n", canM->getBrandName().c_str());
	}

	void update(){
    if(pin != 7){
      value = canM->was;
      float a = (value - 0.5)*64256 + db->steerS.wasOffset;
      //TODO review /steerSensorCounts or *steerSensorCounts???
      angle = a / (db->steerS.steerSensorCounts *((pin == 3 || pin ==5)? 10 : 1));//Fendt Only modifies value by 10
      if(angle<0) angle *= db->steerS.AckermanFix;
    }
  
    //Map WAS
    if(pin == 3 || pin ==5) angle = multiMap<float>(angle, inputWAS, outputWASFendt, 21);
    else angle = multiMap<float>(angle, inputWAS, outputWAS, 21);
	}
private:
  CANManager* canM;
  bool debug = false;
  //WAS Calibration
  float inputWAS[21] =     { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};  //Input WAS do not adjust
  float outputWAS[21] =    { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};
  float outputWASFendt[21]={ -60.00, -54.0, -48.0, -42.3, -36.1, -30.1, -23.4, -17.1, -11.0, -5.5, 0, 5.5, 11.0, 17.1, 23.4, 30.1, 36.1, 42.3, 48.0, 54.0, 60.0};  //Fendt 720 SCR, CPD = 80

  template<typename T>//Rob Tillaart, https://github.com/RobTillaart/MultiMap
  T multiMap(T value, T* _in, T* _out, uint8_t size){
    // take care the value is within range
    // value = constrain(value, _in[0], _in[size-1]);
    if (value <= _in[0]) return _out[0];
    if (value >= _in[size - 1]) return _out[size - 1];

    // search right interval
    uint8_t pos = 1;  // _in[0] already tested
    while (value > _in[pos]) pos++;

    // this will handle all exact "points" in the _in array
    if (value == _in[pos]) return _out[pos];

    // interpolate in the right segment for the rest
    return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
  }
};
#endif
