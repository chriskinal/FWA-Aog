/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library implements the mandatory methods when no imu is used.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef IMUVOID_H
#define IMUVOID_H

#include "Imu.h"

class ImuVoid: public Imu{
public:
	ImuVoid(JsonDB* _db){
		db = _db;
    isOn = false;
    Serial.println("Imu initialised as Void");
	}

	bool parse(){
    return true;
	}
	
private:
};
#endif