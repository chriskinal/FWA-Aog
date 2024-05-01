/*
  This is a library written for the Wt32-AIO project for AgOpenGPS

  Written by Miguel Cebrian, November 30th, 2023.

  This library enables database administration.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef JSONDB_H
#define JSONDB_H

#include "FS.h"
#include <IPAddress.h>
#include "ArduinoJson.h"

#if MICRO_VERSION == 1
  #define FILE_WRITE_BEGIN "r+"
#endif

typedef std::function<void(JsonDocument& doc)> JsonFileHandler;

/////////////////////////////////////////////

struct SteerSettings {
  uint8_t Kp;              // proportional gain
  uint8_t lowPWM;          // band of no action
  int16_t wasOffset;
  uint8_t minPWM;
  uint8_t highPWM;         // max PWM value
  float steerSensorCounts;
  float AckermanFix;        // sent as percent
};      // 11 bytes


struct SteerConfig {
  uint8_t InvertWAS;
  uint8_t IsRelayActiveHigh;    // if zero, active low (default)
  uint8_t MotorDriveDirection;
  uint8_t SingleInputWAS;
  uint8_t CytronDriver;
  uint8_t SteerSwitch;          // 1 if switch selected
  uint8_t SteerButton;          // 1 if button selected
  uint8_t ShaftEncoder;
  uint8_t PressureSensor;
  uint8_t CurrentSensor;
  uint8_t PulseCountMax;
  uint8_t IsDanfoss;
  uint8_t IsUseY_Axis;     //Set to 0 to use X Axis, 1 to use Y avis
};               // 9 bytes


struct Configuration {
  char webFolder[64];
  char steerSettingsFile[64];
  char steerConfigurationFile[64];
  IPAddress eth_ip;
  IPAddress eth_gateway;
  IPAddress eth_subnet;
  IPAddress eth_dns;
  IPAddress server_ip;
  uint16_t server_pcb_port;
  uint16_t server_autosteer_port;
  uint16_t server_ntrip_port;
  uint16_t server_destination_port;
  uint8_t driver_type;
  uint8_t driver_pin[3];
  uint8_t gnss_port;
  uint32_t gnss_baudRate;
  uint8_t imu_type;
  uint8_t imu_port;
  uint8_t imu_axisOrder;
  uint16_t imu_tickRate;
  uint8_t can_brand;
  uint8_t can_mode;
  uint8_t was_type;
  uint8_t was_resolution;
  uint8_t was_pin;
  uint8_t ls_pin;
  uint8_t ls_filter;
  uint8_t remote_pin;
  uint8_t steer_pin;
  uint8_t work_pin;
  uint16_t reportTickRate;
  uint16_t globalTickRate;
};

class JsonDB {
public:
  JsonDB(const char* filename) {
    strcpy(configurationFile, filename);
  }

  FS* fs;
  Configuration conf;
  SteerSettings steerS;
  SteerConfig steerC;
  char configurationFile[50];

  void begin(FS &_fs, bool resetConfFile=false){
    fs = &_fs;
    FIFO[0].file[0]='\0';
    FIFO[0].callback=[](JsonDocument& doc){};
    FIFO[0].type=0;

    if(resetConfFile || !fs->exists(configurationFile)) resetFile(configurationFile);

    configReadCallback = [&](JsonDocument& doc){
      // Copy values from the JsonDocument to the Config
      strcpy(conf.webFolder, doc["webfolders"] | "/index.html");
      strcpy(conf.steerSettingsFile, doc["steerSettingsFile"] | "/steerSettings.json");
      strcpy(conf.steerConfigurationFile, doc["steerConfigurationFile"] | "/steerConfiguration.json");
      conf.eth_ip = IPAddress(doc["eth"]["ip"][0] | 192, doc["eth"]["ip"][1] | 168, doc["eth"]["ip"][2] | 1, doc["eth"]["ip"][3] | 123);
      conf.eth_gateway = IPAddress(doc["eth"]["gateway"][0] | 192, doc["eth"]["gateway"][1] | 168, doc["eth"]["gateway"][2] | 1, doc["eth"]["gateway"][3] | 1);
      conf.eth_subnet = IPAddress(doc["eth"]["subnet"][0] | 255, doc["eth"]["subnet"][1] | 255, doc["eth"]["subnet"][2] | 255, doc["eth"]["subnet"][3] | 0);
      conf.eth_dns = IPAddress(doc["eth"]["dns"][0] | 8, doc["eth"]["dns"][1] | 8, doc["eth"]["dns"][2] | 8, doc["eth"]["dns"][3] | 8);
      conf.server_ip = IPAddress(doc["server"]["ip"][0] | 192, doc["server"]["ip"][1] | 168, doc["server"]["ip"][2] | 1, doc["server"]["ip"][3] | 255);
      conf.server_pcb_port = doc["server"]["pcbPort"] | 5120;
      conf.server_ntrip_port = doc["server"]["ntripPort"] | 2233;
      conf.server_autosteer_port = doc["server"]["autosteerPort"] | 8888;
      conf.server_destination_port = doc["server"]["destinationPort"] | 9999;
      conf.driver_type = doc["driver"]["type"] | 1;
      conf.driver_pin[0] = doc["driver"]["pin"][0] | 0;
      conf.driver_pin[1] = doc["driver"]["pin"][1] | 4;
      conf.driver_pin[2] = doc["driver"]["pin"][2] | 12;
      conf.gnss_port = doc["gnss"]["port"] | 2;
      conf.gnss_baudRate = doc["gnss"]["baudRate"].as<uint32_t>() | 460800;
      conf.imu_type = doc["imu"]["type"] | 1;
      conf.imu_port = doc["imu"]["port"] | 1;
      conf.imu_axisOrder = doc["imu"]["axisOrder"] | 1;
      conf.imu_tickRate = doc["imu"]["tickRate"] | 11000; // run every 10ms (100Hz)
      conf.can_brand = doc["can"]["brand"] | 0;
      conf.can_mode = doc["can"]["mode"] | 0;
      conf.was_type = doc["was"]["type"] | 1;
      conf.was_resolution = doc["was"]["resolution"] | 10;
      conf.was_pin = doc["was"]["pin"] | 14;
      conf.ls_pin = doc["ls"]["pin"] | 39;
      conf.ls_filter = doc["ls"]["filter"] | 2;
      conf.remote_pin = doc["remotePin"] | 39;
      conf.steer_pin = doc["steerPin"] | 36;
      conf.work_pin = doc["workPin"] | 1;
      conf.reportTickRate = doc["reportTickRate"] | 10000; // run every 100ms (10Hz)
      conf.globalTickRate = doc["globalTickRate"] | 10000; // run every 100ms (10Hz)
    };
    
    get(configurationFile, configReadCallback);
    saveConfiguration();

    if(resetConfFile || !fs->exists(conf.steerSettingsFile)) resetFile(conf.steerSettingsFile);
    get(conf.steerSettingsFile,[&](JsonDocument& doc){
      // Copy values from the JsonDocument to the Config
      steerS.Kp = doc["Kp"] | 40;                  // proportional gain
      steerS.lowPWM = doc["lowPWM"] | 10;          // band of no action
      steerS.wasOffset = doc["wasOffset"] | 0;
      steerS.minPWM = doc["minPWM"] | 9;
      steerS.highPWM = doc["highPWM"] | 60;        // max PWM value
      steerS.steerSensorCounts = doc["steerSensorCounts"] | 150;
      steerS.AckermanFix = doc["AckermanFix"] | 1;// sent as percent
    });
    saveSteerSettings();

    if(resetConfFile || !fs->exists(conf.steerConfigurationFile)) resetFile(conf.steerConfigurationFile);
    get(conf.steerConfigurationFile,[&](JsonDocument& doc){
      // Copy values from the JsonDocument to the Config
      steerC.InvertWAS = doc["InvertWAS"] | 0;
      steerC.IsRelayActiveHigh = doc["IsRelayActiveHigh"] | 0; // if zero, active low (default)
      steerC.MotorDriveDirection = doc["MotorDriveDirection"] | 0;
      steerC.SingleInputWAS = doc["SingleInputWAS"] | 1;
      steerC.CytronDriver = doc["CytronDriver"] | 1;
      steerC.SteerSwitch = doc["SteerSwitch"] | 0; // 1 if switch selected
      steerC.SteerButton = doc["SteerButton"] | 0; // 1 if switch selected
      steerC.ShaftEncoder = doc["ShaftEncoder"] | 0;
      steerC.PressureSensor = doc["PressureSensor"] | 0;
      steerC.CurrentSensor = doc["CurrentSensor"] | 0;
      steerC.PulseCountMax = doc["PulseCountMax"] | 5;
      steerC.IsDanfoss = doc["IsDanfoss"] | 0;
      steerC.IsUseY_Axis = doc["IsUseY_Axis"] | 0; //Set to 0 to use X Axis, 1 to use Y avis
    });
    saveSteerConfiguration();

    if(resetConfFile || !fs->exists("/imuOffset.json")){
      fs->remove("/imuOffset.json");
      // in case the json file does not exist creates is as 0
      Serial.println("Failed to open /imuOffset.json file, creating one as 0");
      File file = fs->open("/imuOffset.json", FILE_WRITE);
      file.print("{\"pitch_offset\":0,\"yaw_offset\":0,\"roll_offset\":0}");
      file.close();
    }

    printFile(configurationFile);
    printFile(conf.steerSettingsFile);
    printFile(conf.steerConfigurationFile);
    printFile("/imuOffset.json");
  }

  bool resetFile(const char* filename){
    fs->remove(filename);
    Serial.printf("Failed to open %s file, opening on memory default file\n", filename);
    File file = fs->open(filename, FILE_WRITE);
    file.print("{\"isReseted\":1}");
    file.close();
    return true;
  }

  bool resetConfigurationFiles(){
    fs->remove(configurationFile);
		File file = fs->open(configurationFile, FILE_WRITE);
    file.print("{\"reset\":1}");
    file.close();
    
		file = fs->open("/steerSettings.json", FILE_WRITE);
    file.print("{\"reset\":1}");
    file.close();

		file = fs->open("/steerConfiguration.json", FILE_WRITE);
    file.print("{\"reset\":1}");
    file.close();
    return true;
  }

  void saveConfiguration(){
    get(configurationFile, [&](JsonDocument& doc){
      // Set the values in the document
      doc["webfolders"] = conf.webFolder;
      doc["steerSettingsFile"] = conf.steerSettingsFile;
      doc["steerConfigurationFile"] = conf.steerConfigurationFile;
      doc["eth"]["ip"][0] = conf.eth_ip[0];
      doc["eth"]["ip"][1] = conf.eth_ip[1];
      doc["eth"]["ip"][2] = conf.eth_ip[2];
      doc["eth"]["ip"][3] = conf.eth_ip[3];
      doc["eth"]["gateway"][0] = conf.eth_gateway[0];
      doc["eth"]["gateway"][1] = conf.eth_gateway[1];
      doc["eth"]["gateway"][2] = conf.eth_gateway[2];
      doc["eth"]["gateway"][3] = conf.eth_gateway[3];
      doc["eth"]["subnet"][0] = conf.eth_subnet[0];
      doc["eth"]["subnet"][1] = conf.eth_subnet[1];
      doc["eth"]["subnet"][2] = conf.eth_subnet[2];
      doc["eth"]["subnet"][3] = conf.eth_subnet[3];
      doc["eth"]["dns"][0] = conf.eth_dns[0];
      doc["eth"]["dns"][1] = conf.eth_dns[1];
      doc["eth"]["dns"][2] = conf.eth_dns[2];
      doc["eth"]["dns"][3] = conf.eth_dns[3];
      doc["server"]["ip"][0] = conf.server_ip[0];
      doc["server"]["ip"][1] = conf.server_ip[1];
      doc["server"]["ip"][2] = conf.server_ip[2];
      doc["server"]["ip"][3] = conf.server_ip[3];
      doc["server"]["pcbPort"] = conf.server_pcb_port;
      doc["server"]["ntripPort"] = conf.server_ntrip_port;
      doc["server"]["autosteerPort"] = conf.server_autosteer_port;
      doc["server"]["destinationPort"] = conf.server_destination_port;
      doc["driver"]["type"] = conf.driver_type;
      doc["driver"]["pin"][0] = conf.driver_pin[0];
      doc["driver"]["pin"][1] = conf.driver_pin[1];
      doc["driver"]["pin"][2] = conf.driver_pin[2];
      doc["gnss"]["port"] = conf.gnss_port;
      doc["gnss"]["baudRate"] = conf.gnss_baudRate;
      doc["imu"]["type"] = conf.imu_type;
      doc["imu"]["port"] = conf.imu_port;
      doc["imu"]["axisOrder"] = conf.imu_axisOrder;
      doc["imu"]["tickRate"] = conf.imu_tickRate;
      doc["can"]["brand"] = conf.can_brand;
      doc["can"]["mode"] = conf.can_mode;
      doc["was"]["type"] = conf.was_type;
      doc["was"]["resolution"] = conf.was_resolution;
      doc["was"]["pin"] = conf.was_pin;
      doc["ls"]["pin"] = conf.ls_pin;
      doc["ls"]["filter"] = conf.ls_filter;
      doc["remotePin"] = conf.remote_pin;
      doc["steerPin"] = conf.steer_pin;
      doc["workPin"] = conf.work_pin;
      doc["reportTickRate"] = conf.reportTickRate; // run every 100ms (10Hz)
      doc["globalTickRate"] = conf.globalTickRate; // run every 100ms (10Hz)
		}, 1);
  }

  void saveSteerSettings(){
    get(conf.steerSettingsFile, [&](JsonDocument& doc){
      // Set the values in the document
      doc["Kp"] = steerS.Kp;                  // proportional gain
      doc["lowPWM"] = steerS.lowPWM;          // band of no action
      doc["wasOffset"] = steerS.wasOffset;
      doc["minPWM"] = steerS.minPWM;
      doc["highPWM"] = steerS.highPWM;        // max PWM value
      doc["steerSensorCounts"] = steerS.steerSensorCounts;
      doc["AckermanFix"] = steerS.AckermanFix;// sent as percent
		}, 1);
  }

  void saveSteerConfiguration(){
    get(conf.steerConfigurationFile, [&](JsonDocument& doc){
      // Set the values in the document
      doc["InvertWAS"] = steerC.InvertWAS;
      doc["IsRelayActiveHigh"] = steerC.IsRelayActiveHigh;
      doc["MotorDriveDirection"] = steerC.MotorDriveDirection;
      doc["SingleInputWAS"] = steerC.SingleInputWAS;
      doc["CytronDriver"] = steerC.CytronDriver;
      doc["SteerSwitch"] = steerC.SteerSwitch; // 1 if switch selected
      doc["SteerButton"] = steerC.SteerButton; // 1 if switch selected
      doc["ShaftEncoder"] = steerC.ShaftEncoder;
      doc["PressureSensor"] = steerC.PressureSensor;
      doc["CurrentSensor"] = steerC.CurrentSensor;
      doc["PulseCountMax"] = steerC.PulseCountMax;
      doc["IsDanfoss"] = steerC.IsDanfoss;
      doc["IsUseY_Axis"] = steerC.IsUseY_Axis; //Set to 0 to use X Axis, 1 to use Y avis
		}, 1);
  }

  File open(const char* filename, uint8_t _type=0){
    return fs->open(filename, _type==0?FILE_READ:_type==1?FILE_WRITE:FILE_WRITE_BEGIN);
  }
  
  bool webConfiguration(uint8_t *data){
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error){
      Serial.printf("Error: %s\n", error.c_str());
      return false;
    }
    configReadCallback(doc);
    saveConfiguration();
    return true;
  }

	bool read(const char* filename, JsonFileHandler callback, uint8_t _type=0){
		File file = fs->open(filename, FILE_WRITE_BEGIN);
		if(!file){
			Serial.printf("Failed to open '%s' file\n", filename);
      return false;
		}
    bool e = false;

		JsonDocument doc;
		DeserializationError error = deserializeJson(doc, file);
		if(error){
			Serial.printf("Failed to read Json %s\n", filename);
			Serial.printf("Error: %s\n", error.c_str());
			e = true;
		}else{
      callback(doc);

      if(_type>0){ //only if it is a write operation
        file.seek(0);
        if(serializeJson(doc, file)==0){
          Serial.println("Failed to write to file");
          e = true;
        }else{
          //truncate the file, in case the old size is bigger than the new
          int16_t sizeTruncate = file.size() - file.position();
          if(sizeTruncate>0)
            for(int i=0; i<sizeTruncate; i++) file.print(" ");
        }
      }
    }
		file.close();
    if (e) return false;
		return true;
	}

	bool get(const char* filename, JsonFileHandler callback, uint8_t type=0){
		if(filesInQueue >= bufferSize) return false;
		strcpy(FIFO[filesInQueue].file, filename);
		FIFO[filesInQueue].callback = callback;
		FIFO[filesInQueue].type = type;
		filesInQueue++;
		if(filesInQueue == 1) startNext();
		return true;
	}

  void printFile(const char * path){
      Serial.printf("Reading file: %s\r\n", path);

      File file = fs->open(path);
      if(!file || file.isDirectory()){
          Serial.println("- failed to open file for reading");
          return;
      }

      Serial.println("- read from file:");
      while(file.available()){
          Serial.write(file.read());
      }
      file.close();
      Serial.println();
  }

private:
	const static uint8_t bufferSize = 16;//number of reads that can be saved in the buffer
	uint8_t filesInQueue = 0;
  JsonFileHandler configReadCallback;

	struct QueueEntry{
		char file[80];
		JsonFileHandler callback;
		uint8_t type;
	};
	QueueEntry FIFO[bufferSize]; //buffer to save the queue

	void startNext(){
		if(filesInQueue == 0) return;

		read(FIFO[0].file, FIFO[0].callback, FIFO[0].type);

		removeFromBuffer();
	}

	void removeFromBuffer(){
		filesInQueue--;
		for(uint8_t i = 0; i<filesInQueue; i++){
			strcpy(FIFO[i].file, FIFO[i+1].file);
			FIFO[i].callback = FIFO[i+1].callback;
			FIFO[i].type = FIFO[i+1].type;
		}
		FIFO[filesInQueue].file[0] = '\0';
		FIFO[filesInQueue].callback = [](JsonDocument& doc){};
		FIFO[filesInQueue].type = 0;

		if(filesInQueue>0) startNext();
	}
};
#endif