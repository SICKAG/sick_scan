//
// Created by michael on 10/11/18.
//

#ifndef SICK_SCAN_SICK_SCAN_CONFIG_H
#define SICK_SCAN_SICK_SCAN_CONFIG_H

#include <fstream>
#include <map>
#include <string>

class SickScanConfig
    {
public:
    static SickScanConfig& get()
      {
      static SickScanConfig instance;
      return instance;
      }
    void setValue(std::string key, std::string val);

    void setValue(std::string key, bool val);
    std::string getValue(std::string key);
    bool getBoolValue(std::string key);

private:
    SickScanConfig(){};
    SickScanConfig(const SickScanConfig&);
    SickScanConfig& operator=(const SickScanConfig&);
    std::map<std::string,std::string> storedConfig;
    };
#endif //SICK_SCAN_SICK_SCAN_CONFIG_H
