#pragma once

#include "Parse.h"

std::vector<std::vector<std::string>> Parse::get_data(std::string& data) {
  std::ifstream file(data);
  std::string line;
  std::vector<std::vector<std::string>> dataset;
  while (std::getline(file, line)) {
    std::istringstream iss( line );
    std::string result;
    std::vector<std::string> dataVect;
    if (std::getline(iss, result, '=')) {
        
        dataVect.push_back(result);
        std::string field;
        while (std::getline(iss, field, ',')) {
          dataVect.push_back(field);
        }
    }
    dataset.push_back(dataVect);
  }
}