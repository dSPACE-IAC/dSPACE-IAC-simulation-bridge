#ifndef NPC_CONTROLLER_FILE_UTILS_HPP_
#define NPC_CONTROLLER_FILE_UTILS_HPP_

#include <fstream>
#include <iostream>
#include <sstream>    // std::stringstream
#include <stdexcept>  // std::runtime_error
#include <string>
#include <vector>

namespace controller {
    // Return vector of vectors for a specified number of columns of a csv file
    std::vector<std::vector<double>> read_csv(std::string filename, int columns,
                                              std::string delimiter) {
      /**
       * Read a csv file into a vector of vectors
       * @param filename: name of the csv file
       * @param columns: number of columns in the csv file
       * @param delimiter: delimiter used in the csv file
       * @return: std::vector<std::vector<double>>
       * @source:
       * https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
       */
      std::vector<std::vector<double>> result(columns,
                                              std::vector<double>(0, 0));
      // Reads a CSV file into a vector of <string, vector<int>> pairs where
      // each pair represents <column name, column values>

      // Create a vector of <string, int vector> pairs to store the result

      // Create an input filestream
      std::ifstream myFile(filename);

      // Make sure the file is open
      if (!myFile.is_open())
        throw std::runtime_error("Could not open file");

      // Helper vars
      std::string line, colname;

      // Read data, line by line
      while (std::getline(myFile, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        std::string s = ss.str();
        int i = 0;
        size_t pos = 0;
        std::string token;
        while ((pos = s.find(delimiter)) != std::string::npos) {
          token = s.substr(0, pos);
          result[i].push_back(std::stod(token));
          s.erase(0, pos + delimiter.length());
          i++;
        }
        result[i].push_back(std::stod(s));
      }

      // Close file
      myFile.close();

      return result;
    }
  }  // namespace

#endif
