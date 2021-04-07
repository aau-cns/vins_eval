/// Copyright (C) 2021 Alessandro Fornasier, Control of Networked Systems,
/// University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available in the
/// LICENSE file. No license in patents is granted.
///
/// You can contact the author at alessandro.fornasier@ieee.org

#include "InputDataParser.hpp"

InputDataParser::InputDataParser() {}

void InputDataParser::getIndices(const std::vector<std::string> &header, std::vector<int> &indices) {

  // Temporary index
  int idx;

  // Get indices
  for (const auto &it : categories_) {

    // Get index
    getIndex(header, it, idx);
    indices.push_back(idx);
  }
}

void InputDataParser::parseLine(std::string &line, std::vector<std::string> &data) {

  // Create a stringstream of the current line
  std::stringstream ss(line);

  // Temporary string
  std::string tmp;

  // Extract each cell
  while (std::getline(ss, tmp, ',')) {
    data.push_back(tmp);
  }
}

void InputDataParser::readParseCsv(const std::string filename) {

  std::ifstream file(filename);

  if (!file) {
    throw std::runtime_error("Error opening file \"" + filename + "\". Exit programm.");
  }

  std::cout << "\n----------------------------------------" << std::endl << "File: " << filename << " successfully open." << std::endl;

  // Line, header and data
  std::string line;
  std::vector<std::string> header;
  std::vector<std::vector<double>> data;

  // Indices (indices of the header corresponding to the defined convention)
  std::vector<int> indices;

  // rows counter
  int rows_cnt = 0;

  // Read the column names
  if (file.good())
  {
    // Extract the header (supposed to be the first line) in the file
    std::getline(file, line);

    // Parse the header
    parseLine(line, header);

    // Read data, line by line
    while (std::getline(file, line)) {
      std::vector<double> tmp;
      parseLine(line, tmp);
      data.push_back(tmp);
      ++rows_cnt;
    }

    // Get association (indices) based on the defined convention
    getIndices(header, indices);

    // clear data structure from previous data
    data_.clear();

    // Loop through the data (lines) and fill the data structure
    for (const auto &it : data) {

      // Temporary input data structure
      Input tmp;

      // Fill out temporary input data structure
      tmp.t = it.at(indices.at(0));
      tmp.p << it.at(indices.at(1)), it.at(indices.at(2)), it.at(indices.at(3));
      tmp.q.coeffs() << it.at(indices.at(4)), it.at(indices.at(5)), it.at(indices.at(6)), it.at(indices.at(7));
      tmp.w << it.at(indices.at(8)), it.at(indices.at(9)), it.at(indices.at(10));
      tmp.a << it.at(indices.at(11)), it.at(indices.at(12)), it.at(indices.at(13));

      data_.push_back(tmp);
    }
  }

  file.close();
  std::cout << "File read successfully." << std::endl << "----------------------------------------\n" << std::endl;
}

const std::vector<InputDataParser::Input> &InputDataParser::getData() const {
  if(!data_.empty()) {
    return data_;
  } else {
    throw std::runtime_error("Trying to get data from empty structure, something went wrong when parsing .csv input file...");
  }
}
