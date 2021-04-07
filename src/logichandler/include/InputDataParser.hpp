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

#ifndef INPUTDATAPARSER_HPP
#define INPUTDATAPARSER_HPP

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <Eigen/Eigen>
#include <algorithm>

/**
* @brief Input data parser for FlightGoggles.
*
* This class has a series of functions that allows to generate
* data starting from a trajectory file generated through the
* MultiCopter simulation framework.
*
*/
class InputDataParser {

public:

  /**
  * @brief Input struct to parse a single line of a .csv file
  *
  * timestamp (t)
  * position (p)
  * rotation - quaternion (q) [defined as: x = R(q)x_b]
  * angular velocity (w)
  * acceleration (a)
  */
  struct Input {
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;
  };

  /**
  * @brief constructor
  */
  InputDataParser();

  /**
  * @brief Clear actual data, read a new .csv file and convert to a matrix (vector of vectors)
  *
  * @param filename
  */
  void readParseCsv(const std::string filename);

  /**
  * @brief Get Data red from .csv file
  */
  const std::vector<Input> &getData() const;


  /**
  * @brief Parse a single line of the .csv file
  *
  * overloaded function to parse a single line of a .csv
  * file with a comma as delimeter.
  * This function is overloaded to include either string values
  * (usually the case for headers) or numerical values
  */
  void parseLine(std::string &line, std::vector<std::string> &data);

  template <typename T>
  void parseLine(std::string &line, std::vector<T> &data) {

    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Temporary value
    T tmp;

    // Extract each cell
    while (ss >> tmp) {
      data.push_back(tmp);

      // skip commas
      if(ss.peek() == ',') ss.ignore();
    }
  }

  /**
  * @brief Find association between input file and defined convention
  *
  * The defined convention of the Input structure is -- t,p,q,w,a --
  * This function find the indices of the columns of the input file based
  * on its header in order to correctly associate input data with the
  * Input structure allowing inpput files with shuffled columns or even
  * more columns than the onse that are necessary
  */
  void getIndices(const std::vector<std::string> &header, std::vector<int> &indices);

  /**
  * @brief Find the index of token within the given vector
  */
  template <typename T>
  void getIndex(const std::vector<T> &data, const T &token, int &index) {

    // Iterator
    auto it = find(data.begin(), data.end(), token);

    // Check if element was found
    if (it != data.end())
    {
      // Get the index
      index = it - data.begin();
    }
    else {
      throw std::runtime_error("Required data missing. Exit programm.");
    }
  }


private:

  /**
  * @brief Raw data from a .csv file converted to a matrix (vector of inputs)
  */
  std::vector<Input> data_;

  /**
  * @brief vector of strings in header ordered based on defined convention -- t,p,q,w,a -
  */
  std::vector<std::string> categories_ = {"Time", "x", "y", "z", "qx", "qy", "qz", "qw", "vroll", "vpitch", "vyaw", "ax", "ay", "az"};

};

#endif //INPUTDATAPARSER_HPP
