#include "parser.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "../design/design.hpp"

void Parser::parse(const std::string& filename, Design& design) {
  std::ifstream infile(filename);
  if (!infile.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filename);
  }

  int rows = 0, cols = 0;
  int num_logic_blocks = 0;
  int num_io_pins = 0;
  int num_nets = 0;

  if (!(infile >> rows >> cols >> num_logic_blocks >> num_io_pins >> num_nets)) {
    throw std::runtime_error("Error: Invalid header format in " + filename);
  }

  design.set_chip_dimensions(cols, rows);

  std::unordered_map<std::string, LogicBlock*> blockMap;
  blockMap.reserve(num_logic_blocks);

  std::unordered_map<std::string, IOPin*> pinMap;
  pinMap.reserve(num_io_pins);

  for (int i = 0; i < num_logic_blocks; ++i) {
    std::string name;
    infile >> name;

    LogicBlock* newBlock = new LogicBlock(name);

    design.AddLogicBlock(newBlock);

    blockMap[name] = newBlock;
  }

  for (int i = 0; i < num_io_pins; ++i) {
    std::string name;
    double x, y;
    infile >> name >> x >> y;

    IOPin* newPin = new IOPin(name, x, y);

    design.AddIOPin(newPin);

    pinMap[name] = newPin;
  }

  for (int i = 0; i < num_nets; ++i) {
    std::string name;
    int degree;
    infile >> name >> degree;

    Net* newNet = new Net(name, degree);

    for (int j = 0; j < degree; ++j) {
      std::string termName;
      infile >> termName;
      auto blockIt = blockMap.find(termName);
      if (blockIt != blockMap.end()) {
        LogicBlock* lb = blockIt->second;
        newNet->AddBlock(lb);
        lb->AddNet(newNet);
      } else {
        auto pinIt = pinMap.find(termName);
        if (pinIt != pinMap.end()) {
          IOPin* pin = pinIt->second;
          newNet->AddPin(pin);
        } else {
          throw std::runtime_error("Error: Unknown terminal '" + termName + "' in net '" + name + "'");
        }
      }
    }
    design.AddNet(newNet);
  }
}