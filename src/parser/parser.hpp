#pragma once
#include <string>

class LogicBlock;
class IOPin;
class Net;
class Design;

class Parser {
 public:
  static void parse(const std::string& filename, Design& design);
};