#pragma once
#include <string>

class LogicBlock;
class IOPin;
class Net;
class Design;

class Parser {
 public:
  static void Parse(const std::string& filename, Design& design);
};