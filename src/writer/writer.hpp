#pragma once
#include <string>

class Design;

class Writer {
 public:
  static void Write(const std::string& filename, const Design& design);
};