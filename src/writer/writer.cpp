#include "writer.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../design/design.hpp"

void Writer::write(const std::string& filename, const Design& design) {
  std::ofstream outfile(filename);

  if (!outfile.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filename);
  }

  for (const auto& block : design.logic_blocks()) {
    outfile << block->name() << " " << block->x() << " " << block->y() << "\n";
  }
}