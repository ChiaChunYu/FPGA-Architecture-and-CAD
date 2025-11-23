#include <chrono>
#include <iostream>
#include <stdexcept>

#include "design/design.hpp"
#include "parser/parser.hpp"
#include "placer/placer.hpp"
#include "writer/writer.hpp"

int main(int argc, char* argv[]) {
  if (argc != 3) {
    throw std::runtime_error("Usage: ./placer <input_file> <output_file>");
  }
  auto startTime = std::chrono::high_resolution_clock::now();
  std::string inputFile = argv[1];
  std::string outputFile = argv[2];
  Design design;
  Parser::parse(inputFile, design);
  Placer::RunSA(design);
  Writer::write(outputFile, design);
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = endTime - startTime;
  std::cout << "Total runtime: " << elapsed.count() << " seconds." << std::endl;
  return 0;
}