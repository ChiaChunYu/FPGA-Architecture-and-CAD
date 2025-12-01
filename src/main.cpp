#include <chrono>
#include <iostream>
#include <stdexcept>

#include "design/design.hpp"
#include "parser/parser.hpp"
#include "placer/placer.hpp"
#include "writer/writer.hpp"

int main(int argc, char* argv[]) {
  auto execution_start = std::chrono::steady_clock::now();
  auto startTime = std::chrono::high_resolution_clock::now();

  if (argc != 3) {
    throw std::runtime_error("Usage: ./placer <input_file> <output_file>");
  }

  std::string inputFile = argv[1];
  std::string outputFile = argv[2];
  std::string baseName = outputFile;
  std::size_t dotPos = baseName.find_last_of('.');
  if (dotPos != std::string::npos) {
    baseName = baseName.substr(0, dotPos);
  }
  std::string svg_initial = baseName + "_initial" + ".svg";
  std::string svg_final = baseName + "_final" + ".svg";

  Design design;
  Parser::Parse(inputFile, design);
  Config config;
  Placer placer(design, config, execution_start);
  placer.InitPlace();
  Writer::ExportDesignToSVG(design, svg_initial);
  placer.Run();
  Writer::ExportDesignToSVG(design, svg_final);
  Writer::Write(outputFile, design);

  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = endTime - startTime;
  std::cout << "Total runtime: " << elapsed.count() << " seconds." << std::endl;
  return 0;
}