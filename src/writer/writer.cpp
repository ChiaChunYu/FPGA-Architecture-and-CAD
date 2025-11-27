#include "writer.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../design/design.hpp"

void ExportToSVG(const Design& design, const std::string& filename) {
  std::ofstream out(filename);
  if (!out) {
    std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
    return;
  }

  int w = design.chip_width();
  int h = design.chip_height();

  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
      << "viewBox=\"0 0 " << w << " " << h << "\" "
      << "width=\"600\" height=\"600\" "
      << "style=\"background-color: white;\">\n";

  // For each LogicBlock, draw a rectangle
  for (auto block : design.logic_blocks()) {
    out << "<rect x=\"" << block->x() << "\" y=\"" << block->y() << "\" "
        << "width=\"1\" height=\"1\" "
        << "fill=\"none\" stroke=\"blue\" stroke-width=\"0.05\" />\n";
  }

  // For each IOPin, draw a small circle
  for (auto pin : design.io_pins()) {
    out << "<circle cx=\"" << pin->x() << "\" cy=\"" << pin->y() << "\" "
        << "r=\"0.5\" fill=\"red\" />\n";
  }

  for (auto net : design.nets()) {
    bool has_pin = false;
    if (net->pins().empty()) continue;  // Only draw Nets connected to IO
    auto pin = net->pins()[0];
    for (auto block : net->blocks()) {
      out << "<line x1=\"" << pin->x() << "\" y1=\"" << pin->y() << "\" "
          << "x2=\"" << block->x() << "\" y2=\"" << block->y() << "\" "
          << "stroke=\"green\" stroke-width=\"0.1\" opacity=\"0.5\" />\n";
    }
  }

  out << "</svg>";
  out.close();
  std::cout << "[Output] SVG visualization saved to " << filename << std::endl;
}

// Export Usage Map (congestion heatmap) to SVG
void ExportUsageToSVG(const Design& design, const std::vector<std::vector<int>>& usage_map, const std::string& filename) {
  std::ofstream out(filename);
  if (!out) {
    std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
    return;
  }

  int w = design.chip_width();
  int h = design.chip_height();

  // Find the maximum Usage value for color normalization
  int max_usage = 0;
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      if (usage_map[x][y] > max_usage) {
        max_usage = usage_map[x][y];
      }
    }
  }
  if (max_usage == 0) max_usage = 1;

  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
      << "viewBox=\"0 0 " << w << " " << h << "\" "
      << "width=\"600\" height=\"600\" "
      << "style=\"background-color: white;\">\n";

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      int val = usage_map[x][y];
      if (val == 0) continue;

      double ratio = static_cast<double>(val) / max_usage;

      int color_val = static_cast<int>(255 * (1.0 - ratio));

      out << "<rect x=\"" << x << "\" y=\"" << y << "\" "
          << "width=\"1\" height=\"1\" "
          << "fill=\"rgb(255," << color_val << "," << color_val << ")\" "
          << "stroke=\"none\" />\n";
    }
  }

  for (auto block : design.logic_blocks()) {
    out << "<rect x=\"" << block->x() << "\" y=\"" << block->y() << "\" "
        << "width=\"1\" height=\"1\" "
        << "fill=\"none\" stroke=\"black\" stroke-width=\"0.05\" opacity=\"0.2\" />\n";
  }

  out << "</svg>";
  out.close();
  std::cout << "[Output] Usage Map visualization saved to " << filename << " (Max Usage: " << max_usage << ")" << std::endl;
}

void Writer::Write(const std::string& filename, const Design& design) {
  std::ofstream outfile(filename);

  if (!outfile.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filename);
  }

  for (const auto& block : design.logic_blocks()) {
    outfile << block->name() << " " << block->x() << " " << block->y() << "\n";
  }
  ExportToSVG(design, filename + ".svg");
  ExportUsageToSVG(design, design.GetUsageMap(), filename + "_usage.svg");
}