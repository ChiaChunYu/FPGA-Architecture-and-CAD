#include "writer.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "../design/design.hpp"

void Writer::ExportDesignToSVG(const Design& design, const std::string& filename) {
  std::ofstream out(filename);

  if (!out) {
    std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
    return;
  }

  int width = design.chip_width();
  int height = design.chip_height();
  const auto usage_map = design.GetUsageMap();
  const auto& blocks = design.GetGridGraph();
  int max_usage = 0;

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      if (usage_map[x][y] > max_usage) {
        max_usage = usage_map[x][y];
      }
    }
  }

  if (max_usage == 0) max_usage = 1;

  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
      << "viewBox=\"0 0 " << width << " " << height << "\" "
      << "width=\"600\" height=\"600\" "
      << "style=\"background-color: white;\">\n";

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      int val = usage_map[x][y];
      double ratio = static_cast<double>(val) / max_usage;
      int color_val = static_cast<int>(255 * (1.0 - ratio));

      out << "<rect x=\"" << x << "\" y=\"" << y << "\" "
          << "width=\"1\" height=\"1\" "
          << "fill=\"rgb(255," << color_val << "," << color_val << ")\" "
          << "stroke=\"none\" "
          << "shape-rendering=\"crispEdges\" />\n";
    }
  }

  const std::string line_style = "stroke=\"black\" stroke-width=\"0.05\" stroke-opacity=\"0.8\" stroke-linecap=\"square\"";

  for (int x = 0; x <= width; ++x) {
    for (int y = 0; y < height; ++y) {
      bool left_has_block = (x > 0) && (blocks[x - 1][y] != nullptr);
      bool right_has_block = (x < width) && (blocks[x][y] != nullptr);

      if (left_has_block || right_has_block) {
        out << "<line x1=\"" << x << "\" y1=\"" << y << "\" " << "x2=\"" << x << "\" y2=\"" << (y + 1) << "\" " << line_style << " />\n";
      }
    }
  }

  for (int y = 0; y <= height; ++y) {
    for (int x = 0; x < width; ++x) {
      bool top_has_block = (y > 0) && (blocks[x][y - 1] != nullptr);
      bool bottom_has_block = (y < height) && (blocks[x][y] != nullptr);

      if (top_has_block || bottom_has_block) {
        out << "<line x1=\"" << x << "\" y1=\"" << y << "\" " << "x2=\"" << (x + 1) << "\" y2=\"" << y << "\" " << line_style << " />\n";
      }
    }
  }

  out << "</svg>";
  out.close();
}

void Writer::Write(const std::string& filename, const Design& design) {
  std::ofstream outfile(filename);

  if (!outfile.is_open()) {
    throw std::runtime_error("Error: Could not open file " + filename);
  }

  for (const auto& block : design.logic_blocks()) {
    outfile << block->name() << " " << block->x() << " " << block->y() << "\n";
  }
}
