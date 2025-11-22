#include "design.hpp"

// ==========================================
// LogicBlock Implementation
// ==========================================

LogicBlock::LogicBlock(const std::string& name) : name_(name), x_(0), y_(0) {}

void LogicBlock::AddNet(Net* net) {
  nets_.push_back(net);
}

// ==========================================
// IOPin Implementation
// ==========================================

IOPin::IOPin(const std::string& name, double x, double y)
    : name_(name), x_(x), y_(y) {}

void IOPin::AddNet(Net* net) {
  nets_.push_back(net);
}

// ==========================================
// Net Implementation
// ==========================================

Net::Net(const std::string& name, int degree) : name_(name), degree_(degree) {
  blocks_.reserve(degree);
  pins_.reserve(degree);
}

void Net::AddBlock(LogicBlock* block) {
  blocks_.push_back(block);
}

void Net::AddPin(IOPin* pin) {
  pins_.push_back(pin);
}

// ==========================================
// Design Implementation
// ==========================================

Design::Design() : chip_width_(0), chip_height_(0) {}

void Design::set_chip_dimensions(int width, int height) {
  chip_width_ = width;
  chip_height_ = height;
}

void Design::set_chip_height(int height) {
  chip_height_ = height;
}

void Design::set_chip_width(int width) {
  chip_width_ = width;
}

void Design::AddLogicBlock(LogicBlock* block) {
  logic_blocks_.push_back(block);
}

void Design::AddIOPin(IOPin* pin) {
  io_pins_.push_back(pin);
}

void Design::AddNet(Net* net) {
  nets_.push_back(net);
}