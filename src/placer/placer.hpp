#pragma once

class Design;

class Placer {
 public:
  static double CalculateHPWL(const Design& design);
  static double CalculateCongestionCoefficient(const Design& design);

  static void InitPlace(Design& design);
  static void RunSA(Design& design);
};