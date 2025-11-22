#pragma once

class Design;

class Placer {
 public:
  Placer() = default;

  static void Run(Design& design);

  static void InitPlace(Design& design);

  static double CalculateHPWL(const Design& design);

  static double CalculateCongestionCoefficient(const Design& design);
};