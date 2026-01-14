#include "autotune.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "AUTOTUNE";

// e = {NegativeBig, NegativeSmall, Zero, PositiveSmall, PositiveBig} -> {0, 1, 2, 3, 4}
// e constrains [-10, 10]
// de = {DecreaseFast, DecreaseSlow, NoChange, IncreaseSlow, IncreaseFast} -> {0, 1, 2, 3, 4}
// de constrains [-5, 5]
// Output of Fuzzy: K'p, K'i, K'd = {Small, MediumSmall, Medium, MediumBig, Big} -> {0, 1, 2, 3, 4}
// Fuzzy rule:
/*
    |     E/De      | NegativeBig   | NegativeSmall |   Zero        | PositiveSmall | PositiveBig   |
    |Negative       |     Big       |   Medium       | Medium        |   Big         |    Big        |
    |Zero           |     Medium    |    Medium     | Small         |   Medium      |   Medium      |
    |Positive       |     Big       |    Medium     |   Medium      |    Big        |    Big         |
*/
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x == b) return 1;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}
float leftRamp(float x, float a, float b)
{
    if (x <= a)
        return 1.0f;
    else if (x >= b)
        return 0.0f;
    else
        return (b - x) / (b - a);
}

float rightRamp(float x, float a, float b)
{
    if (x <= a)
        return 0.0f;
    else if (x >= b)
        return 1.0f;
    else
        return (x - a) / (b - a);
}
ErrorMF fuzzifyError(float e) {
  ErrorMF mf;
  mf.N = leftRamp(e, -1, -0);
  mf.Z = triangle(e, -1, 0, 1);
  mf.P = rightRamp(e, 0, 1);
  return mf;
}
DerivativeMF fuzzifyDE(float de) {
  DerivativeMF mf;
  mf.NB_ = leftRamp(de, -20, -10);
  mf.NS_ = triangle(de, -20, -10, -5);
  mf.Z_  = triangle(de, -5, 0, 5);
  mf.PS_ = triangle(de, 5, 10, 20);
  mf.PB_ = rightRamp(de, 10, 20);
  return mf;
}

OutputMF fuzzifyOutput(float out) {
  OutputMF mf;
  mf.S  = triangle(out, -100, -50, 0);
  mf.M  = triangle(out, -50, 0, 50);
  mf.B  = triangle(out, 0, 50, 100);
  return mf;
}

OutputMF inference(ErrorMF e, DerivativeMF de) {
  OutputMF out = {0, 0, 0};

  // Rule table (E rows x De columns):
  // E = Negative: NB->Big, NS->Medium, Z->Medium, PS->Big, PB->Big
  out.B = fmaxf(out.B, fminf(e.N, de.NB_));
  out.M = fmaxf(out.M, fminf(e.N, de.NS_));
  out.M = fmaxf(out.M, fminf(e.N, de.Z_));
  out.B = fmaxf(out.B, fminf(e.N, de.PS_));
  out.B = fmaxf(out.B, fminf(e.N, de.PB_));  

  // E = Zero: NB->Medium, NS->Medium, Z->Small, PS->Medium, PB->Medium
  out.M = fmaxf(out.M, fminf(e.Z, de.NB_));
  out.M = fmaxf(out.M, fminf(e.Z, de.NS_));
  out.S = fmaxf(out.S, fminf(e.Z, de.Z_));
  out.M = fmaxf(out.M, fminf(e.Z, de.PS_));
  out.M = fmaxf(out.M, fminf(e.Z, de.PB_));

  // E = Positive: NB->Big, NS->Medium, Z->Medium, PS->Big, PB->Big
  out.B = fmaxf(out.B, fminf(e.P, de.NB_));
  out.M = fmaxf(out.M, fminf(e.P, de.NS_));
  out.M = fmaxf(out.M, fminf(e.P, de.Z_));
  out.B = fmaxf(out.B, fminf(e.P, de.PS_));
  out.B = fmaxf(out.B, fminf(e.P, de.PB_));

  return out;
}
float defuzzify(OutputMF out, float left, float center, float right) {
  // Centroid method using the center points from fuzzifyOutput() membership functions
  // S: triangle(-100, -50, 0) -> center at -50
  // M: triangle(-50, 0, 50) -> center at 0
  // B: triangle(0, 50, 100) -> center at 50
  float numerator =
      out.S * (left) +
      out.M * (center) +
      out.B * (right);

  float denominator = out.S + out.M + out.B;
  if (denominator == 0) return 0;
  return numerator / denominator;
}