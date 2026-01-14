#ifndef AUTOTUNE_H
#define AUTOTUNE_H
#include "esp_err.h"


float triangle(float x, float a, float b, float c);

typedef struct {
  float N;
  float Z; // Zero
  float P;
} ErrorMF;

typedef struct {
  float NB_; // Negative Big
  float NS_; // Negative Small
  float Z_;  // Zero
  float PS_; // Positive Small
  float PB_; // Positive Big
} DerivativeMF;

typedef struct {
  float S;
  float M;
  float B;
} OutputMF;

ErrorMF fuzzifyError(float e);
DerivativeMF fuzzifyDE(float de);
OutputMF fuzzifyOutput(float out);
OutputMF inference(ErrorMF e, DerivativeMF de);

float defuzzify(OutputMF out, float left, float center, float right);
#endif // AUTOTUNE_H