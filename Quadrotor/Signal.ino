
#include "Signal.h"

// Low pass filter

float Signal::compute_LPF(float prev, float current, float alpha){
   return alpha *current + (1 - alpha)*prev;
}

// Derivative

float Signal::compute_derivative(float prev, float current, float delta_t){
  return (current-prev)*1000000/delta_t;
}

void Signal::update(float new_value, float delta_t){
    raw=new_value;
    float new_filtered = compute_LPF(value,new_value,raw_alpha);
    float new_derivative = compute_derivative(value, new_filtered, delta_t);
    derivative = compute_LPF(derivative, new_derivative, derivative_alpha);
    value = new_filtered;
}
