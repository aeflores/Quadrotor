
#ifndef SIGNAL_H_
#define SIGNAL_H_

class Signal{
private:
    float raw_alpha, derivative_alpha;

    float compute_LPF(float prev,float current, float alpha);
    float compute_derivative(float prev,float current, float delta_t);

public:
    float raw=0,value=0, derivative=0;
    Signal(float raw_alpha, float derivative_alpha): 
        raw_alpha(raw_alpha),
        derivative_alpha(derivative_alpha){};

    void update(float new_value, float delta_t);


};

#endif
