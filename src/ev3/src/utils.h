#pragma once

#include <unistd.h>
#include <math.h>

#define DEBUG false

#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

inline void Sleep(uint32_t msec)
{
    usleep(( msec ) * 1000);
}

inline double scale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline double smooth(double input, double factor, double output)
{
    if (factor > 1){      // check to make sure param's are within range
        factor = .999;
    }
    else if (factor <= 0){
        factor = 0;
    }

    output = (input * (1 - factor)) + (output  *  factor);

    return output;
}

inline double rangeToCm(double range)
{
    return 62749*pow(range, -1.377);
}

//Low pass chebyshev filter order=1 alpha1=0.33333333333333
class  FilterChLp1
{
    public:
        FilterChLp1()
        {
            v[0]=0.0;
        }
    private:
        double v[2];
    public:
        double step(double x) //class II
        {
            v[0] = v[1];
            v[1] = (6.345254188091803416e-1 * x)
                 + (-0.26905083761836068312 * v[0]);
            return
                 (v[0] + v[1]);
        }
};
