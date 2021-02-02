#include "RNG.h"

unsigned long RNG::Randint(int low, int high)
{
    return (unsigned long) (gsl_rng_uniform(gr) * (high - low) + low
        + 0.5); // +0.5 para que al truncar tanto low como high est��n incluidos
}

double RNG::Randfloat(double low, double high)
{
    return (gsl_rng_uniform(gr) * (high - low) + low);
}

void RNG::change(long int seed)
{
    gsl_rng_set(gr, seed);
    _seed = seed;
}