#pragma once
#include <gsl/gsl_rng.h>
#include <gsl/gsl_combination.h>
#include <cstdio>
#include <vector>
#include <cassert>

using namespace std;

class RNG
{
    const gsl_rng_type *GRT;
    gsl_rng *gr;
    long int initial_seed;
    long int _seed;

public:

    RNG(long int seed = 0)
    {
        gsl_rng_env_setup();
        GRT = gsl_rng_default;
        gr = gsl_rng_alloc(GRT);
        gsl_rng_set(gr, seed);
        _seed = seed;
        initial_seed = seed;
    }

    ~RNG()
    {
        gsl_rng_free(gr);
    }

    /*!
     * 生成[low,high]的随机整数
     * @param low
     * @param high
     * @return
     */
    unsigned long Randint(int low, int high);

    /*!
     * 生成[low,high]的随机浮点数
     * @param low
     * @param high
     * @return
     */
    double Randfloat(double low, double high);

    /*!
     * 对数组进行洗牌
     * @param perm
     */
    template<typename T>
    void RandPerm(std::vector<T> &perm)
    {
        for (int i = 0; i < perm.size(); i++) {
            int change = this->Randint(0, perm.size() - 1);
            swap(perm[i],perm[change]);
        }
    }

    /* 改变随机数种子 */
    void change(long int seed);

    void inline show_seed()
    {
        printf("random seed is : %ld\n", _seed);
    }

    inline void restore()
    {
        gsl_rng_set(gr, initial_seed);
        _seed = initial_seed;
    }
};

class COMB
{
    gsl_combination *comb;
    vector<int> combinations;
    bool available;
public:
    COMB(const int c, const int r)
    {
        assert(c >= r);
        comb = gsl_combination_calloc(c, r);
        combinations.reserve(r);
        combinations.clear();
        available = true;
    }

    ~COMB()
    {
        gsl_combination_free(comb);
    };

    bool get_combinations(vector<int> &buffer)
    {
        if (available) {
            combinations.clear();
            for (auto i = 0; i < gsl_combination_k(comb); i++) {
                combinations.push_back(gsl_combination_get(comb, i));
            }
            buffer = combinations;
            available = (gsl_combination_next(comb) == GSL_SUCCESS);
            return true;
        }
        else
            return false;
    }

};