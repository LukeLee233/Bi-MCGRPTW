#pragma once
#include <array>
#include <dirent.h>
#include <sys/types.h>
#include <vector>
#include <limits>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include "RNG.h"
#include <stack>
#include <sys/timeb.h>

#define seed_size 100

#ifdef DEBUG
#   define My_Assert(Expr, Msg) \
    __My_Assert(#Expr, Expr, __FILE__, __LINE__, Msg)

#   define DEBUG_PRINT(stuff) std::cout<<(stuff)<<endl
#else
#   define My_Assert(Expr, Msg) ;

#   define DEBUG_PRINT(stuff) ;
#endif

using std::ofstream;

extern ofstream result_out;

extern ofstream log_out;

static std::array<int, seed_size>
    seed = {12345678, 23456781, 34567812, 45678123, 56781234, 67812345, 78123456, 81234567,
            12435678, 24356781, 43567812, 35678124, 56781243, 67812435, 78124356, 81243567,
            18435672, 84356721, 43567218, 35672184, 56721843, 67218435, 72184356, 21843567,
            18437652, 84376521, 43765218, 37652184, 76521843, 65218437, 52184376, 21843765,
            18473652, 84736521, 47365218, 73652184, 36521847, 65218473, 52184736, 21847365,
            15473682, 54736821, 47368215, 73682154, 36821547, 68215473, 82154736, 21547368,
            15472683, 54726831, 47268315, 72683154, 26831547, 68315472, 83154726, 31547268,
            65472183, 54721836, 47218365, 72183654, 21836547, 18365472, 83654721, 36547218,
            35472186, 54721863, 47218635, 72186354, 21863547, 18635472, 86354721, 63547218,
            35427186, 54271863, 42718635, 27186354, 71863542, 18635427, 86354271, 63542718,
            36427185, 64271853, 42718536, 27185364, 71853642, 18536427, 85364271, 53642718,
            36428175, 64281753, 42817536, 28175364, 81753642, 17536428, 75364281, 53642817,
            36528174, 65281743, 52817436, 28174365};

const static int inf = std::numeric_limits<int>::max();

/*!
 * @details a more powerful assert function:)
 * @param expr_str
 * @param expr
 * @param file
 * @param line
 * @param msg
 */
void __My_Assert(const char *expr_str, bool expr, const char *file, int line, const char *msg);

bool print(std::ostream &os1, std::ostream &os2, const std::string &str);
bool print(std::ostream &os1, const std::string &str);

extern std::array<int, seed_size> seed;


struct instance_num_information
{
    int node_num;
    int edge_num;
    int arc_num;
    int req_edge_num;
    int req_arc_num;
    int req_node_num;
};

enum NeighborOperator
{
    SINGLE_INSERT, DOUBLE_INSERT, SWAP, TWO_OPT, INVERT,
    PRESERT, POSTSERT, MOVE_STRING, PRE_MOVE_STRING,
    POST_MOVE_STRING, SLICE, PRE_SLICE, POST_SLICE,
    EXTRACTION, FLIP, SWAP_ENDS
};


struct MCGRPRoute
{
    int ID = -1;
    int start = -1;
    int end = -1;
    int load = 0;
    double length = 0;
    int num_customers = 0;

    void clear(){
        start = -1;
        end = -1;
        load = 0;
        length = 0;
        num_customers = 0;
    }
    static double accumulate_load(double accumulator, const MCGRPRoute &a)
    {
        return accumulator + a.length;
    };
};

struct MCGRPNeighborInfo
{
public:
    int task_id;
    double distance;

    ~MCGRPNeighborInfo()
    {};
    static bool cmp(const MCGRPNeighborInfo &a, const MCGRPNeighborInfo &b)
    {
        return a.distance < b.distance;
    };
};

struct task
{
    int head_node;
    int tail_node;
    int trave_cost;
    int serv_cost;
    int demand;
    int inverse;
};

struct arc
{
    int tail_node;
    int head_node;
    int trav_cost;
};

struct Individual
{
    std::vector<int> sequence;
    std::vector<int> route_seg_load;
    double total_cost;
    int total_vio_load;
};

struct POPOrder
{
    double val;
    int posi;
};

std::vector<std::string> read_directory(const std::string dir_path);

/*!
 * @details find duplicate elements position
 * @param positions
 * @param seq
 * @param target
 */
vector<int> find_ele_positions(const std::vector<int> &seq, int target);

/*!
 * select number based on probability
 * @param prob probability vector
 * @param ratio ratio vector
 * @param rng random generator
 * @return selected number
 */
double sel_ratio(const std::vector<double> &_prob, const std::vector<double> &ratio, RNG &rng);

//judge whether two float numbers are close
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T a, T b)
{
    return fabs(a - b) <= std::numeric_limits<T>::epsilon();
}

template<typename T>
struct identity
{
    typedef T type;
};

class Generator
{
private:
    int _count = 0;
public:
    Generator(int count = 0): _count(count){};

    int operator()()
    {
        return ++_count;
    }
};

/*!
 * @details convert  delimiter coding to negative coding
 * @param sequence : delimiter coding
 * @return  negative coding
 */
vector<int> get_negative_coding(const vector<int> &sequence);

/*!
 * @details convert negative coding to delimiter coding
 * @param negative_coding : negative coding
 * @return  delimiter coding
 */
vector<int> get_delimiter_coding(const vector<int> &negative_coding);


class MCGRPMOVE
{
public:
    //pairwise tasks in a moving action
    int task1;
    int task2;

    //This info is only used in swap ends operation
    int seq1_cus_num = -1;
    int seq2_cus_num = -1;

    bool considerable = false;
    int total_number_of_routes;
    double delta;

    double new_total_route_length;
    double vio_load_delta;
    vector<int> move_arguments;

    int num_affected_routes;
    vector<int> route_loads;
    vector<int> route_id;
    vector<int> route_custs_num;

    vector<double> route_lens;
    NeighborOperator move_type;


    MCGRPMOVE()
        : task1(-1), task2(-1), num_affected_routes(-1)
    {
        considerable = false;
        delta = 0;
        seq1_cus_num = -1;
        seq2_cus_num = -1;
        new_total_route_length =
            std::numeric_limits<identity<decltype(MCGRPMOVE::new_total_route_length)>::type>::max();

        vio_load_delta = 0;
    };

    MCGRPMOVE(NeighborOperator _move_type)
        : task1(-1), task2(-1), num_affected_routes(-1)
    {
        considerable = false;
        delta = 0;
        new_total_route_length =
            std::numeric_limits<identity<decltype(MCGRPMOVE::new_total_route_length)>::type>::max();
        move_type = _move_type;
        seq1_cus_num = -1;
        seq2_cus_num = -1;
        vio_load_delta = 0;
    }

    void choose_tasks(int _task1, int _task2)
    {
        reset();
        task1 = _task1;
        task2 = _task2;
    }

    void reset()
    {
        if(considerable){
            task1 = -1;
            task2 = -1;
            seq1_cus_num = -1;
            seq2_cus_num = -1;
            total_number_of_routes = 0;
            delta = 0;
            vio_load_delta = 0;
            new_total_route_length =
                std::numeric_limits<identity<decltype(MCGRPMOVE::new_total_route_length)>::type>::max();
            move_arguments.clear();

            num_affected_routes = -1;
            route_loads.clear();
            route_id.clear();
            route_custs_num.clear();

            route_lens.clear();
            considerable = false;
        }

    }

};

inline double get_time_difference(const timeb& start,const timeb& end){
    return (end.time - start.time) + ((end.millitm - start.millitm) * 1.0 / 1000);
}