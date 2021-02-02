#pragma once
#include "NeighborSearch.h"
#include "MCGRP.h"
#include <vector>
#include <thread>
#include <future>
#include <functional>
#include <queue>

using std::vector;

//several distance metrics to evalute the similarity among solutions
enum SIMILARITY
{
    COST_DISTANCE,
    HAMMING_DISTANCE,
    RECOVER_DISTANCE
};

class Memetic
{
private:
    vector<vector<int>> population;                    //solutions in negative coding format, population[种群数][该解的遍历序列]

    vector<vector<double>> distMatrix;                    //距离矩阵，存放各个解之间的距离, distMatrix[种群大小][种群大小]
    vector<double> indi_obj;                    //各解的目标函数值, indi_obj[种群数]
    int popSize;

    int evolve_num;
    int cross_route_num = 1;
    double QNDF_weight;


public:
    NeighBorSearch &ns;

    Memetic(NeighBorSearch &local_search, const int pool_size, const int evolve_steps, int _QNDF_weight);

    void reset();

    void memetic_search(const MCGRP &mcgrp);

    /*!
     * @details initialize the pool
     * @param mcgrp
     */
    void init_population(const MCGRP &mcgrp);

    /*!
     * @details check whether used based similarity metric
     * @param size
     * @param tested_value
     * @param sim
     * @return
     */
    bool is_used(const double tested_value, SIMILARITY sim);

    /*!
     * @details get hamming distance between two solution
     * @param mcgrp
     * @param a
     * @param b
     * @return
     */
    static double hamming_dist(const MCGRP &mcgrp, const vector<int> &a, const vector<int> &b);

    /*!
     * @details remove duplicate task and statistic ommited task
     * @param mcgrp
     * @param XRoutes
     * @param Omitted_task
     */
    void remove_duplicates(const MCGRP &mcgrp, vector<vector<int>> &XRoutes, vector<int> &Omitted_task);

    /*!
     * @details generate distance matrix based on hamming diatance
     * @param mcgrp
     */
    void initDistMatrix(const MCGRP &mcgrp);

    /*!
     * compress routes
     * @param XRoutes
     */
    static void remove_empty_routes(vector<vector<int>> &XRoutes);

    static vector<int> cal_loads(const MCGRP &mcgrp, const vector<vector<int>> &XRoutes);

    /*!
     * @details parse the chosen pool solution
     * @param mcgrp
     * @param p
     * @param sol
     * @param val
     */
    static void parse_solution(const MCGRP &mcgrp, Individual &p, const vector<int> &sol, const double val);

    /*!
     * @details route based crossover
     * @param xed_child
     * @param p1
     * @param p2
     * @param mcgrp
     */
    void route_based_crossover(Individual &xed_child, const Individual &p1, const Individual &p2, const MCGRP &mcgrp);

    /*!
     * @details update the pool
     * @param mcgrp
     * @param new_sol
     * @param new_obj
     * @return
     */
    bool poolUpdate(const MCGRP &mcgrp, const vector<int> new_sol, const double new_obj);
};


class HighSpeedMemetic
{
private:

    struct UNIT{
        vector<int> solution;           // negative coding format
        double obj = numeric_limits<double>::max();

        UNIT(){};
        UNIT(double _obj) : obj(_obj){};
    };


    int hardware_threads;
    vector<UNIT> population;
    vector<vector<double>> distMatrix;                //distance among solutions

    int cross_route_num = 1;

    int popSize;
    int evolve_num;
    double QNDF_weight;

    //for concurrency
    condition_variable data_cond;
    queue<UNIT> citizen_buffer;
    mutex buffer_lock;
    void clear_buffer(){
        queue<UNIT> empty;
        swap(empty, citizen_buffer);
    }


    pair<int,int> choose_parents(const MCGRP &mcgrp);

    struct ROUTE{
        vector<int> seq;
        double length = 0;
        double load = 0;
    };

    struct SOLUTION{
        vector<ROUTE> routes;
        double length = 0;
        double load = 0;

        vector<int> get_route_sequence(int route_id,bool with_dummy){
            My_Assert(route_id>=0 && route_id < routes.size(),"Wrong arguments");
            if(with_dummy){
                return routes[route_id].seq;
            }
            else{
                return vector<int>(routes[route_id].seq.begin()+1,routes[route_id].seq.end()-1);
            }
        }
        vector<int> get_whole_sequence(){
            //without dummy
            vector<int> buffer;
            for(int i = 0;i< routes.size();i++){
                auto tmp = get_route_sequence(i, false);
                tmp.front() = -tmp.front();
                buffer.insert(buffer.end(),tmp.begin(),tmp.end());
            }

            return buffer;
        }

    };

    struct THREAD_MAN{
//        mutex thread_info_lock;
        const int max_allowed_activated_thread;
        int total_threads_num = 0;
        int dead_thread_num = 0;
        int activate_thread_num = 0;

        THREAD_MAN(int threads_num):max_allowed_activated_thread(threads_num){};
    } thread_pool;

    static bool check_integrity(const MCGRP &mcgrp, const SOLUTION &solution, const vector<int>& ommited);
    static bool check_empty_route(const MCGRP &mcgrp,const SOLUTION &solution);

    static SOLUTION unpack_seq(const MCGRP &mcgrp, const vector<int> &neg_seq);


public:

    HighSpeedMemetic(HighSpeedNeighBorSearch &local_search, const int pool_size, const int evolve_steps, int _QNDF_weight);

    inline void reset(){
        population.clear();
        distMatrix.clear();
    }

    static bool valid(const MCGRP& mcgrp,const SOLUTION &solution);


    void memetic_search(const MCGRP &mcgrp);

    void search_child(const MCGRP &mcgrp, vector<int> child);

    static void repair_solution(const MCGRP &mcgrp, SOLUTION & solution);

    /*!
     * @details initialize the pool
     * @param mcgrp
     */
    void init_population(const MCGRP &mcgrp);
    void create_citizen(const MCGRP& mcgrp);

    /*!
     * @details check whether used based similarity metric
     * @param size
     * @param tested_value
     * @param sim
     * @return
     */
    bool repeated(const MCGRP &mcgrp, const double tested_value, const vector<int> &immigrant, SIMILARITY sim);

    static void best_insert_repair(const MCGRP &mcgrp,SOLUTION& solution, vector<int> Omitted_task);
    static void merge_split_repair(const MCGRP &mcgrp,SOLUTION& solution, vector<int> Omitted_task);

    /*!
     * @details remove duplicate task and statistic ommited task
     * @param mcgrp
     * @param XRoutes
     * @param Omitted_task
     */
    static vector<int> remove_duplicates(const MCGRP &mcgrp, SOLUTION &solution);

    /*!
     * @details generate distance matrix based on hamming diatance
     * @param mcgrp
     */
    void initDistMatrix(const MCGRP &mcgrp, SIMILARITY sim);

    /*!
     * @details route based crossover
     * @param xed_child
     * @param p1
     * @param p2
     * @param mcgrp
     */
    vector<int> route_based_crossover(const MCGRP &mcgrp, const vector<int>& parent_a_seq,const vector<int>& parent_b_seq);

    /*!
     * @details update the pool
     * @param mcgrp
     * @param new_sol
     * @param new_obj
     * @return
     */
    bool poolUpdate(const MCGRP &mcgrp, const vector<int> new_sol, const double new_obj);
};


