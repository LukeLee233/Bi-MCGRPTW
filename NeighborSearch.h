#ifndef _NEIGHBORSEARCH_
#define _NEIGHBORSEARCH_

#include "RNG.h"
#include "utils.h"
#include "MCGRP.h"
#include <vector>
#include <limits>
#include "SearchPolicy.h"
#include "ConstructPolicy.h"
#include <memory>
#include <stack>
#include <unordered_set>
#include "config.h"

extern vector<double> ratios;
extern vector<double> prob;

class NeighBorSearch
{
    friend class Memetic;
    friend class SingleInsert;
    friend class DoubleInsert;
    friend class NewSwap;
    friend class Swap;
    friend class TwoOpt;
    friend class NewTwoOpt;
    friend class Presert;
    friend class Postsert;
    friend class MoveString;
    friend class Invert;
    friend class NewFlip;
    friend class Flip;
    friend class NewSwapEnds;
    friend class SwapEnds;
    friend class Policy;

    friend void merge_split(NeighBorSearch &ns, const MCGRP &mcgrp, const int merge_size,const int pseudo_capacity);
    friend struct RouteSegment get_segment_info(const MCGRP &mcgrp,const NeighBorSearch &ns,const int a,const int b);
private:

    const int significant_search_delta = significant_search;
    const double local_ratio_threshold = local_ratio;
    const double infeasible_distance_threshold = infeasible_distance;


    //dynamic info
    int neigh_size = 0;            //different policy use different neighbor size
    std::vector<int> search_space;


    /* current solution */
    //unpacked info
    vector<MCGRPRoute> routes;    //route info
    vector<int> next_array;    //negative means different route
    vector<int> pred_array;
    vector<int> route_id;     //tell the task which route it belong to

    //move operator
    unique_ptr<class Invert> invert;
    unique_ptr<class SingleInsert> single_insert;
    unique_ptr<class DoubleInsert> double_insert;
    unique_ptr<class NewTwoOpt> two_opt;
    unique_ptr<class NewSwap> swap;

public:
    //This is the format which final solution will use
    vector<int> negative_coding_sol;

    //This is the format which searching process will use
    vector<int> delimiter_coding_sol;
    double cur_solution_cost;
    double total_vio_load;

    Policy policy;

    int search_step; // a valid move operation
    int equal_step; // a valid move operation but lead no change in solution objective

    NeighBorSearch(const MCGRP &mcgrp);
    ~NeighBorSearch();

    int local_minimum_likelihood = 0;
    const int local_threshold = local_minimum_threshold;

    double get_fitness(){
        My_Assert(policy.beta != std::numeric_limits<decltype(policy.beta)>::max(), "beta is undefined!");
        return cur_solution_cost + policy.beta * total_vio_load;
    }

    void _neigh_search(const MCGRP &mcgrp, int mode);

    /*!
     * sub proceduren in decode_seq method
     */
    void create_pred_array();

    /*!
     * @details 根据邻域表创建任务的邻域搜索空间
     * @param mcgrp
     * @param task
     */
    void create_search_neighborhood(const MCGRP &mcgrp, const int task);

    /*!
     * @details pack current sol info to delimeter coding format
     * @param p
     */
    void create_individual(const MCGRP &mcgrp, Individual &p);

    /*!
     * eliminate empty route
     * @param seq
     * @param actual_route_num
     */
    void compress_solution(vector<int> &seq, int actual_route_num);

    /*!
 * @details unpack sol info to negative coding format
 * @param del_seq
 */
    void unpack_seq(const std::vector<int> &del_seq, const MCGRP &mcgrp);

    /*!
     * @details neighbor search
     * @param mcgrp
     * @param max_nonimprove_times
     * @param accept [First accept | Bext accept]
     */
    void neighbor_search(const MCGRP &mcgrp);

    /*!
     * @details downhill search
     * @param mcgrp
     */
    void descent_search(const MCGRP &mcgrp);

    /*!
     * @details random threshold search
     * @param mcgrp
     */
    void RTR_search(const MCGRP &mcgrp);

    /*!
     *  @details threshold exploration
     * @param mcgrp
     */
    void threshold_exploration_version_0(const MCGRP &mcgrp);
    void threshold_exploration_version_1(const MCGRP &mcgrp);

    /*!
     * @details descent exploration
     * @param mcgrp
     */
    void descent_exploration_version_0(const MCGRP &mcgrp);

    void descent_exploration_version_1(const MCGRP &mcgrp);

    /*!
     * @details Infeasible search
     * @param mcgrp
     * @param start_point
     */
    void infeasible_exploration(const MCGRP &mcgrp);

    void small_step_infeasible_descent_search(const MCGRP &mcgrp);
    void small_step_infeasible_tabu_search(const MCGRP &mcgrp);


    void large_step_infeasible_search(const MCGRP &mcgrp);

    /*!
     * @details repair overloaded solution
     * @param mcgrp
     */
    void repair_solution(const MCGRP &mcgrp);

    /*!
     * @details Check if any tasks missed.
     */
    bool check_missed(const MCGRP &mcgrp);

    /*!
     * @details check duplicate tasks
     * @param mcgrp
     * @return
     */
    bool check_duplicated(const MCGRP &mcgrp);

    /*!
     * @details This function inserts a sentinel task BEFORE task i
     * @param mcgrp
     * @param i
     */
    void presert_sentinel(const MCGRP &mcgrp, const int i);

    /*!
     * @details inserts sentinel task after i
     * @param mcgrp
     * @param i
     */
    void postsert_sentinel(const MCGRP &mcgrp, const int i);

    void remove_sentinel(const MCGRP &mcgrp);


};





class HighSpeedNeighBorSearch
{
    friend class Memetic;
    friend class SingleInsert;
    friend class DoubleInsert;
    friend class NewSwap;
    friend class Swap;
    friend class TwoOpt;
    friend class NewTwoOpt;
    friend class Presert;
    friend class Postsert;
    friend class MoveString;
    friend class PreMoveString;
    friend class PostMoveString;
    friend class Slice;
    friend class Preslice;
    friend class Postslice;
    friend class Extraction;
    friend class Invert;
    friend class NewFlip;
    friend class Flip;
    friend class NewSwapEnds;
    friend class SwapEnds;
    friend class Policy;

    friend void merge_split(NeighBorSearch &ns, const MCGRP &mcgrp, const int merge_size,const int pseudo_capacity);
    friend void merge_split(class HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int merge_size, const int pseudo_capacity);
    friend struct RouteSegment get_segment_info(const MCGRP &mcgrp,const NeighBorSearch &ns,const int a,const int b);
    friend struct RouteSegment get_segment_info(const MCGRP &mcgrp,HighSpeedNeighBorSearch &ns,const int chosen_task);
    friend struct seg_info get_seg_info(const MCGRP &mcgrp, HighSpeedNeighBorSearch &ns, const int start_task, const int end_task);

private:

    struct TASK_NODE{
        int ID;
        int route_id;   //tell the task which route it belong to
        TASK_NODE *pre;
        TASK_NODE *next;

        TASK_NODE(int id = 0){
            ID = id;
            route_id = -1;
            pre = nullptr;
            next = nullptr;
        }

        void clear(){
            route_id = -1;
            pre = nullptr;
            next = nullptr;
        }
    };

    struct DUMMYPOOL{
        //This class is used to manage dummy task
        struct DUMMY_TASK{
            bool used;
            TASK_NODE task_info;

            DUMMY_TASK() : used(false){};
        };

        stack<int> unused;
        vector<DUMMY_TASK> pool;

        void extend();

        DUMMYPOOL(int initial_size = 10):pool(initial_size){
            for(int i = pool.size() - 1;i > 0; i--){
                unused.push(-i);
                pool[i].task_info.ID = -i;
            }
        }

        TASK_NODE* get_new_dummy(){
            if(unused.empty()){
//                extend();
                cerr<<"Bad new dummy task!";
                abort();
            }

            int new_dummy_id = unused.top();
            unused.pop();
            My_Assert(new_dummy_id < 0 && !pool[-new_dummy_id].used,"Bad allocate for new dummy task");
            pool[-new_dummy_id].used = true;

            return &pool[-new_dummy_id].task_info;
        }

        void free_dummy(int id){
            My_Assert(id < 0,"Bad free dummy task");

            id = -id;

            if(pool[id].used == false){
                cerr<<"Double free!";
                abort();
            }

            pool[id].used = false;
            pool[id].task_info.clear();

            unused.push(pool[id].task_info.ID);
            return;
        }

        void clear(){
            for(int i = 0;i<pool.size();i++){
                if(pool[i].used){
                    pool[i].used = false;
                    unused.push(pool[i].task_info.ID);
                    pool[i].task_info.clear();
                }
            }

        }

    };

    struct SOLUTION{
        DUMMYPOOL dummypool;

        //for better memory management, use vector to implement double linked list
        vector<TASK_NODE> tasks;

        TASK_NODE *very_start;
        TASK_NODE *very_end;


        SOLUTION(unsigned int task_num): tasks(task_num + 1), dummypool(task_num+1){
            for(int id = 1;id<=task_num;id++){
                tasks[id].ID = id;
            }

            very_start = nullptr;
            very_end = nullptr;
        }

        TASK_NODE* operator[](int id)
        {
            //use ID to locate address
            My_Assert(id != 0,"Wrong ID");
            if(id > 0){
                return &tasks[id];
            }
            else{
                id = -id;
                if(dummypool.pool[id].used == false)
                {
                    cerr << "This dummy task info is unavailble!" <<endl;
                    abort();
                }
                return &dummypool.pool[id].task_info;
            }

        }

        void print(){
            if(very_start == nullptr){
                return;
            }

            TASK_NODE * tmp = very_start;
            do{
                cout<<max (tmp->ID, 0) <<'-';
                tmp = tmp->next;
            }while(tmp != very_end);
            cout<<max (tmp->ID, 0)<<endl;
        }

        void clear(){
            dummypool.clear();
            very_start = nullptr;
            very_end = nullptr;
            for(int i = 0;i<tasks.size();i++){
                tasks[i].clear();
            }
        };
    };

    class ROUTESPOOL{
        struct ROUTE_NODE{
            bool used;
            MCGRPRoute route_info;

            ROUTE_NODE() : used(false){};
        };

        stack<int> unused;
        vector<ROUTE_NODE> pool;

    public:
        unordered_set<int> activated_route_id;
        ROUTESPOOL(int initial_size = 20):pool(initial_size){
            for(int i = pool.size() - 1; i>=0 ;i--){
                unused.push(i);
                pool[i].route_info.ID = i;
            }
        }

        MCGRPRoute* operator[](int id)
        {
            if(pool[id].used == false)
            {
                cerr << "This route info is unavailble!" <<endl;
                abort();
            }

            return &pool[id].route_info;
        }

        int allocate_route(){
            if(unused.empty()){
                cerr<<"Bad allocate!";
                abort();
            }

            int id = unused.top();
            unused.pop();
            pool[id].used = true;
            activated_route_id.insert(id);
            return id;
        }

        void free_route(int id){
            if(pool[id].used == false){
                cerr<<"Double free!";
                abort();
            }

            pool[id].used = false;
            pool[id].route_info.clear();

            unused.push(id);
            auto succed = activated_route_id.erase(id);
            My_Assert(succed == 1,"Bad Free");
            return;
        }

        void clear(){
            for(auto i : activated_route_id){
                My_Assert(pool[i].used == true,"Wrong routes!");
                pool[i].used = false;
                pool[i].route_info.clear();
                unused.push(i);
            }

            activated_route_id.clear();
        };

    };

    Policy policy;

    double best_solution_cost;

    const int significant_search_delta = significant_search;
    const double local_ratio_threshold = local_ratio;
    const double infeasible_distance_threshold = infeasible_distance;

    int search_step;

    int equal_step;

    int local_minimum_likelihood = 0;
    const int local_threshold = local_minimum_threshold;


    //dynamic info
    int neigh_size = 0;            //different policy use different neighbor size
    std::vector<int> search_space;
    /* current solution */
    SOLUTION solution;
    ROUTESPOOL routes;
    double cur_solution_cost;

    vector<int> task_set;

    double total_vio_load;

    //move operator
    unique_ptr<class SingleInsert> single_insert;
    unique_ptr<class DoubleInsert> double_insert;
    unique_ptr<class Invert> invert;
    unique_ptr<class NewSwap> swap;
    unique_ptr<class NewTwoOpt> two_opt;
    unique_ptr<class Extraction> extraction;
    unique_ptr<class Slice> slice;
public:

    HighSpeedNeighBorSearch(const MCGRP &mcgrp);
    ~HighSpeedNeighBorSearch();


    vector<int> get_solution();



    void clear();

    inline vector<int> get_tasks_set(){return task_set;}

    inline double get_fitness(){
        My_Assert(policy.beta != std::numeric_limits<decltype(policy.beta)>::max(), "beta is undefined!");
        return cur_solution_cost + policy.beta * total_vio_load;
    }

    inline double get_cur_cost(){
        return cur_solution_cost;
    }

    inline double get_cur_vio_load(){
        return total_vio_load;
    }


    /*!
     * sub proceduren in decode_seq method

    /*!
     * @details 根据邻域表创建任务的邻域搜索空间
     * @param mcgrp
     * @param task
     */
    void create_search_neighborhood(const MCGRP &mcgrp, const int task);

    /*!
     * @details pack current sol info to delimeter coding format
     * @param p
     */
    void create_individual(const MCGRP &mcgrp, Individual &p);


    /*!
 * @details unpack sol info to negative coding format
 * @param del_seq
 */
    void unpack_seq(const std::vector<int> &del_seq, const MCGRP &mcgrp);

    /*!
     * @details neighbor search
     * @param mcgrp
     * @param max_nonimprove_times
     * @param accept [First accept | Bext accept]
     */
    void neighbor_search(const MCGRP &mcgrp);

    /*!
     * @details downhill search
     * @param mcgrp
     */
    void descent_search(const MCGRP &mcgrp);

    /*!
     * @details random threshold search
     * @param mcgrp
     */
    void RTR_search(const MCGRP &mcgrp);

    /*!
     *  @details threshold exploration
     * @param mcgrp
     */
    void threshold_exploration_version_0(const MCGRP &mcgrp);
    void threshold_exploration_version_1(const MCGRP &mcgrp);

    /*!
     * @details descent exploration
     * @param mcgrp
     */
    void descent_exploration_version_0(const MCGRP &mcgrp);

    void descent_exploration_version_1(const MCGRP &mcgrp);

    /*!
     * @details Infeasible search
     * @param mcgrp
     * @param start_point
     */
    void infeasible_exploration(const MCGRP &mcgrp);

    void small_step_infeasible_descent_search(const MCGRP &mcgrp);
    void small_step_infeasible_tabu_search(const MCGRP &mcgrp);

    void large_step_infeasible_search(const MCGRP &mcgrp);

    void update(const MCGRP &mcgrp, const vector<int>& best_buffer,const vector<int>& best_routes);

    /*!
     * @details repair overloaded solution
     * @param mcgrp
     */
    void repair_solution(const MCGRP &mcgrp);

    /*!
     * @details Check if any tasks missed.
     */
    bool missed(const MCGRP &mcgrp);

    /*!
     * @details check duplicate tasks
     * @param mcgrp
     * @return
     */
    bool check_duplicated(const MCGRP &mcgrp);

    bool valid_sol(const MCGRP& mcgrp);

    /*!
     * extract the searched solution to MCGRP instance
     * @param mcgrp: the implementation of a problem
     */
    void trace(const MCGRP &mcgrp);

    /*!
     * the implementation of the neighborhood search given a specific mode
     * @param mcgrp: the implementation of a problem
     * @param mode: search mode
     */
    void _neigh_search(const MCGRP &mcgrp, int mode);

};

#endif