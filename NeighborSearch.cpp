#include "NeighborSearch.h"
#include <iostream>
#include "SingleInsert.h"
#include "DoubleInsert.h"
#include "Swap.h"
#include "Invert.h"
#include "TwoOpt.h"
#include "Slice.h"
#include "Extraction.h"
#include <sys/timeb.h>
#include <vector>
#include <numeric>
#include <bitset>
#include <cmath>
#include <algorithm>
#include "config.h"
#include "utils.h"

using namespace std;

vector<double> ratios = {0.07,0.08,0.09,0.10};
//vector<double> ratios = {0.003,0.004,0.005,0.006};


vector<double> prob = {0.25, 0.25, 0.25, 0.25};


NeighBorSearch::NeighBorSearch(const MCGRP &mcgrp)
:invert(new Invert)
, swap(new NewSwap())
, single_insert(new SingleInsert)
, double_insert(new DoubleInsert)
,two_opt(new NewTwoOpt)
{
    search_step = 0;
    equal_step = 0;
    cur_solution_cost = 0;
    total_vio_load = 0;
    policy.tolerance = 0;


//    nearest_scanning(mcgrp, ns_indi);
//    unpack_seq(ns_indi.sequence, mcgrp);
//    mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
//    policy.beta = cur_solution_cost / double(mcgrp.capacity * 30);
}

NeighBorSearch::~NeighBorSearch() = default;

void NeighBorSearch::neighbor_search(const MCGRP &mcgrp)
{
    if(mcgrp.best_total_route_length < mcgrp.global_best_total_route_length){
        mcgrp.global_best_total_route_length = mcgrp.best_total_route_length;
        mcgrp.global_best_sol_buff = mcgrp.best_sol_buff;
    }

    mcgrp.best_total_route_length = cur_solution_cost;

    int mode;
    if(neighbor_search_mode == "IDPRTR")
        mode = IDPRTR;
    else if (neighbor_search_mode == "RTRIDP")
        mode = RTRIDP;
    else
        mode = (int) mcgrp._rng.Randint(0, 1);

    _neigh_search(mcgrp, mode);

    mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
}

void NeighBorSearch::_neigh_search(const MCGRP &mcgrp, int mode)
{
    if (mode == RTRIDP) {
        cout<<"RTTR -> IDP\n";
        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");

        //Second Infeasible solve
        DEBUG_PRINT("IDP Procedure started...");

        infeasible_exploration(mcgrp);

        DEBUG_PRINT("IDP Procedure done!");

        My_Assert(total_vio_load == 0, "IDP procedure produce an infeasible solution");

        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");

    }
    else if (mode == IDPRTR) {
        cout<<"IDP -> RTTP\n";
        DEBUG_PRINT("IDP Procedure started...");

        DEBUG_PRINT("IDP Procedure started...");

        infeasible_exploration(mcgrp);

        DEBUG_PRINT("IDP Procedure done!");

        My_Assert(total_vio_load == 0, "IDP procedure produce an infeasible solution");

        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");
    }
    else{
        My_Assert(false,"Unknown search policy");
    }
}

void NeighBorSearch::compress_solution(vector<int> &seq, int actual_route_num)
{
    My_Assert(actual_route_num == routes.size() - 1, "Incorrect empty route number!");
    My_Assert(seq.front() == DUMMY && seq.back() == DUMMY, "Start task and end task in sequence should be DUMMY!");

    vector<int> buffer = seq;
    seq.clear();
    for (auto cursor = 0; cursor < buffer.size() - 1; cursor++) {
        if (buffer[cursor] == DUMMY && buffer[cursor + 1] == DUMMY) {
            continue;
        }
        seq.push_back(buffer[cursor]);
    }
    seq.push_back(buffer.back());
}

void NeighBorSearch::unpack_seq(const std::vector<int> &del_seq, const MCGRP &mcgrp)
{

    My_Assert(!del_seq.empty(), "You can't unpack an empty sequence!");
    int whole_task_num = mcgrp.actual_task_num + 2;            //include dummy task and sentinel task

    next_array.clear();
    next_array.resize(whole_task_num, std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max());

    pred_array.clear();
    pred_array.resize(whole_task_num, std::numeric_limits<identity<decltype(pred_array)>::type::value_type>::max());

    route_id.clear();
    route_id.resize(whole_task_num, std::numeric_limits<identity<decltype(route_id)>::type::value_type>::max());

    routes.clear();

    //对于一个新路径的起点用负值表示，表示下一个任务与自己不属于同一路径
    next_array[DUMMY] = -del_seq[1];    //next task of dummy is the first task in first route
    route_id[DUMMY] = -1;          //doesn't belong to any route

    int current_task = -1;    //当前任务
    int serve_num_in_route = 0;    //路径中已服务的任务个数

    int cursor = 0;
    My_Assert(del_seq[cursor] == DUMMY, "sequence should start from dummy task!");
    while (cursor < del_seq.size()) {
        //没有遍历到解终点时
        if (del_seq[cursor] == DUMMY) {
            //find a delimiter
            if (cursor == del_seq.size() - 1)    //遍历到解终点
                break;

            //start a new route
            ++cursor;
            My_Assert(del_seq[cursor] != DUMMY, "More than one dummy task at the beginning of the route!");
            current_task = del_seq[cursor];    //获取新路径中的第一个任务

            if (del_seq[cursor + 1] != DUMMY)    //如果该任务不是该路径中的最后一个实际任务，就直接更新其实际后继任务
                next_array[current_task] = del_seq[cursor + 1];
            else {        //如果该任务是该条路径中最后一个实际任务，就将其后继任务至负，然后更新(不考虑虚拟任务)
                if (cursor < del_seq.size() - 2)
                    next_array[current_task] = -del_seq[cursor + 2];
            }

            /* initialize a new route */
            MCGRPRoute tmp;
            tmp.start = current_task;
            tmp.length = 0;
            tmp.load = 0;
            routes.push_back(tmp);

            //complement last route info
            if (routes.size() > 1) {
                routes[routes.size() - 2].end = del_seq[cursor - 2];    //获得前一条路径的终点实际任务

                /* 上一条路径的长度要加上最后一个任务尾节点到仓库的距离以及最后一个任务的服务成本 */
                routes[routes.size() - 2].length +=
                    (mcgrp.min_cost[mcgrp.inst_tasks[routes[routes.size() - 2].end].tail_node][mcgrp.inst_tasks[DUMMY]
                        .head_node]
                        + mcgrp.inst_tasks[routes[routes.size() - 2].end].serv_cost);
                routes[routes.size() - 2].num_customers = serve_num_in_route;        //更新上一条路径的服务的实际任务数
            }


            /* 更新该路径长度，因为该任务是路径中的第一个任务，所以要加上仓库点到该任务头结点的距离 */
            routes.back().length +=
                mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[current_task].head_node];

            serve_num_in_route = 1;    //该路径实际服务任务数是1
        }
        else {        //如果没有遍历到路径终点
            current_task = del_seq[cursor];    //获取当前任务
            if (del_seq[cursor + 1] == DUMMY)    //如果该任务是该路径中的最后一个实际任务
                next_array[current_task] = -del_seq[cursor + 2];    //将其后继任务置负，然后更新(不考虑虚拟任务)
            else
                next_array[current_task] = del_seq[cursor + 1];//如果该任务不是该路径中的最后一个实际任务，就直接更新其实际后继任务

            /* 更新该路径长度，要加上上一个任务尾节点到该任务头结点的距离和上一个任务的服务成本 */
            routes.back().length +=
                (mcgrp.min_cost[mcgrp.inst_tasks[del_seq[cursor - 1]].tail_node][mcgrp.inst_tasks[current_task].head_node]
                    + mcgrp.inst_tasks[del_seq[cursor - 1]].serv_cost);
            ++serve_num_in_route;        //更新该路径服务实际任务数
        }

        My_Assert(!routes.empty(), "You failed to create a route at the very beginning!");
        routes.back().load += mcgrp.inst_tasks[current_task].demand;    //更新路径负载

        route_id[current_task] = routes.size() - 1;    //更新任务所属路径编号
        ++cursor;
    }

    //complement last route
    current_task = del_seq[del_seq.size() - 2];
    routes.back().end = current_task;    //最后一条路径的终点
    routes.back().length += (mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[DUMMY].head_node]
        + mcgrp.inst_tasks[current_task].serv_cost);    //最后一条路径的长度
    routes.back().num_customers = serve_num_in_route;    //最后一条路径服务的实际任务数


    next_array[current_task] = DUMMY;    //最后一条路径的最后一个任务的后继任务为仓库

    // Now create the associated pred_array implied by the newly created next_array
    create_pred_array();

    total_vio_load = 0;
    for (auto route : routes) {
        total_vio_load += max((route.load - mcgrp.capacity), 0);
    }

    cur_solution_cost = std::accumulate(routes.begin(), routes.end(), 0, MCGRPRoute::accumulate_load);
    negative_coding_sol = get_negative_coding(del_seq);
    delimiter_coding_sol = del_seq;
}

void NeighBorSearch::create_individual(const MCGRP &mcgrp, Individual &p)
{
    My_Assert(!negative_coding_sol.empty(), "Current solution cannot be empty!");

    p.sequence.clear();
    p.sequence = get_delimiter_coding(negative_coding_sol);

    assert(!routes.empty());
    p.route_seg_load.clear();
    p.total_vio_load = 0;
    for (auto route : routes) {
        p.route_seg_load.push_back(route.load);
        p.total_vio_load += max((route.load - mcgrp.capacity), 0);
    }

    p.total_cost = cur_solution_cost;
}

void NeighBorSearch::create_pred_array()
{
    My_Assert(!next_array.empty(), "Next array is empty!");

    int i = DUMMY;
    int j = next_array[i];
    while (j != DUMMY) {
        if (j > 0)
            pred_array[j] = i;
        else
            pred_array[-j] = -i;

        i = abs(j);
        j = next_array[i];

    }
    // The precede task of dummy task is last task in last route
    pred_array[j] = -i;
}

void NeighBorSearch::RTR_search(const MCGRP &mcgrp)
{
    double orig_val_for_uphill;
    double orig_val_for_downhill;
    local_minimum_likelihood = 1;
    int cnt = 1;
    do{
        struct timeb start_time;
        ftime(&start_time);

        threshold_exploration_version_0(mcgrp);
        orig_val_for_uphill = cur_solution_cost;

        do{
            orig_val_for_downhill = cur_solution_cost;
            descent_exploration_version_0(mcgrp);
        } while (cur_solution_cost < orig_val_for_downhill);

        struct timeb end_time;
        ftime(&end_time);

        cout << "Finish a feasible search process, spent: "
             <<fixed << (end_time.time - start_time.time)
                 + ((end_time.millitm - start_time.millitm) * 1.0 / 1000) << 's' << endl;

        if (cur_solution_cost < orig_val_for_uphill){
            DEBUG_PRINT("total_route_length " +  to_string(orig_val_for_uphill));
            DEBUG_PRINT("reset local likelihood");
            local_minimum_likelihood = 1;
            cnt++;
            if (cnt == max_RTR_search_cycle)
            {
                DEBUG_PRINT("maybe a local optimal");
                local_minimum_likelihood = local_threshold;
            }
            mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
        }
        else{
            local_minimum_likelihood++;
        }
    }while(local_minimum_likelihood < local_threshold);
}

void NeighBorSearch::descent_search(const MCGRP &mcgrp)
{
    descent_exploration_version_1(mcgrp);
}

void NeighBorSearch::create_search_neighborhood(const MCGRP &mcgrp, const int task)
{
    search_space.clear();

    for(auto chosen_task:mcgrp.task_neigh_list[task]){
        //chose the task which in the current solution
        auto task_id = chosen_task.task_id;
        if(next_array[task_id] != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()){
            search_space.push_back(task_id);
        }
    }

    if(search_space.size() > neigh_size){
        search_space.resize(neigh_size);
    }

    mcgrp._rng.RandPerm(search_space);
}

void NeighBorSearch::threshold_exploration_version_0(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Uphill and downhill...");

    // Based on the best solution find so far, experiment shows this is better than the policy,especially on big instance
    // which start from current solution.
    policy.benchmark = mcgrp.best_total_route_length;


    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);
    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | TOLERANCE | DELTA_ONLY);

    //you need to decide the dynamic neighbor size when you search based on different policy
        neigh_size = mcgrp.neigh_size;
//    neigh_size = min(10,mcgrp.neigh_size);


    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::INVERT,
        NeighborOperator::TWO_OPT
    };

    int L = mcgrp._rng.Randint(28, 33);

    for (int k = 1; k < L; k++) {
        mcgrp._rng.RandPerm(neighbor_operator);

        vector<int> task_set(mcgrp.actual_task_num);
        std::generate(task_set.begin(), task_set.end(), Generator());
        int chosen_task = -1;

        for (auto cur_operator:neighbor_operator) {
            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);

                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    double_insert->search(*this, mcgrp, chosen_task);
                }
            }


            if (cur_operator == NeighborOperator::SWAP) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    swap->search(*this, mcgrp, chosen_task);
                }
            }

            //Invert move
            if (cur_operator == NeighborOperator::INVERT) {
                if(mcgrp.req_edge_num == 0){
                    static bool once = true;
                    if(once){
                        cout<<"This instance doesn't need INVERT operator!\n";
                        once = false;
                    }
                    continue;
                }
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    invert->search(*this, mcgrp, chosen_task);
                }
            }

            //TwoOpt move
            if (cur_operator == NeighborOperator::TWO_OPT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                }
            }

            if (next_array[chosen_task]
                == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                if (!mcgrp.is_edge(chosen_task)) {
                    cerr << "A non edge task has been missed!\n";
                    abort();
                }
                else {
                    chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                      != std::numeric_limits<
                                                                                          identity<decltype(next_array)>::type::value_type>::max(),
                                                                                  "An edge task has been missed");
                }
            }

            two_opt->search(*this, mcgrp, chosen_task);
        }
    }

    policy.benchmark = 0;
    policy.set(original_policy);
    policy.tolerance = 0;
    neigh_size = 0;
}

void NeighBorSearch::threshold_exploration_version_1(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Uphill and downhill...");

    // Based on the best solution find so far, experiment shows this is better than the policy,especially on big instance
    // which start from current solution.
    policy.benchmark = mcgrp.best_total_route_length;

    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);

    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | TOLERANCE | DELTA_ONLY);

    //you need to decide the dynamic neighbor size when you search based on different policy
    neigh_size = mcgrp.neigh_size;
//    neigh_size = min(10,mcgrp.neigh_size);


    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::INVERT,
        NeighborOperator::TWO_OPT
    };

    double prior_cost;

    do{
        prior_cost = cur_solution_cost;
        policy.benchmark = mcgrp.best_total_route_length;

        mcgrp._rng.RandPerm(neighbor_operator);

        vector<int> task_set(mcgrp.actual_task_num);
        std::generate(task_set.begin(), task_set.end(), Generator());
        int chosen_task = -1;

        for (auto cur_operator:neighbor_operator) {
            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);

                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    double_insert->search(*this, mcgrp, chosen_task);
                }
            }


            if (cur_operator == NeighborOperator::SWAP) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    swap->search(*this, mcgrp, chosen_task);
                }
            }

            //Invert move
            if (cur_operator == NeighborOperator::INVERT) {
                if(mcgrp.req_edge_num == 0){
                    static bool once = true;
                    if(once){
                        cout<<"This instance doesn't need INVERT operator!\n";
                        once = false;
                    }
                    continue;
                }
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    invert->search(*this, mcgrp, chosen_task);
                }
            }

            //TwoOpt move
            if (cur_operator == NeighborOperator::TWO_OPT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    two_opt->search(*this, mcgrp, chosen_task);
                }
            }

        }

        descent_exploration_version_1(mcgrp);

    } while (cur_solution_cost >= prior_cost);


    policy.benchmark = 0;
    policy.set(original_policy);
    policy.tolerance = 0;
    neigh_size = 0;
}

extern struct timeb cur_time;

extern struct timeb instance_start_time;

void NeighBorSearch::descent_exploration_version_0(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Downhill...");

    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | DELTA_ONLY);

    neigh_size = mcgrp.neigh_size;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::INVERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT,
    };

    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());

    int chosen_task = -1;
    mcgrp._rng.RandPerm(neighbor_operator);
    for (auto cur_operator:neighbor_operator) {
        // Single Insert
        if (cur_operator == NeighborOperator::SINGLE_INSERT) {
            double start_val = 0;
            do{
                start_val = cur_solution_cost;
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task] != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }while (cur_solution_cost < start_val);

        }

        //Double Insert
        if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
            double start_val = 0;
            do{
                start_val = cur_solution_cost;
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    double_insert->search(*this, mcgrp, chosen_task);
                }
            }
            while(start_val < cur_solution_cost);
        }

        //Swap
        if (cur_operator == NeighborOperator::SWAP) {
            double start_val = 0;
            do{
                start_val = cur_solution_cost;
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    swap->search(*this, mcgrp, chosen_task);
                }
            }while (cur_solution_cost < start_val);

        }

//            Invert move
        if (cur_operator == NeighborOperator::INVERT) {
            double start_val = 0;
            do{
                start_val = cur_solution_cost;
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    invert->search(*this, mcgrp, chosen_task);
                }
            }while (cur_solution_cost < start_val);
        }

        //TwoOpt move
        if (cur_operator == NeighborOperator::TWO_OPT) {
            double start_val = 0;
            do{
                start_val = cur_solution_cost;
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    two_opt->search(*this, mcgrp, chosen_task);
                }
            }while (cur_solution_cost < start_val);
        }
    }

    policy.set(original_policy);
    neigh_size = 0;
}

void NeighBorSearch::descent_exploration_version_1(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Downhill...");

    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | DELTA_ONLY);

    neigh_size = mcgrp.neigh_size;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::INVERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT,
    };

    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());

    int prior_search_step;
    int prior_equal_step;

    int search_step_delta;
    int equal_step_delta;

    double equal_ratio;

    do{
        prior_equal_step = this->equal_step;
        prior_search_step = this->search_step;

        int chosen_task = -1;
        mcgrp._rng.RandPerm(neighbor_operator);
        for (auto cur_operator:neighbor_operator) {
            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task] != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                      "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    double_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::SWAP) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    swap->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::INVERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    invert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::TWO_OPT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
                                                                                              != std::numeric_limits<
                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
                                                                                          "An edge task has been missed");
                        }
                    }

                    two_opt->search(*this, mcgrp, chosen_task);
                }
            }
        }

        search_step_delta = search_step - prior_search_step;
        equal_step_delta = equal_step - prior_equal_step;

        if (search_step_delta == 0) {
            My_Assert(equal_step_delta == 0,"Wrong ratio!");
            equal_ratio = 0;
        }
        else {
            equal_ratio = double(equal_step_delta) / double(search_step_delta);
        }

        My_Assert(equal_ratio >= 0 && equal_ratio <= 1, "Wrong ratio!");


    } while(
        search_step_delta > significant_search_delta
        && equal_ratio < local_ratio_threshold
    );


    policy.set(original_policy);
    neigh_size = 0;
}

void NeighBorSearch::infeasible_exploration(const MCGRP &mcgrp)
{
    // Based on the best solution find so far, experiment shows this is better than the policy
    // which start from current solution.
    delimiter_coding_sol = get_delimiter_coding(mcgrp.best_sol_buff);
    unpack_seq(delimiter_coding_sol, mcgrp);

    My_Assert(total_vio_load == 0, "Cannot start from an infeasible point!");
    policy.nearest_feasible_cost = cur_solution_cost;
    policy.beta = cur_solution_cost / double(mcgrp.capacity * 15);


    small_step_infeasible_descent_search(mcgrp);
    if (total_vio_load == 0) {
        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }
//    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);

    DEBUG_PRINT("Trigger Infeasible Tabu Search");
    small_step_infeasible_tabu_search(mcgrp);

    if (total_vio_load == 0) {
        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }
    //    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);

    small_step_infeasible_descent_search(mcgrp);
    if (total_vio_load == 0) {
        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }
    //    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);


    //Here used to break the local minimum with merge-split operator
//    large_step_infeasible_search(mcgrp);

    My_Assert(total_vio_load >= 0,"Wrong total violated load!");
    if (total_vio_load == 0) {
        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }
    else {
//        mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);
        repair_solution(mcgrp);
        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }

}

void NeighBorSearch::small_step_infeasible_descent_search(const MCGRP &mcgrp)
{
    auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | FITNESS_ONLY);
    neigh_size = 10;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT
    };

    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());

    this->equal_step = 0;
    this->search_step = 0;

    int prior_search_step;
    int prior_equal_step;

    int search_step_delta;
    int equal_step_delta;

    double equal_ratio;

    static int too_far = 0;
    static int too_near = 0;
    static int count = 0;

    do {
        prior_equal_step = this->equal_step;
        prior_search_step = this->search_step;
        int chosen_task = -1;
        mcgrp._rng.RandPerm(neighbor_operator);


        if(total_vio_load != 0){
            //start location should not be considered
            count++;

            double distance_to_feasible_zone = double (total_vio_load) / policy.nearest_feasible_cost;

            if (distance_to_feasible_zone > infeasible_distance_threshold)
                too_far++;
            else
                too_near++;

            if (count % 5 == 0) {

                if (too_near == 5) {
                    DEBUG_PRINT("Infeasible search too near");
                    policy.beta /= 2;
                }
                else if (too_far == 5) {
                    DEBUG_PRINT("Infeasible search too far");
                    policy.beta *= 2;
                }

                too_far = 0;
                too_near = 0;
                count = 0;
            }
        }

        for (auto cur_operator:neighbor_operator) {
            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);

                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task]
                            != std::numeric_limits<
                            identity<decltype(next_array)>::type::value_type>::max(),
                            "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task]
                            != std::numeric_limits<
                            identity<decltype(next_array)>::type::value_type>::max(),
                            "An edge task has been missed");
                        }
                    }

                    double_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::SWAP) {
                mcgrp._rng.RandPerm(task_set);

                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task]
                            != std::numeric_limits<
                            identity<decltype(next_array)>::type::value_type>::max(),
                            "An edge task has been missed");
                        }
                    }

                    swap->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::TWO_OPT) {
                mcgrp._rng.RandPerm(task_set);

                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (next_array[chosen_task]
                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(next_array[chosen_task]
                                          != std::numeric_limits<
                                              identity<decltype(next_array)>::type::value_type>::max(),
                                      "An edge task has been missed");
                        }
                    }

                    two_opt->search(*this, mcgrp, chosen_task);
                }
            }
        }

        search_step_delta = search_step - prior_search_step;
        equal_step_delta = equal_step - prior_equal_step;

        if(search_step_delta == 0 && equal_step_delta == 0){
            equal_ratio = 0;
        }
        else{
            equal_ratio = double(equal_step_delta) / double(search_step_delta);
        }

        My_Assert(equal_ratio >=0 && equal_ratio <= 1,"Wrong ratio!");
    }while(
        search_step_delta > significant_search_delta
            && equal_ratio < local_ratio_threshold
        );


    policy.set(original_policy);
    neigh_size = 0;
}

void NeighBorSearch::small_step_infeasible_tabu_search(const MCGRP &mcgrp)
{
    policy.benchmark = mcgrp.best_total_route_length;
    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);
    neigh_size = 10;

    auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | TOLERANCE | FITNESS_ONLY);

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT
        };

    mcgrp._rng.RandPerm(neighbor_operator);

    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    int chosen_task = -1;

    for (auto cur_operator:neighbor_operator) {
        if (cur_operator == NeighborOperator::SINGLE_INSERT) {
            mcgrp._rng.RandPerm(task_set);

            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (next_array[chosen_task]
                    == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                    if (!mcgrp.is_edge(chosen_task)) {
                        cerr << "A non edge task has been missed!\n";
                        abort();
                    }
                    else {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(next_array[chosen_task]
                                      != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                  "An edge task has been missed");
                    }
                }

                single_insert->search(*this, mcgrp, chosen_task);
            }
        }

        if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
            mcgrp._rng.RandPerm(task_set);
            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (next_array[chosen_task]
                    == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                    if (!mcgrp.is_edge(chosen_task)) {
                        cerr << "A non edge task has been missed!\n";
                        abort();
                    }
                    else {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(next_array[chosen_task]
                                      != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                  "An edge task has been missed");
                    }
                }

                double_insert->search(*this, mcgrp, chosen_task);
            }
        }


        if (cur_operator == NeighborOperator::SWAP) {
            mcgrp._rng.RandPerm(task_set);
            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (next_array[chosen_task]
                    == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                    if (!mcgrp.is_edge(chosen_task)) {
                        cerr << "A non edge task has been missed!\n";
                        abort();
                    }
                    else {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(next_array[chosen_task]
                                      != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                  "An edge task has been missed");
                    }
                }

                swap->search(*this, mcgrp, chosen_task);
            }
        }

        if (cur_operator == NeighborOperator::TWO_OPT) {
            mcgrp._rng.RandPerm(task_set);
            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (next_array[chosen_task]
                    == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
                    if (!mcgrp.is_edge(chosen_task)) {    //非边任务一定在解序列中
                        cerr << "A non edge task has been missed!\n";
                        abort();
                    }
                    else {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(next_array[chosen_task]
                                      != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                                  "An edge task has been missed");
                    }
                }

                two_opt->search(*this, mcgrp, chosen_task);
            }
        }
    }

    policy.benchmark = 0;
    policy.set(original_policy);
    policy.tolerance = 0;
    neigh_size = 0;
}

void NeighBorSearch::large_step_infeasible_search(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Trigger large step break movement");
    //decide how many routes need to be merged, self-adaptive
//    int merge_size = routes.size() / 3;
    int merge_size = 10;

    //enlarge the capacity for infeasible search to decide whether use infeasible consideration
//    double scale = mcgrp.capacity * policy.beta;
//    int pseudo_capacity = int(mcgrp.capacity *(1+ (scale / (policy.beta*sqrt(1+scale*scale)))));
    int pseudo_capacity = mcgrp.capacity;
    merge_split(*this, mcgrp, merge_size, pseudo_capacity);

//    mcgrp.check_best_infeasible_solution(cur_solution_cost,beta,total_vio_load,negative_coding_sol);
}

void NeighBorSearch::repair_solution(const MCGRP &mcgrp)
{
    // Repair infeasible solution
    // Only violated load here no missed task, duplicated task problem here


    My_Assert(!check_missed(mcgrp), "some task missed!");
    My_Assert(!check_duplicated(mcgrp), "duplicated task!");
    My_Assert(total_vio_load > 0, "This is not a infeasible task!");

    DEBUG_PRINT("Repair infeasible solution...");

    struct Route
    {
        vector<int> task_seq;
        int load;
    };

    vector<Route> original_route_tasks(routes.size());

    //generate tasks distribution in routes
    int cursor = -1;
    for (auto task : negative_coding_sol) {
        if (task < 0) {
            if (cursor >= 0) {
                original_route_tasks[cursor].task_seq.push_back(DUMMY);
            }
            cursor++;
            original_route_tasks[cursor].task_seq.push_back(DUMMY);

            original_route_tasks[cursor].task_seq.push_back(-task);
            original_route_tasks[cursor].load += mcgrp.inst_tasks[-task].demand;
        }
        else if (task > 0) {
            original_route_tasks[cursor].task_seq.push_back(task);
            original_route_tasks[cursor].load += mcgrp.inst_tasks[task].demand;
        }
        else {
            My_Assert(false, "dummy task cannot occur in negative coding solution!");
        }
    }
    original_route_tasks.back().task_seq.push_back(DUMMY);


    //the task set which need to be re-inserted
    vector<int> candidate_tasks;
    vector<Route> satisfied_route_tasks;
    for (auto current_route = 0; current_route < original_route_tasks.size(); current_route++) {
        if (original_route_tasks[current_route].load > mcgrp.capacity) {
            for (auto task : original_route_tasks[current_route].task_seq) {
                if (task != DUMMY) {
                    candidate_tasks.push_back(task);
                }
            }
        }
        else {
            satisfied_route_tasks.push_back(original_route_tasks[current_route]);
        }
    }


    //insert task
    int chosen_route = -1;
    int chosen_pos = -1;
    int best_delta = numeric_limits<decltype(best_delta)>::max();
    int delta = numeric_limits<decltype(delta)>::max();
    for (auto task : candidate_tasks) {
        //reset chosen information before insert
        //for faster execution time, I omit inverse task of edge task!:)
        chosen_route = -1;
        chosen_pos = -1;
        best_delta = numeric_limits<decltype(best_delta)>::max();

        auto demand = mcgrp.inst_tasks[task].demand;
        for (int row = 0; row < satisfied_route_tasks.size(); row++) {
            //load constraint check
            if (demand + satisfied_route_tasks[row].load > mcgrp.capacity) {
                continue;
            }
            else {
                //find a best position to insert
                for (int col = 1; col < satisfied_route_tasks[row].task_seq.size(); col++) {
                    double ab = mcgrp.min_cost[mcgrp.inst_tasks[col - 1].tail_node][mcgrp.inst_tasks[col].head_node];
                    double
                        a_task = mcgrp.min_cost[mcgrp.inst_tasks[col - 1].tail_node][mcgrp.inst_tasks[task].head_node];
                    double task_b = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[col].head_node];
                    delta = -ab + a_task + mcgrp.inst_tasks[task].serv_cost + task_b;
                    if (delta < best_delta) {
                        best_delta = delta;
                        chosen_route = row;
                        chosen_pos = col;
                    }
                }
            }
        }

        //means a new route need to be created
        if (chosen_route == -1) {
            satisfied_route_tasks.push_back(Route());
            satisfied_route_tasks.back().task_seq.push_back(DUMMY);
            satisfied_route_tasks.back().task_seq.push_back(task);
            satisfied_route_tasks.back().task_seq.push_back(DUMMY);
            satisfied_route_tasks.back().load = mcgrp.inst_tasks[task].demand;
        }
        else {
            satisfied_route_tasks[chosen_route].task_seq
                .insert(satisfied_route_tasks[chosen_route].task_seq.begin() + chosen_pos, task);
            satisfied_route_tasks[chosen_route].load += mcgrp.inst_tasks[task].demand;
        }
    }

    vector<int> new_feasible_sol;
    new_feasible_sol.push_back(DUMMY);
    for (auto route : satisfied_route_tasks) {
        for (auto task:route.task_seq) {
            if (task != DUMMY) {
                new_feasible_sol.push_back(task);
            }
        }
        new_feasible_sol.push_back(DUMMY);
    }

    //Update neighbor search info
    unpack_seq(new_feasible_sol, mcgrp);
    delimiter_coding_sol = get_delimiter_coding(negative_coding_sol);

    My_Assert(total_vio_load == 0, "Repair method seems doesn't work properly!");
}

bool NeighBorSearch::check_missed(const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());

    for (auto task:task_set) {
        if (next_array[task]
            == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
            if (!mcgrp.is_edge(task)) {
                cerr << "Non edge task " << task << " has been missed!\n";
                abort();
            }
            else {
                int inverse_task = mcgrp.inst_tasks[task].inverse;
                My_Assert(next_array[inverse_task]
                              != std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max(),
                          "Edge task has been missed");
            }
        }
    }

    return false;

}

bool NeighBorSearch::check_duplicated(const MCGRP &mcgrp)
{
    //offset one
    vector<int> task_set(mcgrp.actual_task_num + 1, 0);


    for (auto task : delimiter_coding_sol) {
        if (task == DUMMY) {
            continue;
        }

        My_Assert(task_set[task] == 0, "You have serviced this task before!");
        task_set[task]++;
        if (mcgrp.is_edge(task)) {
            task_set[mcgrp.inst_tasks[task].inverse]++;
        }
    }

    return false;
}

void NeighBorSearch::presert_sentinel(const MCGRP &mcgrp, const int i){

    My_Assert(i >= 1 && i <= mcgrp.actual_task_num,"Invalid task!");

    auto ite = find(delimiter_coding_sol.begin(),delimiter_coding_sol.end(),i);
    My_Assert( ite != delimiter_coding_sol.end(),"Task doesn't exist in current sol!");

    //1.handle delimiter_coding_sol
    //presert
    delimiter_coding_sol.insert(ite, mcgrp.sentinel);


    //2.handle pred and next array
    // pre_i is what used to be before i, maybe in the different route
    // ...pre_i-i...    ---->    ...pre_i-sentinel-i...
    int pre_i = pred_array[i];

    // sentinel is now followed by i
    next_array[mcgrp.sentinel] = i;

    // The element who used to be after i is now followed by sentinel
    if (pre_i > 0)
        next_array[pre_i] = mcgrp.sentinel;
    else
        // equal to zero or less than zero all means a different route
        next_array[abs(pre_i)] = -mcgrp.sentinel;


    pred_array[i] = mcgrp.sentinel;
    pred_array[mcgrp.sentinel] = pre_i;


    //3.handle route info
    int i_route = route_id[i];
    int start_i = routes[i_route].start;

    route_id[mcgrp.sentinel] = i_route;

    // Modify the start of i's route
    if(start_i == i){
        routes[i_route].start = mcgrp.sentinel;
    }

}

void NeighBorSearch::postsert_sentinel(const MCGRP &mcgrp, const int i)
{
//    int post_i, dummy; //pre_i,
//    int start, end, start_i, end_i;
//    int i_route;
    //double tu, uv, tv, iu, ju, ij, u_loss, i_gain, i_change, i_length, u_length;

    My_Assert(i >= 1 && i <= mcgrp.actual_task_num,"Invalid task!");

    auto ite = find(delimiter_coding_sol.begin(),delimiter_coding_sol.end(),i);
    My_Assert( ite != delimiter_coding_sol.end(),"Task doesn't exist in current sol!");

    //1.handle delimiter_coding_sol
    //postsert
    delimiter_coding_sol.insert(ite + 1, mcgrp.sentinel);

    //2.handle pred and next array
    // post_i is what used to be after i, maybe in the different route
    int post_i = next_array[i];

    // sentinel is now following i
    pred_array[mcgrp.sentinel] = i;

    // The task which used to be after i is now follow sentinel
    if (post_i > 0)
        pred_array[post_i] = mcgrp.sentinel;
    else
        // equal to zero or less than zero all means a different route
        pred_array[-post_i] = -mcgrp.sentinel;


    next_array[i] = mcgrp.sentinel;
    next_array[mcgrp.sentinel] = post_i;

    //3.handle route info
    const int i_route = route_id[i];
    const int end_i = routes[i_route].end;


    route_id[mcgrp.sentinel] = i_route;

    // Modify the end of i's route
    if (end_i == i){
        routes[i_route].end = mcgrp.sentinel;
    }

}

void NeighBorSearch::remove_sentinel(const MCGRP &mcgrp){
    const int pre_sentinel = pred_array[mcgrp.sentinel];
    const int post_sentinel = next_array[mcgrp.sentinel];

    My_Assert(pre_sentinel != numeric_limits<int>::max() && post_sentinel != numeric_limits<int>::max(),"invalid indices!");

    const int sentinel_route = route_id[mcgrp.sentinel];
    const int route_start = routes[sentinel_route].start;
    const int route_end = routes[sentinel_route].end;

    //1.handle delimiter_coding_sol
    auto ite = find(delimiter_coding_sol.begin(),delimiter_coding_sol.end(),mcgrp.sentinel);
    My_Assert( ite != delimiter_coding_sol.end(),"Sentinel doesn't exist in current sol!");
    delimiter_coding_sol.erase(ite);


    //2.handle next and pred
    next_array[abs(pre_sentinel)] = post_sentinel;
    pred_array[abs(post_sentinel)] = pre_sentinel;
    next_array[mcgrp.sentinel] = std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max();
    pred_array[mcgrp.sentinel] = std::numeric_limits<identity<decltype(pred_array)>::type::value_type>::max();


    //3.handle routes info and modify next and pred array
    route_id[mcgrp.sentinel] = std::numeric_limits<identity<decltype(route_id)>::type::value_type>::max();
    if(route_start == mcgrp.sentinel){
        //sentinel is located in the start position
        My_Assert(post_sentinel > 0,"post sentinel error in removal");
        routes[sentinel_route].start = post_sentinel;
        next_array[abs(pre_sentinel)] = -post_sentinel;
    }
    else if (route_end == mcgrp.sentinel)
    {
        My_Assert(pre_sentinel > 0,"pre sentinel error in removal");
        routes[sentinel_route].end = pre_sentinel;
        pred_array[abs(post_sentinel)] = -pre_sentinel;
    }
}



/*
 * High speed local search algorithm
 *
 *
 *
 *
 *
 *
 */

void HighSpeedNeighBorSearch::DUMMYPOOL::extend(){
    unsigned int new_size = pool.size() * 2;
    unsigned int old_size = pool.size();

    //2^n growth
//    vector<DUMMY_TASK> pool_buffer;
//
//    std::move(std::begin(pool), std::end(pool), std::back_inserter(pool_buffer));
//    My_Assert(pool_buffer.size() == new_size / 2,"Incorrect move!");
    pool.resize(new_size);

//    pool = pool_buffer;

    //refill unused
    for(auto i = old_size ; i < new_size ; i++){
        unused.push(i);
    }

    return;
}



HighSpeedNeighBorSearch::HighSpeedNeighBorSearch(const MCGRP &mcgrp)
: solution(mcgrp.actual_task_num)
,routes(mcgrp.actual_task_num)
,task_set(mcgrp.actual_task_num)
,single_insert(new SingleInsert)
,double_insert(new DoubleInsert)
,two_opt(new NewTwoOpt)
, invert(new Invert)
, swap(new NewSwap)
, extraction(new Extraction)
, slice(new Slice)
{
    search_step = 0;
    equal_step = 0;
    cur_solution_cost = numeric_limits<decltype(best_solution_cost)>::max();
    total_vio_load = 0;
    best_solution_cost = numeric_limits<decltype(best_solution_cost)>::max();
    std::generate(task_set.begin(), task_set.end(), Generator());
    solution.print();
}

HighSpeedNeighBorSearch::~HighSpeedNeighBorSearch() = default;

void HighSpeedNeighBorSearch::neighbor_search(const MCGRP &mcgrp)
{
    int mode;
    if(neighbor_search_mode == "IDPRTR")
        mode = IDPRTR;
    else if (neighbor_search_mode == "RTRIDP")
        mode = RTRIDP;
    else
        mode = (int) mcgrp._rng.Randint(0, 1);

    _neigh_search(mcgrp, mode);

    trace(mcgrp);
}

void HighSpeedNeighBorSearch::_neigh_search(const MCGRP &mcgrp, int mode){
    if (mode == RTRIDP) {
        cout<<"RTTR -> IDP\n";
        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");

        //Second Infeasible solve
        DEBUG_PRINT("IDP Procedure started...");

        infeasible_exploration(mcgrp);

        DEBUG_PRINT("IDP Procedure done!");

        My_Assert(total_vio_load == 0, "IDP procedure produce an infeasible solution");

        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");

    }
    else if (mode == IDPRTR) {
        cout<<"IDP -> RTTP\n";
        DEBUG_PRINT("IDP Procedure started...");

        infeasible_exploration(mcgrp);

        DEBUG_PRINT("IDP Procedure done!");

        My_Assert(total_vio_load == 0, "IDP procedure produce an infeasible solution");

        DEBUG_PRINT("RTTP Procedure started...");

        RTR_search(mcgrp);

        DEBUG_PRINT("RTTP Procedure done!");
    }
    else{
        My_Assert(false,"Unknown search policy");
    }
}

void HighSpeedNeighBorSearch::unpack_seq(const std::vector<int> &del_seq, const MCGRP &mcgrp)
{

    My_Assert(del_seq.size() > 2, "You can't unpack an empty sequence!");

    //parse solution
    TASK_NODE *dummy = nullptr;
    dummy = solution.dummypool.get_new_dummy();
    My_Assert(solution.very_start == nullptr, "Incorrect state of solution");
    solution.very_start = dummy;

    //link first dummy
    My_Assert(del_seq[0] == DUMMY && del_seq[1] != DUMMY, "sequence should start from dummy task!");
    solution.very_start->next = &solution.tasks[del_seq[1]];
    solution.tasks[del_seq[1]].pre = solution.very_start;

    int cursor = 1;
    while (true) {
        if (del_seq[cursor] == DUMMY) {
            if (cursor == del_seq.size() - 1) {
                //...0
                My_Assert(dummy->next == nullptr, "This dummy task has been used!");
                solution.very_end = dummy;
                break;
            }

            My_Assert(del_seq[cursor + 1] != DUMMY, "too much dummy seq!");

            //...0-a...
            My_Assert(dummy != nullptr, "null dummy task!");
            My_Assert(dummy->next == nullptr, "this dummy task has been used!");
            dummy->next = &solution.tasks[del_seq[cursor + 1]];
            solution.tasks[del_seq[cursor + 1]].pre = dummy;

            cursor++;
        }
        else {
            if (del_seq[cursor + 1] == DUMMY) {
                //...a-0...
                // a new dummy task is going to be created
                dummy = solution.dummypool.get_new_dummy();

                //connect dummy task;
                solution.tasks[del_seq[cursor]].next = dummy;
                dummy->pre = &solution.tasks[del_seq[cursor]];
                cursor++;
            }
            else {
                //...a-b...
                solution.tasks[del_seq[cursor]].next = &solution.tasks[del_seq[cursor + 1]];
                solution.tasks[del_seq[cursor + 1]].pre = &solution.tasks[del_seq[cursor]];
                cursor++;
            }
        }
    }


    vector<vector<int>> seg;
    vector<int> buffer;
    for (cursor = 1; cursor < del_seq.size(); cursor++) {
        if (del_seq[cursor] == DUMMY) {
            seg.push_back(buffer);
            buffer.clear();
        }
        else {
            buffer.push_back(del_seq[cursor]);
        }
    }

    total_vio_load = 0;
    cur_solution_cost = 0;
    for (int i = 0; i < seg.size(); i++) {
        MCGRPRoute *new_route = routes[routes.allocate_route()];
        new_route->start = seg[i].front();
        new_route->end = seg[i].back();
        new_route->length = 0;
        new_route->load = 0;
        new_route->num_customers = seg[i].size();

        //0-a...
        new_route->length += mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[new_route->start].head_node];
        new_route->length += mcgrp.inst_tasks[DUMMY].serv_cost;

        for (int j = 0; j < seg[i].size() - 1; j++) {
            //...b-c...
            new_route->length += mcgrp.inst_tasks[seg[i][j]].serv_cost;
            new_route->length +=
                mcgrp.min_cost[mcgrp.inst_tasks[seg[i][j]].tail_node][mcgrp.inst_tasks[seg[i][j + 1]].head_node];
            new_route->load += mcgrp.inst_tasks[seg[i][j]].demand;
            My_Assert(solution[seg[i][j]]->route_id == -1, "The task has been parsed!");
            solution[seg[i][j]]->route_id = new_route->ID;
        }

        //...d-0
        new_route->length += mcgrp.inst_tasks[new_route->end].serv_cost;
        new_route->length +=
            mcgrp.min_cost[mcgrp.inst_tasks[new_route->end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        new_route->length += mcgrp.inst_tasks[DUMMY].serv_cost;

        new_route->load += mcgrp.inst_tasks[new_route->end].demand;
        My_Assert(solution[new_route->end]->route_id == -1, "The task has been parsed!");
        solution[new_route->end]->route_id = new_route->ID;

        total_vio_load += max((new_route->load - mcgrp.capacity), 0);
        cur_solution_cost += new_route->length;
    }

    My_Assert(valid_sol(mcgrp),"Wrong state");
}

vector<int> HighSpeedNeighBorSearch::get_solution(){
    My_Assert(solution.very_start != nullptr,"Empty solution");

    vector<int> buffer;

    TASK_NODE * tmp = solution.very_start;
    do{
        buffer.push_back(max(tmp->ID,0));
        tmp = tmp->next;
    }while(tmp != solution.very_end);

    buffer.push_back(max(tmp->ID,0));

    return buffer;
}

void HighSpeedNeighBorSearch::clear(){
    total_vio_load = 0;
    best_solution_cost = numeric_limits<decltype(best_solution_cost)>::max();
    cur_solution_cost = 0;
    solution.clear();
    routes.clear();
}


void HighSpeedNeighBorSearch::create_individual(const MCGRP &mcgrp, Individual &p)
{
    p.sequence = get_solution();

    My_Assert(!routes.activated_route_id.empty(),"Routes info is empty!");
    p.route_seg_load.clear();
    p.total_vio_load = 0;
    for (auto id : routes.activated_route_id) {
        p.route_seg_load.push_back(routes[id]->load);
        p.total_vio_load += max((routes[id]->load - mcgrp.capacity), 0);
    }

    My_Assert(p.total_vio_load == total_vio_load,"incorrect total vio load!");
    p.total_cost = cur_solution_cost;
}

void HighSpeedNeighBorSearch::RTR_search(const MCGRP &mcgrp)
{
    double orig_val_for_uphill;
    double orig_val_for_downhill;
    local_minimum_likelihood = 1;
    int cnt = 1;
    do{
        struct timeb start_time;
        ftime(&start_time);

        threshold_exploration_version_0(mcgrp);
        orig_val_for_uphill = cur_solution_cost;

        do{
            orig_val_for_downhill = cur_solution_cost;
            descent_exploration_version_0(mcgrp);
        } while (cur_solution_cost < orig_val_for_downhill);

        struct timeb end_time;
        ftime(&end_time);

        cout << "Finish a feasible search process, spent: "
             <<fixed << get_time_difference(start_time,end_time) << 's' << endl;

        if (cur_solution_cost < orig_val_for_uphill){
            DEBUG_PRINT("total_route_length " +  to_string(orig_val_for_uphill));
            DEBUG_PRINT("reset local likelihood");
            local_minimum_likelihood = 1;
            cnt++;
            if (cnt == max_RTR_search_cycle) {
                DEBUG_PRINT("maybe a local optimal");
                local_minimum_likelihood = local_threshold;
            }
        }
        else{
            local_minimum_likelihood++;
        }
    }while(local_minimum_likelihood < local_threshold);
}

void HighSpeedNeighBorSearch::descent_search(const MCGRP &mcgrp)
{
    descent_exploration_version_0(mcgrp);
}

void HighSpeedNeighBorSearch::create_search_neighborhood(const MCGRP &mcgrp, const int task)
{
    search_space.clear();

    for(auto chosen_task:mcgrp.task_neigh_list[task]){
        //chose the task which in the current solution
        auto task_id = chosen_task.task_id;
        if(solution.tasks[task_id].next != nullptr){
            search_space.push_back(task_id);
        }
    }

    if(search_space.size() > neigh_size){
        search_space.resize(neigh_size);
    }

    mcgrp._rng.RandPerm(search_space);
}

bool HighSpeedNeighBorSearch::valid_sol(const MCGRP& mcgrp) {
    if(solution.very_start->ID >= 0 || solution.very_end->ID >= 0){
        return false;
    }

    auto cur_task = solution.very_start;

    double cost = 0;
    while(cur_task != solution.very_end){
        cost += mcgrp.inst_tasks[max(cur_task->ID, 0)].serv_cost;
        cost += mcgrp.min_cost[mcgrp.inst_tasks[max(cur_task->ID, 0)].tail_node][mcgrp.inst_tasks[max(cur_task->next->ID, 0)].head_node];
        cur_task = cur_task->next;
    }
    cost += mcgrp.inst_tasks[max(cur_task->ID, 0)].serv_cost;

    double vio_load = 0;
    double routes_cost_sum = 0;
    for(auto id : routes.activated_route_id){
        if(routes[id]->load > mcgrp.capacity){
            vio_load += routes[id]->load - mcgrp.capacity;
        }
        routes_cost_sum += routes[id]->length;
    }

    return routes_cost_sum == cost && cost == cur_solution_cost && vio_load == total_vio_load;
}

void HighSpeedNeighBorSearch::threshold_exploration_version_0(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Uphill and downhill...");

    // Based on the best solution find so far, experiment shows this is better than the policy,especially on big instance
    // which start from current solution.
    policy.benchmark = this->best_solution_cost;


    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);
    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | TOLERANCE | DELTA_ONLY);

    //you need to decide the dynamic neighbor size when you search based on different policy
    neigh_size = mcgrp.neigh_size;
//    neigh_size = min(10,mcgrp.neigh_size);


    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::INVERT,
        NeighborOperator::SLICE,
        NeighborOperator::EXTRACTION,
        NeighborOperator::TWO_OPT
    };

    int L = mcgrp._rng.Randint(28, 33);

    for (int k = 1; k < L; k++) {
        mcgrp._rng.RandPerm(neighbor_operator);

        int chosen_task = -1;

        for (auto cur_operator:neighbor_operator) {
            mcgrp._rng.RandPerm(task_set);

            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (solution[chosen_task]->next == nullptr) {
                    if (mcgrp.is_edge(chosen_task)) {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(solution[chosen_task]->next != nullptr,"An edge task has been missed");
                    }
                    else {
                        My_Assert(false,"A non edge task has been missed!");
                    }
                }

                switch (cur_operator){
                    case SINGLE_INSERT:
                        single_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case DOUBLE_INSERT:
                        double_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case SWAP:
                        swap->search(*this, mcgrp, chosen_task);
                        break;
                    case INVERT:
                        if(mcgrp.req_edge_num != 0){
                            invert->search(*this, mcgrp, chosen_task);
                        }
                        break;
                    case TWO_OPT:
                        two_opt->search(*this, mcgrp, chosen_task);
                        break;
                    case SLICE:
                        slice->search(*this,mcgrp,chosen_task);
                        break;
                    case EXTRACTION:
                        extraction->search(*this,mcgrp,chosen_task);
                        break;
                    default:
                        My_Assert(false,"unknown operator!");
                }

            }
        }
    }

    policy.benchmark = 0;
    policy.set(original_policy);
    policy.tolerance = 0;
    neigh_size = 0;
}

//void NeighBorSearch::threshold_exploration_version_1(const MCGRP &mcgrp)
//{
//    DEBUG_PRINT("Uphill and downhill...");
//
//    // Based on the best solution find so far, experiment shows this is better than the policy,especially on big instance
//    // which start from current solution.
//    policy.benchmark = mcgrp.best_total_route_length;
//
//    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);
//
//    const auto original_policy = policy.get();
//    policy.set(BEST_ACCEPT | TOLERANCE | DELTA_ONLY);
//
//    //you need to decide the dynamic neighbor size when you search based on different policy
//    neigh_size = mcgrp.neigh_size;
////    neigh_size = min(10,mcgrp.neigh_size);
//
//
//    vector<NeighborOperator> neighbor_operator{
//        NeighborOperator::SINGLE_INSERT,
//        NeighborOperator::DOUBLE_INSERT,
//        NeighborOperator::SWAP,
//        NeighborOperator::INVERT,
//        NeighborOperator::TWO_OPT
//    };
//
//    double prior_cost;
//
//    do{
//        prior_cost = cur_solution_cost;
//        policy.benchmark = mcgrp.best_total_route_length;
//
//        mcgrp._rng.RandPerm(neighbor_operator);
//
//        vector<int> task_set(mcgrp.actual_task_num);
//        std::generate(task_set.begin(), task_set.end(), Generator());
//        int chosen_task = -1;
//
//        for (auto cur_operator:neighbor_operator) {
//            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
//                mcgrp._rng.RandPerm(task_set);
//
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    SingleInsert::search(*this, mcgrp, chosen_task);
//                }
//            }
//
//            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    DoubleInsert::search(*this, mcgrp, chosen_task);
//                }
//            }
//
//
//            if (cur_operator == NeighborOperator::SWAP) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    NewSwap::search(*this, mcgrp, chosen_task);
//                }
//            }
//
//            //Invert move
//            if (cur_operator == NeighborOperator::INVERT) {
//                if(mcgrp.req_edge_num == 0){
//                    static bool once = true;
//                    if(once){
//                        cout<<"This instance doesn't need INVERT operator!\n";
//                        once = false;
//                    }
//                    continue;
//                }
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    Invert::search(*this, mcgrp, chosen_task);
//                }
//            }
//
//            //TwoOpt move
//            if (cur_operator == NeighborOperator::TWO_OPT) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    NewTwoOpt::search(*this, mcgrp, chosen_task);
//                }
//            }
//
//        }
//
//        descent_exploration_version_1(mcgrp);
//
//    } while (cur_solution_cost >= prior_cost);
//
//
//    policy.benchmark = 0;
//    policy.set(original_policy);
//    policy.tolerance = 0;
//    neigh_size = 0;
//}


void HighSpeedNeighBorSearch::descent_exploration_version_0(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Downhill...");

    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | DELTA_ONLY);

    neigh_size = mcgrp.neigh_size;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::INVERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT,
    };

    int chosen_task = -1;
    mcgrp._rng.RandPerm(neighbor_operator);
    for (auto cur_operator : neighbor_operator) {
        double start_val = 0;
        do{
            start_val = cur_solution_cost;
            mcgrp._rng.RandPerm(task_set);

            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (solution[chosen_task]->next == nullptr) {
                    if (mcgrp.is_edge(chosen_task)) {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                        My_Assert(solution[chosen_task]->next != nullptr,"An edge task has been missed");
                    }
                    else {
                        My_Assert(false,"A non edge task has been missed!");
                    }
                }

                switch (cur_operator){
                    case SINGLE_INSERT:
                        single_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case DOUBLE_INSERT:
                        double_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case SWAP:
                        swap->search(*this, mcgrp, chosen_task);
                        break;
                    case INVERT:
                        if(mcgrp.req_edge_num != 0){
                            invert->search(*this, mcgrp, chosen_task);
                        }
                        break;
                    case TWO_OPT:
                        two_opt->search(*this, mcgrp, chosen_task);
                        break;
                    default:
                        My_Assert(false,"unknown operator!");
                }
            }
        }while (cur_solution_cost < start_val);
    }

    policy.set(original_policy);
    neigh_size = 0;
}

void HighSpeedNeighBorSearch::descent_exploration_version_1(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Downhill...");

    const auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | DELTA_ONLY);

    neigh_size = mcgrp.neigh_size;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::INVERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT,
    };

    int prior_search_step;
    int prior_equal_step;

    int search_step_delta;
    int equal_step_delta;

    double equal_ratio;

    do{
        prior_equal_step = this->equal_step;
        prior_search_step = this->search_step;

        int chosen_task = -1;
        mcgrp._rng.RandPerm(neighbor_operator);
        for (auto cur_operator:neighbor_operator) {
            if (cur_operator == NeighborOperator::SINGLE_INSERT) {
                mcgrp._rng.RandPerm(task_set);
                for (int i = 0; i < mcgrp.actual_task_num; i++) {
                    chosen_task = task_set[i];
                    if (solution.tasks[chosen_task].next == nullptr) {
                        if (!mcgrp.is_edge(chosen_task)) {
                            cerr << "A non edge task has been missed!\n";
                            abort();
                        }
                        else {
                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                            My_Assert(solution.tasks[chosen_task].next != nullptr,
                                      "An edge task has been missed");
                        }
                    }

                    single_insert->search(*this, mcgrp, chosen_task);
                }
            }

            if (cur_operator == NeighborOperator::DOUBLE_INSERT) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    DoubleInsert::search(*this, mcgrp, chosen_task);
//                }
            }

            if (cur_operator == NeighborOperator::SWAP) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    NewSwap::search(*this, mcgrp, chosen_task);
//                }
            }

            if (cur_operator == NeighborOperator::INVERT) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    Invert::search(*this, mcgrp, chosen_task);
//                }
            }

            if (cur_operator == NeighborOperator::TWO_OPT) {
//                mcgrp._rng.RandPerm(task_set);
//                for (int i = 0; i < mcgrp.actual_task_num; i++) {
//                    chosen_task = task_set[i];
//                    if (next_array[chosen_task]
//                        == std::numeric_limits<identity<decltype(next_array)>::type::value_type>::max()) {
//                        if (!mcgrp.is_edge(chosen_task)) {
//                            cerr << "A non edge task has been missed!\n";
//                            abort();
//                        }
//                        else {
//                            chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(next_array[chosen_task]
//                                                                                              != std::numeric_limits<
//                                                                                                  identity<decltype(next_array)>::type::value_type>::max(),
//                                                                                          "An edge task has been missed");
//                        }
//                    }
//
//                    NewTwoOpt::search(*this, mcgrp, chosen_task);
//                }
            }
        }

        search_step_delta = search_step - prior_search_step;
        equal_step_delta = equal_step - prior_equal_step;

        if (search_step_delta == 0) {
            My_Assert(equal_step_delta == 0,"Wrong ratio!");
            equal_ratio = 0;
        }
        else {
            equal_ratio = double(equal_step_delta) / double(search_step_delta);
        }

        My_Assert(equal_ratio >= 0 && equal_ratio <= 1, "Wrong ratio!");


    } while(
        search_step_delta > significant_search_delta
            && equal_ratio < local_ratio_threshold
        );


    policy.set(original_policy);
    neigh_size = 0;
}

void HighSpeedNeighBorSearch::infeasible_exploration(const MCGRP &mcgrp)
{
    // Based on the best solution find so far, experiment shows this is better than the policy
    // which start from current solution.
    this->clear();
    unpack_seq(get_delimiter_coding(mcgrp.best_sol_buff), mcgrp);

    My_Assert(total_vio_load == 0, "Cannot start from an infeasible point!");
    policy.nearest_feasible_cost = cur_solution_cost;
    policy.beta = cur_solution_cost / double(mcgrp.capacity * 15);

    small_step_infeasible_descent_search(mcgrp);

//    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);

    DEBUG_PRINT("Trigger Infeasible Tabu Search");
    small_step_infeasible_tabu_search(mcgrp);

    //    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);

    small_step_infeasible_descent_search(mcgrp);

    //    mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);


    //Here used to break the local minimum with merge-split operator
    large_step_infeasible_search(mcgrp);

    My_Assert(total_vio_load >= 0,"Wrong total violated load!");
    if (total_vio_load > 0) {
//        mcgrp.check_best_infeasible_solution(cur_solution_cost,policy.beta,total_vio_load,negative_coding_sol);
        repair_solution(mcgrp);
    }

    policy.nearest_feasible_cost = 0;
    policy.beta = 0;
    this->best_solution_cost = cur_solution_cost;
}

void HighSpeedNeighBorSearch::trace(const MCGRP &mcgrp){
    if (total_vio_load == 0)
    {
        if(cur_solution_cost < this->best_solution_cost){
            this->best_solution_cost = cur_solution_cost;
        }

        vector<int> negative_coding_sol;
        if(cur_solution_cost < mcgrp.best_total_route_length){
            negative_coding_sol = get_negative_coding(get_solution());
        }

        mcgrp.check_best_solution(cur_solution_cost, negative_coding_sol);
    }
}


void HighSpeedNeighBorSearch::small_step_infeasible_descent_search(const MCGRP &mcgrp)
{
    auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | DOWNHILL | FITNESS_ONLY);
    neigh_size = 10;

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT
    };

    equal_step = 0;
    search_step = 0;

    int prior_search_step;
    int prior_equal_step;

    int search_step_delta;
    int equal_step_delta;

    double equal_ratio;

    static int too_far = 0;
    static int too_near = 0;
    static int count = 0;

    do {
        prior_equal_step = this->equal_step;
        prior_search_step = this->search_step;
        int chosen_task = -1;
        mcgrp._rng.RandPerm(neighbor_operator);

        if(total_vio_load > 0){
            //start location should not be considered
            count++;

            double distance_to_feasible_zone = double (total_vio_load) / policy.nearest_feasible_cost;

            if (distance_to_feasible_zone > infeasible_distance_threshold)
                too_far++;
            else
                too_near++;

            if (count % 5 == 0) {

                if (too_near == 5) {
                    DEBUG_PRINT("Infeasible search too near");
                    policy.beta /= 2;
                }
                else if (too_far == 5) {
                    DEBUG_PRINT("Infeasible search too far");
                    policy.beta *= 2;
                }

                too_far = 0;
                too_near = 0;
                count = 0;
            }
        }


        for (auto cur_operator:neighbor_operator) {
            mcgrp._rng.RandPerm(task_set);

            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (solution[chosen_task]->next == nullptr) {
                    if (mcgrp.is_edge(chosen_task)) {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(
                            solution[chosen_task]->next != nullptr, "An edge task has been missed");
                    }
                    else { My_Assert(false, "A non edge task has been missed!");
                    }
                }

                switch (cur_operator) {
                    case SINGLE_INSERT:single_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case DOUBLE_INSERT:double_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case SWAP:swap->search(*this, mcgrp, chosen_task);
                        break;
                    case TWO_OPT:two_opt->search(*this, mcgrp, chosen_task);
                        break;
                    default:My_Assert(false, "unknown operator!");
                }
            }
        }

        search_step_delta = search_step - prior_search_step;
        equal_step_delta = equal_step - prior_equal_step;

        if(search_step_delta == 0 && equal_step_delta == 0){
            equal_ratio = 0;
        }
        else{
            equal_ratio = double(equal_step_delta) / double(search_step_delta);
        }

        My_Assert(equal_ratio >=0 && equal_ratio <= 1,"Wrong ratio!");
    }while(
        search_step_delta > significant_search_delta
            && equal_ratio < local_ratio_threshold
        );


    policy.set(original_policy);
    neigh_size = 0;
}

void HighSpeedNeighBorSearch::small_step_infeasible_tabu_search(const MCGRP &mcgrp)
{
    policy.benchmark = this->best_solution_cost;
    policy.tolerance = sel_ratio(prob, ratios, mcgrp._rng);
    neigh_size = 10;

    auto original_policy = policy.get();
    policy.set(BEST_ACCEPT | TOLERANCE | FITNESS_ONLY);

    vector<NeighborOperator> neighbor_operator{
        NeighborOperator::SINGLE_INSERT,
        NeighborOperator::DOUBLE_INSERT,
        NeighborOperator::SWAP,
        NeighborOperator::TWO_OPT
    };


    int L = mcgrp._rng.Randint(28, 33);

    for(int k = 0;k<L;k++){
        int chosen_task = -1;
        mcgrp._rng.RandPerm(neighbor_operator);

        for (auto cur_operator:neighbor_operator) {
            mcgrp._rng.RandPerm(task_set);

            for (int i = 0; i < mcgrp.actual_task_num; i++) {
                chosen_task = task_set[i];
                if (solution[chosen_task]->next == nullptr) {
                    if (mcgrp.is_edge(chosen_task)) {
                        chosen_task = mcgrp.inst_tasks[chosen_task].inverse;My_Assert(
                            solution[chosen_task]->next != nullptr, "An edge task has been missed");
                    }
                    else { My_Assert(false, "A non edge task has been missed!");
                    }
                }

                switch (cur_operator) {
                    case SINGLE_INSERT:single_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case DOUBLE_INSERT:double_insert->search(*this, mcgrp, chosen_task);
                        break;
                    case SWAP:swap->search(*this, mcgrp, chosen_task);
                        break;
                    case TWO_OPT:two_opt->search(*this, mcgrp, chosen_task);
                        break;
                    default:My_Assert(false, "unknown operator!");
                }
            }
        }

    }

    policy.benchmark = 0;
    policy.set(original_policy);
    policy.tolerance = 0;
    neigh_size = 0;
}

void HighSpeedNeighBorSearch::large_step_infeasible_search(const MCGRP &mcgrp)
{
    DEBUG_PRINT("Trigger large step break movement");
    //decide how many routes need to be merged, self-adaptive
//    int merge_size = routes.size() / 3;
    int merge_size = 10;

    //enlarge the capacity for infeasible search to decide whether use infeasible consideration
//    double scale = mcgrp.capacity * policy.beta;
//    int pseudo_capacity = int(mcgrp.capacity *(1+ (scale / (policy.beta*sqrt(1+scale*scale)))));
    int pseudo_capacity = mcgrp.capacity;
    merge_split(*this, mcgrp, merge_size, pseudo_capacity);

    trace(mcgrp);

//    mcgrp.check_best_infeasible_solution(cur_solution_cost,beta,total_vio_load,negative_coding_sol);
}

void HighSpeedNeighBorSearch::update(const MCGRP &mcgrp, const vector<int>& best_buffer,const vector<int>& best_routes){
    My_Assert(valid_sol(mcgrp),"Wrong state!");

    //remove old info
    for(auto route_id : best_routes){
        My_Assert(routes.activated_route_id.find(route_id)!=routes.activated_route_id.end(),"Invalid route");
        cur_solution_cost -= routes[route_id]->length;
        if(routes[route_id]->load > mcgrp.capacity){
            total_vio_load -= (routes[route_id]->load - mcgrp.capacity);
        }

        //remove solution
        vector<int> seq;
        int cur_task = routes[route_id]->start;
        while (cur_task != routes[route_id]->end){
            seq.push_back(cur_task);
            cur_task = solution[cur_task]->next->ID;
        }
        seq.push_back(cur_task);

        int dummy_marker = 0;
        HighSpeedNeighBorSearch::TASK_NODE *reserve_dummy;
        bool very_end_case = false;
        if(solution[seq.back()]->next == solution.very_end){
            dummy_marker = solution[seq.front()]->pre->ID;
            reserve_dummy = solution[seq.back()]->next;
            very_end_case = true;
        }
        else{
            dummy_marker = solution[seq.back()]->next->ID;
            reserve_dummy = solution[seq.front()]->pre;
        }
        My_Assert(dummy_marker < 0 ,"Wrong tasks");

        if(very_end_case){
            solution[dummy_marker]->pre->next = reserve_dummy;
            reserve_dummy->pre = solution[dummy_marker]->pre;
        }
        else{
            reserve_dummy->next = solution[dummy_marker]->next;
            solution[dummy_marker]->next->pre = reserve_dummy;
        }

        solution.dummypool.free_dummy(dummy_marker);

        for(auto task: seq){
            solution[task]->clear();
        }

        //free old route
        routes.free_route(route_id);
    }

    My_Assert(valid_sol(mcgrp),"Wrong state!");

    My_Assert(best_buffer.front() == DUMMY && best_buffer.back() == DUMMY,"Wrong state");


    vector<vector<int >> seqs;
    vector<int> buffer;
    for(int i = 1;i<best_buffer.size();i++){
        if(best_buffer[i] == DUMMY){
            seqs.push_back(buffer);
            buffer.clear();
        }
        else{
            buffer.push_back(best_buffer[i]);
        }
    }

    for(int row = 0;row<seqs.size();row++){
        int new_route = routes.allocate_route();
        double length = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[seqs[row].front()].head_node];
        double load = 0;

        for(int col = 0;col <seqs[row].size() - 1;col++){
            length += mcgrp.inst_tasks[seqs[row][col]].serv_cost
                + mcgrp.min_cost[mcgrp.inst_tasks[seqs[row][col]].tail_node][mcgrp.inst_tasks[seqs[row][col + 1]].head_node];

            load += mcgrp.inst_tasks[seqs[row][col]].demand;

            solution[seqs[row][col]]->route_id = new_route;
        }

        length += mcgrp.inst_tasks[seqs[row].back()].serv_cost
            + mcgrp.min_cost[mcgrp.inst_tasks[seqs[row].back()].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        load += mcgrp.inst_tasks[seqs[row].back()].demand;
        solution[seqs[row].back()]->route_id = new_route;


        routes[new_route]->length = length;
        routes[new_route]->load = load;
        routes[new_route]->start = seqs[row].front();
        routes[new_route]->end = seqs[row].back();
        routes[new_route]->num_customers = seqs[row].size();

        if(load >mcgrp.capacity){
            total_vio_load += (load - mcgrp.capacity);
        }
        cur_solution_cost += length;

        //handle solution
        if(seqs[row].size() == 1){
            auto task = seqs[row][0];
            auto new_dummy = solution.dummypool.get_new_dummy();
            new_dummy->pre = solution.very_end->pre;
            new_dummy->next = solution[task];
            new_dummy->pre->next = new_dummy;
            solution[task]->pre = new_dummy;
            solution[task]->next = solution.very_end;
            solution.very_end->pre = solution[task];
        }
        else{
            My_Assert(seqs[row].size()>1,"Wrong state");

            auto new_dummy = solution.dummypool.get_new_dummy();
            new_dummy->pre = solution.very_end->pre;
            new_dummy->next = solution[seqs[row].front()];

            new_dummy->pre->next = new_dummy;

            solution[seqs[row].front()]->pre = new_dummy;
            solution[seqs[row].front()]->next = solution[seqs[row][1]];

            for(auto i = 1;i < seqs[row].size()-1;i++) {
                auto pre_task = seqs[row][i-1];
                auto task = seqs[row][i];
                auto next_task = seqs[row][i+1];
                My_Assert(solution[task]->pre == nullptr && solution[task]->next == nullptr,"Wrong state!");
                solution[task]->next = solution[next_task];
                solution[task]->pre = solution[pre_task];
            }

            solution[seqs[row].back()]->pre = solution[seqs[row][seqs[row].size() - 2]];
            solution[seqs[row].back()]->next = solution.very_end;

            solution.very_end->pre = solution[seqs[row].back()];

        }

    }

    My_Assert(missed(mcgrp) && check_duplicated(mcgrp),"Wrong result!");
    My_Assert(valid_sol(mcgrp),"Wrong state!");
}

void HighSpeedNeighBorSearch::repair_solution(const MCGRP &mcgrp)
{
    // Repair infeasible solution
    // Only violated load here
    // No missed task, duplicated task problem here

    My_Assert(missed(mcgrp), "Some task missed!");
    My_Assert(check_duplicated(mcgrp), "Duplicated task!");
    My_Assert(total_vio_load > 0, "This is not a infeasible task!");

    DEBUG_PRINT("Repair infeasible solution...");

    struct Route
    {
        int route_id;
        vector<int> task_seq;
        int load;
    };

    vector<Route> satisfied_routes;
    vector<Route> violated_routes;

    //generate tasks distribution in routes
    for(auto route_id : routes.activated_route_id){
        Route tmp;
        tmp.route_id = route_id;
        tmp.load = routes[route_id]->load;
        int cur_task = routes[route_id]->start;
        while(cur_task != routes[route_id]->end){
            tmp.task_seq.push_back(cur_task);
            cur_task = solution[cur_task]->next->ID;
        }
        tmp.task_seq.push_back(cur_task);

        if(tmp.load > mcgrp.capacity){
            violated_routes.push_back(tmp);
        }
        else{
            satisfied_routes.push_back(tmp);
        }
    }

    My_Assert(!violated_routes.empty(),"Wrong state");
    //the task set which need to be re-inserted
    vector<int> candidate_tasks;
    for (auto current_route : violated_routes) {
        //remove the violated routes info
        cur_solution_cost -= routes[current_route.route_id]->length;
        total_vio_load -= (routes[current_route.route_id]->load - mcgrp.capacity);


        routes.free_route(current_route.route_id);

        //remove solution
        int dummy_marker = 0;
        HighSpeedNeighBorSearch::TASK_NODE *reserve_dummy;
        bool very_end_case = false;
        if(solution[current_route.task_seq.back()]->next == solution.very_end){
            dummy_marker = solution[current_route.task_seq.front()]->pre->ID;
            reserve_dummy = solution[current_route.task_seq.back()]->next;
            very_end_case = true;
        }
        else{
            dummy_marker = solution[current_route.task_seq.back()]->next->ID;
            reserve_dummy = solution[current_route.task_seq.front()]->pre;
        }
        My_Assert(dummy_marker < 0 ,"Wrong tasks");

        if(very_end_case){
            My_Assert(reserve_dummy == solution.very_end,"Wrong state");
            if(solution[dummy_marker] != solution.very_start){
                solution[dummy_marker]->pre->next = reserve_dummy;
                reserve_dummy->pre = solution[dummy_marker]->pre;
                solution.dummypool.free_dummy(dummy_marker);
            }
            else{
                solution[dummy_marker]->next = reserve_dummy;
                reserve_dummy->pre = solution[dummy_marker];
            }

        }
        else{
            reserve_dummy->next = solution[dummy_marker]->next;
            solution[dummy_marker]->next->pre = reserve_dummy;
            solution.dummypool.free_dummy(dummy_marker);
        }


        for(auto task: current_route.task_seq){
            solution[task]->clear();
            candidate_tasks.push_back(task);
        }


    }

    My_Assert(valid_sol(mcgrp),"Wrong validation");

    //insert task to repair
    HighSpeedNeighBorSearch::TASK_NODE *left_task;
    HighSpeedNeighBorSearch::TASK_NODE *right_task;
    int chosen_route = -1;
    int best_delta = numeric_limits<decltype(best_delta)>::max();
    int delta = numeric_limits<decltype(delta)>::max();
    for (auto task : candidate_tasks) {
        //reset chosen information before insert
        //for faster execution time, I omit inverse task of edge task!:)
        chosen_route = -1;
        best_delta = numeric_limits<decltype(best_delta)>::max();
        left_task = nullptr;
        right_task = nullptr;

        auto demand = mcgrp.inst_tasks[task].demand;

        for (int row = 0; row < satisfied_routes.size(); row++) {
            //load constraint check
            if (demand + satisfied_routes[row].load > mcgrp.capacity) {
                continue;
            }
            else {
                int a = 0;
                int b = 0;
                double ab  = 0;
                double a_task = 0;
                double task_b = 0;

                //find a best position to insert
                a = solution[satisfied_routes[row].task_seq.front()]->pre->ID;
                My_Assert(a < 0,"Wrong task");
                b = solution[satisfied_routes[row].task_seq.front()]->ID;
                My_Assert(b > 0,"Wrong task");

                ab = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[b].head_node];
                a_task = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[task].head_node];
                task_b = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[b].head_node];
                delta = -ab + a_task + mcgrp.inst_tasks[task].serv_cost + task_b;

                if (delta < best_delta) {
                    best_delta = delta;
                    chosen_route = row;
                    left_task = solution[a];
                    right_task = solution[b];
                }

                for (int col = 1; col < satisfied_routes[row].task_seq.size() - 1; col++) {
                    a = solution[satisfied_routes[row].task_seq[col]]->ID;
                    b = solution[satisfied_routes[row].task_seq[col + 1]]->ID;
                    My_Assert(a > 0,"Wrong task");
                    My_Assert(b > 0,"Wrong task");

                    ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
                    a_task = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[task].head_node];
                    task_b = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[b].head_node];
                    delta = -ab + a_task + mcgrp.inst_tasks[task].serv_cost + task_b;

                    if (delta < best_delta) {
                        best_delta = delta;
                        chosen_route = row;
                        left_task = solution[a];
                        right_task = solution[b];
                    }
                }

                a = solution[satisfied_routes[row].task_seq.back()]->ID;
                My_Assert(a > 0,"Wrong task");
                b = solution[satisfied_routes[row].task_seq.back()]->next->ID;
                My_Assert(b < 0,"Wrong task");

                ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
                a_task = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[task].head_node];
                task_b = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
                delta = -ab + a_task + mcgrp.inst_tasks[task].serv_cost + task_b;

                if (delta < best_delta) {
                    best_delta = delta;
                    chosen_route = row;
                    left_task = solution[a];
                    right_task = solution[b];
                }
            }
        }

        //means a new route need to be created
        if (chosen_route == -1) {
            My_Assert(left_task == nullptr && right_task == nullptr,"Wrong tasks");
            const auto new_route = routes.allocate_route();

            satisfied_routes.push_back(Route());
            satisfied_routes.back().task_seq.push_back(task);
            satisfied_routes.back().load = mcgrp.inst_tasks[task].demand;
            satisfied_routes.back().route_id = new_route;

            const auto dummy_task = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[task].head_node];
            const auto task_dummy = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

            routes[new_route]->length = dummy_task + mcgrp.inst_tasks[task].serv_cost + task_dummy;
            routes[new_route]->num_customers = 1;
            routes[new_route]->load = mcgrp.inst_tasks[task].demand;

            routes[new_route]->start = task;
            routes[new_route]->end = task;

            solution[task]->route_id = new_route;

            cur_solution_cost += routes[new_route]->length;

            //handle solution
            auto new_dummy = solution.dummypool.get_new_dummy();
            new_dummy->pre = solution.very_end->pre;
            new_dummy->next = solution[task];
            new_dummy->pre->next = new_dummy;
            solution[task]->pre = new_dummy;
            solution[task]->next = solution.very_end;
            solution.very_end->pre = solution[task];

        }
        else {
            int a = left_task->ID;
            int b = right_task->ID;
            if(a<0){
                satisfied_routes[chosen_route].task_seq.insert(satisfied_routes[chosen_route].task_seq.begin(),task);
            }
            else if (b<0){
                satisfied_routes[chosen_route].task_seq.insert(satisfied_routes[chosen_route].task_seq.end(),task);
            }
            else{
                auto ite = find(satisfied_routes[chosen_route].task_seq.begin(),satisfied_routes[chosen_route].task_seq.end(),b);
                My_Assert(ite != satisfied_routes[chosen_route].task_seq.end(),"Cannot find the task!");
                satisfied_routes[chosen_route].task_seq.insert(ite, task);
            }

            satisfied_routes[chosen_route].load += mcgrp.inst_tasks[task].demand;


            const auto route_id = satisfied_routes[chosen_route].route_id;
            routes[route_id]->length += best_delta;
            routes[route_id]->num_customers += 1;
            routes[route_id]->load += demand;

            solution[task]->route_id = route_id;

            if(a<0){
                routes[route_id]->start = task;
            } else if(b < 0){
                routes[route_id]->end = task;
            }

            cur_solution_cost += best_delta;

            //handle solution
            solution[a]->next = solution[task];
            solution[b]->pre = solution[task];
            solution[task]->pre = solution[a];
            solution[task]->next = solution[b];
        }

        My_Assert(valid_sol(mcgrp), "Repair method doesn't work properly!");
    }

    My_Assert(missed(mcgrp), "Some task missed!");
    My_Assert(check_duplicated(mcgrp), "Duplicated task!");
    My_Assert(total_vio_load == 0, "This is not a infeasible task!");

    My_Assert(valid_sol(mcgrp), "Repair method doesn't work properly!");


}

bool HighSpeedNeighBorSearch::missed(const MCGRP &mcgrp)
{
    for (auto task_id : task_set) {
        if (solution.tasks[task_id].next == nullptr) {
            if (!mcgrp.is_edge(task_id)) {
                return false;
            }
            else {
                int inverse_task_id = mcgrp.inst_tasks[task_id].inverse;
                if(solution.tasks[inverse_task_id].next == nullptr){
                    return false;
                }
            }
        }
    }

    return true;
}

bool HighSpeedNeighBorSearch::check_duplicated(const MCGRP &mcgrp)
{
    //offset one
    vector<int> tasks(mcgrp.actual_task_num + 1, 0);

    vector<int> delimiter_coding_sol = get_solution();
    for (auto task : delimiter_coding_sol) {
        if (task == DUMMY) {
            continue;
        }

        if(tasks[task] != 0){
            return false;
        }

        tasks[task]++;
        if (mcgrp.is_edge(task)) {
            tasks[mcgrp.inst_tasks[task].inverse]++;
        }
    }

    return true;
}
