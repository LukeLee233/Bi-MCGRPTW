//
// Created by luke on 2019/11/29.
//

#include <algorithm>
#include "ConstructPolicy.h"
#include "RNG.h"
#include <unordered_map>

using namespace std;


void nearest_scanning(const MCGRP &mcgrp, Individual &rs_indi)
{
    int serve_task_num = mcgrp.req_arc_num + mcgrp.req_node_num + mcgrp.req_edge_num;

    int load;
    int trial;
    int mindist;

    std::vector<int> unserved_task_id_set;

    std::vector<int> candidate_task_set;

    std::vector<int> nearest_task_set;

    int current_task;
    int chosen_task;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    rs_indi.sequence.clear();
    rs_indi.route_seg_load.clear();

    //insert dummy task at the very beginning
    rs_indi.sequence.push_back(DUMMY);

    //统计未服务任务
    for (int i = 1; i <= mcgrp.actual_task_num; i++) {
        unserved_task_id_set.push_back(i);
    }

    load = 0;
    trial = 0;
    while (trial < serve_task_num) {
        current_task = rs_indi.sequence.back();

        //Find all the tasks that satisfy the capacity constraint
        candidate_task_set.clear();
        for (auto unserved_task : unserved_task_id_set) {
            if (mcgrp.inst_tasks[unserved_task].demand <= mcgrp.capacity - load) {
                candidate_task_set.push_back(unserved_task);
            }
        }


        if (candidate_task_set.empty()) {
            rs_indi.sequence.push_back(DUMMY);
            rs_indi.route_seg_load.push_back(load);
            load = 0;
            continue;
        }

        mindist = std::numeric_limits<decltype(mindist)>::max();

        //Find the nearest task from the current cndidate task set
        for (auto candidate_task : candidate_task_set) {
            if (mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[candidate_task].head_node]
                < mindist) {
                mindist = mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[candidate_task]
                    .head_node];
                nearest_task_set.clear();
                nearest_task_set.push_back(candidate_task);
            }
            else if (
                mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[candidate_task].head_node]
                    == mindist) {
                nearest_task_set.push_back(candidate_task);
            }
        }


        //If multiple tasks both satisfy the capacity constraint and are closest, randomly choose one
        int k = (int) mcgrp._rng.Randint(0, nearest_task_set.size() - 1);
        chosen_task = nearest_task_set[k];


        trial++;
        rs_indi.sequence.push_back(chosen_task);

        load += mcgrp.inst_tasks[chosen_task].demand;

        unserved_task_id_set.erase(std::find_if(unserved_task_id_set.begin(),
                                                unserved_task_id_set.end(),
                                                [chosen_task](const int &elem) -> bool
                                                { return elem == chosen_task; }));

        if (mcgrp.inst_tasks[chosen_task].inverse != ARC_NO_INVERSE
            && mcgrp.inst_tasks[chosen_task].inverse != NODE_NO_INVERSE) {
            int inverse_task = mcgrp.inst_tasks[chosen_task].inverse;
            unserved_task_id_set.erase(std::find_if(unserved_task_id_set.begin(),
                                                    unserved_task_id_set.end(),
                                                    [inverse_task](const int &elem) -> bool
                                                    { return elem == inverse_task; }));
        }
    }

    if (rs_indi.sequence.back() != DUMMY) {
        rs_indi.sequence.push_back(DUMMY);
        rs_indi.route_seg_load.push_back(load);
    }

    rs_indi.total_cost = mcgrp.get_task_seq_total_cost(rs_indi.sequence);
    rs_indi.total_vio_load = mcgrp.get_total_vio_load(rs_indi.route_seg_load);
    My_Assert(rs_indi.total_vio_load == 0, "Should construct a feasible sequence here!");
}

const int max_merge_set_num = 20;

void merge_split(NeighBorSearch &ns, const MCGRP &mcgrp, const int merge_size, const int pseudo_capacity)
{
    My_Assert(!ns.delimiter_coding_sol.empty(), "Neighbor search doesn't hold a sequence!");

    if (ns.routes.size() < merge_size) {
        return;
    }


    COMB combination_manager(ns.routes.size(), merge_size);

    int count = 0;
    vector<int> chosen_routes_id;

    double best_choice = numeric_limits<decltype(best_choice)>::max();
    vector<int> best_buffer;

    while (count < max_merge_set_num && combination_manager.get_combinations(chosen_routes_id)) {
        count++;

        vector<vector<int>> task_routes(ns.routes.size());

        //generate tasks disturbution in routes
        int cursor = -1;
        for (auto task : ns.negative_coding_sol) {
            if (task < 0) {
                cursor++;
                task_routes[cursor].push_back(-task);
            }
            else if (task > 0) {
                task_routes[cursor].push_back(task);
            }
            else {
                My_Assert(false, "dummy task cannot occur in negative coding soluiton!");
            }
        }

        //extract chosen tasks
        vector<int> candidate_tasks;
        for (auto route_id : chosen_routes_id) {
            for (auto task : task_routes[route_id]) {
                candidate_tasks.push_back(task);

                if (mcgrp.is_edge(task)) {
                    candidate_tasks.push_back(mcgrp.inst_tasks[task].inverse);
                }
            }
        }


        vector<int> seq_buffer;
        seq_buffer = split_task(ns.policy,mcgrp, candidate_tasks,pseudo_capacity);
        My_Assert(seq_buffer.back() == DUMMY, "incorrect sequence!");
        seq_buffer.pop_back();

        //concatenate unchosen routes
        for (auto route_id = 0; route_id < ns.routes.size(); route_id++) {
            if (find(chosen_routes_id.begin(), chosen_routes_id.end(), route_id) == chosen_routes_id.end()) {
                seq_buffer.push_back(DUMMY);
                for (auto task : task_routes[route_id]) {
                    seq_buffer.push_back(task);
                }
            }
        }
        My_Assert(seq_buffer.back() != DUMMY, "incorrect sequence!");
        seq_buffer.push_back(DUMMY);

        //Best Accept
        Individual candidate_choice = mcgrp.parse_delimiter_seq(seq_buffer);
        if (candidate_choice.total_cost < best_choice) {
            best_buffer = seq_buffer;
        }
    }

    //Update neighbor search info
    ns.unpack_seq(best_buffer, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);
    //    ns.create_individual(mcgrp, ns.ns_indi);

    //check for missed
    ns.check_missed(mcgrp);
}



void merge_split(class HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int merge_size, const int pseudo_capacity){
    if (ns.routes.activated_route_id.size() < merge_size) {
        DEBUG_PRINT("Too few routes to merge!");
        return;
    }
    else if(ns.routes.activated_route_id.size() == merge_size){
        DEBUG_PRINT("Merge whole routes");
        ns.clear();

        auto candidate_tasks = ns.get_tasks_set();
        auto seq_buffer = split_task(ns.policy,mcgrp, candidate_tasks,pseudo_capacity);

        ns.unpack_seq(get_delimiter_coding(mcgrp.best_sol_buff), mcgrp);

        return;
    }

    My_Assert(ns.valid_sol(mcgrp),"Wrong state!");


    //generate tasks disturbution in routes
    unordered_map<int,vector<int>> task_routes;
    for(auto route_id : ns.routes.activated_route_id){
        int cur_task = ns.routes[route_id]->start;
        vector<int> buffer;
        while (cur_task != ns.routes[route_id]->end){
            buffer.push_back(cur_task);
            cur_task = ns.solution[cur_task]->next->ID;
        }
        buffer.push_back(cur_task);
        task_routes.emplace(route_id,buffer);
    }


    vector<int> total_routes_id;
    for(auto i:ns.routes.activated_route_id){
        total_routes_id.push_back(i);
    }
    COMB combination_manager(ns.routes.activated_route_id.size(), merge_size);
    vector<int> permutation;


    double best_choice = numeric_limits<decltype(best_choice)>::max();
    vector<int> best_buffer;
    vector<int> best_routes;

    int count = 0;
    while (count < max_merge_set_num && combination_manager.get_combinations(permutation)) {
        count++;
        My_Assert(permutation.size() == merge_size,"Wrong state!");

        vector<int> chosen_routes_id;
        for(auto i: permutation){
            chosen_routes_id.push_back(total_routes_id[i]);
        }

        //extract chosen tasks
        vector<int> candidate_tasks;
        for (auto route_id : chosen_routes_id) {
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");


            for (auto task : task_routes.at(route_id)) {
                candidate_tasks.push_back(task);

                if (mcgrp.is_edge(task)) {
                    candidate_tasks.push_back(mcgrp.inst_tasks[task].inverse);
                }
            }
        }


        vector<int> seq_buffer = split_task(ns.policy,mcgrp, candidate_tasks,pseudo_capacity);
        My_Assert(seq_buffer.front() == DUMMY && seq_buffer.back() == DUMMY, "Incorrect sequence!");

        //Best Accept
        Individual res = mcgrp.parse_delimiter_seq(seq_buffer);
        double fitness = res.total_cost + ns.policy.beta * res.total_vio_load;
        if (fitness < best_choice) {
            best_choice = fitness;
            best_buffer = seq_buffer;
            best_routes = chosen_routes_id;
        }
    }

    //Update neighbor search info
    My_Assert(!best_buffer.empty() && !best_routes.empty(),"Wrong state");
    ns.update(mcgrp,best_buffer,best_routes);

}



vector<int> split_task(Policy& policy, const MCGRP &mcgrp, const vector<int> &tasks,const int constraint)
{
    vector<int> merge_sequence;

    Individual res;
    double fitness;
    double buffer;


    /*-----------------Nearest L2 distance merge policy--------------------------*/
    merge_sequence = nearest_growing(mcgrp, tasks, constraint);
    Individual nearest_L2_indi;
    nearest_L2_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    res = nearest_L2_indi;
    fitness = res.total_cost + policy.beta * res.total_vio_load;
    /*-----------------Nearest L2 distance merge policy--------------------------*/

    /*-----------------Furthest L2 distance merge policy--------------------------*/
    merge_sequence = nearest_depot_growing(mcgrp, tasks,constraint);
    Individual nearest_depot_indi;
    nearest_depot_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = nearest_depot_indi.total_cost + policy.beta * nearest_depot_indi.total_vio_load;
    if (buffer < fitness) {
        res = nearest_depot_indi;
        fitness = buffer;
    }
    /*-----------------Furthest L2 distance merge policy--------------------------*/

    /*-----------------Max yield merge policy--------------------------*/
    merge_sequence = maximum_yield_growing(mcgrp, tasks,constraint);
    Individual maximum_yield_indi;
    maximum_yield_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = maximum_yield_indi.total_cost + policy.beta * maximum_yield_indi.total_vio_load;
    if (buffer < fitness) {
        res = maximum_yield_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/

    /*-----------------Min yield merge policy--------------------------*/
    merge_sequence = minimum_yield_growing(mcgrp, tasks,constraint);
    Individual minimum_yield_indi;
    minimum_yield_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = minimum_yield_indi.total_cost + policy.beta * minimum_yield_indi.total_vio_load;
    if (buffer < fitness) {
        res = minimum_yield_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/

    /*-----------------mixtured merge policy--------------------------*/
    merge_sequence = mixtured_growing(mcgrp, tasks,constraint);
    Individual mixtured_indi;
    mixtured_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = mixtured_indi.total_cost + policy.beta * mixtured_indi.total_vio_load;
    if (buffer < fitness) {
        res = mixtured_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/

    return res.sequence;

}

vector<int> nearest_growing(const MCGRP &mcgrp, vector<int> tasks, const int constraint)
{
    vector<int> sequence;

    sequence.push_back(DUMMY);
    int current_task;
    int current_load = 0;
    vector<int> candidate_tasks;
    vector<int> nearest_tasks;

    while (!tasks.empty()) {
        current_task = sequence.back();

        candidate_tasks.clear();
        for (auto task : tasks) {
            if (mcgrp.inst_tasks[task].demand <= constraint - current_load) {
                candidate_tasks.push_back(task);
            }
        }

        //check whether need to create a new route
        if (candidate_tasks.empty()) {
            sequence.push_back(DUMMY);
            current_load = 0;
            continue;
        }

        double min_dist = numeric_limits<decltype(min_dist)>::max();
        nearest_tasks.clear();
        for (auto task:candidate_tasks) {
            if (mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[task].head_node] < min_dist) {
                min_dist = mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[task].head_node];
                nearest_tasks.clear();
                nearest_tasks.push_back(task);
            }
            else if (mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[task].head_node]
                == min_dist) {
                nearest_tasks.push_back(task);
            }
            else {
                continue;
            }
        }

        My_Assert(!nearest_tasks.empty(), "you cannot have an empty nearest task set!");

        int chosen_task = nearest_tasks[0];
        sequence.push_back(chosen_task);
        current_load += mcgrp.inst_tasks[chosen_task].demand;


        auto ite = find(tasks.begin(), tasks.end(), chosen_task);
        My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
        tasks.erase(ite);

        if (mcgrp.is_edge(chosen_task)) {
            ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);
        }
    }

    if (sequence.back() != DUMMY) {
        sequence.push_back(DUMMY);
    }

    return sequence;
}

vector<int> nearest_depot_growing(const MCGRP &mcgrp, vector<int> tasks, const int constraint)
{
    vector<int> sequence;

    sequence.push_back(DUMMY);
    int current_load = 0;
    vector<int> candidate_tasks;
    vector<int> nearest_tasks;

    while (!tasks.empty()) {
        candidate_tasks.clear();
        for (auto task : tasks) {
            if (mcgrp.inst_tasks[task].demand <= constraint - current_load) {
                candidate_tasks.push_back(task);
            }
        }

        //check whether need to create a new route
        if (candidate_tasks.empty()) {
            sequence.push_back(DUMMY);
            current_load = 0;
            continue;
        }

        double min_dist = numeric_limits<decltype(min_dist)>::max();
        nearest_tasks.clear();

        for (auto task:candidate_tasks) {
            if (mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node] < min_dist) {
                min_dist = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
                nearest_tasks.clear();
                nearest_tasks.push_back(task);
            }
            else if (mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node] == min_dist) {
                nearest_tasks.push_back(task);
            }
            else {
                continue;
            }
        }

        My_Assert(!nearest_tasks.empty(), "you cannot have an empty nearest task set!");

        int chosen_task = nearest_tasks[0];
        sequence.push_back(chosen_task);
        current_load += mcgrp.inst_tasks[chosen_task].demand;


        auto ite = find(tasks.begin(), tasks.end(), chosen_task);
        My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
        tasks.erase(ite);

        if (mcgrp.is_edge(chosen_task)) {
            ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);
        }
    }

    if (sequence.back() != DUMMY) {
        sequence.push_back(DUMMY);
    }

    return sequence;
}

vector<int> maximum_yield_growing(const MCGRP &mcgrp, vector<int> tasks, const int constraint)
{
    vector<int> sequence;

    sequence.push_back(DUMMY);
    int current_load = 0;
    vector<int> candidate_tasks;
    vector<int> maximum_tasks;

    while (!tasks.empty()) {
        candidate_tasks.clear();
        for (auto task : tasks) {
            if (mcgrp.inst_tasks[task].demand <= constraint - current_load) {
                candidate_tasks.push_back(task);
            }
        }

        //check whether need to create a new route
        if (candidate_tasks.empty()) {
            sequence.push_back(DUMMY);
            current_load = 0;
            continue;
        }

        double max_yield = -1;
        maximum_tasks.clear();

        for (auto task:candidate_tasks) {
            if (mcgrp.get_yield(task) > max_yield) {
                max_yield = mcgrp.get_yield(task);
                maximum_tasks.clear();
                maximum_tasks.push_back(task);
            }
            else if (mcgrp.get_yield(task) == max_yield) {
                maximum_tasks.push_back(task);
            }
            else {
                continue;
            }
        }

        My_Assert(!maximum_tasks.empty(), "you cannot have an empty nearest task set!");

        int chosen_task = maximum_tasks[0];
        sequence.push_back(chosen_task);
        current_load += mcgrp.inst_tasks[chosen_task].demand;


        auto ite = find(tasks.begin(), tasks.end(), chosen_task);
        My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
        tasks.erase(ite);

        if (mcgrp.is_edge(chosen_task)) {
            ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);
        }
    }

    if (sequence.back() != DUMMY) {
        sequence.push_back(DUMMY);
    }

    return sequence;
}

vector<int> minimum_yield_growing(const MCGRP &mcgrp, vector<int> tasks, const int constraint)
{
    vector<int> sequence;

    sequence.push_back(DUMMY);
    int current_load = 0;
    vector<int> candidate_tasks;
    vector<int> minimum_tasks;

    while (!tasks.empty()) {
        candidate_tasks.clear();
        for (auto task : tasks) {
            if (mcgrp.inst_tasks[task].demand <= constraint - current_load) {
                candidate_tasks.push_back(task);
            }
        }

        //check whether need to create a new route
        if (candidate_tasks.empty()) {
            sequence.push_back(DUMMY);
            current_load = 0;
            continue;
        }

        double min_yield = numeric_limits<decltype(min_yield)>::max();
        minimum_tasks.clear();

        for (auto task:candidate_tasks) {
            if (mcgrp.get_yield(task) < min_yield) {
                min_yield = mcgrp.get_yield(task);
                minimum_tasks.clear();
                minimum_tasks.push_back(task);
            }
            else if (mcgrp.get_yield(task) == min_yield) {
                minimum_tasks.push_back(task);
            }
            else {
                continue;
            }
        }

        My_Assert(!minimum_tasks.empty(), "you cannot have an empty nearest task set!");

        int chosen_task = minimum_tasks[0];
        sequence.push_back(chosen_task);
        current_load += mcgrp.inst_tasks[chosen_task].demand;


        auto ite = find(tasks.begin(), tasks.end(), chosen_task);
        My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
        tasks.erase(ite);

        if (mcgrp.is_edge(chosen_task)) {
            ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);
        }
    }

    if (sequence.back() != DUMMY) {
        sequence.push_back(DUMMY);
    }

    return sequence;
}

vector<int> mixtured_growing(const MCGRP &mcgrp, vector<int> tasks, const int constraint)
{
    vector<int> sequence;

    sequence.push_back(DUMMY);
    int current_load = 0;
    vector<int> candidate_tasks;
    vector<int> potential_tasks;

    while (!tasks.empty()) {
        candidate_tasks.clear();
        for (auto task : tasks) {
            if (mcgrp.inst_tasks[task].demand <= constraint - current_load) {
                candidate_tasks.push_back(task);
            }
        }

        //check whether need to create a new route
        if (candidate_tasks.empty()) {
            sequence.push_back(DUMMY);
            current_load = 0;
            continue;
        }

        if (current_load < constraint / 2) {
            //minmum depot didtance
            double min_dist = numeric_limits<decltype(min_dist)>::max();
            potential_tasks.clear();

            for (auto task:candidate_tasks) {
                if (mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node] < min_dist) {
                    min_dist = mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
                    potential_tasks.clear();
                    potential_tasks.push_back(task);
                }
                else if (mcgrp.min_cost[mcgrp.inst_tasks[task].tail_node][mcgrp.inst_tasks[DUMMY].head_node]
                    == min_dist) {
                    potential_tasks.push_back(task);
                }
                else {
                    continue;
                }
            }

            My_Assert(!potential_tasks.empty(), "you cannot have an empty nearest task set!");

            int chosen_task = potential_tasks[0];
            sequence.push_back(chosen_task);
            current_load += mcgrp.inst_tasks[chosen_task].demand;


            auto ite = find(tasks.begin(), tasks.end(), chosen_task);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);

            if (mcgrp.is_edge(chosen_task)) {
                ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
                My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
                tasks.erase(ite);
            }
        }
        else {
            //maximum yield merging
            double max_yield = -1;
            potential_tasks.clear();

            for (auto task:candidate_tasks) {
                if (mcgrp.get_yield(task) > max_yield) {
                    max_yield = mcgrp.get_yield(task);
                    potential_tasks.clear();
                    potential_tasks.push_back(task);
                }
                else if (mcgrp.get_yield(task) == max_yield) {
                    potential_tasks.push_back(task);
                }
                else {
                    continue;
                }
            }

            My_Assert(!potential_tasks.empty(), "you cannot have an empty nearest task set!");

            int chosen_task = potential_tasks[0];
            sequence.push_back(chosen_task);
            current_load += mcgrp.inst_tasks[chosen_task].demand;

            auto ite = find(tasks.begin(), tasks.end(), chosen_task);
            My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
            tasks.erase(ite);

            if (mcgrp.is_edge(chosen_task)) {
                ite = find(tasks.begin(), tasks.end(), mcgrp.inst_tasks[chosen_task].inverse);
                My_Assert(ite != tasks.end(), "Cannot find chosen tasks!");
                tasks.erase(ite);
            }
        }
    }

    if (sequence.back() != DUMMY) {
        sequence.push_back(DUMMY);
    }

    return sequence;
}