#include "DoubleInsert.h"
#include "MoveString.h"
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;


bool DoubleInsert::search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
{
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    MCGRPMOVE BestM;

    ns.create_search_neighborhood(mcgrp, chosen_task);

    vector<int> chosen_seq; //参与扰动的任务序列
    chosen_seq = get_successor_tasks(ns, chosen_task);        //保证参与扰动序列在同一路径内

    My_Assert(chosen_seq.size()>0,"You cannot generate an empty sequence!");

    if (chosen_seq.size() != length) {
        DEBUG_PRINT("The length after chosen task is not two in the double insert");
        return false;
    }

    int b = chosen_seq.front();

    for(auto neighbor_task : ns.search_space) {
        My_Assert(neighbor_task != b, "neighbor task can't be itself!");

        if (neighbor_task != DUMMY) {
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;

            if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                // doesn't overlap
                if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                    if (ns.policy.has_rule(FIRST_ACCEPT)) {
                        move(ns, mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)) {
                        if (ns.policy.check_result(move_result, BestM))
                            BestM = move_result;
                    }
                    else {
                        My_Assert(false, "Unknown accept rule!");
                    }
                }
            }
        }
        else {
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here and b can't be dummy neither
            //each start and end location of each route will be considered
            //total 2 x route_nums cases

            int current_start = abs(ns.next_array[DUMMY]);

            while (current_start != DUMMY) {
                // Consider the start location
                int j = current_start;

                if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                    // doesn't overlap
                    if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                        if (ns.policy.has_rule(FIRST_ACCEPT)) {
                            move(ns, mcgrp);
                            return true;
                        }
                        else if (ns.policy.has_rule(BEST_ACCEPT)) {
                            if (ns.policy.check_result(move_result, BestM))
                                BestM = move_result;
                        }
                        else {
                            My_Assert(false, "Unknown accept rule!");
                        }
                    }
                }

                // Consider the end location
                const int current_route = ns.route_id[current_start];
                const int current_end = ns.routes[current_route].end;
                j = current_end;
                if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                    // doesn't overlap
                    if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                        if (ns.policy.has_rule(FIRST_ACCEPT)) {
                            move(ns, mcgrp);
                            return true;
                        }
                        else if (ns.policy.has_rule(BEST_ACCEPT)) {
                            if (ns.policy.check_result(move_result, BestM))
                                BestM = move_result;
                        }
                        else {
                            My_Assert(false, "Unknown accept rule!");
                        }
                    }
                }

                // Advance to next route's starting task
                current_start = abs(ns.next_array[current_end]);
            }
        }
    }

    if (ns.policy.has_rule(FIRST_ACCEPT)) {
        DEBUG_PRINT("No actual move: First Accept Rule");
        return false;
    }
    else if (ns.policy.has_rule(BEST_ACCEPT)) {
        if (BestM.considerable == false) {
            DEBUG_PRINT("No actual move: Best Accept Rule");
            return false;
        }
        else {
            move_result = BestM;
            move(ns, mcgrp);
            return true;
        }
    }
    else {
        My_Assert(false, "Unknown accept rule");
    }

}

vector<int> DoubleInsert::get_successor_tasks(NeighBorSearch &ns, const int chosen_task)
{
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    int current_node;
    vector<int> buffer;

    current_node = chosen_task;
    buffer.push_back(current_node);
    while (buffer.size() < length) {
        current_node = ns.next_array[current_node];

        if (current_node <= DUMMY) {
            return buffer;
        }
        else {
            buffer.push_back(current_node);
        }
    }

    return buffer;
}

bool DoubleInsert::considerable_move(NeighBorSearch &ns,
                                     const MCGRP &mcgrp,
                                     vector<int> disturbance_seq,
                                     const int j)
{
    My_Assert(j != DUMMY,"You cannot insert a dummy task!");
    My_Assert(!disturbance_seq.empty(),"disturbance seq can't be empty!");
    My_Assert(find(disturbance_seq.begin(),disturbance_seq.end(),DUMMY) == disturbance_seq.end(),"You cannot insert a dummy task!");
    My_Assert(find(disturbance_seq.begin(),disturbance_seq.end(),j) == disturbance_seq.end(),"Overlap");

    const int k = max(ns.next_array[j], DUMMY);

    if(k == disturbance_seq.front()){
        //suppose disturbance is a-b
        // ...i-j-k(a)-b...
        //no need do postsert
        move_result.reset();
        move_result.move_type = NeighborOperator::DOUBLE_INSERT;
        return false;
    }
    else{
        if (move_string.considerable_move(ns, mcgrp, disturbance_seq, j)
            && ns.policy.check_move(move_string.move_result)) {
            move_result = move_string.move_result;
            return true;
        }
        else {
            move_result.reset();
            move_result.move_type = NeighborOperator::DOUBLE_INSERT;
            return false;
        }
    }
}

void DoubleInsert::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a double insert move");
    My_Assert(move_result.considerable,"Invalid predictions");

    if(move_result.move_type == NeighborOperator::MOVE_STRING){
        move_string.move_result = move_result;
        move_string.move(ns, mcgrp);
    }
    else{
        My_Assert(false,"Uknown operator");
    }

    if (ns.total_vio_load == 0) {
        mcgrp.check_best_solution(ns.cur_solution_cost, ns.negative_coding_sol);
    }

//    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.policy.beta,ns.total_vio_load,ns.negative_coding_sol);


    move_result.reset();
    move_result.move_type = NeighborOperator::DOUBLE_INSERT;
}

void DoubleInsert::unit_test(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    mcgrp._rng.RandPerm(task_set);    //shuffle tasks

    auto original_policy = ns.policy.get();
    ns.policy.set(FIRST_ACCEPT | TOLERANCE | DELTA_ONLY);
    ns.policy.beta = 0.5;
    ns.policy.tolerance = 0.003;
    ns.neigh_size = mcgrp.neigh_size;

    int chosen_task = -1;
    for (int i = 0; i < mcgrp.actual_task_num; i++) {
        chosen_task = task_set[i];

        if (ns.next_array[chosen_task] == std::numeric_limits<identity<decltype(ns
            .next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
            if (!mcgrp.is_edge(chosen_task)) {    //非边任务一定在解序列中
                cerr << "A non edge task has been missed!\n";
                abort();
            }
            else {
                chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                My_Assert(ns.next_array[chosen_task]
                              != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max(),
                          "An edge task has been missed");
            }
        }

        search(ns, mcgrp, chosen_task);
    }

    ns.neigh_size = 0;
    ns.policy.set(original_policy);
    ns.policy.beta = 0;
    ns.policy.tolerance = 0;
}



/*
 * High Speed
 */


bool DoubleInsert::search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
{
    My_Assert(chosen_task >= 1 && chosen_task <= mcgrp.actual_task_num,"Wrong task");

    presert_times = 0;
    postsert_times = 0;

    MCGRPMOVE BestM;

    ns.create_search_neighborhood(mcgrp, chosen_task);

    vector<int> chosen_seq = get_successor_tasks(ns, chosen_task);

    My_Assert(chosen_seq.size()>0,"You cannot generate an empty sequence!");

    if (chosen_seq.size() != length) {
        DEBUG_PRINT("The length after chosen task is not two in the double insert");
        return false;
    }

    int b = chosen_seq.front();

    for(auto neighbor_task : ns.search_space) {
        My_Assert(neighbor_task != b, "neighbor task can't be itself!");

        if (neighbor_task != DUMMY) {
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;

            if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                // doesn't overlap
                if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                    if (ns.policy.has_rule(FIRST_ACCEPT)) {
                        move(ns, mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)) {
                        if (ns.policy.check_result(move_result, BestM))
                            BestM = move_result;
                    }
                    else {
                        My_Assert(false, "Unknown accept rule!");
                    }
                }
            }
        }
        else {
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here and b can't be dummy neither
            //each start and end location of each route will be considered
            //total 2 x route_nums cases

            int current_start = ns.solution.very_start->next->ID;

            while (ns.solution[current_start]->next != ns.solution.very_end) {
                // Consider the start location
                int j = current_start;

                if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                    // doesn't overlap
                    if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                        if (ns.policy.has_rule(FIRST_ACCEPT)) {
                            move(ns, mcgrp);
                            return true;
                        }
                        else if (ns.policy.has_rule(BEST_ACCEPT)) {
                            if (ns.policy.check_result(move_result, BestM))
                                BestM = move_result;
                        }
                        else {
                            My_Assert(false, "Unknown accept rule!");
                        }
                    }
                }

                // Consider the end location
                const int current_route = ns.solution[current_start]->route_id;
                const int current_end = ns.routes[current_route]->end;
                j = current_end;
                if (std::find(chosen_seq.begin(), chosen_seq.end(), j) == chosen_seq.end()) {
                    // doesn't overlap
                    if (considerable_move(ns, mcgrp, chosen_seq, j)) {
                        if (ns.policy.has_rule(FIRST_ACCEPT)) {
                            move(ns, mcgrp);
                            return true;
                        }
                        else if (ns.policy.has_rule(BEST_ACCEPT)) {
                            if (ns.policy.check_result(move_result, BestM))
                                BestM = move_result;
                        }
                        else {
                            My_Assert(false, "Unknown accept rule!");
                        }
                    }
                }

                // Advance to next route's starting task
                current_start = ns.solution[current_end]->next->next->ID;
            }
        }
    }

    if (ns.policy.has_rule(FIRST_ACCEPT)) {
        DEBUG_PRINT("No actual move: First Accept Rule");
        return false;
    }
    else if (ns.policy.has_rule(BEST_ACCEPT)) {
        if (BestM.considerable == false) {
            DEBUG_PRINT("No actual move: Best Accept Rule");
            return false;
        }
        else {
            move_result = BestM;
            move(ns, mcgrp);
            return true;
        }
    }
    else {
        My_Assert(false, "Unknown accept rule");
    }

    DEBUG_PRINT("Double insert presert times:" + to_string(presert_times));
    DEBUG_PRINT("Double insert postsert times:" + to_string(postsert_times));
}

vector<int> DoubleInsert::get_successor_tasks(HighSpeedNeighBorSearch &ns, const int chosen_task)
{
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    int current_node;
    vector<int> buffer;

    current_node = chosen_task;
    buffer.push_back(current_node);
    while (buffer.size() < length) {
        current_node = ns.solution[current_node]->next->ID;

        if (current_node < 0) {
            return buffer;
        }
        else {
            buffer.push_back(current_node);
        }
    }

    return buffer;
}

bool DoubleInsert::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, vector<int> disturbance_seq, const int j)
{

    My_Assert(j >= 1 && j <= mcgrp.actual_task_num,"Wrong task");

    My_Assert(!disturbance_seq.empty(),"disturbance seq can't be empty!");
    My_Assert(all_of(disturbance_seq.begin(),disturbance_seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");
    My_Assert(find(disturbance_seq.begin(),disturbance_seq.end(),j) == disturbance_seq.end(),"Overlap");

    const int i = max(ns.solution[j]->pre->ID, 0);
    const int k = max(ns.solution[j]->next->ID, 0);

    //suppose disturbance sequenec is dis(0)-dis(1)-dis(2)
    if(disturbance_seq.back() == i){
        //...dis(0)-dis(1)-dis(2)(i)-j-k...
        //no need to presert
        postsert_times++;
        if (post_move_string.considerable_move(ns, mcgrp, disturbance_seq, j) && ns.policy.check_move(post_move_string.move_result)) {
            move_result = post_move_string.move_result;
            return true;
        }
        else {
            move_result.reset();
            move_result.move_type = NeighborOperator::DOUBLE_INSERT;
            return false;
        }
    }
    else if (disturbance_seq.front() == k){
        //...i-j-dis(0)(k)-dis(1)-dis(2)...
        //no need to postsert
        presert_times++;
        if (pre_move_string.considerable_move(ns, mcgrp, disturbance_seq, j) && ns.policy.check_move(pre_move_string.move_result)) {
            move_result = pre_move_string.move_result;
            return true;
        }
        else {
            move_result.reset();
            move_result.move_type = NeighborOperator::DOUBLE_INSERT;
            return false;
        }
    }
    else{
        //...dis(0)-dis(1)-dis(2)...
        //...i-j-k...

        //check presert and postsert
        pre_move_string.considerable_move(ns, mcgrp, disturbance_seq, j);
        post_move_string.considerable_move(ns, mcgrp, disturbance_seq, j);

        if(pre_move_string.move_result.considerable && post_move_string.move_result.considerable){
            //Both considerable
            if(pre_move_string.move_result.delta <= post_move_string.move_result.delta){
                presert_times++;
                if(ns.policy.check_move(pre_move_string.move_result)){
                    move_result = pre_move_string.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::DOUBLE_INSERT;
                    return false;
                }
            }
            else{
                postsert_times++;
                if(ns.policy.check_move(post_move_string.move_result)){
                    move_result = post_move_string.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::DOUBLE_INSERT;
                    return false;
                }
            }
        }
        else if (pre_move_string.move_result.considerable){
            //only presert is considerable
            presert_times++;
            if(ns.policy.check_move(pre_move_string.move_result)){
                move_result = pre_move_string.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::DOUBLE_INSERT;
                return false;
            }
        }
        else if (post_move_string.move_result.considerable){
            postsert_times++;
            if(ns.policy.check_move(post_move_string.move_result)){
                move_result = post_move_string.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::DOUBLE_INSERT;
                return false;
            }
        }
        else{
            //No considerable
            move_result.reset();
            move_result.move_type = NeighborOperator::DOUBLE_INSERT;
            return false;
        }
    }

    My_Assert(false,"Cannot reach here!");
}

void DoubleInsert::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a double insert move");

    My_Assert(move_result.considerable,"Invalid predictions");

    if(move_result.move_type == NeighborOperator::PRE_MOVE_STRING){
        pre_move_string.move_result = move_result;
        pre_move_string.move(ns,mcgrp);
    }
    else if (move_result.move_type == NeighborOperator::POST_MOVE_STRING)
    {
        post_move_string.move_result = move_result;
        post_move_string.move(ns,mcgrp);
    }
    else{
        My_Assert(false,"Uknown operator");
    }

    ns.trace(mcgrp);


//    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.policy.beta,ns.total_vio_load,ns.negative_coding_sol);

    move_result.reset();
    move_result.move_type = NeighborOperator::DOUBLE_INSERT;
}

void DoubleInsert::unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    mcgrp._rng.RandPerm(task_set);    //shuffle tasks

    auto original_policy = ns.policy.get();
    ns.policy.set(FIRST_ACCEPT | TOLERANCE | FITNESS_ONLY);
    ns.policy.beta = 0.5;
    ns.policy.tolerance = 0.003;
    ns.neigh_size = mcgrp.neigh_size;

    int chosen_task = -1;
    for (int i = 0; i < mcgrp.actual_task_num; i++) {
        chosen_task = task_set[i];

        if (ns.solution[chosen_task]->next == nullptr) {
            if (mcgrp.is_edge(chosen_task)) {
                chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                My_Assert(ns.solution[chosen_task]->next != nullptr,"An edge task has been missed");
            }
            else {
                My_Assert(false,"A non edge task has been missed!");
            }
        }

        search(ns, mcgrp, chosen_task);
    }

    ns.neigh_size = 0;
    ns.policy.set(original_policy);
    ns.policy.beta = 0;
    ns.policy.tolerance = 0;
}