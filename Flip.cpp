#include "Flip.h"
#include <algorithm>

using namespace std;

MCGRPMOVE Flip::move_result = MCGRPMOVE(NeighborOperator::FLIP);

vector<int> Flip::get_sequence(NeighBorSearch &ns, const int start, const int end)
{

    const auto start_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), start);
    const auto end_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), end);

    My_Assert(start_ite != ns.delimiter_coding_sol.end() && end_ite != ns.delimiter_coding_sol.end() && start_ite != end_ite,
              "Can't find task!");

    vector<int> buffer;
    buffer.reserve((ns.delimiter_coding_sol.size() / 2));

    for (auto ite = start_ite; ite != end_ite; ite++) {
        buffer.push_back(*ite);
    }
    buffer.push_back(*end_ite);

    return buffer;
}

bool Flip::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task)
{
    My_Assert(start_task != DUMMY && end_task != DUMMY, "Flip can't handle dummy task!");

    vector<int> original_sequence = get_sequence(ns, start_task, end_task);

    const int start_route = ns.route_id[start_task];
    const int end_route = ns.route_id[end_task];
    My_Assert(start_route == end_route, "can't flip cross routes!");

    move_result.choose_tasks(start_task, end_task);
    move_result.move_arguments = original_sequence;

    vector<int> candidate_seq;
    candidate_seq.reserve(original_sequence.size());
    for (auto cursor = 0; cursor < original_sequence.size(); cursor++) {
        //Base on probobality
        if (mcgrp.is_edge(original_sequence[cursor]) && mcgrp._rng.Randfloat(0, 1) > 0.5) {
            candidate_seq.push_back(mcgrp.inst_tasks[original_sequence[cursor]].inverse);
        }
        else {
            candidate_seq.push_back(original_sequence[cursor]);
        }
    }

    const int pre_start = max(ns.pred_array[start_task], DUMMY);
    const int post_end = max(ns.next_array[end_task], DUMMY);

    move_result.total_number_of_routes = ns.routes.size();

    vector<int> actual_seq;
    actual_seq.reserve(original_sequence.size());
    double delta = 0;
    if (original_sequence == candidate_seq) {
        delta -= mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[original_sequence.front()]
            .head_node];
        for (auto cursor = 0; cursor < original_sequence.size() - 1; cursor++) {
            delta -= mcgrp.inst_tasks[original_sequence[cursor]].serv_cost;
            delta -= mcgrp.min_cost[mcgrp.inst_tasks[original_sequence[cursor]].tail_node][mcgrp
                .inst_tasks[original_sequence[cursor + 1]].head_node];
        }
        delta -= mcgrp.inst_tasks[original_sequence.back()].serv_cost;
        delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[original_sequence.back()].tail_node][mcgrp.inst_tasks[post_end].head_node];


        delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[original_sequence.back()].head_node];
        for (auto cursor = original_sequence.size() - 1; cursor > 0; --cursor) {
            delta += mcgrp.inst_tasks[original_sequence[cursor]].serv_cost;
            delta += mcgrp.min_cost[mcgrp.inst_tasks[original_sequence[cursor]].tail_node][mcgrp
                .inst_tasks[original_sequence[cursor - 1]].head_node];
        }
        delta += mcgrp.inst_tasks[original_sequence.front()].serv_cost;
        delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[original_sequence.front()].tail_node][mcgrp.inst_tasks[post_end].head_node];

        actual_seq = original_sequence;
    }
    else {
        double original_delta = 0;
        original_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[original_sequence.front()]
                .head_node];
        for (auto cursor = 0; cursor < original_sequence.size() - 1; cursor++) {
            original_delta -= mcgrp.inst_tasks[original_sequence[cursor]].serv_cost;
            original_delta -= mcgrp.min_cost[mcgrp.inst_tasks[original_sequence[cursor]].tail_node][mcgrp
                .inst_tasks[original_sequence[cursor + 1]].head_node];
        }
        original_delta -= mcgrp.inst_tasks[original_sequence.back()].serv_cost;
        original_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[original_sequence.back()].tail_node][mcgrp.inst_tasks[post_end].head_node];


        original_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[original_sequence.back()].head_node];
        for (auto cursor = original_sequence.size() - 1; cursor > 0; --cursor) {
            original_delta += mcgrp.inst_tasks[original_sequence[cursor]].serv_cost;
            original_delta += mcgrp.min_cost[mcgrp.inst_tasks[original_sequence[cursor]].tail_node][mcgrp
                .inst_tasks[original_sequence[cursor - 1]].head_node];
        }
        original_delta += mcgrp.inst_tasks[original_sequence.front()].serv_cost;
        original_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[original_sequence.front()].tail_node][mcgrp.inst_tasks[post_end].head_node];

        //////////////////////////////////////////////////////////////////////////////////////////////////
        double candidate_delta = 0;
        candidate_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[original_sequence.front()].head_node];
        for (auto cursor = 0; cursor < candidate_seq.size() - 1; cursor++) {
            candidate_delta -= mcgrp.inst_tasks[original_sequence[cursor]].serv_cost;
            candidate_delta -=
                mcgrp.min_cost[mcgrp.inst_tasks[original_sequence[cursor]].tail_node][mcgrp.inst_tasks[original_sequence[cursor
                    + 1]].head_node];
        }
        candidate_delta -= mcgrp.inst_tasks[original_sequence.back()].serv_cost;
        candidate_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[original_sequence.back()].tail_node][mcgrp.inst_tasks[post_end].head_node];


        candidate_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[pre_start].tail_node][mcgrp.inst_tasks[candidate_seq.back()].head_node];
        for (auto cursor = candidate_seq.size() - 1; cursor > 0; --cursor) {
            candidate_delta += mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
            candidate_delta +=
                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp.inst_tasks[candidate_seq[cursor
                    - 1]].head_node];
        }
        candidate_delta += mcgrp.inst_tasks[candidate_seq.front()].serv_cost;
        candidate_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.front()].tail_node][mcgrp.inst_tasks[post_end].head_node];

        if (original_delta <= candidate_delta) {
            delta = original_delta;
            actual_seq = original_sequence;
        }
        else {
            delta = candidate_delta;
            actual_seq = candidate_seq;
        }

    }

    move_result.num_affected_routes = 1;

    move_result.route_id.push_back(start_route);

    move_result.delta = delta;
    move_result.route_lens.push_back(ns.routes[start_route].length + move_result.delta);

    move_result.route_loads.push_back(ns.routes[start_route].load);

    move_result.route_custs_num.push_back(ns.routes[start_route].num_customers);

    reverse(actual_seq.begin(), actual_seq.end());
    move_result.move_arguments.insert(move_result.move_arguments.end(), actual_seq.begin(), actual_seq.end());

    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
    move_result.considerable = true;

    return true;
}

void Flip::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    My_Assert(move_result.move_arguments.size() % 2 == 0, "move arguments should be even!");

    //extract move arguments
    vector<int> original_seq(move_result.move_arguments.begin(),
                             move_result.move_arguments.begin() + move_result.move_arguments.size() / 2);
    vector<int> actual_seq
        (move_result.move_arguments.begin() + move_result.move_arguments.size() / 2, move_result.move_arguments.end());

    My_Assert(original_seq.size() == actual_seq.size(),
              "Sequence should be equal!");


    const int start_task = original_seq.front();
    const int end_task = original_seq.back();

    Individual individual;
    ns.create_individual(mcgrp, individual);

    My_Assert(ns.delimiter_coding_sol == individual.sequence,
              "Inconsistency between negative coding and delimiter coding.");

    //check for missed
    ns.check_missed(mcgrp);

    const auto ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), start_task);
    const auto ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), end_task);

    My_Assert(ite_start != individual.sequence.end() && ite_end != individual.sequence.end(),
              "Can't find corresponding disturbance task in sequence!");

    int cursor = 0;
    for (auto ite = ite_start; ite != ite_end; ite++) {
        *ite = actual_seq[cursor];
        ++cursor;
    }
    *ite_end = actual_seq.back();

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
//    ns.create_individual(mcgrp, ns.ns_indi);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");

    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

    //check for missed
    ns.check_missed(mcgrp);


    move_result.reset();
    move_result.move_type = NeighborOperator::FLIP;

    ns.search_step++;
}



bool NewFlip::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task){
    My_Assert(start_task != DUMMY && end_task != DUMMY, "Flip can't handle dummy task!");
    My_Assert(ns.route_id[start_task] == ns.route_id[end_task],"Flip attempted using different routes!");

    vector<int> candidate_seq = get_sequence(ns,start_task, end_task);
    //No need flipping
    if(candidate_seq.size() <= 1)
        return false;

    move_result.reset();
    move_result.choose_tasks(start_task, end_task);
    move_result.move_arguments = candidate_seq;

    double delta = 0;
    delta -= mcgrp.min_cost[mcgrp.inst_tasks[start_task].tail_node][mcgrp.inst_tasks[candidate_seq.front()].head_node];
    for(int cur = 0; cur < candidate_seq.size() - 1; cur++){
        delta -= mcgrp.inst_tasks[candidate_seq[cur]].serv_cost;
        delta -= mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cur]].tail_node][mcgrp.inst_tasks[candidate_seq[cur+1]].head_node];
    }
    delta -= mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
    delta -= mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[end_task].head_node];

    vector<int> actual_seq;
    reverse(candidate_seq.begin(),candidate_seq.end());

    if(mcgrp.is_edge(candidate_seq.front())){
        int inverse = mcgrp.inst_tasks[candidate_seq.front()].inverse;
        delta += mcgrp.min_cost[mcgrp.inst_tasks[start_task].tail_node][mcgrp.inst_tasks[inverse].head_node];
    }
    else{
        delta += mcgrp.min_cost[mcgrp.inst_tasks[start_task].tail_node][mcgrp.inst_tasks[candidate_seq.front()].head_node];
    }

    for(int cur = 0;cur<candidate_seq.size()-1;cur++){
        if(mcgrp.is_edge(candidate_seq[cur])){
            int inverse = mcgrp.inst_tasks[candidate_seq[cur]].inverse;
            actual_seq.push_back(inverse);
            delta += mcgrp.inst_tasks[inverse].serv_cost;
            if(mcgrp.is_edge(candidate_seq[cur+1])){
                int next_inverse = mcgrp.inst_tasks[candidate_seq[cur + 1]].inverse;
                delta += mcgrp.min_cost[mcgrp.inst_tasks[inverse].tail_node][mcgrp.inst_tasks[next_inverse].head_node];
            }
            else{
                delta += mcgrp.min_cost[mcgrp.inst_tasks[inverse].tail_node][mcgrp.inst_tasks[candidate_seq[cur + 1]].head_node];
            }
        }
        else{
            actual_seq.push_back(candidate_seq[cur]);
            delta += mcgrp.inst_tasks[candidate_seq[cur]].serv_cost;
            if(mcgrp.is_edge(candidate_seq[cur+1])){
                int next_inverse = mcgrp.inst_tasks[candidate_seq[cur + 1]].inverse;
                delta += mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cur]].tail_node][mcgrp.inst_tasks[next_inverse].head_node];
            }
            else{
                delta += mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cur]].tail_node][mcgrp.inst_tasks[candidate_seq[cur + 1]].head_node];
            }
        }
    }

    if(mcgrp.is_edge(candidate_seq.back())){
        int inverse = mcgrp.inst_tasks[candidate_seq.back()].inverse;
        actual_seq.push_back(inverse);
        delta += mcgrp.inst_tasks[inverse].serv_cost;
        delta += mcgrp.min_cost[mcgrp.inst_tasks[inverse].tail_node][mcgrp.inst_tasks[end_task].head_node];
    }
    else{
        actual_seq.push_back(candidate_seq.back());
        delta += mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
        delta += mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[end_task].head_node];
    }

    My_Assert(candidate_seq.size() == actual_seq.size(),"actual sequence doesn't have the same length with candidate sequence!");

    move_result.num_affected_routes = 1;

    int route_id = ns.route_id[start_task];
    move_result.route_id.push_back(route_id);
    move_result.delta = delta;

    move_result.route_lens.push_back(ns.routes[route_id].length + move_result.delta);
    move_result.route_loads.push_back(ns.routes[route_id].load);

    move_result.route_custs_num.push_back(ns.routes[route_id].num_customers);
    move_result.move_arguments.insert(move_result.move_arguments.end(), actual_seq.begin(), actual_seq.end());
    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
    move_result.considerable = true;

    return true;
}

void NewFlip::move(NeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt: swapends move");

    My_Assert(move_result.considerable,"Invalid predictions");
    My_Assert(move_result.move_arguments.size() % 2 == 0, "move arguments should be even!");

    //extract move arguments
    vector<int> original_seq(move_result.move_arguments.begin(),
                             move_result.move_arguments.begin() + move_result.move_arguments.size() / 2);
    vector<int> actual_seq
        (move_result.move_arguments.begin() + move_result.move_arguments.size() / 2, move_result.move_arguments.end());

    My_Assert(find(original_seq.begin(),original_seq.end(),DUMMY) == original_seq.end(),
        "Dummy tasks involve flip!");

    Individual individual;
    ns.create_individual(mcgrp, individual);

    My_Assert(ns.delimiter_coding_sol == individual.sequence,
              "Inconsistency between negative coding and delimiter coding.");

    //check for missed
    My_Assert(!ns.check_missed(mcgrp),"Some task has been missed!");


    const int start_task = original_seq.front();
    const int end_task = original_seq.back();

    const auto ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), start_task);
    const auto ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), end_task);


    My_Assert(ite_start != individual.sequence.end() && ite_end != individual.sequence.end(),
              "Can't find corresponding disturbance task in sequence!");

    int cursor = 0;
    for (auto ite = ite_start; ite != ite_end; ite++) {
        *ite = actual_seq[cursor];
        ++cursor;
    }
    *ite_end = actual_seq.back();

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");

    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

    //check for missed
    My_Assert(!ns.check_missed(mcgrp),"Some task has been missed!");

    move_result.reset();
    move_result.move_type = NeighborOperator::FLIP;

    ns.search_step++;
}


vector<int> NewFlip::get_sequence(NeighBorSearch &ns, const int start, const int end)
{
    //Two ends are pillars which doesn't involve inverse
    const auto start_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), start);
    const auto end_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), end);

    My_Assert(start_ite != ns.delimiter_coding_sol.end() && end_ite != ns.delimiter_coding_sol.end() && start_ite != end_ite,
              "Can't find task!");

    vector<int> buffer;
    buffer.reserve((ns.delimiter_coding_sol.size() / 2));

    for (auto ite = start_ite + 1; ite != end_ite; ite++) {
        buffer.push_back(*ite);
    }
//    buffer.push_back(*end_ite);

    return buffer;
}


/*
 *
 */

bool NewFlip::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task){
    vector<int> candidate_seq = get_sequence(ns,start_task, end_task);

    //No need flipping
    if(candidate_seq.size() <= 1){
        move_result.reset();
        return false;
    }

    My_Assert(all_of(candidate_seq.begin(),candidate_seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");
    My_Assert(ns.solution[candidate_seq.front()]->route_id == ns.solution[candidate_seq.back()]->route_id,"Flip attempted using different routes!");

    move_result.choose_tasks(start_task, end_task);
    move_result.move_arguments = candidate_seq;

    double delta = 0;
    start_task = max(start_task,0);
    end_task = max(end_task,0);

    delta -= mcgrp.min_cost[mcgrp.inst_tasks[start_task].tail_node][mcgrp.inst_tasks[candidate_seq.front()].head_node];
    for(int cur = 0; cur < candidate_seq.size() - 1; cur++){
        delta -= mcgrp.inst_tasks[candidate_seq[cur]].serv_cost;
        delta -= mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cur]].tail_node][mcgrp.inst_tasks[candidate_seq[cur+1]].head_node];
    }
    delta -= mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
    delta -= mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[end_task].head_node];

    reverse(candidate_seq.begin(),candidate_seq.end());

    delta += mcgrp.min_cost[mcgrp.inst_tasks[start_task].tail_node][mcgrp.inst_tasks[candidate_seq.front()].head_node];

    for(int cur = 0;cur<candidate_seq.size()-1;cur++){
        delta += mcgrp.inst_tasks[candidate_seq[cur]].serv_cost;
        delta += mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cur]].tail_node][mcgrp.inst_tasks[candidate_seq[cur + 1]].head_node];
    }

    delta += mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
    delta += mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[end_task].head_node];


    move_result.num_affected_routes = 1;

    const int route_id = ns.solution[candidate_seq.front()]->route_id;
    move_result.route_id.push_back(route_id);
    move_result.delta = delta;

    move_result.route_lens.push_back(ns.routes[route_id]->length + move_result.delta);
    move_result.route_loads.push_back(ns.routes[route_id]->load);

    move_result.route_custs_num.push_back(ns.routes[route_id]->num_customers);
    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
    move_result.considerable = true;

    return true;
}

void NewFlip::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt: flip move");

    My_Assert(move_result.considerable,"Invalid predictions");

    //extract move arguments
    vector<int> seq(move_result.move_arguments.begin(), move_result.move_arguments.end());
    My_Assert(all_of(seq.begin(),seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");
    My_Assert(ns.solution[move_result.task1]->next->ID == seq.front() && ns.solution[move_result.task2]->pre->ID == seq.back(),"Wrong task");

    My_Assert(move_result.num_affected_routes == 1,"Wrong result info");

    //Modify routes info
    const int route_id = move_result.route_id[0];
    ns.routes[route_id]->length = move_result.route_lens[0];

    const int start_task = move_result.task1;
    const int end_task = move_result.task2;

    if(ns.solution[start_task]->ID < 0){
        ns.routes[route_id]->start = seq.back();
    }

    if(ns.solution[end_task]->ID < 0){
        ns.routes[route_id]->end = seq.front();
    }

    //handle solution
    for(auto task : seq){
        auto tmp = ns.solution[task]->pre;
        ns.solution[task]->pre = ns.solution[task]->next;
        ns.solution[task]->next = tmp;
    }

    ns.solution[start_task]->next = ns.solution[seq.back()];
    ns.solution[end_task]->pre = ns.solution[seq.front()];
    ns.solution[seq.front()]->next = ns.solution[end_task];
    ns.solution[seq.back()]->pre = ns.solution[start_task];


    //modify global info
    ns.cur_solution_cost += move_result.delta;
    ns.total_vio_load += move_result.vio_load_delta;
    My_Assert(ns.valid_sol(mcgrp),"Prediction wrong!");

    if(move_result.delta == 0){
        ns.equal_step++;
    }

    move_result.reset();
    ns.search_step++;
}

vector<int> NewFlip::get_sequence(HighSpeedNeighBorSearch &ns, const int start, const int end)
{
    vector<int> buffer;
    int cur = ns.solution[start]->next->ID;

    while (cur != end){
        buffer.push_back(cur);
        cur = ns.solution[cur]->next->ID;
    }

    return buffer;
}