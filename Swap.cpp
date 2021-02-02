#include "Swap.h"
#include <iostream>
#include <vector>
#include <algorithm>


using namespace std;

MCGRPMOVE Swap::move_result = MCGRPMOVE(NeighborOperator::SWAP);

bool Swap::search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
{
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    MCGRPMOVE BestM;

    // Create the search_space
    ns.create_search_neighborhood(mcgrp, chosen_task);

    int candidate_task = -1;
    for (int i = 0; i < ns.search_space.size(); i++) {
        candidate_task = ns.search_space[i];

        if (ns.next_array[candidate_task]
            == std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max()) {    //保证参与扰动的任务位于可行空间内
            if (mcgrp.is_edge(candidate_task)) {
                candidate_task = mcgrp.inst_tasks[candidate_task].inverse;
                My_Assert(ns.next_array[candidate_task]
                              != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max(),
                          "An edge task has been missed!");
            }
            else {
                cerr << "An one direction task has been forgotten!\n"
                     << "Exception location: swap::search!\n";
                abort();
            }
        }

        if (candidate_task != DUMMY &&
            // if two swap tasks are too close, information may get confused :(
            // the problem is stemmed from edge task
            abs(distance(ns, chosen_task, candidate_task)) > 1
//        chosen_task != candidate_task
            ) {
            if (considerable_move(ns, mcgrp, chosen_task, candidate_task)
                && ns.policy.check_move(move_result)) {
                if (ns.policy.has_rule(FIRST_ACCEPT)) {
                    move(ns, mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)) {
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else {
                    cerr << "Unknown accept rule!\n";
                    abort();
                }
            }
        }
    }

    if (BestM.considerable) {
        move_result = BestM;
        move(ns, mcgrp);
        return true;
    }
    else {
        return false;
    }

}

bool Swap::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int u, int i)
{
    My_Assert(u != i, "two task need to be different!");
    My_Assert(u != DUMMY && i != DUMMY, "Swap can't not handle dummy task!");
    My_Assert(ns.next_array[u] != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max()
                  || ns.next_array[i]
                      != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max(),
              "task u or i are not in current solution");

    const int original_u = u;
    const int original_i = i;



    int u_route, i_route;
    u_route = ns.route_id[u];
    i_route = ns.route_id[i];

    // can check feasibility easily
    int u_load_delta = 0;
    int i_load_delta = 0;
    double vio_load_delta = 0;
    if (u_route != i_route) {

        u_load_delta = mcgrp.inst_tasks[i].demand - mcgrp.inst_tasks[u].demand;
        i_load_delta = mcgrp.inst_tasks[u].demand - mcgrp.inst_tasks[i].demand;

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }    // route that used to contain u is infeasible

            if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;    // route that used to contain i is infeasible
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (u_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[u_route].load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += u_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[u_route].load - mcgrp.capacity + u_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += u_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[u_route].load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[u_route].load;
                    }
                }
            }

            if (i_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[i_route].load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += i_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[i_route].load - mcgrp.capacity + i_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += i_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[i_route].load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[i_route].load;
                    }
                }
            }
        }
    }

    int t, v;
    t = max(ns.pred_array[u], DUMMY);
    v = max(ns.next_array[u], DUMMY);

    int h, j;
    h = max(ns.pred_array[i], DUMMY);
    j = max(ns.next_array[i], DUMMY);


    int u_tilde = u;
    int i_tilde = i;
    //Total four situations need to be considered here
    if (mcgrp.is_edge(u)) {
        u_tilde = mcgrp.inst_tasks[u].inverse;
    }
    if (mcgrp.is_edge(i)) {
        i_tilde = mcgrp.inst_tasks[i].inverse;
    }

    My_Assert(mcgrp.inst_tasks[u].demand == mcgrp.inst_tasks[u_tilde].demand
                  && mcgrp.inst_tasks[i].demand == mcgrp.inst_tasks[i_tilde].demand,
              "Inverse task should have same demand");

    const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
    const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
    const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
    const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
    const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
    const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
    const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];
    const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

    const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
    const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];

    const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
    const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

    double u_delta;
    u_delta = -tu - uv - mcgrp.inst_tasks[u].serv_cost;
    if (ti + iv <= ti_tilde + i_tildev) {
        u_delta = u_delta + ti + iv + mcgrp.inst_tasks[i].serv_cost;
    }
    else {
        //choose its inverse
        i = i_tilde;
        u_delta = u_delta + ti_tilde + i_tildev + mcgrp.inst_tasks[i_tilde].serv_cost;
    }

    double i_delta;
    i_delta = -hi - ij - mcgrp.inst_tasks[i].serv_cost;
    if (hu + uj <= hu_tilde + u_tildej) {
        i_delta = i_delta + hu + uj + mcgrp.inst_tasks[u].serv_cost;
    }
    else {
        //choose its inverse
        u = u_tilde;
        i_delta = i_delta + hu_tilde + u_tildej + mcgrp.inst_tasks[u_tilde].serv_cost;
    }

    move_result.choose_tasks(original_u, original_i);

    move_result.move_arguments.push_back(original_u);
    move_result.move_arguments.push_back(original_i);

    move_result.total_number_of_routes = ns.routes.size();
    if (u_route == i_route) {
        const double delta = u_delta + i_delta;
        move_result.num_affected_routes = 1;
        move_result.delta = delta;

        move_result.route_id.push_back(u_route);

        move_result.route_lens.push_back(ns.routes[u_route].length + delta);
        move_result.route_loads.push_back(ns.routes[u_route].load);

        move_result.route_custs_num.push_back(ns.routes[u_route].num_customers); // no change

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;


        move_result.considerable = true;
    }
    else {
        // Different routes
        move_result.num_affected_routes = 2;
        move_result.delta = u_delta + i_delta;

        move_result.route_id.push_back(u_route);
        move_result.route_id.push_back(i_route);

        move_result.route_lens.push_back(ns.routes[u_route].length + u_delta);
        move_result.route_lens.push_back(ns.routes[i_route].length + i_delta);

        int u_route_load = ns.routes[u_route].load - mcgrp.inst_tasks[u].demand + mcgrp.inst_tasks[i].demand;
        int i_route_load = ns.routes[i_route].load - mcgrp.inst_tasks[i].demand + mcgrp.inst_tasks[u].demand;
        move_result.route_loads.push_back(u_route_load);
        move_result.route_loads.push_back(i_route_load);


        move_result.route_custs_num.push_back(ns.routes[u_route].num_customers);
        move_result.route_custs_num.push_back(ns.routes[i_route].num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;
    }

    return true;
}

void Swap::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    My_Assert(move_result.move_arguments.size() == 4, "Incorrect move arguments!");
    My_Assert(move_result.move_type == NeighborOperator::SWAP, "Incorrect move type");

    const int original_u = move_result.move_arguments[0];
    const int original_i = move_result.move_arguments[1];
    const int actual_u = move_result.move_arguments[2];
    const int actual_i = move_result.move_arguments[3];

    My_Assert(original_i != DUMMY && original_u != DUMMY, "Swap can't handle dummy task!");
    Individual individual;
    ns.create_individual(mcgrp, individual);

    auto ite_u = std::find(individual.sequence.begin(), individual.sequence.end(), original_u);

    My_Assert(ite_u != individual.sequence.end(),
              "Can't find corresponding task u in sequence!");

    auto ite_i = std::find(individual.sequence.begin(), individual.sequence.end(), original_i);
    My_Assert(ite_i != individual.sequence.end(),
              "Can't find corresponding task i in sequence!");

    *ite_u = actual_i;
    *ite_i = actual_u;

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

    if (ns.total_vio_load == 0) {
        mcgrp.check_best_solution(ns.cur_solution_cost, ns.negative_coding_sol);
    }

//    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.beta,ns.total_vio_load,ns.negative_coding_sol);

//    if(ns.cur_solution_cost < ns.policy.cost_benchmark){
//        ns.policy.cost_benchmark = ns.cur_solution_cost;
//    }

    move_result.reset();
    move_result.move_type = NeighborOperator::SWAP;

    ns.search_step++;
}

void Swap::unit_test(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    mcgrp._rng.RandPerm(task_set);    //shuffle tasks

    ns.policy.set(BEST_ACCEPT | TOLERANCE | DELTA_ONLY);
    ns.policy.beta = 0;

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
}

int Swap::distance(const NeighBorSearch &ns, const int task1, const int task2)
{
    auto ite1 = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), task1);
    auto ite2 = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), task2);

    My_Assert(ite1 != ns.delimiter_coding_sol.end() && ite2 != ns.delimiter_coding_sol.end(),
              "tasks can't be found in current sequence!");

    return ite1 - ite2;
}




bool NewSwap::search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
{
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    MCGRPMOVE BestM;

    ns.create_search_neighborhood(mcgrp, chosen_task);

    int b = chosen_task;
    for(auto neighbor_task : ns.search_space) {
        My_Assert(neighbor_task != b, "neighbor task can't be itself!");

        if (neighbor_task != DUMMY) {
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;
            if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
                move(ns, mcgrp);
                return true;
            }
            else {
                move_result.reset();
                return true;
            }
        }
        else {
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here but b can't be dummy
            //each start and end location of each route will be considered
            //total 2 x route_nums cases

            int current_start = abs(ns.next_array[DUMMY]);

            while (current_start != DUMMY) {
                // Consider the start location
                int j = current_start;

                if (b != j) { //not the same task
                    if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
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
                if (b != j) {
                    if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
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

bool NewSwap::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int i, int u)
{
    My_Assert(u != i, "two task need to be different!");
    My_Assert(u != DUMMY && i != DUMMY, "Swap can't not handle dummy task!");


    const int original_u = u;
    const int original_i = i;


    int u_route, i_route;
    u_route = ns.route_id[u];
    i_route = ns.route_id[i];

    // can check feasibility easily
    int u_load_delta = 0;
    int i_load_delta = 0;
    double vio_load_delta = 0;
    if (u_route != i_route) {

        u_load_delta = mcgrp.inst_tasks[i].demand - mcgrp.inst_tasks[u].demand;
        i_load_delta = mcgrp.inst_tasks[u].demand - mcgrp.inst_tasks[i].demand;

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }    // route that used to contain u is infeasible

            if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;    // route that used to contain i is infeasible
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (u_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[u_route].load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += u_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[u_route].load - mcgrp.capacity + u_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += u_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[u_route].load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[u_route].load;
                    }
                }
            }

            if (i_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[i_route].load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += i_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[i_route].load - mcgrp.capacity + i_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[i_route].load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += i_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[i_route].load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[i_route].load;
                    }
                }
            }
        }
    }

    const int t = max(ns.pred_array[u], DUMMY);
    const int v = max(ns.next_array[u], DUMMY);

    const int h = max(ns.pred_array[i], DUMMY);
    const int j = max(ns.next_array[i], DUMMY);


    const int distance_ = distance(ns,i,u);
    My_Assert(distance_ != 0,"distance can't be zero!");


    //These two info are used when two tasks are not in the same route
    double u_delta = 0;
    double i_delta = 0;

    double delta = 0;

    if(distance_ == -1){
        //...h-i-u-v...
        DEBUG_PRINT("Swap:two tasks are neighbor");
        My_Assert(i_route == u_route,"Close tasks should be in the same route!");
        My_Assert(j == u,"j task must be equal with u task!");

        double deltas[4];
        if (mcgrp.is_edge(u) && mcgrp.is_edge(i)) {
            const int u_tilde = mcgrp.inst_tasks[u].inverse;
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];
            const double ui_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildei = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i].head_node];


            const double u_tildei_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i_tilde].head_node];

            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u-i_tilde-v...
            deltas[1] = hu + mcgrp.inst_tasks[u].serv_cost + ui_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;

            //...h-u_tilde-i-v...
            deltas[2] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u_tilde-i_tilde-v...
            deltas[3] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;

            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<4;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u;
                    i = i_tilde;
                    break;
                case 2:
                    u = u_tilde;
                    i = i;
                    break;
                case 3:
                    u = u_tilde;
                    i = i_tilde;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }

        }
        else if (mcgrp.is_edge(u)){

            const int u_tilde = mcgrp.inst_tasks[u].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildei = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i].head_node];


            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u_tilde-i-v...
            deltas[1] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei + mcgrp.inst_tasks[i].serv_cost + iv;

            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<2;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u_tilde;
                    i = i;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }
        }
        else if (mcgrp.is_edge(i)){
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];
            const double ui_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i_tilde].head_node];



            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u-i_tilde-v...
            deltas[1] = hu + mcgrp.inst_tasks[u].serv_cost + ui_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;


            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<2;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u;
                    i = i_tilde;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }
        }
        else{
            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            double best_delta = deltas[0];

            delta = best_delta + base;
        }
    }
    else if (distance_ == 1){
        //...t-u-i-j...
        DEBUG_PRINT("Swap:two tasks are neighbor");
        My_Assert(i_route == u_route,"Close tasks should be in the same route!");
        My_Assert(v == i,"v task must be equal with i task!");

        double deltas[4];
        if (mcgrp.is_edge(u) && mcgrp.is_edge(i)) {
            const int u_tilde = mcgrp.inst_tasks[u].inverse;
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double i_tildeu = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u].head_node];
            const double iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

            const double i_tildeu_tilde =
                mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u_tilde].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i-u_tilde-j...
            deltas[1] = ti + mcgrp.inst_tasks[i].serv_cost + iu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            //...t-i_tilde-u-j...
            deltas[2] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i_tilde-u_tilde-j...
            deltas[3] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 4; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:i = i;
                    u = u;
                    break;
                case 1:i = i;
                    u = u_tilde;
                    break;
                case 2:i = i_tilde;
                    u = u;
                    break;
                case 3:i = i_tilde;
                    u = u_tilde;
                    break;
                default:My_Assert(false, "unknown cases!");
            }
        }
        else if (mcgrp.is_edge(i)){
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double i_tildeu = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i_tilde-u-j...
            deltas[1] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu + mcgrp.inst_tasks[u].serv_cost + uj;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 2; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:
                    i = i;
                    u = u;
                    break;
                case 1:
                    i = i_tilde;
                    u = u;
                    break;
                default:My_Assert(false, "unknown cases!");
            }

        }
        else if (mcgrp.is_edge(u)){
            const int u_tilde = mcgrp.inst_tasks[u].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...h-i-u_tilde-j...
            deltas[1] = ti + mcgrp.inst_tasks[i].serv_cost + iu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 2; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:i = i;
                    u = u;
                    break;
                case 1:i = i;
                    u = u_tilde;
                    break;
                default:My_Assert(false, "unknown cases!");
            }
        }
        else{
            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];


            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;


            double best_delta = deltas[0];

            delta = best_delta + base;
        }
    }
    else{
        //...t-u-v...h-i-j...
        // OR
        //...h-i-j...t-u-v...
        int u_tilde = u;
        int i_tilde = i;
        //Total four situations need to be considered here
        if (mcgrp.is_edge(u)) {
            u_tilde = mcgrp.inst_tasks[u].inverse;
        }
        if (mcgrp.is_edge(i)) {
            i_tilde = mcgrp.inst_tasks[i].inverse;
        }

        const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
        const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
        const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
        const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
        const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

        const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
        const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];

        const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
        const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

        u_delta = -tu - uv - mcgrp.inst_tasks[u].serv_cost;
        if (ti + iv <= ti_tilde + i_tildev) {
            u_delta = u_delta + ti + iv + mcgrp.inst_tasks[i].serv_cost;
        }
        else {
            //choose its inverse
            i = i_tilde;
            u_delta = u_delta + ti_tilde + i_tildev + mcgrp.inst_tasks[i_tilde].serv_cost;
        }

        i_delta = -hi - ij - mcgrp.inst_tasks[i].serv_cost;
        if (hu + uj <= hu_tilde + u_tildej) {
            i_delta = i_delta + hu + uj + mcgrp.inst_tasks[u].serv_cost;
        }
        else {
            //choose its inverse
            u = u_tilde;
            i_delta = i_delta + hu_tilde + u_tildej + mcgrp.inst_tasks[u_tilde].serv_cost;
        }

        delta = u_delta + i_delta;

    }

    move_result.choose_tasks(original_u, original_i);

    move_result.move_arguments.push_back(original_u);
    move_result.move_arguments.push_back(original_i);

    move_result.total_number_of_routes = ns.routes.size();
    if (u_route == i_route) {
        move_result.num_affected_routes = 1;
        move_result.delta = delta;

        move_result.route_id.push_back(u_route);

        move_result.route_lens.push_back(ns.routes[u_route].length + delta);
        move_result.route_loads.push_back(ns.routes[u_route].load);

        move_result.route_custs_num.push_back(ns.routes[u_route].num_customers); // no change

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;


        move_result.considerable = true;
    }
    else {
        // Different routes
        move_result.num_affected_routes = 2;
        move_result.delta = delta;

        move_result.route_id.push_back(u_route);
        move_result.route_id.push_back(i_route);

        move_result.route_lens.push_back(ns.routes[u_route].length + u_delta);
        move_result.route_lens.push_back(ns.routes[i_route].length + i_delta);

        int u_route_load = ns.routes[u_route].load - mcgrp.inst_tasks[u].demand + mcgrp.inst_tasks[i].demand;
        int i_route_load = ns.routes[i_route].load - mcgrp.inst_tasks[i].demand + mcgrp.inst_tasks[u].demand;
        move_result.route_loads.push_back(u_route_load);
        move_result.route_loads.push_back(i_route_load);


        move_result.route_custs_num.push_back(ns.routes[u_route].num_customers);
        move_result.route_custs_num.push_back(ns.routes[i_route].num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;
    }


    return true;
}

void NewSwap::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute an swap move");

    My_Assert(move_result.considerable,"Invalid predictions");
    My_Assert(move_result.move_arguments.size() == 4, "Incorrect move arguments!");

    const int original_u = move_result.move_arguments[0];
    const int original_i = move_result.move_arguments[1];
    const int actual_u = move_result.move_arguments[2];
    const int actual_i = move_result.move_arguments[3];

    My_Assert(original_i != DUMMY && original_u != DUMMY, "Swap can't handle dummy task!");
    Individual individual;
    ns.create_individual(mcgrp, individual);

    auto ite_u = std::find(individual.sequence.begin(), individual.sequence.end(), original_u);

    My_Assert(ite_u != individual.sequence.end(),
              "Can't find corresponding task u in sequence!");

    auto ite_i = std::find(individual.sequence.begin(), individual.sequence.end(), original_i);
    My_Assert(ite_i != individual.sequence.end(),
              "Can't find corresponding task i in sequence!");

    *ite_u = actual_i;
    *ite_i = actual_u;

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");

    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

    if (ns.total_vio_load == 0) {
        mcgrp.check_best_solution(ns.cur_solution_cost, ns.negative_coding_sol);
    }

//    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.policy.beta,ns.total_vio_load,ns.negative_coding_sol);


    move_result.reset();

    ns.search_step++;
}

void NewSwap::unit_test(NeighBorSearch &ns, const MCGRP &mcgrp)
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

int NewSwap::distance(const NeighBorSearch &ns, const int task1, const int task2)
{
    auto ite1 = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), task1);
    auto ite2 = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), task2);

    My_Assert(ite1 != ns.delimiter_coding_sol.end() && ite2 != ns.delimiter_coding_sol.end(),
              "tasks can't be found in current sequence!");

    return ite1 - ite2;
}


/*
 * High speed
 */

bool NewSwap::search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
{
    My_Assert(chosen_task >= 1 && chosen_task <= mcgrp.actual_task_num,"Wrong task");

    MCGRPMOVE BestM;

    ns.create_search_neighborhood(mcgrp, chosen_task);

    int b = chosen_task;


    for(auto neighbor_task : ns.search_space) {
        My_Assert(neighbor_task != b, "neighbor task can't be itself!");

        if (neighbor_task != DUMMY) {
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;
            if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
                move(ns, mcgrp);
                return true;
            }
            else {
                move_result.reset();
                return true;
            }
        }
        else {
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here but b can't be dummy
            //each start and end location of each route will be considered
            //total 2 x route_nums cases

            int current_start = ns.solution.very_start->next->ID;

            while (current_start != DUMMY) {
                // Consider the start location
                int j = current_start;

                if (b != j) { //not the same task
                    if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
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
                if (b != j) {
                    if (considerable_move(ns, mcgrp, b, j) && ns.policy.check_move(move_result)) {
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

}

bool NewSwap::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int i, int u)
{
    My_Assert(u != i, "two task need to be different!");
    My_Assert(i >= 1 && i <= mcgrp.actual_task_num,"Wrong task");
    My_Assert(u >= 1 && u <= mcgrp.actual_task_num,"Wrong task");


    const int original_u = u;
    const int original_i = i;


    int u_route, i_route;
    u_route = ns.solution[u]->route_id;
    i_route = ns.solution[i]->route_id;

    // can check feasibility easily
    int u_load_delta = 0;
    int i_load_delta = 0;
    double vio_load_delta = 0;
    if (u_route != i_route) {

        u_load_delta = mcgrp.inst_tasks[i].demand - mcgrp.inst_tasks[u].demand;
        i_load_delta = mcgrp.inst_tasks[u].demand - mcgrp.inst_tasks[i].demand;

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route]->load + u_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }    // route that used to contain u is infeasible

            if (ns.routes[i_route]->load + i_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;    // route that used to contain i is infeasible
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (u_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[u_route]->load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[u_route]->load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += u_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[u_route]->load - mcgrp.capacity + u_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[u_route]->load + u_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += u_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[u_route]->load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[u_route]->load;
                    }
                }
            }

            if (i_load_delta >= 0) {
                //vio_load_delta may get bigger
                if (ns.routes[i_route]->load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    if (ns.routes[i_route]->load >= mcgrp.capacity) {
                        //if the route u already over loaded
                        vio_load_delta += i_load_delta;
                    }
                    else {
                        vio_load_delta += ns.routes[i_route]->load - mcgrp.capacity + i_load_delta;
                    }
                }
            }
            else {
                //vio_load_delta may get less
                if (ns.routes[i_route]->load + i_load_delta > mcgrp.capacity) {
                    //if result is overload
                    vio_load_delta += i_load_delta;
                }
                else {
                    //the result will be not overload
                    if (ns.routes[i_route]->load > mcgrp.capacity) {
                        //the route used to be overload
                        vio_load_delta += mcgrp.capacity - ns.routes[i_route]->load;
                    }
                }
            }
        }
    }

    const int t = max(ns.solution[u]->pre->ID, 0);
    const int v = max(ns.solution[u]->next->ID, 0);

    const int h = max(ns.solution[i]->pre->ID, 0);
    const int j = max(ns.solution[i]->next->ID, 0);


    //These two info are used when two tasks are not in the same route
    double u_delta = 0;
    double i_delta = 0;

    double delta = 0;

    if(j == u){
        //...h-i-u-v...
        DEBUG_PRINT("Swap:two tasks are neighbor");
        My_Assert(i_route == u_route,"Close tasks should be in the same route!");
        My_Assert(t == i,"t task must be equal with i task!");

        double deltas[4];
        if (mcgrp.is_edge(u) && mcgrp.is_edge(i)) {
            const int u_tilde = mcgrp.inst_tasks[u].inverse;
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];
            const double ui_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildei = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i].head_node];


            const double u_tildei_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i_tilde].head_node];

            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u-i_tilde-v...
            deltas[1] = hu + mcgrp.inst_tasks[u].serv_cost + ui_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;

            //...h-u_tilde-i-v...
            deltas[2] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u_tilde-i_tilde-v...
            deltas[3] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;

            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<4;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u;
                    i = i_tilde;
                    break;
                case 2:
                    u = u_tilde;
                    i = i;
                    break;
                case 3:
                    u = u_tilde;
                    i = i_tilde;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }

        }
        else if (mcgrp.is_edge(u)){

            const int u_tilde = mcgrp.inst_tasks[u].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildei = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[i].head_node];


            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u_tilde-i-v...
            deltas[1] = hu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildei + mcgrp.inst_tasks[i].serv_cost + iv;

            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<2;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u_tilde;
                    i = i;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }
        }
        else if (mcgrp.is_edge(i)){
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];
            const double ui_tilde = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i_tilde].head_node];



            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            //...h-u-i_tilde-v...
            deltas[1] = hu + mcgrp.inst_tasks[u].serv_cost + ui_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildev;


            int cases = 0;
            double best_delta = deltas[0];
            for(int cursor = 1;cursor<2;cursor++){
                if(deltas[cursor] < best_delta){
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases){
                case 0:
                    u = u;
                    i = i;
                    break;
                case 1:
                    u = u;
                    i = i_tilde;
                    break;
                default:
                    My_Assert(false,"unknown cases!");
            }
        }
        else{
            const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
            const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];

            //remove the original connections firstly
            double base = -hi - mcgrp.inst_tasks[i].serv_cost - iu - mcgrp.inst_tasks[u].serv_cost - uv;

            //add new connections secondly
            //...h-u-i-v...
            deltas[0] = hu + mcgrp.inst_tasks[u].serv_cost + ui + mcgrp.inst_tasks[i].serv_cost + iv;

            double best_delta = deltas[0];

            delta = best_delta + base;
        }
    }
    else if (v == i){
        //...t-u-i-j...
        DEBUG_PRINT("Swap:two tasks are neighbor");
        My_Assert(i_route == u_route,"Close tasks should be in the same route!");
        My_Assert(u == h,"u task must be equal with h task!");

        double deltas[4];
        if (mcgrp.is_edge(u) && mcgrp.is_edge(i)) {
            const int u_tilde = mcgrp.inst_tasks[u].inverse;
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double i_tildeu = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u].head_node];
            const double iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

            const double i_tildeu_tilde =
                mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u_tilde].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i-u_tilde-j...
            deltas[1] = ti + mcgrp.inst_tasks[i].serv_cost + iu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            //...t-i_tilde-u-j...
            deltas[2] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i_tilde-u_tilde-j...
            deltas[3] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 4; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:i = i;
                    u = u;
                    break;
                case 1:i = i;
                    u = u_tilde;
                    break;
                case 2:i = i_tilde;
                    u = u;
                    break;
                case 3:i = i_tilde;
                    u = u_tilde;
                    break;
                default:My_Assert(false, "unknown cases!");
            }
        }
        else if (mcgrp.is_edge(i)){
            const int i_tilde = mcgrp.inst_tasks[i].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
            const double i_tildeu = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[u].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...t-i_tilde-u-j...
            deltas[1] = ti_tilde + mcgrp.inst_tasks[i_tilde].serv_cost + i_tildeu + mcgrp.inst_tasks[u].serv_cost + uj;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 2; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:
                    i = i;
                    u = u;
                    break;
                case 1:
                    i = i_tilde;
                    u = u;
                    break;
                default:My_Assert(false, "unknown cases!");
            }

        }
        else if (mcgrp.is_edge(u)){
            const int u_tilde = mcgrp.inst_tasks[u].inverse;

            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

            const double iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
            const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;

            //...h-i-u_tilde-j...
            deltas[1] = ti + mcgrp.inst_tasks[i].serv_cost + iu_tilde + mcgrp.inst_tasks[u_tilde].serv_cost + u_tildej;

            int cases = 0;
            double best_delta = deltas[0];
            for (int cursor = 1; cursor < 2; cursor++) {
                if (deltas[cursor] < best_delta) {
                    cases = cursor;
                    best_delta = deltas[cursor];
                }
            }

            delta = best_delta + base;

            //decide which task should be swapped finally
            switch (cases) {
                case 0:i = i;
                    u = u;
                    break;
                case 1:i = i;
                    u = u_tilde;
                    break;
                default:My_Assert(false, "unknown cases!");
            }
        }
        else{
            const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
            const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
            const double ui = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[i].head_node];
            const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
            const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];


            //remove the original connections firstly
            double base = -tu - mcgrp.inst_tasks[u].serv_cost - ui - mcgrp.inst_tasks[i].serv_cost - ij;

            //add new connections secondly
            //...t-i-u-j...
            deltas[0] = ti + mcgrp.inst_tasks[i].serv_cost + iu + mcgrp.inst_tasks[u].serv_cost + uj;


            double best_delta = deltas[0];

            delta = best_delta + base;
        }
    }
    else{
        //...t-u-v...h-i-j...
        // OR
        //...h-i-j...t-u-v...
        int u_tilde = u;
        int i_tilde = i;
        //Total four situations need to be considered here
        if (mcgrp.is_edge(u)) {
            u_tilde = mcgrp.inst_tasks[u].inverse;
        }
        if (mcgrp.is_edge(i)) {
            i_tilde = mcgrp.inst_tasks[i].inverse;
        }

        const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[i].head_node];
        const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];
        const double ti = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i].head_node];
        const double iv = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[v].head_node];
        const double hu = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];

        const double ti_tilde = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[i_tilde].head_node];
        const double i_tildev = mcgrp.min_cost[mcgrp.inst_tasks[i_tilde].tail_node][mcgrp.inst_tasks[v].head_node];

        const double hu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
        const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];

        u_delta = -tu - uv - mcgrp.inst_tasks[u].serv_cost;
        if (ti + iv <= ti_tilde + i_tildev) {
            u_delta = u_delta + ti + iv + mcgrp.inst_tasks[i].serv_cost;
        }
        else {
            //choose its inverse
            i = i_tilde;
            u_delta = u_delta + ti_tilde + i_tildev + mcgrp.inst_tasks[i_tilde].serv_cost;
        }

        i_delta = -hi - ij - mcgrp.inst_tasks[i].serv_cost;
        if (hu + uj <= hu_tilde + u_tildej) {
            i_delta = i_delta + hu + uj + mcgrp.inst_tasks[u].serv_cost;
        }
        else {
            //choose its inverse
            u = u_tilde;
            i_delta = i_delta + hu_tilde + u_tildej + mcgrp.inst_tasks[u_tilde].serv_cost;
        }

        delta = u_delta + i_delta;

    }

    move_result.choose_tasks(original_u, original_i);

    move_result.move_arguments.push_back(original_u);
    move_result.move_arguments.push_back(original_i);

    move_result.total_number_of_routes = ns.routes.activated_route_id.size();

    if (u_route == i_route) {

        move_result.num_affected_routes = 1;
        move_result.delta = delta;

        move_result.route_id.push_back(u_route);

        move_result.route_lens.push_back(ns.routes[u_route]->length + delta);

        move_result.route_loads.push_back(ns.routes[u_route]->load);

        move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }
    else {
        // Different routes
        move_result.num_affected_routes = 2;
        move_result.delta = delta;

        move_result.route_id.push_back(u_route);
        move_result.route_id.push_back(i_route);

        move_result.route_lens.push_back(ns.routes[u_route]->length + u_delta);
        move_result.route_lens.push_back(ns.routes[i_route]->length + i_delta);

        int u_route_load = ns.routes[u_route]->load - mcgrp.inst_tasks[u].demand + mcgrp.inst_tasks[i].demand;
        int i_route_load = ns.routes[i_route]->load - mcgrp.inst_tasks[i].demand + mcgrp.inst_tasks[u].demand;
        move_result.route_loads.push_back(u_route_load);
        move_result.route_loads.push_back(i_route_load);


        move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers);
        move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.move_arguments.push_back(u);
        move_result.move_arguments.push_back(i);

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }

    My_Assert(false,"Cannot reach here!");
}

void NewSwap::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a swap move");

    My_Assert(move_result.considerable,"Invalid predictions");
    My_Assert(move_result.move_arguments.size() == 4, "Incorrect move arguments!");


    const int original_u = move_result.move_arguments[0];
    const int original_i = move_result.move_arguments[1];
    const int actual_u = move_result.move_arguments[2];
    const int actual_i = move_result.move_arguments[3];

    My_Assert(original_u >= 1 && original_u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(actual_u >= 1 && actual_u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(original_i >= 1 && original_i <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(actual_i >= 1 && actual_i <= mcgrp.actual_task_num,"Wrong arguments");

    if(original_u != actual_u && original_i != actual_i) {
        //Edge task has been moved
        //both task switch
        My_Assert(mcgrp.inst_tasks[original_u].inverse == actual_u, "This is not the inverse");
        My_Assert(mcgrp.inst_tasks[original_i].inverse == actual_i,"Wrong tasks");
        My_Assert(ns.solution[original_u]->next != nullptr && ns.solution[actual_u]->next == nullptr,"Wrong arguments");
        My_Assert(ns.solution[original_i]->next != nullptr && ns.solution[actual_i]->next == nullptr,"Wrong arguments");

        if(move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[route_id]->num_customers > 1,"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[route_id]->start = actual_i;
            }
            else if (ns.solution[original_u]->next->ID < 0){
                ns.routes[route_id]->end = actual_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[route_id]->start = actual_u;
            }
            else if(ns.solution[original_i]->next->ID < 0){
                ns.routes[route_id]->end = actual_u;
            }


            ns.solution[actual_u]->route_id = route_id;
            ns.solution[actual_i]->route_id = route_id;
        }
        else{
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[u_route]->length = move_result.route_lens[0];
            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[u_route]->load = move_result.route_loads[0];
            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[u_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[u_route]->start = actual_i;
            }

            if (ns.solution[original_u]->next->ID < 0){
                ns.routes[u_route]->end = actual_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[i_route]->start = actual_u;
            }

            if(ns.solution[original_i]->next->ID < 0){
                ns.routes[i_route]->end = actual_u;
            }

            ns.solution[actual_u]->route_id = i_route;
            ns.solution[actual_i]->route_id = u_route;
        }

        //handle solution
        if(ns.solution[original_u]->next == ns.solution[original_i]){
            //...t-u-i-j...
            const auto original_i_next = ns.solution[original_i]->next;
            const auto original_u_pre = ns.solution[original_u]->pre;

            original_i_next->pre = ns.solution[actual_u];
            original_u_pre->next = ns.solution[actual_i];

            ns.solution[actual_i]->pre = original_u_pre;
            ns.solution[actual_i]->next = ns.solution[actual_u];
            ns.solution[actual_u]->pre = ns.solution[actual_i];
            ns.solution[actual_u]->next = original_i_next;
        }
        else if (ns.solution[original_i]->next == ns.solution[original_u]){
            //...h-i-u-v...
            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_u_next = ns.solution[original_u]->next;

            original_i_pre->next = ns.solution[actual_u];
            original_u_next->pre = ns.solution[actual_i];

            ns.solution[actual_i]->pre = ns.solution[actual_u];
            ns.solution[actual_i]->next = original_u_next;
            ns.solution[actual_u]->pre = original_i_pre;
            ns.solution[actual_u]->next = ns.solution[actual_i];
        }
        else{
            //...h-i-j...
            //...t-u-v...
            ns.solution[original_i]->pre->next = ns.solution[actual_u];
            ns.solution[original_i]->next->pre = ns.solution[actual_u];

            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_i_next = ns.solution[original_i]->next;
            ns.solution[actual_i]->pre = ns.solution[original_u]->pre;
            ns.solution[actual_i]->next = ns.solution[original_u]->next;

            ns.solution[original_u]->next->pre = ns.solution[actual_i];
            ns.solution[original_u]->pre->next = ns.solution[actual_i];

            ns.solution[actual_u]->pre = original_i_pre;
            ns.solution[actual_u]->next = original_i_next;
        }

        //clear original i.u task info
        ns.solution[original_i]->clear();
        ns.solution[original_u]->clear();
    }
    else if (original_u != actual_u){
        //single task switch
        My_Assert(mcgrp.inst_tasks[original_u].inverse == actual_u,"Wrong tasks");
        My_Assert(ns.solution[original_u]->next != nullptr && ns.solution[actual_u]->next == nullptr,
                  "Wrong arguments");

        if(move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[route_id]->num_customers > 1,"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[route_id]->start = original_i;
            }
            else if (ns.solution[original_u]->next->ID < 0){
                ns.routes[route_id]->end = original_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[route_id]->start = actual_u;
            }
            else if(ns.solution[original_i]->next->ID < 0){
                ns.routes[route_id]->end = actual_u;
            }

            ns.solution[actual_u]->route_id = route_id;
        }
        else{
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[u_route]->length = move_result.route_lens[0];
            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[u_route]->load = move_result.route_loads[0];
            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[u_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[u_route]->start = original_i;
            }

            if (ns.solution[original_u]->next->ID < 0){
                ns.routes[u_route]->end = original_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[i_route]->start = actual_u;
            }

            if(ns.solution[original_i]->next->ID < 0){
                ns.routes[i_route]->end = actual_u;
            }

            ns.solution[actual_u]->route_id = i_route;
            ns.solution[original_i]->route_id = u_route;
        }

        //handle solution
        if(ns.solution[original_u]->next == ns.solution[original_i]){
            //...t-u-i-j...
            const auto original_i_next = ns.solution[original_i]->next;
            const auto original_u_pre = ns.solution[original_u]->pre;

            original_i_next->pre = ns.solution[actual_u];
            original_u_pre->next = ns.solution[original_i];

            ns.solution[original_i]->pre = original_u_pre;
            ns.solution[original_i]->next = ns.solution[actual_u];
            ns.solution[actual_u]->pre = ns.solution[original_i];
            ns.solution[actual_u]->next = original_i_next;
        }
        else if (ns.solution[original_i]->next == ns.solution[original_u]){
            //...h-i-u-v...
            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_u_next = ns.solution[original_u]->next;

            original_i_pre->next = ns.solution[actual_u];
            original_u_next->pre = ns.solution[original_i];

            ns.solution[original_i]->pre = ns.solution[actual_u];
            ns.solution[original_i]->next = original_u_next;
            ns.solution[actual_u]->pre = original_i_pre;
            ns.solution[actual_u]->next = ns.solution[original_i];
        }
        else{
            //...h-i-j...
            //...t-u-v...
            ns.solution[original_i]->pre->next = ns.solution[actual_u];
            ns.solution[original_i]->next->pre = ns.solution[actual_u];

            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_i_next = ns.solution[original_i]->next;
            ns.solution[original_i]->pre = ns.solution[original_u]->pre;
            ns.solution[original_i]->next = ns.solution[original_u]->next;

            ns.solution[original_u]->next->pre = ns.solution[original_i];
            ns.solution[original_u]->pre->next = ns.solution[original_i];

            ns.solution[actual_u]->pre = original_i_pre;
            ns.solution[actual_u]->next = original_i_next;
        }

        //clear original u task info
        ns.solution[original_u]->clear();
    }
    else if (original_i != actual_i){
        //single task switch
        My_Assert(mcgrp.inst_tasks[original_i].inverse == actual_i,"Wrong tasks");
        My_Assert(ns.solution[original_i]->next != nullptr && ns.solution[actual_i]->next == nullptr,
                  "Wrong arguments");


        if(move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[route_id]->num_customers > 1,"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[route_id]->start = actual_i;
            }
            else if (ns.solution[original_u]->next->ID < 0){
                ns.routes[route_id]->end = actual_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[route_id]->start = original_u;
            }
            else if(ns.solution[original_i]->next->ID < 0){
                ns.routes[route_id]->end = original_u;
            }

            ns.solution[actual_i]->route_id = route_id;
        }
        else{
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[u_route]->length = move_result.route_lens[0];
            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[u_route]->load = move_result.route_loads[0];
            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[u_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[u_route]->start = actual_i;
            }

            if (ns.solution[original_u]->next->ID < 0){
                ns.routes[u_route]->end = actual_i;
            }

            if (ns.solution[original_i]->pre->ID < 0){
                ns.routes[i_route]->start = original_u;
            }

            if(ns.solution[original_i]->next->ID < 0){
                ns.routes[i_route]->end = original_u;
            }

            ns.solution[original_u]->route_id = i_route;
            ns.solution[actual_i]->route_id = u_route;
        }

        //handle solution
        if(ns.solution[original_u]->next == ns.solution[original_i]){
            //...t-u-i-j...
            const auto original_i_next = ns.solution[original_i]->next;
            const auto original_u_pre = ns.solution[original_u]->pre;

            original_i_next->pre = ns.solution[original_u];
            original_u_pre->next = ns.solution[actual_i];

            ns.solution[actual_i]->pre = original_u_pre;
            ns.solution[actual_i]->next = ns.solution[original_u];
            ns.solution[original_u]->pre = ns.solution[actual_i];
            ns.solution[original_u]->next = original_i_next;
        }
        else if (ns.solution[original_i]->next == ns.solution[original_u]){
            //...h-i-u-v...
            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_u_next = ns.solution[original_u]->next;

            original_i_pre->next = ns.solution[original_u];
            original_u_next->pre = ns.solution[actual_i];

            ns.solution[actual_i]->pre = ns.solution[original_u];
            ns.solution[actual_i]->next = original_u_next;
            ns.solution[original_u]->pre = original_i_pre;
            ns.solution[original_u]->next = ns.solution[actual_i];
        }
        else{
            //...h-i-j...
            //...t-u-v...
            ns.solution[original_i]->pre->next = ns.solution[original_u];
            ns.solution[original_i]->next->pre = ns.solution[original_u];

            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_i_next = ns.solution[original_i]->next;
            ns.solution[actual_i]->pre = ns.solution[original_u]->pre;
            ns.solution[actual_i]->next = ns.solution[original_u]->next;

            ns.solution[original_u]->next->pre = ns.solution[actual_i];
            ns.solution[original_u]->pre->next = ns.solution[actual_i];

            ns.solution[original_u]->pre = original_i_pre;
            ns.solution[original_u]->next = original_i_next;
        }

        //clear original i task info
        ns.solution[original_i]->clear();
    }
    else{
        //no task switch
        //Modify routes info
        My_Assert(original_i == actual_i && original_u == actual_u,"Wrong arguments");
        if(move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[route_id]->num_customers > 1,"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[route_id]->start = original_i;
            }
            else if(ns.solution[original_u]->next->ID < 0){
                ns.routes[route_id]->end = original_i;
            }

            if(ns.solution[original_i]->pre->ID < 0){
                ns.routes[route_id]->start = original_u;
            }
            else if(ns.solution[original_i]->next->ID < 0){
                ns.routes[route_id]->end = original_u;
            }

        }
        else{
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[u_route]->length = move_result.route_lens[0];
            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[u_route]->load = move_result.route_loads[0];
            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[u_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            if(ns.solution[original_u]->pre->ID < 0){
                ns.routes[u_route]->start = original_i;
            }

            if(ns.solution[original_u]->next->ID < 0){
                ns.routes[u_route]->end = original_i;
            }

            if(ns.solution[original_i]->pre->ID < 0){
                ns.routes[i_route]->start = original_u;
            }

            if(ns.solution[original_i]->next->ID < 0){
                ns.routes[i_route]->end = original_u;
            }

            ns.solution[original_u]->route_id = i_route;
            ns.solution[original_i]->route_id = u_route;
        }

        //handle solution
        if(ns.solution[original_u]->next == ns.solution[original_i]){
            //...t-u-i-j...
            const auto original_i_next = ns.solution[original_i]->next;
            const auto original_u_pre = ns.solution[original_u]->pre;

            original_i_next->pre = ns.solution[original_u];
            original_u_pre->next = ns.solution[original_i];

            ns.solution[original_i]->pre = original_u_pre;
            ns.solution[original_i]->next = ns.solution[original_u];
            ns.solution[original_u]->pre = ns.solution[original_i];
            ns.solution[original_u]->next = original_i_next;
        }
        else if (ns.solution[original_i]->next == ns.solution[original_u]){
            //...h-i-u-v...
            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_u_next = ns.solution[original_u]->next;

            original_i_pre->next = ns.solution[original_u];
            original_u_next->pre = ns.solution[original_i];

            ns.solution[original_i]->pre = ns.solution[original_u];
            ns.solution[original_i]->next = original_u_next;
            ns.solution[original_u]->pre = original_i_pre;
            ns.solution[original_u]->next = ns.solution[original_i];
        }
        else{
            //...h-i-j...
            //...t-u-v...
            ns.solution[original_i]->pre->next = ns.solution[original_u];
            ns.solution[original_i]->next->pre = ns.solution[original_u];

            const auto original_i_pre = ns.solution[original_i]->pre;
            const auto original_i_next = ns.solution[original_i]->next;
            ns.solution[original_i]->pre = ns.solution[original_u]->pre;
            ns.solution[original_i]->next = ns.solution[original_u]->next;

            ns.solution[original_u]->next->pre = ns.solution[original_i];
            ns.solution[original_u]->pre->next = ns.solution[original_i];

            ns.solution[original_u]->pre = original_i_pre;
            ns.solution[original_u]->next = original_i_next;
        }
    }


    //modify global info
    ns.cur_solution_cost += move_result.delta;
    ns.total_vio_load += move_result.vio_load_delta;
    My_Assert(ns.valid_sol(mcgrp),"Prediction wrong!");

    if(move_result.delta == 0){
        ns.equal_step++;
    }

    ns.trace(mcgrp);

//    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.policy.beta,ns.total_vio_load,ns.negative_coding_sol);

    move_result.reset();
    ns.search_step++;
}

void NewSwap::unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    mcgrp._rng.RandPerm(task_set);    //shuffle tasks

    auto original_policy = ns.policy.get();
    ns.policy.set(FIRST_ACCEPT | DOWNHILL | DELTA_ONLY);
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
