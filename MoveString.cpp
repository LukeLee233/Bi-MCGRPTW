#include "MoveString.h"
#include "NeighborSearch.h"
#include "PostSert.h"
#include "Presert.h"
#include <algorithm>



bool MoveString::considerable_move(NeighBorSearch &ns,
                                   const class MCGRP &mcgrp,
                                   vector<int> disturbance_seq,
                                   const int u)
{

    //Evaluates the move of inserting the string i-j-k(disturbance seq) between u and v (i.e.h-i-j-k-l & t-u-v-w )
    //* @details yielding h-l & t-u-i-j-k-v-w
    // task u cannot be dummy task
    My_Assert(u != DUMMY, "Move string cannot handle situation when u is dummy");

    My_Assert(std::find(distrubance_seq.begin(), distrubance_seq.end(), DUMMY) == distrubance_seq.end(),
              "Disturbance sequence can't cross routes!");




    const int i_route = ns.route_id[disturbance_seq.front()];
    const int u_route = ns.route_id[u];

    int v = max(ns.next_array[u], DUMMY);
    //another consistency check
    if (v != DUMMY) {
        My_Assert(u_route == ns.route_id[v], "MoveString u and v not in same route");
    }

    //of course, disturbance sequence can't has task u and v, is can't insert itself!
    //forbid overlap
    My_Assert(find(distrubance_seq.begin(), distrubance_seq.end(), u) == distrubance_seq.end()
    && find(distrubance_seq.begin(), distrubance_seq.end(), v) == distrubance_seq.end(),"overlapping!");



    const int h = max(ns.pred_array[disturbance_seq.front()], DUMMY);
    const int l = max(ns.next_array[disturbance_seq.back()], DUMMY);

    // Now check loads;
    int i_load_delta = 0;
    int u_load_delta = 0;
    double vio_load_delta = 0;
    //not the same route
    if (i_route != u_route) {
        for (auto task : disturbance_seq) {
            u_load_delta += mcgrp.inst_tasks[task].demand;
        }

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (ns.routes[u_route].load + u_load_delta > mcgrp.capacity) {
                //if insert task to route u and over load
                if (ns.routes[u_route].load >= mcgrp.capacity) {
                    //if the route u already over loaded
                    vio_load_delta += u_load_delta;
                }
                else {
                    vio_load_delta += ns.routes[u_route].load + u_load_delta - mcgrp.capacity;
                }
            }

            //i_route vio-load calculate
            if (ns.routes[i_route].load > mcgrp.capacity) {
                //if remove task from route i and over load
                if (ns.routes[i_route].load - u_load_delta >= mcgrp.capacity) {
                    //if still over loaded
                    vio_load_delta -= u_load_delta;
                }
                else {
                    vio_load_delta -= (ns.routes[i_route].load - mcgrp.capacity);
                }
            }
        }

        i_load_delta = -u_load_delta;
    }

    int start_i, end_i;
    start_i = ns.routes[i_route].start;
    end_i = ns.routes[i_route].end;

    double i_route_length_delta = 0;
    double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
    double kl = mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[l].head_node];
    double hl = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[l].head_node];
    i_route_length_delta = hl - (hi + kl);
    for (int cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
        i_route_length_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[cursor
                + 1]].head_node];
        i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
    }
    i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;

    //If you consider all possible situations of edge_tasks, you will have to compare 2^disturbance_seq.size() potential moves
    //which is costly, so I random invert the edge tasks given the prob as a sample to compare with the original task;
    double u_route_length_delta = 0;
    double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
    u_route_length_delta -= uv;

    vector<int> candidate_disturbance_seq;
    for (auto cursor = 0; cursor < disturbance_seq.size(); cursor++) {
        //Base on probability
        if (mcgrp.is_edge(disturbance_seq[cursor]) && mcgrp._rng.Randfloat(0, 1) > 0.5) {
            candidate_disturbance_seq.push_back(mcgrp.inst_tasks[disturbance_seq[cursor]].inverse);
        }
        else {
            candidate_disturbance_seq.push_back(disturbance_seq[cursor]);
        }
    }

    vector<int> actual_disturbance_seq;
    //means no inverse happen
    if (candidate_disturbance_seq == disturbance_seq) {
        for (auto cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
            u_route_length_delta +=
                mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[
                    cursor + 1]].head_node];
            u_route_length_delta += mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
        }
        u_route_length_delta += mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;
        //handle ends
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[v].head_node];

        actual_disturbance_seq = disturbance_seq;
    }
        //inverse happens
    else {
        auto candidate_u_route_length_delta = u_route_length_delta;

        for (auto cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
            u_route_length_delta +=
                mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[
                    cursor + 1]].head_node];
            u_route_length_delta += mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
        }
        u_route_length_delta += mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;
        //handle ends
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[v].head_node];


        for (auto cursor = 0; cursor < candidate_disturbance_seq.size() - 1; cursor++) {
            candidate_u_route_length_delta +=
                mcgrp.min_cost[mcgrp.inst_tasks[candidate_disturbance_seq[cursor]].tail_node][mcgrp
                    .inst_tasks[candidate_disturbance_seq[cursor + 1]].head_node];
            candidate_u_route_length_delta += mcgrp.inst_tasks[candidate_disturbance_seq[cursor]].serv_cost;
        }
        candidate_u_route_length_delta += mcgrp.inst_tasks[candidate_disturbance_seq.back()].serv_cost;
        //handle ends
        candidate_u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[candidate_disturbance_seq.front()]
                .head_node];
        candidate_u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[candidate_disturbance_seq.back()].tail_node][mcgrp.inst_tasks[v].head_node];

        //lower is better
        if (candidate_u_route_length_delta < u_route_length_delta) {
            actual_disturbance_seq = candidate_disturbance_seq;
            u_route_length_delta = candidate_u_route_length_delta;
        }
        else {
            actual_disturbance_seq = disturbance_seq;
        }
    }



    move_result.choose_tasks(disturbance_seq.front(), u);

    move_result.move_arguments = disturbance_seq;

    // Check if we need to reduce the # of routes here
    if (start_i == disturbance_seq.front() && end_i == disturbance_seq.back())
        move_result.total_number_of_routes = ns.routes.size() - 1;
    else
        move_result.total_number_of_routes = ns.routes.size();


    // Check feasibility
    if (i_route == u_route) {

        move_result.num_affected_routes = 1;

        move_result.route_id.push_back(i_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;
        move_result.route_lens.push_back(ns.routes[i_route].length + move_result.delta);

        move_result.route_loads.push_back(ns.routes[i_route].load);

        move_result.route_custs_num.push_back(ns.routes[i_route].num_customers);


        move_result.move_arguments
            .insert(move_result.move_arguments.end(), actual_disturbance_seq.begin(), actual_disturbance_seq.end());
        move_result.move_arguments.push_back(u);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;
    }
    else {
        // Different routes!
        move_result.num_affected_routes = 2;
        move_result.route_id.push_back(i_route);
        move_result.route_id.push_back(u_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;

        move_result.route_lens.push_back(ns.routes[i_route].length + i_route_length_delta);
        move_result.route_lens.push_back(ns.routes[u_route].length + u_route_length_delta);

        move_result.route_loads.push_back(ns.routes[i_route].load + i_load_delta);
        move_result.route_loads.push_back(ns.routes[u_route].load + u_load_delta);

        move_result.route_custs_num.push_back(ns.routes[i_route].num_customers - actual_disturbance_seq.size());
        move_result.route_custs_num.push_back(ns.routes[u_route].num_customers + actual_disturbance_seq.size());


        move_result.move_arguments
            .insert(move_result.move_arguments.end(), actual_disturbance_seq.begin(), actual_disturbance_seq.end());
        move_result.move_arguments.push_back(u);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;


        move_result.considerable = true;
    }

    return true;
}

void MoveString::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a double insert:postsert move");

    My_Assert(move_result.considerable,"Invalid predictions");


    My_Assert(move_result.move_arguments.size() % 2 != 0, "move arguments should be odd!");

    //extract move arguments
    const int u = move_result.move_arguments.back();
    vector<int> original_disturbance_sequence(move_result.move_arguments.begin(),
                                              move_result.move_arguments.begin()
                                                  + (move_result.move_arguments.size() - 1) / 2);

    vector<int>
        actual_disturbance_sequence(move_result.move_arguments.begin() + (move_result.move_arguments.size() - 1) / 2,
                                    move_result.move_arguments.end() - 1);

    My_Assert(original_disturbance_sequence.size() == actual_disturbance_sequence.size(),
              "Disturbance sequence should be equal!");

    Individual individual;
    ns.create_individual(mcgrp, individual);

    My_Assert(ns.delimiter_coding_sol == individual.sequence,
              "Inconsistency between negative coding and delimiter coding.");

    auto ite_i =
        std::find(individual.sequence.begin(), individual.sequence.end(), original_disturbance_sequence.front());
    auto
        ite_k = std::find(individual.sequence.begin(), individual.sequence.end(), original_disturbance_sequence.back());

    My_Assert(ite_i != individual.sequence.end() && ite_k != individual.sequence.end(),
              "Can't find corresponding disturbance task in sequence!");
    individual.sequence.erase(ite_i, ite_k + 1);

    auto ite_u = std::find(individual.sequence.begin(), individual.sequence.end(), u);

    My_Assert(ite_u != individual.sequence.end(),
              "Can't find corresponding task in sequence!");
    individual.sequence.insert(ite_u + 1, actual_disturbance_sequence.begin(), actual_disturbance_sequence.end());


    if (move_result.total_number_of_routes != ns.routes.size()) {
        ns.compress_solution(individual.sequence, move_result.total_number_of_routes);
    }

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);
    //    ns.create_individual(mcgrp, ns.ns_indi);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");

    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

//    if(ns.cur_solution_cost < ns.policy.cost_benchmark){
//        ns.policy.cost_benchmark = ns.cur_solution_cost;
//    }

    move_result.reset();
    move_result.move_type = NeighborOperator::MOVE_STRING;

    ns.search_step++;
}

//---------------------------------------------------------------------------------------------



bool PostMoveString::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, vector<int> disturbance_seq, const int u)
{
    // task u cannot be dummy task
    My_Assert(u>=1 && u<=mcgrp.actual_task_num,"Wrong task");
    My_Assert(all_of(distrubance_seq.begin(),distrubance_seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");

    if(u == ns.solution[disturbance_seq.back()]->next->ID){
        // Nothing to do
        move_result.reset();
        return false;
    }

    const int i_route = ns.solution[disturbance_seq.front()]->route_id;
    const int u_route = ns.solution[u]->route_id;

    // Check load feasibility easily
    int load_delta = 0;
    double vio_load_delta = 0;
    //not the same route
    if (i_route != u_route) {
        for (auto task : disturbance_seq) {
            load_delta += mcgrp.inst_tasks[task].demand;
        }

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route]->load + load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (ns.routes[u_route]->load + load_delta > mcgrp.capacity) {
                //if insert task to route u and over load
                if (ns.routes[u_route]->load >= mcgrp.capacity) {
                    //if the route u already over loaded
                    vio_load_delta += load_delta;
                }
                else {
                    vio_load_delta += ns.routes[u_route]->load + load_delta - mcgrp.capacity;
                }
            }

            //i_route vio-load calculate
            if (ns.routes[i_route]->load > mcgrp.capacity) {
                //if remove task from route i and over load
                if (ns.routes[i_route]->load - load_delta >= mcgrp.capacity) {
                    //if still over loaded
                    vio_load_delta -= load_delta;
                }
                else {
                    vio_load_delta -= (ns.routes[i_route]->load - mcgrp.capacity);
                }
            }
        }
    }



    const int v = max(ns.solution[u]->next->ID, 0);

    const int h = max(ns.solution[disturbance_seq.front()]->pre->ID, 0);
    const int l = max(ns.solution[disturbance_seq.back()]->next->ID, 0);

    const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
    const double kl = mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[l].head_node];
    const double hl = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[l].head_node];

    double i_route_length_delta = hl - (hi + kl);
    for (int cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
        i_route_length_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[cursor
                + 1]].head_node];
        i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
    }
    i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;


    const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
    double u_route_length_delta = -uv;

    for (auto cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[
                cursor + 1]].head_node];
        u_route_length_delta += mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
    }
    u_route_length_delta += mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;
    //handle ends
    u_route_length_delta +=
        mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
    u_route_length_delta +=
        mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[v].head_node];


    move_result.choose_tasks(disturbance_seq.front(), u);

    move_result.move_arguments = disturbance_seq;
    move_result.move_arguments.push_back(u);

    // Check if we need to reduce the # of routes here
    const int start_i = ns.routes[i_route]->start;
    const int end_i = ns.routes[i_route]->end;
    if (start_i == disturbance_seq.front() && end_i == disturbance_seq.back())
        move_result.total_number_of_routes = ns.routes.activated_route_id.size() - 1;
    else
        move_result.total_number_of_routes = ns.routes.activated_route_id.size();


    if (i_route == u_route) {

        move_result.num_affected_routes = 1;

        move_result.route_id.push_back(i_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;
        move_result.route_lens.push_back(ns.routes[i_route]->length + move_result.delta);

        move_result.route_loads.push_back(ns.routes[i_route]->load);

        move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }
    else {
        // Different routes!
        move_result.num_affected_routes = 2;
        move_result.route_id.push_back(i_route);
        move_result.route_id.push_back(u_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;

        move_result.route_lens.push_back(ns.routes[i_route]->length + i_route_length_delta);
        move_result.route_lens.push_back(ns.routes[u_route]->length + u_route_length_delta);

        move_result.route_loads.push_back(ns.routes[i_route]->load - load_delta);
        move_result.route_loads.push_back(ns.routes[u_route]->load + load_delta);

        move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers - disturbance_seq.size());
        move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers + disturbance_seq.size());

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }

    My_Assert(false,"Cannot reach here!");
}

void PostMoveString::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a double insert:postsert move");
    My_Assert(move_result.considerable,"Invalid predictions");

    //extract move arguments
    const int u = move_result.move_arguments.back();
    vector<int> disturbance_sequence(move_result.move_arguments.begin(),move_result.move_arguments.end() - 1);

    My_Assert(u >= 1 && u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(all_of(disturbance_sequence.begin(),disturbance_sequence.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong arguments");

    if (move_result.total_number_of_routes != ns.routes.activated_route_id.size()) {
        //need to eliminate an empty route
        My_Assert(move_result.total_number_of_routes == ns.routes.activated_route_id.size() - 1,"Incorrect routes number");
        My_Assert(move_result.num_affected_routes == 2,"Incorrect routes number");
        const int i_route = move_result.route_id[0];
        const int u_route = move_result.route_id[1];
        My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
        My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
        My_Assert(ns.routes[i_route]->start == disturbance_sequence.front() && ns.routes[i_route]->end == disturbance_sequence.back(),"Wrong route");

        //Modify routes info
        ns.routes[u_route]->length = move_result.route_lens[1];

        ns.routes[u_route]->load = move_result.route_loads[1];

        ns.routes[u_route]->num_customers = move_result.route_custs_num[1];

        for(auto task:disturbance_sequence){
            ns.solution[task]->route_id = u_route;
        }

        if(ns.solution[u]->next->ID < 0){
            ns.routes[u_route]->end = disturbance_sequence.back();
        }

        ns.routes.free_route(i_route);

        //handle solution
        //record next dummy task of actual u
        int dummy_marker = 0;
        My_Assert(ns.solution[disturbance_sequence.front()]->pre->ID < 0 && ns.solution[disturbance_sequence.back()]->next->ID < 0,"Route is not empty!");
        if(ns.solution[disturbance_sequence.back()]->next == ns.solution.very_end){
            dummy_marker = ns.solution[disturbance_sequence.front()]->pre->ID;
        }
        else{
            dummy_marker = ns.solution[disturbance_sequence.back()]->next->ID;
        }

        //handle solution
        //remove
        ns.solution[disturbance_sequence.front()]->pre->next = ns.solution[disturbance_sequence.back()]->next;
        ns.solution[disturbance_sequence.back()]->next->pre = ns.solution[disturbance_sequence.front()]->pre;

        //postsert
        ns.solution[disturbance_sequence.front()]->pre = ns.solution[u];
        ns.solution[disturbance_sequence.back()]->next = ns.solution[u]->next;
        ns.solution[u]->next->pre = ns.solution[disturbance_sequence.back()];
        ns.solution[u]->next = ns.solution[disturbance_sequence.front()];

        //free dummy marker
        ns.solution[dummy_marker]->pre->next = ns.solution[dummy_marker]->next;
        ns.solution[dummy_marker]->next->pre = ns.solution[dummy_marker]->pre;
        ns.solution.dummypool.free_dummy(dummy_marker);
    }
    else {
        //no routes eliminates
        //Modify routes info
        if (move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[disturbance_sequence.front()]->pre->ID < 0){
                ns.routes[route_id]->start = ns.solution[disturbance_sequence.back()]->next->ID;
            }

            if(ns.solution[disturbance_sequence.back()]->next->ID < 0){
                ns.routes[route_id]->end = ns.solution[disturbance_sequence.front()]->pre->ID;
            }

            if(ns.solution[u]->next->ID < 0){
                ns.routes[route_id]->end = disturbance_sequence.back();
            }
        }
        else{
            const int i_route = move_result.route_id[0];
            const int u_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[i_route]->length = move_result.route_lens[0];
            ns.routes[u_route]->length = move_result.route_lens[1];

            ns.routes[i_route]->load = move_result.route_loads[0];
            ns.routes[u_route]->load = move_result.route_loads[1];

            ns.routes[i_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[u_route]->num_customers = move_result.route_custs_num[1];

            for(auto task:disturbance_sequence){
                ns.solution[task]->route_id = u_route;
            }

            if(ns.solution[disturbance_sequence.front()]->pre->ID < 0){
                ns.routes[i_route]->start = ns.solution[disturbance_sequence.back()]->next->ID;
            }

            if(ns.solution[disturbance_sequence.back()]->next->ID < 0){
                ns.routes[i_route]->end = ns.solution[disturbance_sequence.front()]->pre->ID;
            }

            if(ns.solution[u]->next->ID < 0){
                ns.routes[u_route]->end = disturbance_sequence.back();
            }
        }

        //handle solution
        //remove
        ns.solution[disturbance_sequence.front()]->pre->next = ns.solution[disturbance_sequence.back()]->next;
        ns.solution[disturbance_sequence.back()]->next->pre = ns.solution[disturbance_sequence.front()]->pre;

        //postsert
        ns.solution[disturbance_sequence.front()]->pre = ns.solution[u];
        ns.solution[disturbance_sequence.back()]->next = ns.solution[u]->next;
        ns.solution[u]->next->pre = ns.solution[disturbance_sequence.back()];
        ns.solution[u]->next = ns.solution[disturbance_sequence.front()];

    }


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

//---------------------------------------------------------------------------------------------



bool PreMoveString::considerable_move(HighSpeedNeighBorSearch &ns, const class MCGRP &mcgrp, vector<int> disturbance_seq, const int u)
{

    // task u cannot be dummy task
    My_Assert(u >= 1 && u <= mcgrp.actual_task_num,"Wrong task");
    My_Assert(all_of(disturbance_seq.begin(), disturbance_seq.end(), [&](int i){return i>=1 && i<=mcgrp.actual_task_num;}), "Wrong task");

    if(u == ns.solution[disturbance_seq.front()]->pre->ID){
        // Nothing to do
        move_result.reset();
        return false;
    }

    const int i_route = ns.solution[disturbance_seq.front()]->route_id;
    const int u_route = ns.solution[u]->route_id;

    // Check load feasibility easily
    int load_delta = 0;
    double vio_load_delta = 0;
    //not the same route
    if (i_route != u_route) {
        for (auto task : disturbance_seq) {
            load_delta += mcgrp.inst_tasks[task].demand;
        }

        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[u_route]->load + load_delta > mcgrp.capacity) {
                move_result.reset();
                return false;
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //u_route vio-load calculate
            if (ns.routes[u_route]->load + load_delta > mcgrp.capacity) {
                //if insert task to route u and over load
                if (ns.routes[u_route]->load >= mcgrp.capacity) {
                    //if the route u already over loaded
                    vio_load_delta += load_delta;
                }
                else {
                    vio_load_delta += ns.routes[u_route]->load + load_delta - mcgrp.capacity;
                }
            }

            //i_route vio-load calculate
            if (ns.routes[i_route]->load > mcgrp.capacity) {
                //if remove task from route i and over load
                if (ns.routes[i_route]->load - load_delta >= mcgrp.capacity) {
                    //if still over loaded
                    vio_load_delta -= load_delta;
                }
                else {
                    vio_load_delta -= (ns.routes[i_route]->load - mcgrp.capacity);
                }
            }
        }
    }



    const int t = max(ns.solution[u]->pre->ID, 0);

    const int h = max(ns.solution[disturbance_seq.front()]->pre->ID, 0);
    const int l = max(ns.solution[disturbance_seq.back()]->next->ID, 0);

    const double hi = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
    const double kl = mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[l].head_node];
    const double hl = mcgrp.min_cost[mcgrp.inst_tasks[h].tail_node][mcgrp.inst_tasks[l].head_node];

    double i_route_length_delta = hl - (hi + kl);

    for (int cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
        i_route_length_delta -=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[cursor
                + 1]].head_node];
        i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
    }
    i_route_length_delta -= mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;


    const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
    double u_route_length_delta = -tu;

    for (auto cursor = 0; cursor < disturbance_seq.size() - 1; cursor++) {
        u_route_length_delta +=
            mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq[cursor]].tail_node][mcgrp.inst_tasks[disturbance_seq[
                cursor + 1]].head_node];
        u_route_length_delta += mcgrp.inst_tasks[disturbance_seq[cursor]].serv_cost;
    }

    u_route_length_delta += mcgrp.inst_tasks[disturbance_seq.back()].serv_cost;
    //handle ends
    u_route_length_delta +=
        mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[disturbance_seq.front()].head_node];
    u_route_length_delta +=
        mcgrp.min_cost[mcgrp.inst_tasks[disturbance_seq.back()].tail_node][mcgrp.inst_tasks[u].head_node];


    move_result.choose_tasks(disturbance_seq.front(), u);

    move_result.move_arguments = disturbance_seq;
    move_result.move_arguments.push_back(u);


    // Check if we need to reduce the # of routes here
    const int start_i = ns.routes[i_route]->start;
    const int end_i = ns.routes[i_route]->end;
    if (start_i == disturbance_seq.front() && end_i == disturbance_seq.back())
        move_result.total_number_of_routes = ns.routes.activated_route_id.size() - 1;
    else
        move_result.total_number_of_routes = ns.routes.activated_route_id.size();


    if (i_route == u_route) {

        move_result.num_affected_routes = 1;

        move_result.route_id.push_back(i_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;
        move_result.route_lens.push_back(ns.routes[i_route]->length + move_result.delta);

        move_result.route_loads.push_back(ns.routes[i_route]->load);

        move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers);

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }
    else {
        // Different routes!
        move_result.num_affected_routes = 2;
        move_result.route_id.push_back(i_route);
        move_result.route_id.push_back(u_route);

        move_result.delta = u_route_length_delta + i_route_length_delta;

        move_result.route_lens.push_back(ns.routes[i_route]->length + i_route_length_delta);
        move_result.route_lens.push_back(ns.routes[u_route]->length + u_route_length_delta);

        move_result.route_loads.push_back(ns.routes[i_route]->load - load_delta);
        move_result.route_loads.push_back(ns.routes[u_route]->load + load_delta);

        move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers - disturbance_seq.size());
        move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers + disturbance_seq.size());

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.vio_load_delta = vio_load_delta;

        move_result.considerable = true;

        return true;
    }

    My_Assert(false,"Cannot reach here!");
}

void PreMoveString::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a double insert:postsert move");
    My_Assert(move_result.considerable,"Invalid predictions");

    //extract move arguments
    const int u = move_result.move_arguments.back();
    vector<int> disturbance_sequence(move_result.move_arguments.begin(),move_result.move_arguments.end() - 1);

    My_Assert(u >= 1 && u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(all_of(disturbance_sequence.begin(),disturbance_sequence.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong arguments");

    if (move_result.total_number_of_routes != ns.routes.activated_route_id.size()) {
        //need to eliminate an empty route
        My_Assert(move_result.total_number_of_routes == ns.routes.activated_route_id.size() - 1,"Incorrect routes number");
        My_Assert(move_result.num_affected_routes == 2,"Incorrect routes number");
        const int i_route = move_result.route_id[0];
        const int u_route = move_result.route_id[1];
        My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
        My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
        My_Assert(ns.routes[i_route]->start == disturbance_sequence.front() && ns.routes[i_route]->end == disturbance_sequence.back(),"Wrong route");

        //Modify routes info
        ns.routes[u_route]->length = move_result.route_lens[1];

        ns.routes[u_route]->load = move_result.route_loads[1];

        ns.routes[u_route]->num_customers = move_result.route_custs_num[1];

        for(auto task:disturbance_sequence){
            ns.solution[task]->route_id = u_route;
        }

        if(ns.solution[u]->pre->ID < 0){
            ns.routes[u_route]->start = disturbance_sequence.front();
        }

        ns.routes.free_route(i_route);

        //handle solution
        //record next dummy task of actual u
        int dummy_marker = 0;
        My_Assert(ns.solution[disturbance_sequence.front()]->pre->ID < 0 && ns.solution[disturbance_sequence.back()]->next->ID < 0,"Route is not empty!");
        if(ns.solution[disturbance_sequence.back()]->next == ns.solution.very_end){
            dummy_marker = ns.solution[disturbance_sequence.front()]->pre->ID;
        }
        else{
            dummy_marker = ns.solution[disturbance_sequence.back()]->next->ID;
        }

        //handle solution
        //remove
        ns.solution[disturbance_sequence.front()]->pre->next = ns.solution[disturbance_sequence.back()]->next;
        ns.solution[disturbance_sequence.back()]->next->pre = ns.solution[disturbance_sequence.front()]->pre;

        //presert
        ns.solution[disturbance_sequence.front()]->pre = ns.solution[u]->pre;
        ns.solution[disturbance_sequence.back()]->next = ns.solution[u];
        ns.solution[u]->pre->next = ns.solution[disturbance_sequence.front()];
        ns.solution[u]->pre = ns.solution[disturbance_sequence.back()];

        //free dummy marker
        ns.solution[dummy_marker]->pre->next = ns.solution[dummy_marker]->next;
        ns.solution[dummy_marker]->next->pre = ns.solution[dummy_marker]->pre;
        ns.solution.dummypool.free_dummy(dummy_marker);
    }
    else{
        //no routes eliminates
        //Modify routes info
        if (move_result.num_affected_routes == 1) {
            //No need to change route id
            const int route_id = move_result.route_id[0];
            My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[route_id]->length = move_result.route_lens[0];

            if(ns.solution[disturbance_sequence.front()]->pre->ID < 0){
                ns.routes[route_id]->start = ns.solution[disturbance_sequence.back()]->next->ID;
            }

            if(ns.solution[disturbance_sequence.back()]->next->ID < 0){
                ns.routes[route_id]->end = ns.solution[disturbance_sequence.front()]->pre->ID;
            }

            if(ns.solution[u]->pre->ID < 0){
                ns.routes[route_id]->start = disturbance_sequence.front();
            }
        }
        else{
            const int i_route = move_result.route_id[0];
            const int u_route = move_result.route_id[1];

            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[i_route]->length = move_result.route_lens[0];
            ns.routes[u_route]->length = move_result.route_lens[1];

            ns.routes[i_route]->load = move_result.route_loads[0];
            ns.routes[u_route]->load = move_result.route_loads[1];

            ns.routes[i_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[u_route]->num_customers = move_result.route_custs_num[1];

            for(auto task:disturbance_sequence){
                ns.solution[task]->route_id = u_route;
            }

            if(ns.solution[disturbance_sequence.front()]->pre->ID < 0){
                ns.routes[i_route]->start = ns.solution[disturbance_sequence.back()]->next->ID;
            }

            if(ns.solution[disturbance_sequence.back()]->next->ID < 0){
                ns.routes[i_route]->end = ns.solution[disturbance_sequence.front()]->pre->ID;
            }

            if(ns.solution[u]->pre->ID < 0){
                ns.routes[u_route]->start = disturbance_sequence.front();
            }
        }

        //handle solution
        //remove
        ns.solution[disturbance_sequence.front()]->pre->next = ns.solution[disturbance_sequence.back()]->next;
        ns.solution[disturbance_sequence.back()]->next->pre = ns.solution[disturbance_sequence.front()]->pre;

        //presert
        ns.solution[disturbance_sequence.front()]->pre = ns.solution[u]->pre;
        ns.solution[disturbance_sequence.back()]->next = ns.solution[u];
        ns.solution[u]->pre->next = ns.solution[disturbance_sequence.front()];
        ns.solution[u]->pre = ns.solution[disturbance_sequence.back()];

    }


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