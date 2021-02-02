#include "PostSert.h"
#include <algorithm>

using std::max;


bool Postsert::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int u, const int i)
{
    const int original_u = u;


    int start_u, end_u;
    int t, v, j, u_tilde, demand_change, i_route_load, u_route_load, i_route, u_route;
    double tu, uv, tv, iu, ij, iu_tilde, u_tildej;
    double uj, delta, delta1;
    double u_delta, i_delta, i_delta1, i_route_length, u_route_length;


    i_route = ns.route_id[i];
    u_route = ns.route_id[u];

    if (ns.next_array[i] == u) {
        // Nothing to do
        move_result.reset();
        return false;
    }

    // Can check load feasibility easily
    double vio_load_delta = 0;
    if (u_route != i_route) {
        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[i_route].load + mcgrp.inst_tasks[u].demand > mcgrp.capacity) {
                move_result.reset();
                return false;
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //i_route vio-load calculate
            if (ns.routes[i_route].load + mcgrp.inst_tasks[u].demand > mcgrp.capacity) {
                //if insert task to route i and over load
                if (ns.routes[i_route].load >= mcgrp.capacity) {
                    //if the route i already over loaded
                    vio_load_delta += mcgrp.inst_tasks[u].demand;
                }
                else {
                    vio_load_delta += ns.routes[i_route].load + mcgrp.inst_tasks[u].demand - mcgrp.capacity;
                }
            }

            //u_route vio-load calculate
            if (ns.routes[u_route].load > mcgrp.capacity) {
                //if remove task from route u and over load
                if (ns.routes[u_route].load - mcgrp.inst_tasks[u].demand >= mcgrp.capacity) {
                    //if still over loaded
                    vio_load_delta -= mcgrp.inst_tasks[u].demand;
                }
                else {
                    vio_load_delta -= (ns.routes[u_route].load - mcgrp.capacity);
                }
            }
        }
    }

    move_result.choose_tasks(original_u, i);
    move_result.move_arguments.push_back(original_u);

    if (mcgrp.is_edge(u)) {    //���ڱ�����
        // First, calculate the relevant nodes and distances
        t = max(ns.pred_array[u], DUMMY);
        v = max(ns.next_array[u], DUMMY);
        j = max(ns.next_array[i], DUMMY);

        u_tilde = mcgrp.inst_tasks[u].inverse;

        tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        tv = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[v].head_node];

        iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
        uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];
        ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];

        iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
        u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];


        u_delta = tv - tu - uv - mcgrp.inst_tasks[u].serv_cost;
        i_delta = iu + uj - ij + mcgrp.inst_tasks[u].serv_cost;
        i_delta1 = iu_tilde + u_tildej - ij + mcgrp.inst_tasks[u_tilde].serv_cost;
        delta = i_delta + u_delta;
        delta1 = i_delta1 + u_delta;

        if (delta1 < delta) {   //inverse task is better
            u = u_tilde;
            iu = iu_tilde;
            uj = u_tildej;
            i_delta = i_delta1;
            delta = delta1;
        }

        start_u = ns.routes[u_route].start;
        end_u = ns.routes[u_route].end;

        if (u_route == i_route) {
            // u and i were in the same route originally
            // No overall change in the load here - we just shifted u around
            demand_change = 0;
            // The total length of route i is now old_length + delta
            i_route_length = ns.routes[i_route].length + delta;
            u_route_length = i_route_length;
            i_route_load = ns.routes[i_route].load + demand_change;
            u_route_load = ns.routes[u_route].load - demand_change;
        }
        else {
            // Different routes
            demand_change = mcgrp.inst_tasks[u].demand;
            i_route_length = ns.routes[i_route].length + i_delta;
            u_route_length = ns.routes[u_route].length + u_delta;
            i_route_load = ns.routes[i_route].load + demand_change;
            u_route_load = ns.routes[u_route].load - demand_change;

        }

        // Check if we need to reduce the # of routes here
        // The original u route only has task u itself
        if (start_u == end_u)
            move_result.total_number_of_routes = ns.routes.size() - 1;
        else
            move_result.total_number_of_routes = ns.routes.size();

        if (u_route == i_route) {
            move_result.num_affected_routes = 1;
            move_result.delta = delta;

            move_result.route_id.push_back(u_route);

            move_result.route_lens.push_back(u_route_length);

            move_result.route_loads.push_back(u_route_load);

            move_result.route_custs_num.push_back(ns.routes[u_route].num_customers); // no change

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;


            move_result.move_type = NeighborOperator::POSTSERT;
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

            move_result.route_lens.push_back(u_route_length);
            move_result.route_lens.push_back(i_route_length);


            move_result.route_loads.push_back(u_route_load);
            move_result.route_loads.push_back(i_route_load);

            My_Assert(u != DUMMY, "You cannot postsert a dummy task");
            move_result.route_custs_num.push_back(ns.routes[u_route].num_customers - 1);
            move_result.route_custs_num.push_back(ns.routes[i_route].num_customers + 1);

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
            move_result.move_type = NeighborOperator::POSTSERT;

            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;


            move_result.considerable = true;
        }

    }
    else {
        // First, calculate the relevant nodes and distances
        t = max(ns.pred_array[u], DUMMY);
        v = max(ns.next_array[u], DUMMY);
        j = max(ns.next_array[i], DUMMY);

        tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        tv = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[v].head_node];

        iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
        uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];
        ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];

        u_delta = tv - tu - uv - mcgrp.inst_tasks[u].serv_cost;
        i_delta = iu + uj - ij + mcgrp.inst_tasks[u].serv_cost;
        delta = i_delta + u_delta;

        start_u = ns.routes[u_route].start;
        end_u = ns.routes[u_route].end;

        if (u_route == i_route) {
            // u and i were in the same route originally
            // No overall change in the load here - we just shifted u around
            demand_change = 0;
            // The total length of route i is now old_length + delta
            i_route_length = ns.routes[i_route].length + delta;
            u_route_length = i_route_length;
            i_route_load = ns.routes[i_route].load + demand_change;
            u_route_load = ns.routes[u_route].load - demand_change;
        }
        else {
            // Different routes
            demand_change = mcgrp.inst_tasks[u].demand;

            i_route_length = ns.routes[i_route].length + i_delta;
            u_route_length = ns.routes[u_route].length + u_delta;
            i_route_load = ns.routes[i_route].load + demand_change;
            u_route_load = ns.routes[u_route].load - demand_change;
        }

        // Check if we need to reduce the # of routes here
        // The original u route only has task u itself
        if (start_u == end_u)
            move_result.total_number_of_routes = ns.routes.size() - 1;
        else
            move_result.total_number_of_routes = ns.routes.size();

        if (u_route == i_route) {
            move_result.num_affected_routes = 1;
            move_result.delta = delta;

            move_result.route_id.push_back(u_route);

            move_result.route_lens.push_back(u_route_length);

            move_result.route_loads.push_back(u_route_load);

            move_result.route_custs_num.push_back(ns.routes[u_route].num_customers); // no change

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

            move_result.move_type = NeighborOperator::POSTSERT;
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

            move_result.route_lens.push_back(u_route_length);
            move_result.route_lens.push_back(i_route_length);

            move_result.route_loads.push_back(u_route_load);
            move_result.route_loads.push_back(i_route_load);

            My_Assert(u != DUMMY, "You cannot postsert a dummy task");
            move_result.route_custs_num.push_back(ns.routes[u_route].num_customers - 1);
            move_result.route_custs_num.push_back(ns.routes[i_route].num_customers + 1);

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

            move_result.move_type = NeighborOperator::POSTSERT;
            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;

            move_result.considerable = true;
        }

    }

    return true;
}

void Postsert::move(NeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a single insert:postsert move");

    My_Assert(move_result.considerable,"Invalid predictions");
    //  Indeed, I don't the previous implemantation is good,
    //  since you have to care too much about the special cases, which is too redundant.
    //  A better solution I think is to rebuild the correnponding delimiter coding format
    //  then use negative decoding to get result :).

    Individual individual;
    ns.create_individual(mcgrp, individual);

    // Indeed this method can be optimized,but I want to check the consistency here,
    // so I just rebuild the packed info.
    My_Assert(ns.delimiter_coding_sol == individual.sequence,
              "Inconsistency between negative coding and delimiter coding.");

    int original_u = move_result.move_arguments[0], actual_u = move_result.move_arguments[1],
        i = move_result.move_arguments[2];

    auto ite_u = std::find(individual.sequence.begin(), individual.sequence.end(), original_u);

    My_Assert(ite_u != individual.sequence.end(), "Can't find task u!");
    individual.sequence.erase(ite_u);

    auto ite_i = std::find(individual.sequence.begin(), individual.sequence.end(), i);

    My_Assert(ite_i != individual.sequence.end(), "Can't find task i!");
    //postsert
    individual.sequence.insert(ite_i + 1, actual_u);

    if (move_result.total_number_of_routes != ns.routes.size()) {
        ns.compress_solution(individual.sequence, move_result.total_number_of_routes);
    }

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
//    ns.create_individual(mcgrp, ns.ns_indi);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");


    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

//    if(ns.cur_solution_cost < ns.policy.cost_benchmark){
//        ns.policy.cost_benchmark = ns.cur_solution_cost;
//    }

    move_result.reset();
    move_result.move_type = NeighborOperator::POSTSERT;


    ns.search_step++;
}

bool Postsert::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int u, const int i)
{
    My_Assert(i>=1 && i<=mcgrp.actual_task_num,"Wrong task");
    My_Assert(u>=1 && u<=mcgrp.actual_task_num,"Wrong task");

    if(ns.solution[u]->pre->ID == i){
        // Nothing to do
        move_result.reset();
        return false;
    }

    const int i_route = ns.solution[i]->route_id;
    const int u_route = ns.solution[u]->route_id;


    // Can check load feasibility easily
    double vio_load_delta = 0;
    if (u_route != i_route) {
        if (ns.policy.has_rule(DELTA_ONLY)) {
            if (ns.routes[i_route]->load + mcgrp.inst_tasks[u].demand > mcgrp.capacity) {
                move_result.reset();
                return false;
            }
        }
        else if (ns.policy.has_rule(FITNESS_ONLY)) {
            //i_route vio-load calculate
            if (ns.routes[i_route]->load + mcgrp.inst_tasks[u].demand > mcgrp.capacity) {
                //if insert task to route i and over load
                if (ns.routes[i_route]->load >= mcgrp.capacity) {
                    //if the route i already over loaded
                    vio_load_delta += mcgrp.inst_tasks[u].demand;
                }
                else {
                    vio_load_delta += ns.routes[i_route]->load + mcgrp.inst_tasks[u].demand - mcgrp.capacity;
                }
            }

            //u_route vio-load calculate
            if (ns.routes[u_route]->load > mcgrp.capacity) {
                //if remove task from route u and over load
                if (ns.routes[u_route]->load - mcgrp.inst_tasks[u].demand >= mcgrp.capacity) {
                    //if still over loaded
                    vio_load_delta -= mcgrp.inst_tasks[u].demand;
                }
                else {
                    vio_load_delta -= (ns.routes[u_route]->load - mcgrp.capacity);
                }
            }
        }
    }

    const int original_u = u;

    move_result.choose_tasks(original_u, i);
    move_result.move_arguments.push_back(original_u);

    if (mcgrp.is_edge(u)) {
        const int t = max(ns.solution[u]->pre->ID, 0);
        const int v = max(ns.solution[u]->next->ID, 0);
        const int j = max(ns.solution[i]->next->ID, 0);

        const int u_tilde = mcgrp.inst_tasks[u].inverse;

        const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        const double tv = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[v].head_node];

        const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];
        const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];

        const double iu_tilde = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u_tilde].head_node];
        const double u_tildej = mcgrp.min_cost[mcgrp.inst_tasks[u_tilde].tail_node][mcgrp.inst_tasks[j].head_node];


        const double u_delta = tv - tu - uv - mcgrp.inst_tasks[u].serv_cost;
        double i_delta = iu + uj - ij + mcgrp.inst_tasks[u].serv_cost;
        const double i_delta1 = iu_tilde + u_tildej - ij + mcgrp.inst_tasks[u_tilde].serv_cost;
        double delta = i_delta + u_delta;
        const double delta1 = i_delta1 + u_delta;

        if (delta1 < delta) {
            u = u_tilde;
            i_delta = i_delta1;
            delta = delta1;
        }

        const int start_u = ns.routes[u_route]->start;
        const int end_u = ns.routes[u_route]->end;
        // Check if we need to reduce the # of routes here
        // The original u route only has task u itself
        if (start_u == end_u)
            move_result.total_number_of_routes = ns.routes.activated_route_id.size() - 1;
        else
            move_result.total_number_of_routes = ns.routes.activated_route_id.size();


        if (u_route == i_route) {
            // u and i were in the same route originally
            // The total length of route i is now old_length + delta
            const double u_route_length = ns.routes[u_route]->length + delta;
            const double u_route_load = ns.routes[u_route]->load;

            move_result.num_affected_routes = 1;
            move_result.delta = delta;

            move_result.route_id.push_back(u_route);

            move_result.route_lens.push_back(u_route_length);

            move_result.route_loads.push_back(u_route_load);

            move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers); // no change

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;

            move_result.considerable = true;

            return true;
        }
        else {
            // Different routes
            const double demand_change = mcgrp.inst_tasks[u].demand;
            const double i_route_length = ns.routes[i_route]->length + i_delta;
            const double u_route_length = ns.routes[u_route]->length + u_delta;
            const double i_route_load = ns.routes[i_route]->load + demand_change;
            const double u_route_load = ns.routes[u_route]->load - demand_change;

            move_result.num_affected_routes = 2;
            move_result.delta = delta;

            move_result.route_id.push_back(u_route);
            move_result.route_id.push_back(i_route);

            move_result.route_lens.push_back(u_route_length);
            move_result.route_lens.push_back(i_route_length);

            move_result.route_loads.push_back(u_route_load);
            move_result.route_loads.push_back(i_route_load);

            My_Assert(u != DUMMY, "You cannot postsert a dummy task");
            move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers - 1);
            move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers + 1);

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;

            move_result.considerable = true;

            return true;
        }
    }
    else {
        const int t = max(ns.solution[u]->pre->ID, 0);
        const int v = max(ns.solution[u]->next->ID, 0);
        const int j = max(ns.solution[i]->next->ID, 0);

        const double tu = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uv = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[v].head_node];
        const double tv = mcgrp.min_cost[mcgrp.inst_tasks[t].tail_node][mcgrp.inst_tasks[v].head_node];

        const double iu = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[u].head_node];
        const double uj = mcgrp.min_cost[mcgrp.inst_tasks[u].tail_node][mcgrp.inst_tasks[j].head_node];
        const double ij = mcgrp.min_cost[mcgrp.inst_tasks[i].tail_node][mcgrp.inst_tasks[j].head_node];

        const double u_delta = tv - tu - uv - mcgrp.inst_tasks[u].serv_cost;
        const double i_delta = iu + uj - ij + mcgrp.inst_tasks[u].serv_cost;
        const double delta = i_delta + u_delta;

        // Check if we need to reduce the # of routes here
        // The original u route only has task u itself
        const int start_u = ns.routes[u_route]->start;
        const int end_u = ns.routes[u_route]->end;
        if (start_u == end_u)
            move_result.total_number_of_routes = ns.routes.activated_route_id.size() - 1;
        else
            move_result.total_number_of_routes = ns.routes.activated_route_id.size();


        if (u_route == i_route) {
            // u and i were in the same route originally
            // No overall change in the load here - we just shifted u around
            // The total length of route i is now old_length + delta
            const double u_route_length = ns.routes[u_route]->length + delta;
            const double u_route_load = ns.routes[u_route]->load;

            move_result.num_affected_routes = 1;
            move_result.delta = delta;

            move_result.route_loads.push_back(u_route_load);
            move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers);
            move_result.route_id.push_back(u_route);

            move_result.route_lens.push_back(u_route_length);

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;

            move_result.considerable = true;

            return true;
        }
        else {
            // Different routes
            const double demand_change = mcgrp.inst_tasks[u].demand;
            const double i_route_length = ns.routes[i_route]->length + i_delta;
            const double u_route_length = ns.routes[u_route]->length + u_delta;
            const double i_route_load = ns.routes[i_route]->load + demand_change;
            const double u_route_load = ns.routes[u_route]->load - demand_change;

            move_result.num_affected_routes = 2;
            move_result.delta = delta;

            move_result.route_id.push_back(u_route);
            move_result.route_id.push_back(i_route);

            move_result.route_lens.push_back(u_route_length);
            move_result.route_lens.push_back(i_route_length);

            move_result.route_loads.push_back(u_route_load);
            move_result.route_loads.push_back(i_route_load);

            My_Assert(u != DUMMY, "You cannot postsert a dummy task");
            move_result.route_custs_num.push_back(ns.routes[u_route]->num_customers - 1);
            move_result.route_custs_num.push_back(ns.routes[i_route]->num_customers + 1);

            move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

            move_result.move_arguments.push_back(u);
            move_result.move_arguments.push_back(i);

            move_result.vio_load_delta = vio_load_delta;

            move_result.considerable = true;

            return true;
        }
    }

    My_Assert(false,"Cannot reach here!");
}

void Postsert::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    DEBUG_PRINT("execute a single insert:postsert move");
    My_Assert(move_result.considerable,"Invalid predictions");

    const int original_u = move_result.move_arguments[0];
    const int actual_u = move_result.move_arguments[1];
    const int i = move_result.move_arguments[2];

    My_Assert(original_u >= 1 && original_u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(actual_u >= 1 && actual_u <= mcgrp.actual_task_num,"Wrong arguments");
    My_Assert(i >= 1 && i <= mcgrp.actual_task_num,"Wrong arguments");

    if(original_u != actual_u) {
        //Edge task has been moved
        My_Assert(mcgrp.inst_tasks[original_u].inverse == actual_u, "This is not the inverse");
        My_Assert(ns.solution[original_u]->next != nullptr && ns.solution[actual_u]->next == nullptr,
                  "Wrong arguments");

        if (move_result.total_number_of_routes != ns.routes.activated_route_id.size()) {
            //new task switch
            //routes eliminates
            //Modify routes info
            My_Assert(move_result.total_number_of_routes == ns.routes.activated_route_id.size() - 1,"Incorrect routes number");
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];
            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[u_route]->num_customers == 1,"Wrong route");

            //Modify routes info
            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");

            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            ns.solution[actual_u]->route_id = i_route;

            if(ns.solution[i]->next->ID < 0){
                ns.routes[i_route]->end = actual_u;
            }

            ns.routes.free_route(u_route);

            //handle solution
            //record next dummy task of actual u
            int dummy_marker = 0;
            if(ns.solution[original_u]->next == ns.solution.very_end){
                dummy_marker = ns.solution[original_u]->pre->ID;
            }
            else{
                dummy_marker = ns.solution[original_u]->next->ID;
            }

            //remove
            ns.solution[original_u]->pre->next = ns.solution[original_u]->next;
            ns.solution[original_u]->next->pre = ns.solution[original_u]->pre;

            //postsert
            ns.solution[actual_u]->pre = ns.solution[i];
            ns.solution[actual_u]->next = ns.solution[i]->next;
            ns.solution[i]->next->pre = ns.solution[actual_u];
            ns.solution[i]->next = ns.solution[actual_u];

            //free dummy marker
            ns.solution[dummy_marker]->pre->next = ns.solution[dummy_marker]->next;
            ns.solution[dummy_marker]->next->pre = ns.solution[dummy_marker]->pre;
            ns.solution.dummypool.free_dummy(dummy_marker);

        }
        else{
            //new task switch
            //no routes eliminates
            //Modify routes info
            if(move_result.num_affected_routes == 1) {
                //No need to change route id
                const int route_id = move_result.route_id[0];
                My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");

                ns.routes[route_id]->length = move_result.route_lens[0];

                if(ns.solution[original_u]->pre->ID < 0){
                    ns.routes[route_id]->start = ns.solution[original_u]->next->ID;
                }

                if(ns.solution[original_u]->next->ID < 0){
                    ns.routes[route_id]->end = ns.solution[original_u]->pre->ID;
                }

                if(ns.solution[i]->next->ID < 0){
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
                    ns.routes[u_route]->start = ns.solution[original_u]->next->ID;
                }

                if(ns.solution[original_u]->next->ID < 0){
                    ns.routes[u_route]->end = ns.solution[original_u]->pre->ID;
                }

                if(ns.solution[i]->next->ID < 0){
                    ns.routes[i_route]->end = actual_u;
                }

                ns.solution[actual_u]->route_id = i_route;
            }

            //handle solution
            //remove
            ns.solution[original_u]->pre->next = ns.solution[original_u]->next;
            ns.solution[original_u]->next->pre = ns.solution[original_u]->pre;

            //postsert
            ns.solution[actual_u]->pre = ns.solution[i];
            ns.solution[actual_u]->next = ns.solution[i]->next;
            ns.solution[i]->next->pre = ns.solution[actual_u];
            ns.solution[i]->next = ns.solution[actual_u];

        }

        //clear original u task info
        ns.solution[original_u]->clear();
    }
    else{
        if(move_result.total_number_of_routes != ns.routes.activated_route_id.size()){
            //no new task switch
            //need to eliminate an empty route
            My_Assert(move_result.total_number_of_routes == ns.routes.activated_route_id.size() - 1,"Incorrect routes number");
            const int u_route = move_result.route_id[0];
            const int i_route = move_result.route_id[1];
            My_Assert(ns.routes.activated_route_id.find(u_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes.activated_route_id.find(i_route)!=ns.routes.activated_route_id.end(),"Invalid route");
            My_Assert(ns.routes[u_route]->num_customers == 1,"Wrong route");

            //Modify routes info
            ns.routes[i_route]->length = move_result.route_lens[1];

            ns.routes[i_route]->load = move_result.route_loads[1];

            ns.routes[i_route]->num_customers = move_result.route_custs_num[1];

            ns.solution[actual_u]->route_id = i_route;

            if(ns.solution[i]->next->ID < 0){
                ns.routes[i_route]->end = actual_u;
            }

            ns.routes.free_route(u_route);

            //handle solution
            //record next dummy task of actual u
            int dummy_marker = 0;
            if(ns.solution[actual_u]->next == ns.solution.very_end){
                dummy_marker = ns.solution[actual_u]->pre->ID;
            }
            else{
                dummy_marker = ns.solution[actual_u]->next->ID;
            }

            //remove
            ns.solution[actual_u]->pre->next = ns.solution[actual_u]->next;
            ns.solution[actual_u]->next->pre = ns.solution[actual_u]->pre;

            //postsert
            ns.solution[actual_u]->pre = ns.solution[i];
            ns.solution[actual_u]->next = ns.solution[i]->next;
            ns.solution[i]->next->pre = ns.solution[actual_u];
            ns.solution[i]->next = ns.solution[actual_u];

            //free dummy marker
            ns.solution[dummy_marker]->pre->next = ns.solution[dummy_marker]->next;
            ns.solution[dummy_marker]->next->pre = ns.solution[dummy_marker]->pre;
            ns.solution.dummypool.free_dummy(dummy_marker);
        }
        else{
            //no new task switch
            //no routes eliminates
            //Modify routes info
            if(move_result.num_affected_routes == 1) {
                //No need to change route id
                const int route_id = move_result.route_id[0];
                My_Assert(ns.routes.activated_route_id.find(route_id)!=ns.routes.activated_route_id.end(),"Invalid route");

                ns.routes[route_id]->length = move_result.route_lens[0];

                if(ns.solution[actual_u]->pre->ID < 0){
                    ns.routes[route_id]->start = ns.solution[actual_u]->next->ID;
                }

                if(ns.solution[actual_u]->next->ID < 0){
                    ns.routes[route_id]->end = ns.solution[actual_u]->pre->ID;
                }

                if(ns.solution[i]->next->ID < 0){
                    ns.routes[route_id]->end = actual_u;
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

                ns.solution[actual_u]->route_id = i_route;

                if(ns.solution[actual_u]->pre->ID < 0){
                    ns.routes[u_route]->start = ns.solution[actual_u]->next->ID;
                }

                if(ns.solution[actual_u]->next->ID < 0){
                    ns.routes[u_route]->end = ns.solution[actual_u]->pre->ID;
                }

                if(ns.solution[i]->next->ID < 0){
                    ns.routes[i_route]->end = actual_u;
                }
            }

            //handle solution
            //remove
            ns.solution[actual_u]->pre->next = ns.solution[actual_u]->next;
            ns.solution[actual_u]->next->pre = ns.solution[actual_u]->pre;

            //postsert
            ns.solution[actual_u]->pre = ns.solution[i];
            ns.solution[actual_u]->next = ns.solution[i]->next;
            ns.solution[i]->next->pre = ns.solution[actual_u];
            ns.solution[i]->next = ns.solution[actual_u];
        }
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