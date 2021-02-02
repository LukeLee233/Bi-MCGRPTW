//
// Created by luke on 2020/1/26.
//

#include "Slice.h"
#include <algorithm>

using namespace std;

bool Slice::search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task){
    //No search space in Slice operator, No accept rule for invert operator
    pre_slice_times = 0;
    post_slice_times = 0;

    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    if (considerable_move(ns, mcgrp, chosen_task) && ns.policy.check_move(move_result)) {
        move(ns, mcgrp);
        return true;
    }
    else
    {
        move_result.reset();
        return false;
    }
}

bool Slice::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp,const int b){
    My_Assert(b >= 1 && b <= mcgrp.actual_task_num,"Wrong task");

    const int a = ns.solution[b]->pre->ID;
    const int c = ns.solution[b]->next->ID;

    if( a < 0 && c < 0){
        //No need do slice
        //dummy-b-dummy
        move_result.move_type = NeighborOperator::SLICE;
        move_result.reset();
        return false;
    }
    if(a < 0){
        //No need do pre slice
        //dummy-b-c...
        post_slice_times++;
        if (postslice.considerable_move(ns, mcgrp, b) && ns.policy.check_move(postslice.move_result)) {
            move_result = postslice.move_result;
            return true;
        }
        else {
            move_result.reset();
            move_result.move_type = NeighborOperator::SLICE;
            return false;
        }
    }
    else if (c < 0){
        //No need do post slice
        //...a-b-dummy
        pre_slice_times++;
        if (preslice.considerable_move(ns, mcgrp, b) && ns.policy.check_move(preslice.move_result)) {
            move_result = preslice.move_result;
            return true;
        }
        else {
            move_result.reset();
            move_result.move_type = NeighborOperator::SLICE;
            return false;
        }
    }
    else{
        //check presert and postsert
        //...a-b-c...
        preslice.considerable_move(ns, mcgrp, b);
        postslice.considerable_move(ns, mcgrp, b);

        if(preslice.move_result.considerable && postslice.move_result.considerable){
            //Both considerable
            if(preslice.move_result.delta <= postslice.move_result.delta){
                pre_slice_times++;
                if(ns.policy.check_move(preslice.move_result)){
                    move_result = preslice.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::SLICE;
                    return false;
                }
            }
            else{
                post_slice_times++;
                if(ns.policy.check_move(postslice.move_result)){
                    move_result = postslice.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::SLICE;
                    return false;
                }
            }
        }
        else if (preslice.move_result.considerable){
            //only presert is considerable
            pre_slice_times++;
            if(ns.policy.check_move(preslice.move_result)){
                move_result = preslice.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::SLICE;
                return false;
            }
        }
        else if (postslice.move_result.considerable) {
            post_slice_times++;
            if (ns.policy.check_move( postslice.move_result)) {
                move_result = postslice.move_result;
                return true;
            }
            else {
                move_result.reset();
                move_result.move_type = NeighborOperator::SLICE;
                return false;
            }
        }
        else{
            //No considerable
            move_result.reset();
            move_result.move_type = NeighborOperator::SLICE;
            return false;
        }
    }

    My_Assert(false,"Cannot reach here!");
}

void Slice::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a slice move");

    My_Assert(move_result.considerable,"Invalid predictions");

    if (move_result.move_type == NeighborOperator::PRE_SLICE) {
        preslice.move_result = move_result;
        preslice.move(ns, mcgrp);
    }
    else if (move_result.move_type == NeighborOperator::POST_SLICE) {
        postslice.move_result = move_result;
        postslice.move(ns, mcgrp);
    }
    else{
        My_Assert(false,"Unknown operator");
    }

    ns.trace(mcgrp);

    move_result.reset();
    move_result.move_type = NeighborOperator::SLICE;
}

void Slice::unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
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

        if (ns.solution[chosen_task]->next == nullptr) {
            if (mcgrp.is_edge(chosen_task)) {
                chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                My_Assert(ns.solution[chosen_task]->next != nullptr, "An edge task has been missed");
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

/*
 * Pre Slice
 */


struct RouteSegment
{
    // Contains information about a particular segment of a route.
    int segment_start;
    int segment_end;
    int num_custs;
    int load;
    double len;
};

RouteSegment get_segment_info(const MCGRP &mcgrp,HighSpeedNeighBorSearch &ns,const int chosen_task);


bool Preslice::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int b){
    My_Assert(b>=1 && b<=mcgrp.actual_task_num,"Wrong task");
    My_Assert(ns.solution[b]->pre->ID > 0, "Wrong task");

    move_result.reset();

    const auto seg_after_b = get_segment_info(mcgrp,ns, b);

    move_result.task1 = b;
    if(seg_after_b.num_custs != 0){
        move_result.move_arguments.push_back(b);
        move_result.move_arguments.push_back(seg_after_b.segment_end);

        double new_load_delta = seg_after_b.load + mcgrp.inst_tasks[b].demand;
        My_Assert(new_load_delta<=mcgrp.capacity,"Wrong tasks");

        const auto b_route = ns.solution[b]->route_id;
        move_result.route_id.push_back(b_route);

        const auto b_route_load = ns.routes[b_route]->load - new_load_delta;
        const auto new_route_load = new_load_delta;
        move_result.route_loads.push_back(b_route_load);
        move_result.route_loads.push_back(new_route_load);

        const int a = ns.solution[b]->pre->ID;
        const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
        const auto bc = mcgrp.min_cost[mcgrp.inst_tasks[b].tail_node][mcgrp.inst_tasks[seg_after_b.segment_start].head_node];
        const auto c_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_b.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto dummy_b = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[b].head_node];

        const double b_route_length = ns.routes[b_route]->length - ab - mcgrp.inst_tasks[b].serv_cost - bc - seg_after_b.len - c_dummy + a_dummy;
        const double new_route_length =  dummy_b +  mcgrp.inst_tasks[b].serv_cost + bc + seg_after_b.len + c_dummy;
        move_result.route_lens.push_back(b_route_length);
        move_result.route_lens.push_back(new_route_length);

        const auto delta = -ab + a_dummy + dummy_b;
        move_result.delta = delta;

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.route_custs_num.push_back(ns.routes[b_route]->num_customers - seg_after_b.num_custs - 1);
        move_result.route_custs_num.push_back(1 + seg_after_b.num_custs);


        move_result.vio_load_delta = 0;

        move_result.considerable = true;

        return true;
    }
    else{
        move_result.move_arguments.push_back(b);

        double new_load_delta =  mcgrp.inst_tasks[b].demand;
        My_Assert(new_load_delta<=mcgrp.capacity,"Wrong tasks");

        const auto b_route = ns.solution[b]->route_id;
        move_result.route_id.push_back(b_route);

        const auto b_route_load = ns.routes[b_route]->load - new_load_delta;
        const auto new_route_load = new_load_delta;
        move_result.route_loads.push_back(b_route_load);
        move_result.route_loads.push_back(new_route_load);

        const int a = ns.solution[b]->pre->ID;
        const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
        const auto a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto dummy_b = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[b].head_node];
        const auto b_dummy = mcgrp.min_cost[mcgrp.inst_tasks[b].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        const double b_route_length = ns.routes[b_route]->length - ab - mcgrp.inst_tasks[b].serv_cost - b_dummy + a_dummy;
        const double new_route_length =  dummy_b + mcgrp.inst_tasks[b].serv_cost + b_dummy;
        move_result.route_lens.push_back(b_route_length);
        move_result.route_lens.push_back(new_route_length);

        const auto delta = -ab + a_dummy + dummy_b;
        move_result.delta = delta;

        move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

        move_result.route_custs_num.push_back(ns.routes[b_route]->num_customers - 1);
        move_result.route_custs_num.push_back(1);

        move_result.vio_load_delta = 0;

        move_result.considerable = true;

        return true;
    }

}

void Preslice::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a slice : pre-slice move");
    My_Assert(move_result.considerable,"Invalid predictions");

    //Modify routes info
    const int sliced_route = move_result.route_id[0];
    const auto new_route = ns.routes.allocate_route();
    My_Assert(ns.routes.activated_route_id.find(sliced_route)!=ns.routes.activated_route_id.end(),"Invalid route");
    My_Assert(ns.routes.activated_route_id.find(new_route)!=ns.routes.activated_route_id.end(),"Invalid route");

    ns.routes[sliced_route]->length = move_result.route_lens[0];
    ns.routes[new_route]->length = move_result.route_lens[1];

    ns.routes[sliced_route]->num_customers = move_result.route_custs_num[0];
    ns.routes[new_route]->num_customers = move_result.route_custs_num[1];

    ns.routes[sliced_route]->load = move_result.route_loads[0];
    ns.routes[new_route]->load = move_result.route_loads[1];


    int new_route_start;
    int new_route_end;
    if(move_result.route_custs_num[1] == 1){
        new_route_start = move_result.move_arguments[0];
        new_route_end = move_result.move_arguments[0];
    }
    else {
        new_route_start = move_result.move_arguments[0];
        new_route_end = move_result.move_arguments[1];
    }


    My_Assert(new_route_start >= 1 && new_route_start <= mcgrp.actual_task_num,"Wrong task");
    My_Assert(new_route_end >= 1 && new_route_end <= mcgrp.actual_task_num,"Wrong task");

    ns.routes[sliced_route]->end = ns.solution[new_route_start]->pre->ID;

    ns.routes[new_route]->start = new_route_start;
    ns.routes[new_route]->end = new_route_end;

    for(auto cur = new_route_start;cur != ns.solution[new_route_end]->next->ID;cur = ns.solution[cur]->next->ID){
        ns.solution[cur]->route_id = new_route;
    }

    //handle solution
    auto new_dummy = ns.solution.dummypool.get_new_dummy();
    new_dummy->pre = ns.solution[new_route_start]->pre;
    new_dummy->next = ns.solution[new_route_start];
    ns.solution[new_route_start]->pre->next = new_dummy;
    ns.solution[new_route_start]->pre = new_dummy;


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


/*
 * Post Slice
 */


bool Postslice::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int b){
    My_Assert(b>=1 && b<=mcgrp.actual_task_num,"Wrong task");
    My_Assert(ns.solution[b]->next->ID > 0, "Wrong task");

    move_result.reset();

    const auto seg_after_b = get_segment_info(mcgrp,ns, b);

    My_Assert(seg_after_b.num_custs != 0,"Wrong tasks");

    move_result.task1 = b;

    move_result.move_arguments.push_back(seg_after_b.segment_start);
    move_result.move_arguments.push_back(seg_after_b.segment_end);

    double new_load_delta = seg_after_b.load;
    My_Assert(new_load_delta<=mcgrp.capacity,"Wrong tasks");

    const auto b_route = ns.solution[b]->route_id;
    move_result.route_id.push_back(b_route);

    const auto b_route_load = ns.routes[b_route]->load - new_load_delta;
    const auto new_route_load = new_load_delta;
    move_result.route_loads.push_back(b_route_load);
    move_result.route_loads.push_back(new_route_load);

    const int c = ns.solution[b]->next->ID;
    My_Assert(c>=1 && c<=mcgrp.actual_task_num,"Wrong task");
    const auto bc = mcgrp.min_cost[mcgrp.inst_tasks[b].tail_node][mcgrp.inst_tasks[seg_after_b.segment_start].head_node];
    const auto dummy_c = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[c].head_node];
    const auto b_dummy = mcgrp.min_cost[mcgrp.inst_tasks[b].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
    const auto d_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_b.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

    const double b_route_length = ns.routes[b_route]->length - bc - seg_after_b.len - d_dummy + b_dummy;
    const double new_route_length =  dummy_c + seg_after_b.len + d_dummy;
    move_result.route_lens.push_back(b_route_length);
    move_result.route_lens.push_back(new_route_length);

    const auto delta = -bc + b_dummy + dummy_c;
    move_result.delta = delta;

    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

    move_result.route_custs_num.push_back(ns.routes[b_route]->num_customers - seg_after_b.num_custs);
    move_result.route_custs_num.push_back(seg_after_b.num_custs);

    move_result.vio_load_delta = 0;

    move_result.considerable = true;

    return true;
}

void Postslice::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a slice : post-slice move");
    My_Assert(move_result.considerable,"Invalid predictions");

    //Modify routes info
    const int sliced_route = move_result.route_id[0];
    const auto new_route = ns.routes.allocate_route();
    My_Assert(ns.routes.activated_route_id.find(sliced_route)!=ns.routes.activated_route_id.end(),"Invalid route");
    My_Assert(ns.routes.activated_route_id.find(new_route)!=ns.routes.activated_route_id.end(),"Invalid route");

    ns.routes[sliced_route]->length = move_result.route_lens[0];
    ns.routes[new_route]->length = move_result.route_lens[1];

    ns.routes[sliced_route]->num_customers = move_result.route_custs_num[0];
    ns.routes[new_route]->num_customers = move_result.route_custs_num[1];

    ns.routes[sliced_route]->load = move_result.route_loads[0];
    ns.routes[new_route]->load = move_result.route_loads[1];

    int new_route_start;
    int new_route_end;
    new_route_start = move_result.move_arguments[0];
    new_route_end = move_result.move_arguments[1];

    My_Assert(new_route_start >= 1 && new_route_start <= mcgrp.actual_task_num,"Wrong task");
    My_Assert(new_route_end >= 1 && new_route_end <= mcgrp.actual_task_num,"Wrong task");

    ns.routes[sliced_route]->end = ns.solution[new_route_start]->pre->ID;

    ns.routes[new_route]->start = new_route_start;
    ns.routes[new_route]->end = new_route_end;

    for(auto cur = new_route_start;cur != ns.solution[new_route_end]->next->ID;cur = ns.solution[cur]->next->ID){
        ns.solution[cur]->route_id = new_route;
    }

    //handle solution
    auto new_dummy = ns.solution.dummypool.get_new_dummy();
    new_dummy->pre = ns.solution[new_route_start]->pre;
    new_dummy->next = ns.solution[new_route_start];
    ns.solution[new_route_start]->pre->next = new_dummy;
    ns.solution[new_route_start]->pre = new_dummy;


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