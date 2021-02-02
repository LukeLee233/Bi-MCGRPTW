#include "SwapEnds.h"
#include <algorithm>

using namespace std;

//MCGRPMOVE SwapEnds::move_result = MCGRPMOVE(NeighborOperator::SWAP_ENDS);
//
//bool
//SwapEnds::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, vector<int> chosen_seq, vector<int> candidate_seq)
//{
//    My_Assert(find(chosen_seq.begin(), chosen_seq.end(), DUMMY) == chosen_seq.end(),
//              "Chosen sequence can't have dummy task!");
//
//    My_Assert(find(candidate_seq.begin(), candidate_seq.end(), DUMMY) == candidate_seq.end(),
//              "Candidate sequence can't have dummy task!");
//
//    const int chosen_route = ns.route_id[chosen_seq.front()];
//    const int candidate_route = ns.route_id[candidate_seq.front()];
//
//    My_Assert(chosen_route != candidate_route, "Two sequences should be different routes!");
//
//    move_result.choose_tasks(chosen_seq.front(), candidate_seq.front());
//    move_result.move_arguments = chosen_seq;
//    move_result.move_arguments.insert(move_result.move_arguments.end(), candidate_seq.begin(), candidate_seq.end());
//
//    //check load
//    int chosen_seq_load = 0;
//    for (auto task : chosen_seq) {
//        chosen_seq_load += mcgrp.inst_tasks[task].demand;
//    }
//    int candidate_seq_load = 0;
//    for (auto task : candidate_seq) {
//        candidate_seq_load += mcgrp.inst_tasks[task].demand;
//    }
//
//    if (ns.routes[chosen_route].load - chosen_seq_load + candidate_seq_load > mcgrp.capacity ||
//        ns.routes[candidate_route].load - candidate_seq_load + chosen_seq_load > mcgrp.capacity) {
//        move_result.reset();
//        return false;
//    }
//
//    vector<int> potential_chosen_seq;
//    potential_chosen_seq.reserve(chosen_seq.size());
//
//    vector<int> potential_candidate_seq;
//    potential_chosen_seq.reserve(candidate_seq.size());
//
//    for (auto cursor = 0; cursor < chosen_seq.size(); cursor++) {
//        //Base on probability
//        if (mcgrp.is_edge(chosen_seq[cursor]) && mcgrp._rng.Randfloat(0, 1) > 0.5) {
//            potential_chosen_seq.push_back(mcgrp.inst_tasks[chosen_seq[cursor]].inverse);
//        }
//        else {
//            potential_chosen_seq.push_back(chosen_seq[cursor]);
//        }
//    }
//
//    for (auto cursor = 0; cursor < candidate_seq.size(); cursor++) {
//        //Base on probability
//        if (mcgrp.is_edge(candidate_seq[cursor]) && mcgrp._rng.Randfloat(0, 1) > 0.5) {
//            potential_candidate_seq.push_back(mcgrp.inst_tasks[candidate_seq[cursor]].inverse);
//        }
//        else {
//            potential_candidate_seq.push_back(candidate_seq[cursor]);
//        }
//    }
//
//    const int pre_chosen_start = max(ns.pred_array[chosen_seq.front()], DUMMY);
//    const int post_chosen_end = max(ns.next_array[chosen_seq.back()], DUMMY);
//    const int pre_candidate_start = max(ns.pred_array[candidate_seq.front()], DUMMY);
//    const int post_candidate_end = max(ns.next_array[candidate_seq.back()], DUMMY);
//
//    double chosen_delta = 0;
//    double candidate_delta = 0;
//
//    vector<int> actual_chosen_seq;
//    actual_chosen_seq.reserve(chosen_seq.size());
//    vector<int> actual_candidate_seq;
//    actual_candidate_seq.reserve(candidate_seq.size());
//
//    if (chosen_seq == potential_chosen_seq && candidate_seq == potential_candidate_seq) {
//        //no need to do compare
//        chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[chosen_seq.front()]
//                .head_node];
//
//        for (auto cursor = 0; cursor < chosen_seq.size() - 1; cursor++) {
//            chosen_delta -= mcgrp.inst_tasks[chosen_seq[cursor]].serv_cost;
//            chosen_delta -=
//                mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq[cursor]].tail_node][mcgrp.inst_tasks[chosen_seq[cursor + 1]]
//                    .head_node];
//        }
//        chosen_delta -= mcgrp.inst_tasks[chosen_seq.back()].serv_cost;
//        chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq.back()].tail_node][mcgrp.inst_tasks[post_chosen_end].head_node];
//
//        chosen_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[candidate_seq.back()]
//                .head_node];
//        for (auto cursor = candidate_seq.size() - 1; cursor > 0; --cursor) {
//            chosen_delta += mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
//            chosen_delta +=
//                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp.inst_tasks[candidate_seq[cursor
//                    - 1]].head_node];
//        }
//        chosen_delta += mcgrp.inst_tasks[candidate_seq.front()].serv_cost;
//        chosen_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.front()].tail_node][mcgrp.inst_tasks[post_chosen_end]
//                .head_node];
//
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        candidate_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[candidate_seq.front()]
//                .head_node];
//
//        for (auto cursor = 0; cursor < candidate_seq.size() - 1; cursor++) {
//            candidate_delta -= mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
//            candidate_delta -=
//                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp.inst_tasks[candidate_seq[cursor
//                    + 1]].head_node];
//        }
//        candidate_delta -= mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
//        candidate_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[post_candidate_end]
//                .head_node];
//
//        candidate_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[chosen_seq.back()]
//                .head_node];
//        for (auto cursor = chosen_seq.size() - 1; cursor > 0; --cursor) {
//            candidate_delta += mcgrp.inst_tasks[chosen_seq[cursor]].serv_cost;
//            candidate_delta +=
//                mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq[cursor]].tail_node][mcgrp.inst_tasks[chosen_seq[cursor - 1]]
//                    .head_node];
//        }
//        candidate_delta += mcgrp.inst_tasks[chosen_seq.front()].serv_cost;
//        candidate_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq.front()].tail_node][mcgrp.inst_tasks[post_candidate_end]
//                .head_node];
//
//        actual_chosen_seq = chosen_seq;
//        actual_candidate_seq = candidate_seq;
//    }
//    else {
//        //chose the one will lead to less cost
//        //connect original candidate seq
//        double original_chosen_delta = 0;
//        original_chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[chosen_seq.front()]
//                .head_node];
//
//        for (auto cursor = 0; cursor < chosen_seq.size() - 1; cursor++) {
//            original_chosen_delta -= mcgrp.inst_tasks[chosen_seq[cursor]].serv_cost;
//            original_chosen_delta -=
//                mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq[cursor]].tail_node][mcgrp.inst_tasks[chosen_seq[cursor + 1]]
//                    .head_node];
//        }
//        original_chosen_delta -= mcgrp.inst_tasks[chosen_seq.back()].serv_cost;
//        original_chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq.back()].tail_node][mcgrp.inst_tasks[post_chosen_end].head_node];
//
//        original_chosen_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[candidate_seq.back()]
//                .head_node];
//        for (auto cursor = candidate_seq.size() - 1; cursor > 0; --cursor) {
//            original_chosen_delta += mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
//            original_chosen_delta +=
//                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp.inst_tasks[candidate_seq[cursor
//                    - 1]].head_node];
//        }
//        original_chosen_delta += mcgrp.inst_tasks[candidate_seq.front()].serv_cost;
//        original_chosen_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.front()].tail_node][mcgrp.inst_tasks[post_chosen_end]
//                .head_node];
//
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //connect original chosen seq
//        double original_candidate_delta = 0;
//
//        original_candidate_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[candidate_seq.front()]
//                .head_node];
//
//        for (auto cursor = 0; cursor < candidate_seq.size() - 1; cursor++) {
//            original_candidate_delta -= mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
//            original_candidate_delta -=
//                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp.inst_tasks[candidate_seq[cursor
//                    + 1]].head_node];
//        }
//        original_candidate_delta -= mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
//        original_candidate_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp.inst_tasks[post_candidate_end]
//                .head_node];
//
//        original_candidate_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[chosen_seq.back()]
//                .head_node];
//        for (auto cursor = chosen_seq.size() - 1; cursor > 0; --cursor) {
//            original_candidate_delta += mcgrp.inst_tasks[chosen_seq[cursor]].serv_cost;
//            original_candidate_delta +=
//                mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq[cursor]].tail_node][mcgrp.inst_tasks[chosen_seq[cursor - 1]]
//                    .head_node];
//        }
//        original_candidate_delta += mcgrp.inst_tasks[chosen_seq.front()].serv_cost;
//        original_candidate_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq.front()].tail_node][mcgrp.inst_tasks[post_candidate_end]
//                .head_node];
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //connect potential candidate seq
//        double potential_chosen_delta = 0;
//        potential_chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[chosen_seq.front()]
//                .head_node];
//
//        for (auto cursor = 0; cursor < chosen_seq.size() - 1; cursor++) {
//            potential_chosen_delta -= mcgrp.inst_tasks[chosen_seq[cursor]].serv_cost;
//            potential_chosen_delta -= mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq[cursor]].tail_node][mcgrp
//                .inst_tasks[chosen_seq[cursor + 1]].head_node];
//        }
//        potential_chosen_delta -= mcgrp.inst_tasks[chosen_seq.back()].serv_cost;
//        potential_chosen_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[chosen_seq.back()].tail_node][mcgrp.inst_tasks[post_chosen_end]
//                .head_node];
//
//        potential_chosen_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_chosen_start].tail_node][mcgrp.inst_tasks[potential_candidate_seq
//                .back()].head_node];
//        for (auto cursor = potential_candidate_seq.size() - 1; cursor > 0; --cursor) {
//            potential_chosen_delta += mcgrp.inst_tasks[potential_candidate_seq[cursor]].serv_cost;
//            potential_chosen_delta += mcgrp.min_cost[mcgrp.inst_tasks[potential_candidate_seq[cursor]].tail_node][mcgrp
//                .inst_tasks[potential_candidate_seq[cursor - 1]].head_node];
//        }
//        potential_chosen_delta += mcgrp.inst_tasks[potential_candidate_seq.front()].serv_cost;
//        potential_chosen_delta += mcgrp.min_cost[mcgrp.inst_tasks[potential_candidate_seq.front()].tail_node][mcgrp
//            .inst_tasks[post_chosen_end].head_node];
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        //connect potential chosen seq
//        double potential_candidate_delta = 0;
//
//        potential_candidate_delta -=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[candidate_seq
//                .front()].head_node];
//
//        for (auto cursor = 0; cursor < candidate_seq.size() - 1; cursor++) {
//            potential_candidate_delta -= mcgrp.inst_tasks[candidate_seq[cursor]].serv_cost;
//            potential_candidate_delta -=
//                mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq[cursor]].tail_node][mcgrp
//                    .inst_tasks[candidate_seq[cursor + 1]].head_node];
//        }
//        potential_candidate_delta -= mcgrp.inst_tasks[candidate_seq.back()].serv_cost;
//        potential_candidate_delta -= mcgrp.min_cost[mcgrp.inst_tasks[candidate_seq.back()].tail_node][mcgrp
//            .inst_tasks[post_candidate_end].head_node];
//
//        potential_candidate_delta +=
//            mcgrp.min_cost[mcgrp.inst_tasks[pre_candidate_start].tail_node][mcgrp.inst_tasks[potential_chosen_seq
//                .back()].head_node];
//        for (auto cursor = potential_chosen_seq.size() - 1; cursor > 0; --cursor) {
//            potential_candidate_delta += mcgrp.inst_tasks[potential_chosen_seq[cursor]].serv_cost;
//            potential_candidate_delta += mcgrp.min_cost[mcgrp.inst_tasks[potential_chosen_seq[cursor]].tail_node][mcgrp
//                .inst_tasks[potential_chosen_seq[cursor - 1]].head_node];
//        }
//        potential_candidate_delta += mcgrp.inst_tasks[potential_chosen_seq.front()].serv_cost;
//        potential_candidate_delta += mcgrp.min_cost[mcgrp.inst_tasks[potential_chosen_seq.front()].tail_node][mcgrp
//            .inst_tasks[post_candidate_end].head_node];
//
//        ////////////////////////////////////////////////////////////////////////////////////////////////
//        if(original_chosen_delta <= potential_chosen_delta){
//            chosen_delta = original_chosen_delta;
//            actual_candidate_seq = candidate_seq;
//        }
//        else{
//            chosen_delta = potential_chosen_delta;
//            actual_candidate_seq = potential_candidate_seq;
//        }
//
//        if(original_candidate_delta <= potential_candidate_delta){
//            candidate_delta = original_candidate_delta;
//            actual_chosen_seq = chosen_seq;
//        }
//        else{
//            candidate_delta = potential_candidate_delta;
//            actual_chosen_seq = potential_chosen_seq;
//        }
//    }
//
//    move_result.num_affected_routes = 2;
//
//    move_result.route_id.push_back(chosen_route);
//    move_result.route_id.push_back(candidate_route);
//
//    move_result.delta = chosen_delta + candidate_delta;
//
//    move_result.seq1_cus_num = chosen_seq.size();
//    move_result.seq2_cus_num = candidate_seq.size();
//
//    move_result.route_lens.push_back(ns.routes[chosen_route].length + chosen_delta);
//    move_result.route_lens.push_back(ns.routes[candidate_route].length + candidate_delta);
//
//
//    move_result.route_loads.push_back(ns.routes[chosen_route].load - chosen_seq_load + candidate_seq_load);
//    move_result.route_loads.push_back(ns.routes[candidate_route].load - candidate_seq_load + chosen_seq_load);
//
//
//    move_result.route_custs_num
//        .push_back(ns.routes[chosen_route].num_customers - actual_chosen_seq.size() + actual_candidate_seq.size());
//    move_result.route_custs_num
//        .push_back(ns.routes[candidate_route].num_customers - actual_candidate_seq.size() + actual_chosen_seq.size());
//
//
//    reverse(actual_chosen_seq.begin(), actual_chosen_seq.end());
//    move_result.move_arguments
//        .insert(move_result.move_arguments.end(), actual_chosen_seq.begin(), actual_chosen_seq.end());
//    reverse(actual_candidate_seq.begin(), actual_candidate_seq.end());
//    move_result.move_arguments
//        .insert(move_result.move_arguments.end(), actual_candidate_seq.begin(), actual_candidate_seq.end());
//
//
//    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;
//    move_result.considerable = true;
//
//    return true;
//}
//
//void SwapEnds::move(NeighBorSearch &ns, const MCGRP &mcgrp)
//{
//    My_Assert(move_result.move_arguments.size() == (move_result.seq1_cus_num + move_result.seq2_cus_num) * 2,
//              "move arguments number is not correct!");
//
//    //extract move arguments
//    vector<int> original_chosen_seq(move_result.move_arguments.begin(),
//                                    move_result.move_arguments.begin() + move_result.seq1_cus_num);
//
//    vector<int> original_candidate_seq(move_result.move_arguments.begin() + move_result.seq1_cus_num,
//                                       move_result.move_arguments.begin() + move_result.seq1_cus_num
//                                           + move_result.seq2_cus_num);
//
//    vector<int> actual_chosen_seq(move_result.move_arguments.begin() + move_result.seq1_cus_num + move_result.seq2_cus_num,
//                                  move_result.move_arguments.begin() + 2 * move_result.seq1_cus_num + move_result.seq2_cus_num);
//
//    vector<int>
//        actual_candidate_seq(move_result.move_arguments.begin() + 2 * move_result.seq1_cus_num + move_result.seq2_cus_num,
//                             move_result.move_arguments.end());
//
//    My_Assert(original_chosen_seq.size() == actual_chosen_seq.size()
//                  && original_candidate_seq.size() == actual_candidate_seq.size(),
//              "move sequence is not consistent!");
//
//    const int chosen_start_task = original_chosen_seq.front();
//    const int chosen_end_task = original_chosen_seq.back();
//    const int candidate_start_task = original_candidate_seq.front();
//    const int candidate_end_task = original_candidate_seq.back();
//
//    Individual individual;
//    ns.create_individual(mcgrp, individual);
//
//    My_Assert(ns.delimiter_coding_sol == individual.sequence,
//              "Inconsistency between negative coding and delimiter coding.");
//
//    const auto chosen_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), chosen_start_task);
//    const auto chosen_ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), chosen_end_task);
//
//    My_Assert(chosen_ite_start != individual.sequence.end() && chosen_ite_end != individual.sequence.end(),
//              "Can't find corresponding chosen tasks in sequence!");
//
//    const auto original_candidate_ite_start =
//        std::find(individual.sequence.begin(), individual.sequence.end(), candidate_start_task);
//    const auto original_candidate_ite_end =
//        std::find(individual.sequence.begin(), individual.sequence.end(), candidate_end_task);
//
//    My_Assert(original_candidate_ite_start != individual.sequence.end()
//                  && original_candidate_ite_end != individual.sequence.end(),
//              "Can't find corresponding candidate tasks in sequence!");
//
//
//    individual.sequence.erase(chosen_ite_start, chosen_ite_end + 1);
//    individual.sequence.insert(chosen_ite_start, actual_candidate_seq.begin(), actual_candidate_seq.end());
//
//    //The location info has been updated
//    auto updated_candidate_ite_start = individual.sequence.end();
//    auto updated_candidate_ite_end = individual.sequence.end();
//
//    //rescale the searching range
//    if (original_candidate_ite_start < chosen_ite_start) {
//        updated_candidate_ite_start = std::find(individual.sequence.begin(), chosen_ite_start, candidate_start_task);
//        updated_candidate_ite_end = updated_candidate_ite_start + original_candidate_seq.size() - 1;
//    }
//    else if (original_candidate_ite_start > chosen_ite_start) {
//        updated_candidate_ite_start =
//            std::find(chosen_ite_start + actual_candidate_seq.size(), individual.sequence.end(), candidate_start_task);
//        updated_candidate_ite_end = updated_candidate_ite_start + original_candidate_seq.size() - 1;
//    }
//    else {
//        cerr << "can't handle two same sequence!/n";
//        abort();
//    }
//
//    My_Assert(updated_candidate_ite_start != individual.sequence.end()
//                  && updated_candidate_ite_end != individual.sequence.end(),
//              "Can't find corresponding candidate tasks in sequence!");
//
//    individual.sequence.erase(updated_candidate_ite_start, updated_candidate_ite_end + 1);
//    individual.sequence.insert(updated_candidate_ite_start, actual_chosen_seq.begin(), actual_chosen_seq.end());
//
//
//    auto prior_cost = ns.cur_solution_cost;
//
//    //Update neighbor search info
//    ns.unpack_seq(individual.sequence, mcgrp);
////    ns.create_individual(mcgrp, ns.ns_indi);
//    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);
//
//    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");
//
//
//    if(ns.cur_solution_cost == prior_cost){
//        ns.equal_step++;
//    }
//
////    if(ns.cur_solution_cost < ns.policy.cost_benchmark){
////        ns.policy.cost_benchmark = ns.cur_solution_cost;
////    }
//
//    move_result.reset();
//    move_result.move_type = NeighborOperator::SWAP_ENDS;
//
//    ns.search_step++;
//}


struct RouteSegment
{
    // Contains information about a particular segment of a route.
    int segment_start;
    int segment_end;
    int num_custs;
    int load;
    double len;
};

RouteSegment get_segment_info(const MCGRP &mcgrp,const NeighBorSearch &ns,const int a,const int b){
    /// Calculates the length, load, and # of customers on the segment
    /// of the route between nodes a and b.  Assumes that a is before b
    /// in the route - this is not checked! closed interval
    /// Example:  a-i-j-b has
    /// length: d(a,i)+d(i,j)+d(j,b)
    /// load:   a + i + j + b
    /// #:      4

    My_Assert(a != DUMMY || b != DUMMY,"can't parse two dummy tasks");

    RouteSegment buffer;
    if (a == b)
    {
        // Degenerate case...
        buffer.segment_start = b;
        buffer.segment_end = a;
        buffer.len = mcgrp.inst_tasks[a].serv_cost;
        buffer.load = mcgrp.inst_tasks[a].demand;
        buffer.num_custs = 1;
        return buffer;
    }

    buffer.len = 0;
    buffer.segment_start = a;
    buffer.segment_end = b;
    buffer.num_custs = 0;
    buffer.load = 0;

    // Special cases
    if (a == DUMMY)
    {
        //we want the segment dummy-i-j-...-k-b-...
        My_Assert(b != DUMMY,"Two dummy!");

        buffer.segment_start = ns.routes[ns.route_id[b]].start;
        buffer.segment_end = b;
        buffer.len += (mcgrp.inst_tasks[DUMMY].serv_cost +
            mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[buffer.segment_start].head_node]);
    }
    else if (b == DUMMY)
    {
        // we want the segment ...a-i-j-...-k-dummy
        My_Assert(a != DUMMY,"Two dummy!");

        buffer.segment_start = a;
        buffer.segment_end = ns.routes[ns.route_id[a]].end;
        buffer.len += (mcgrp.min_cost[mcgrp.inst_tasks[buffer.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node]
                    + mcgrp.inst_tasks[DUMMY].serv_cost);
    }

    // Now calculate the length, load, and # customer on this segment
    int current_task = buffer.segment_start;

    while (current_task != buffer.segment_end)
    {
        int next_task = max(ns.next_array[current_task], DUMMY);
        buffer.len += (mcgrp.inst_tasks[current_task].serv_cost+
            mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[next_task].head_node]);

        buffer.load += mcgrp.inst_tasks[current_task].demand;

        if (current_task != mcgrp.sentinel)
            buffer.num_custs += 1;

        current_task = next_task;
    }

    //handle last task in the segment
    buffer.len += mcgrp.inst_tasks[current_task].serv_cost;
    buffer.load += mcgrp.inst_tasks[current_task].demand;
    if (current_task != mcgrp.sentinel)
        buffer.num_custs += 1;

    return buffer;
}

bool NewSwapEnds::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, const int chosen_task, const int neighbor_task){
    /// Example: ( a & v input): VRP_DEPOT-i-a-b-j-k-l-VRP_DEPOT and VRP_DEPOT-t-u-v-w-x-y-z-VRP_DEPOT becomes
    /// VRP_DEPOT-i-a-w-x-y-z-VRP_DEPOT and VRP_DEPOT-t-u-v-b-j-k-l-VRP_DEPOT

    const int a = chosen_task;
    const int v = neighbor_task;
    const int a_route = ns.route_id[a];
    const int v_route = ns.route_id[v];

    My_Assert(a != DUMMY && v !=DUMMY,"Swap ends called with depot; Move doesn't make sense");
    My_Assert(a_route != v_route,"swap ends called with a and v in same route!");

    const int w = max(ns.next_array[v], DUMMY);
    const int b = max(ns.next_array[a], DUMMY);

    const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
    const auto vw = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[w].head_node];
    const auto aw = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[w].head_node];
    const auto vb = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[b].head_node];

    const double delta = - ab - vw + aw + vb;

    RouteSegment seg_a = get_segment_info(mcgrp,ns, DUMMY, a);
    RouteSegment seg_v = get_segment_info(mcgrp,ns, DUMMY, v);

    const auto a_load_delta = -(ns.routes[a_route].load - seg_a.load) + (ns.routes[v_route].load - seg_v.load);
    const auto v_load_delta = -(ns.routes[v_route].load - seg_v.load) + (ns.routes[a_route].load - seg_a.load);

    //handle load constraint
    double vio_load_delta = 0;
    if (ns.policy.has_rule(DELTA_ONLY)) {
        if (ns.routes[a_route].load + a_load_delta > mcgrp.capacity
        || ns.routes[v_route].load + v_load_delta > mcgrp.capacity) {
            move_result.reset();
            return false;
        }
    }
    else if (ns.policy.has_rule(FITNESS_ONLY)) {

        //a_route vio-load calculate
        if (ns.routes[a_route].load + a_load_delta > mcgrp.capacity) {
            //if insert task to route a and over load
            if (ns.routes[a_route].load >= mcgrp.capacity) {
                //if the route a already over loaded
                vio_load_delta += a_load_delta;
            }
            else {
                vio_load_delta += ns.routes[a_route].load + a_load_delta - mcgrp.capacity;
            }
        }

        //v_route vio-load calculate
        if (ns.routes[v_route].load + v_load_delta > mcgrp.capacity) {
            //if insert task to route v and over load
            if (ns.routes[v_route].load >= mcgrp.capacity) {
                //if the route v already over loaded
                vio_load_delta += v_load_delta;
            }
            else {
                vio_load_delta += ns.routes[v_route].load + v_load_delta - mcgrp.capacity;
            }
        }
    }

    move_result.reset();
    move_result.num_affected_routes = 2;
    move_result.route_id.push_back(a_route);
    move_result.route_id.push_back(v_route);
    move_result.delta = delta;

    auto new_a_len = seg_a.len + aw + (ns.routes[v_route].length - seg_v.len - vw);
    auto new_v_len = seg_v.len + vb + (ns.routes[a_route].length - seg_a.len - ab);
    move_result.route_lens.push_back(new_a_len);
    move_result.route_lens.push_back(new_v_len);

    auto new_a_load = ns.routes[a].load + a_load_delta;
    auto new_v_load = ns.routes[v].load + v_load_delta;
    move_result.route_loads.push_back(new_a_load);
    move_result.route_loads.push_back(new_v_load);

    auto new_a_custs_num = seg_a.num_custs + (ns.routes[v_route].num_customers - seg_v.num_custs);
    auto new_v_custs_num = seg_v.num_custs + (ns.routes[a_route].num_customers - seg_a.num_custs);
    move_result.route_custs_num.push_back(new_a_custs_num);
    move_result.route_custs_num.push_back(new_v_custs_num);

    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

    move_result.total_number_of_routes = ns.routes.size();


    vector<int> a_seq;
    vector<int> v_seq;

    int cur_task = ns.next_array[a];
    a_seq.push_back(a);
    while(cur_task > DUMMY){
        if(cur_task != mcgrp.sentinel){
            a_seq.push_back(cur_task);
        }
        cur_task = ns.next_array[cur_task];
    }

    cur_task = ns.next_array[v];
    v_seq.push_back(v);
    while(cur_task > DUMMY){
        if(cur_task != mcgrp.sentinel) {
            v_seq.push_back(cur_task);
        }
        cur_task = ns.next_array[cur_task];
    }

    move_result.move_arguments = a_seq;
    move_result.move_arguments.insert(move_result.move_arguments.end(),v_seq.begin(),v_seq.end());

    move_result.task1 = a;
    move_result.task2 = v;

    move_result.vio_load_delta = vio_load_delta;

    move_result.considerable = true;

    return true;
}

void NewSwapEnds::move(NeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt:swap-ends move");

    My_Assert(move_result.considerable,"Invalid predictions");

    const int a = move_result.task1;
    const int v = move_result.task2;

    My_Assert(a != DUMMY && v != DUMMY,"cannot handle dummy tasks!");
    My_Assert(ns.route_id[a] != ns.route_id[v],"cannot handle same route");

    //Extract the seq which needs swapped
    vector<int> a_seq;
    vector<int> v_seq;

    My_Assert(a == move_result.move_arguments.front(),"Incorrect arguments");
    int i;
    for(i = 1; move_result.move_arguments[i] != v;i++){
        a_seq.push_back(move_result.move_arguments[i]);
    }

    My_Assert(v == move_result.move_arguments[i],"Incorrect arguments");
    i++;
    for(;i<move_result.move_arguments.size();i++){
        v_seq.push_back(move_result.move_arguments[i]);
    }

    Individual individual;
    ns.create_individual(mcgrp, individual);

    My_Assert(ns.delimiter_coding_sol == individual.sequence,
              "Inconsistency between negative coding and delimiter coding.");

    My_Assert(!ns.check_missed(mcgrp),"Some task has been missed!");


    if(a_seq.size() > 0 && v_seq.size() > 0){
        vector<int> first_seg;
        vector<int> middle_seg;
        vector<int> last_seg;

        const auto a_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), a_seq.front());
        const auto a_ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), a_seq.back());

        My_Assert(a_ite_start != individual.sequence.end() && a_ite_end != individual.sequence.end(),
                  "Can't find corresponding chosen tasks in sequence!");

        auto v_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), v_seq.front());
        auto v_ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), v_seq.back());

        My_Assert(v_ite_start != individual.sequence.end()
                      && v_ite_end != individual.sequence.end(),
                  "Can't find corresponding candidate tasks in sequence!");

        My_Assert(v_ite_start != a_ite_start,"can't handle two same sequence!");
        if (v_ite_start < a_ite_start) {
            first_seg = vector<int>(individual.sequence.begin(),v_ite_start);
            middle_seg = vector<int>(v_ite_end+1,a_ite_start);
            last_seg = vector<int>(a_ite_end + 1,individual.sequence.end());

            vector<int> buffer;
            buffer = first_seg;
            buffer.insert(buffer.end(),a_seq.begin(),a_seq.end());
            buffer.insert(buffer.end(),middle_seg.begin(),middle_seg.end());
            buffer.insert(buffer.end(),v_seq.begin(),v_seq.end());
            buffer.insert(buffer.end(),last_seg.begin(),last_seg.end());

            individual.sequence = buffer;
        }
        else {
            first_seg = vector<int>(individual.sequence.begin(),a_ite_start);
            middle_seg = vector<int>(a_ite_end+1,v_ite_start);
            last_seg = vector<int>(v_ite_end + 1,individual.sequence.end());

            vector<int> buffer;
            buffer = first_seg;
            buffer.insert(buffer.end(),v_seq.begin(),v_seq.end());
            buffer.insert(buffer.end(),middle_seg.begin(),middle_seg.end());
            buffer.insert(buffer.end(),a_seq.begin(),a_seq.end());
            buffer.insert(buffer.end(),last_seg.begin(),last_seg.end());

            individual.sequence = buffer;
        }

    }
    else if (a_seq.size() == 0 && v_seq.size() > 0){
        DEBUG_PRINT("a sequence is empty");
        My_Assert(a != mcgrp.sentinel,"Invalid arguments!");

        //task a doesn't have subsequence, just need to concatenate v's sequence after a
        const auto v_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), v_seq.front());
        const auto v_ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), v_seq.back());

        My_Assert(v_ite_start != individual.sequence.end()
                      && v_ite_end != individual.sequence.end(),
                  "Can't find corresponding candidate tasks in sequence!");

        individual.sequence.erase(v_ite_start, v_ite_end + 1);

        const auto a_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), a);

        My_Assert(a_ite_start != individual.sequence.end(),
                  "Can't find corresponding chosen task in sequence!");

        individual.sequence.insert(a_ite_start+1, v_seq.begin(), v_seq.end());
    }
    else if (a_seq.size() > 0 && v_seq.size() == 0){
        DEBUG_PRINT("v sequence is empty");
        My_Assert(v != mcgrp.sentinel,"Invalid arguments!");

        //task v doesn't have subsequence, just need to concatenate a's sequence after v
        const auto a_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), a_seq.front());
        const auto a_ite_end = std::find(individual.sequence.begin(), individual.sequence.end(), a_seq.back());

        My_Assert(a_ite_start != individual.sequence.end()
                      && a_ite_end != individual.sequence.end(),
                  "Can't find corresponding candidate tasks in sequence!");

        individual.sequence.erase(a_ite_start, a_ite_end + 1);

        const auto v_ite_start = std::find(individual.sequence.begin(), individual.sequence.end(), v);

        My_Assert(v_ite_start != individual.sequence.end(),
                  "Can't find corresponding chosen task in sequence!");

        individual.sequence.insert(v_ite_start+1, a_seq.begin(), a_seq.end());
    }
    else{
        My_Assert(false,"Both sequences are empty this make no sense!");
    }

    auto prior_cost = ns.cur_solution_cost;

    //Update neighbor search info
    ns.unpack_seq(individual.sequence, mcgrp);
    ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

    My_Assert(prior_cost + move_result.delta == ns.cur_solution_cost,"Wrong prediction!");

    if(ns.policy.has_rule(DELTA_ONLY)){
        My_Assert(ns.total_vio_load == 0,"You can't generate an infeasible solution in the feasible search!");
    }


    if(ns.cur_solution_cost == prior_cost){
        ns.equal_step++;
    }

    My_Assert(!ns.check_missed(mcgrp),"Some task has been missed!");


    move_result.reset();
    move_result.move_type = NeighborOperator::SWAP_ENDS;

    ns.search_step++;
}

/*
 * High Speed
 */

RouteSegment get_segment_info(const MCGRP &mcgrp,HighSpeedNeighBorSearch &ns,const int chosen_task){
    /// Calculates the length, load, and # of customers of the segment
    //  after chosen task.
    /// Example:  chosen_task-a-i-j-b-dummy has
    /// length: d(a,i)+d(i,j)+d(j,b)
    /// load:   a + i + j + b
    /// #:      4


    RouteSegment buffer;

    buffer.segment_start = ns.solution[chosen_task]->next->ID;

    if(buffer.segment_start < 0){
        buffer.segment_start = 0;
        buffer.segment_end = 0;
        buffer.num_custs = 0;
        buffer.load = 0;
        buffer.len = 0;
        return buffer;
    }

    const int route = ns.solution[buffer.segment_start]->route_id;
    buffer.segment_end = ns.routes[route]->end;
    buffer.num_custs = 0;
    buffer.load = 0;

    buffer.len = 0;

    // Now calculate the length, load, and # customer on this segment
    int current_task = buffer.segment_start;

    while (current_task != buffer.segment_end)
    {
        int next_task = ns.solution[current_task]->next->ID;
        buffer.len += (mcgrp.inst_tasks[current_task].serv_cost +
            mcgrp.min_cost[mcgrp.inst_tasks[current_task].tail_node][mcgrp.inst_tasks[next_task].head_node]);

        buffer.load += mcgrp.inst_tasks[current_task].demand;

        buffer.num_custs += 1;

        current_task = next_task;
    }

    //handle last task in the segment
    buffer.len += mcgrp.inst_tasks[current_task].serv_cost;
    buffer.load += mcgrp.inst_tasks[current_task].demand;
    buffer.num_custs += 1;

    return buffer;
}


bool NewSwapEnds::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int chosen_task, const int neighbor_task,const int chosen_route,const int neighbor_route){
    /// Example: ( a & v input): VRP_DEPOT-i-a-b-j-k-l-VRP_DEPOT and VRP_DEPOT-t-u-v-w-x-y-z-VRP_DEPOT becomes
    /// VRP_DEPOT-i-a-w-x-y-z-VRP_DEPOT and VRP_DEPOT-t-u-v-b-j-k-l-VRP_DEPOT

    move_result.reset();

    const int a_route = chosen_route;
    const int v_route = neighbor_route;

    My_Assert(a_route != v_route,"swap ends called with a and v in same route!");

    //get sequence that need to be swap
    RouteSegment seg_after_a = get_segment_info(mcgrp,ns,chosen_task);
    RouteSegment seg_after_v = get_segment_info(mcgrp,ns, neighbor_task);


    const auto a_load_delta = -seg_after_a.load + seg_after_v.load;
    const auto v_load_delta = -seg_after_v.load + seg_after_a.load;

    //handle load constraint
    double vio_load_delta = 0;
    if (ns.policy.has_rule(DELTA_ONLY)) {
        if (ns.routes[a_route]->load + a_load_delta > mcgrp.capacity
            || ns.routes[v_route]->load + v_load_delta > mcgrp.capacity) {
            return false;
        }
    }
    else if (ns.policy.has_rule(FITNESS_ONLY)) {
        My_Assert(v_load_delta == -a_load_delta,"Wrong arguments");

        if(a_load_delta >= 0){
            //a_route vio-load calculate
            if (ns.routes[a_route]->load + a_load_delta > mcgrp.capacity) {
                //if insert task to route a and over load
                if (ns.routes[a_route]->load >= mcgrp.capacity) {
                    //if the route a already over loaded
                    vio_load_delta += a_load_delta;
                }
                else {
                    vio_load_delta += ns.routes[a_route]->load + a_load_delta - mcgrp.capacity;
                }
            }

            My_Assert(v_load_delta <= 0,"Wrong arguments!");
            if (ns.routes[v_route]->load > mcgrp.capacity) {
                //if insert task to route a and over load
                if (ns.routes[v_route]->load + v_load_delta >= mcgrp.capacity) {
                    //if the route v still over loaded
                    vio_load_delta += v_load_delta;
                }
                else {
                    vio_load_delta -= (ns.routes[v_route]->load - mcgrp.capacity);
                }
            }
        }
        else{
            if (ns.routes[a_route]->load > mcgrp.capacity) {
                //if insert task to route a and over load
                if (ns.routes[a_route]->load + a_load_delta >= mcgrp.capacity) {
                    //if the route v still over loaded
                    vio_load_delta += a_load_delta;
                }
                else {
                    vio_load_delta -= (ns.routes[a_route]->load - mcgrp.capacity);
                }
            }

            My_Assert(v_load_delta >= 0,"Wrong arguments!");
            //v_route vio-load calculate
            if (ns.routes[v_route]->load + v_load_delta > mcgrp.capacity) {
                //if insert task to route v and over load
                if (ns.routes[v_route]->load >= mcgrp.capacity) {
                    //if the route v already over loaded
                    vio_load_delta += v_load_delta;
                }
                else {
                    vio_load_delta += ns.routes[v_route]->load + v_load_delta - mcgrp.capacity;
                }
            }
        }

    }


    double delta;

    //a can be dummy and b can be dummy too
    const int a = max(chosen_task,0);
    const int v = max(neighbor_task,0);

    if(seg_after_a.num_custs == 0 && seg_after_v.num_custs == 0){
        //means no actual move happens
        //...-a-[]-0
        //...-v-[]-0
        return false;
    }
    else if (seg_after_a.num_custs == 0){
        //a segment is empty
        //0-a-[]-0
        //0-v-[w]-0
        My_Assert(v_route == ns.solution[seg_after_v.segment_start]->route_id,"Wrong routes");
        const int w = max(ns.solution[neighbor_task]->next->ID, 0);

        const auto vw = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[w].head_node];
        const auto aw = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[w].head_node];

        const auto a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto v_dummy = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        delta = - a_dummy + aw - vw  + v_dummy;

        const auto seg_after_v_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_v.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        auto new_a_len = ns.routes[a_route]->length - a_dummy + aw + seg_after_v.len + seg_after_v_dummy;
        auto new_v_len = ns.routes[v_route]->length - vw - seg_after_v.len - seg_after_v_dummy + v_dummy;
        move_result.route_lens.push_back(new_a_len);
        move_result.route_lens.push_back(new_v_len);

        move_result.seq1_cus_num = seg_after_a.num_custs;
        move_result.seq2_cus_num = seg_after_v.num_custs;
    }
    else if (seg_after_v.num_custs == 0){
        //v segment is empty
        //0-a-[b]-0
        //0-v-[]-0
        My_Assert(a_route == ns.solution[seg_after_a.segment_start]->route_id,"Wrong routes");

        const int b = max(ns.solution[chosen_task]->next->ID, 0);

        const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
        const auto vb = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[b].head_node];

        const auto a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto v_dummy = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        delta = - v_dummy + vb - ab + a_dummy;

        const auto seg_after_a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_a.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        auto new_a_len = ns.routes[a_route]->length - ab - seg_after_a.len - seg_after_a_dummy + a_dummy;
        auto new_v_len = ns.routes[v_route]->length - v_dummy + vb + seg_after_a.len + seg_after_a_dummy;
        move_result.route_lens.push_back(new_a_len);
        move_result.route_lens.push_back(new_v_len);

        move_result.seq1_cus_num = seg_after_a.num_custs;
        move_result.seq2_cus_num = seg_after_v.num_custs;
    }
    else{
        //normal case,two segments are not empty
        //0-a-[b]-0
        //0-v-[w]-0
        My_Assert(a_route == ns.solution[seg_after_a.segment_start]->route_id,"Wrong routes");
        My_Assert(v_route == ns.solution[seg_after_v.segment_start]->route_id,"Wrong routes");

        const int b = max(ns.solution[chosen_task]->next->ID, 0);
        const int w = max(ns.solution[neighbor_task]->next->ID, 0);

        const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
        const auto vw = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[w].head_node];
        const auto aw = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[w].head_node];
        const auto vb = mcgrp.min_cost[mcgrp.inst_tasks[v].tail_node][mcgrp.inst_tasks[b].head_node];

        delta = - ab - vw + aw + vb;

        const auto seg_after_a_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_a.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        const auto seg_after_v_dummy = mcgrp.min_cost[mcgrp.inst_tasks[seg_after_v.segment_end].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

        auto new_a_len = ns.routes[a_route]->length - ab - seg_after_a.len - seg_after_a_dummy  + aw + seg_after_v.len + seg_after_v_dummy;
        auto new_v_len = ns.routes[v_route]->length - vw - seg_after_v.len - seg_after_v_dummy  + vb + seg_after_a.len + seg_after_a_dummy;
        move_result.route_lens.push_back(new_a_len);
        move_result.route_lens.push_back(new_v_len);

        move_result.seq1_cus_num = seg_after_a.num_custs;
        move_result.seq2_cus_num = seg_after_v.num_custs;
    }

    move_result.delta = delta;

    move_result.num_affected_routes = 2;
    My_Assert(move_result.route_id.empty(),"Wrong statement");
    move_result.route_id.push_back(a_route);
    move_result.route_id.push_back(v_route);


    auto new_a_load = ns.routes[a_route]->load + a_load_delta;
    auto new_v_load = ns.routes[v_route]->load + v_load_delta;
    move_result.route_loads.push_back(new_a_load);
    move_result.route_loads.push_back(new_v_load);

    auto new_a_custs_num = ns.routes[a_route]->num_customers - seg_after_a.num_custs + seg_after_v.num_custs;
    auto new_v_custs_num = ns.routes[v_route]->num_customers - seg_after_v.num_custs + seg_after_a.num_custs;
    move_result.route_custs_num.push_back(new_a_custs_num);
    move_result.route_custs_num.push_back(new_v_custs_num);

    move_result.new_total_route_length = ns.cur_solution_cost + move_result.delta;

    move_result.total_number_of_routes = ns.routes.activated_route_id.size();

    vector<int> a_seq;
    a_seq.push_back(chosen_task);
    int cur_task = ns.solution[chosen_task]->ID;
    for(auto i = 0;i < seg_after_a.num_custs;i++) {
        cur_task = ns.solution[cur_task]->next->ID;
        a_seq.push_back(cur_task);
    }

    vector<int> v_seq;
    v_seq.push_back(neighbor_task);
    cur_task = ns.solution[neighbor_task]->ID;
    for(auto i = 0;i < seg_after_v.num_custs;i++) {
        cur_task = ns.solution[cur_task]->next->ID;
        v_seq.push_back(cur_task);
    }


    move_result.move_arguments = a_seq;
    move_result.move_arguments.insert(move_result.move_arguments.end(),v_seq.begin(),v_seq.end());

    move_result.task1 = chosen_task;
    move_result.task2 = neighbor_task;

    move_result.vio_load_delta = vio_load_delta;

    move_result.considerable = true;

    return true;
}



void NewSwapEnds::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt:swap-ends move");

    My_Assert(move_result.considerable,"Invalid predictions");

    const int a = move_result.task1;
    const int v = move_result.task2;

    //Extract the seq which needs swapped
    vector<int> a_seq;
    vector<int> v_seq;

    My_Assert(a == move_result.move_arguments.front(),"Incorrect arguments");
    int i;
    for(i = 1; move_result.move_arguments[i] != v;i++){
        a_seq.push_back(move_result.move_arguments[i]);
    }

    My_Assert(v == move_result.move_arguments[i],"Incorrect arguments");
    i++;
    for(;i<move_result.move_arguments.size();i++){
        v_seq.push_back(move_result.move_arguments[i]);
    }

    My_Assert(all_of(a_seq.begin(), a_seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");
    My_Assert(all_of(v_seq.begin(), v_seq.end(),[&](int i){return i>=1 && i<=mcgrp.actual_task_num;}),"Wrong task");

    My_Assert(a_seq.size() == move_result.seq1_cus_num && v_seq.size() == move_result.seq2_cus_num,"Wrong task");

    My_Assert(move_result.num_affected_routes == 2,"Wrong routes number");

    const int a_route = move_result.route_id[0];
    const int v_route = move_result.route_id[1];
    My_Assert(a_route != v_route,"Wrong routes!");
    My_Assert(ns.routes.activated_route_id.find(a_route)!=ns.routes.activated_route_id.end(),"Invalid route");
    My_Assert(ns.routes.activated_route_id.find(v_route)!=ns.routes.activated_route_id.end(),"Invalid route");

    if(move_result.seq1_cus_num != 0 && move_result.seq2_cus_num != 0){
        //Modify routes info
        My_Assert(!a_seq.empty() && !v_seq.empty(),"Wrong arguments");
        My_Assert(ns.solution[a]->next->ID == a_seq.front(),"Wrong task");
        My_Assert(ns.solution[v]->next->ID == v_seq.front(),"Wrong task");

        ns.routes[a_route]->length = move_result.route_lens[0];
        ns.routes[v_route]->length = move_result.route_lens[1];

        ns.routes[a_route]->load = move_result.route_loads[0];
        ns.routes[v_route]->load = move_result.route_loads[1];

        ns.routes[a_route]->num_customers = move_result.route_custs_num[0];
        ns.routes[v_route]->num_customers = move_result.route_custs_num[1];

        for(auto task : a_seq){
            ns.solution[task]->route_id = v_route;
        }

        for(auto task: v_seq){
            ns.solution[task]->route_id = a_route;
        }

        if(ns.solution[a_seq.front()]->pre->ID < 0){
            ns.routes[a_route]->start = v_seq.front();
        }

        ns.routes[a_route]->end = v_seq.back();

        if(ns.solution[v_seq.front()]->pre->ID < 0){
            ns.routes[v_route]->start = a_seq.front();
        }

        ns.routes[v_route]->end = a_seq.back();

        //handle solution
        //swap two segments
        const auto a_final_dummy = ns.solution[ns.solution[a_seq.back()]->next->ID];
        const auto v_final_dummy = ns.solution[ns.solution[v_seq.back()]->next->ID];

        ns.solution[a]->next = ns.solution[v_seq.front()];
        ns.solution[v_seq.back()]->next = a_final_dummy;
        ns.solution[v_seq.front()]->pre = ns.solution[a];
        a_final_dummy->pre = ns.solution[v_seq.back()];

        ns.solution[v]->next = ns.solution[a_seq.front()];
        ns.solution[a_seq.back()]->next = v_final_dummy;
        ns.solution[a_seq.front()]->pre = ns.solution[v];
        v_final_dummy->pre = ns.solution[a_seq.back()];

    }
    else if (move_result.seq1_cus_num == 0){
        My_Assert(move_result.seq2_cus_num != 0,"Wrong arguments");
        My_Assert(a_seq.empty() && !v_seq.empty(),"Wrong arguments");
        My_Assert(ns.solution[v]->next->ID == v_seq.front(),"Wrong task");

        if(v_seq.front() == ns.routes[v_route]->start && v_seq.back() == ns.routes[v_route]->end) {
            //v route need to be eliminated
            //Modify routes info

            ns.routes[a_route]->length = move_result.route_lens[0];

            ns.routes[a_route]->load = move_result.route_loads[0];

            ns.routes[a_route]->num_customers = move_result.route_custs_num[0];

            for(auto task: v_seq){
                ns.solution[task]->route_id = a_route;
            }

            ns.routes[a_route]->end = v_seq.back();

            ns.routes.free_route(v_route);

            //handle solution
            //record next dummy task of seg v
            int dummy_marker = 0;
            HighSpeedNeighBorSearch::TASK_NODE *reserve_dummy;
            bool very_end_case = false;
            if(ns.solution[v_seq.back()]->next == ns.solution.very_end){
                dummy_marker = ns.solution[v_seq.front()]->pre->ID;
                reserve_dummy = ns.solution[v_seq.back()]->next;
                very_end_case = true;
            }
            else{
                dummy_marker = ns.solution[v_seq.back()]->next->ID;
                reserve_dummy = ns.solution[v_seq.front()]->pre;
            }

            //handle solution
            //swap two segments
            const auto a_final_dummy = ns.solution[ns.solution[a]->next->ID];


            ns.solution[a]->next = ns.solution[v_seq.front()];
            ns.solution[v_seq.back()]->next = a_final_dummy;
            ns.solution[v_seq.front()]->pre = ns.solution[a];
            a_final_dummy->pre = ns.solution[v_seq.back()];

            //free dummy marker
            if(very_end_case){
                ns.solution[dummy_marker]->pre->next = reserve_dummy;
                reserve_dummy->pre = ns.solution[dummy_marker]->pre;
            }
            else{
                ns.solution[dummy_marker]->next->pre = reserve_dummy;
                reserve_dummy->next = ns.solution[dummy_marker]->next;
            }
            ns.solution.dummypool.free_dummy(dummy_marker);
        }
        else{
            //Modify routes info
            ns.routes[a_route]->length = move_result.route_lens[0];
            ns.routes[v_route]->length = move_result.route_lens[1];

            ns.routes[a_route]->load = move_result.route_loads[0];
            ns.routes[v_route]->load = move_result.route_loads[1];

            ns.routes[a_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[v_route]->num_customers = move_result.route_custs_num[1];

            for(auto task: v_seq){
                ns.solution[task]->route_id = a_route;
            }

            ns.routes[a_route]->end = v_seq.back();

            ns.routes[v_route]->end = v;


            //handle solution
            //swap two segments
            const auto a_final_dummy = ns.solution[ns.solution[a]->next->ID];
            const auto v_final_dummy = ns.solution[ns.solution[v_seq.back()]->next->ID];


            ns.solution[a]->next = ns.solution[v_seq.front()];
            ns.solution[v_seq.back()]->next = a_final_dummy;
            ns.solution[v_seq.front()]->pre = ns.solution[a];
            a_final_dummy->pre = ns.solution[v_seq.back()];

            ns.solution[v]->next = v_final_dummy;
            v_final_dummy->pre = ns.solution[v];
        }

    }
    else if (move_result.seq2_cus_num == 0){
        My_Assert(move_result.seq1_cus_num != 0,"Wrong arguments");
        My_Assert(!a_seq.empty() && v_seq.empty(),"Wrong arguments");
        My_Assert(ns.solution[a]->next->ID == a_seq.front(),"Wrong task");

        if(a_seq.front() == ns.routes[a_route]->start && a_seq.back() == ns.routes[a_route]->end){
            //a route need to be eliminated
            //Modify routes info

            ns.routes[v_route]->length = move_result.route_lens[1];

            ns.routes[v_route]->load = move_result.route_loads[1];

            ns.routes[v_route]->num_customers = move_result.route_custs_num[1];

            for(auto task: a_seq){
                ns.solution[task]->route_id = v_route;
            }

            ns.routes[v_route]->end = a_seq.back();

            ns.routes.free_route(a_route);

            //handle solution
            //record next dummy task of seg v
            int dummy_marker = 0;
            HighSpeedNeighBorSearch::TASK_NODE *reserve_dummy;
            bool very_end_case = false;
            if(ns.solution[a_seq.back()]->next == ns.solution.very_end){
                dummy_marker = ns.solution[a_seq.front()]->pre->ID;
                reserve_dummy = ns.solution[a_seq.back()]->next;
                very_end_case = true;
            }
            else{
                dummy_marker = ns.solution[a_seq.back()]->next->ID;
                reserve_dummy = ns.solution[a_seq.front()]->pre;
            }

            //handle solution
            //swap two segments
            const auto v_final_dummy = ns.solution[ns.solution[v]->next->ID];

            ns.solution[v]->next = ns.solution[a_seq.front()];
            ns.solution[a_seq.back()]->next = v_final_dummy;
            ns.solution[a_seq.front()]->pre = ns.solution[v];
            v_final_dummy->pre = ns.solution[a_seq.back()];

            //free dummy marker
            if(very_end_case){
                ns.solution[dummy_marker]->pre->next = reserve_dummy;
                reserve_dummy->pre = ns.solution[dummy_marker]->pre;
            }
            else{
                ns.solution[dummy_marker]->next->pre = reserve_dummy;
                reserve_dummy->next = ns.solution[dummy_marker]->next;
            }


            ns.solution.dummypool.free_dummy(dummy_marker);
        }
        else{
            //Modify routes info
            ns.routes[a_route]->length = move_result.route_lens[0];
            ns.routes[v_route]->length = move_result.route_lens[1];

            ns.routes[a_route]->load = move_result.route_loads[0];
            ns.routes[v_route]->load = move_result.route_loads[1];

            ns.routes[a_route]->num_customers = move_result.route_custs_num[0];
            ns.routes[v_route]->num_customers = move_result.route_custs_num[1];

            for(auto task: a_seq){
                ns.solution[task]->route_id = v_route;
            }

            ns.routes[v_route]->end = a_seq.back();

            ns.routes[a_route]->end = a;

            //handle solution
            //swap two segments
            const auto a_final_dummy = ns.solution[ns.solution[a_seq.back()]->next->ID];
            const auto v_final_dummy = ns.solution[ns.solution[v]->next->ID];


            ns.solution[v]->next = ns.solution[a_seq.front()];
            ns.solution[a_seq.back()]->next = v_final_dummy;
            ns.solution[a_seq.front()]->pre = ns.solution[v];
            v_final_dummy->pre = ns.solution[a_seq.back()];

            ns.solution[a]->next = a_final_dummy;
            a_final_dummy->pre = ns.solution[a];
        }

    }
    else{
        My_Assert(false,"Wrong arguments");
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