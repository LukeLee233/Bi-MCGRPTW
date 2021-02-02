#include "TwoOpt.h"
#include <algorithm>

using namespace std;

//MCGRPMOVE TwoOpt::move_result = MCGRPMOVE(NeighborOperator::TWO_OPT);
//
//vector<int> TwoOpt::get_successor_tasks(NeighBorSearch &ns, const int chosen_task, const int _length)
//{
//    int current_node;
//    vector<int> tmp;
//
//    current_node = chosen_task;
//    tmp.push_back(current_node);
//    while (tmp.size() < _length) {
//        current_node = ns.next_array[current_node];
//
//        if (current_node < 0) {
//            return tmp;
//        }
//        else {
//            //special case
//            if (current_node == DUMMY) {
//                My_Assert(abs(ns.pred_array[current_node]) == ns.delimiter_coding_sol[ns.delimiter_coding_sol.size() - 2],
//                          "This is not the last task in last route!");
//                return tmp;
//            }
//            else {
//                tmp.push_back(current_node);
//            }
//        }
//    }
//    return tmp;
//}
//
//bool TwoOpt::search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task)
//{
//    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");
//
//    MCGRPMOVE BestM;
//
//    // Create the search_space
//    ns.create_search_neighborhood(mcgrp, chosen_task);
//
//    //first sequence with 2-opt
//    vector<int> chosen_seq;
//    chosen_seq.reserve(length);
//    chosen_seq = get_successor_tasks(ns, chosen_task, length);        //保证参与扰动序列在同一路径内
//
//    if (chosen_seq.empty()) {
//        return false;
//    }
//
//    int candidate_task = -1;
//    for (int i = 0; i < ns.search_space.size(); i++) {
//        //explore chosen task's neighbourhood
//        candidate_task = ns.search_space[i];
//
//        if (candidate_task == DUMMY) {
//            // Here I decide to skip handle the situation of dummy candidate,
//            // cause this operator is fairly time consuming
//            continue;
//        }
//
//        if (ns.next_array[candidate_task]
//            == std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max()) {
//            //feasible disturbance
//            if (mcgrp.is_edge(candidate_task)) {
//                candidate_task = mcgrp.inst_tasks[candidate_task].inverse;
//                My_Assert(ns.next_array[candidate_task]
//                              != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max(),
//                          "candidate task must in the current solution!");
//            }
//            else {
//                cerr << "An one direction task has been forgotten!\n"
//                     << "Exception location: Two-opt::search!\n";
//                abort();
//            }
//        }
//
//        //second sequence
//        vector<int> candidate_seq;
//        candidate_seq.reserve(length);
//        candidate_seq = get_successor_tasks(ns, candidate_task, length);
//
//        if (!overlap(chosen_seq, candidate_seq) && considerable_move(ns, mcgrp, chosen_seq, candidate_seq)) {
//            if (ns.policy.has_rule(FIRST_ACCEPT)) {
//                move(ns, mcgrp);
//                return true;
//            }
//            else if (ns.policy.has_rule(BEST_ACCEPT)) {
//                if (ns.policy.check_result(move_result,BestM))
//                    BestM = move_result;
//            }
//            else {
//                cerr << "Unknown accept rule!\n";
//                abort();
//            }
//        }
//    }
//
//    if (BestM.considerable) {
//        move_result = BestM;
//        move(ns, mcgrp);
//        return true;
//    }
//    else {
//        return false;
//    }
//}
//
//bool TwoOpt::overlap(const vector<int> &chosen_seq, const vector<int> &candidate_seq)
//{
//    for (auto task : chosen_seq) {
//        for (auto another_task : candidate_seq) {
//            if (task == another_task)
//                return true;
//        }
//    }
//
//    return false;
//}
//
//bool TwoOpt::considerable_move(NeighBorSearch &ns,
//                               const MCGRP &mcgrp,
//                               vector<int> chosen_seq,
//                               vector<int> candidate_seq)
//{
//
//    My_Assert(find(chosen_seq.begin(), chosen_seq.end(), DUMMY) == chosen_seq.end()
//                  && find(candidate_seq.begin(), candidate_seq.end(), DUMMY) == candidate_seq.end() != DUMMY,
//              "You cannot handle dummy tasks within 2-opt!");
//
//    My_Assert(!overlap(chosen_seq, candidate_seq), "Two seqs are overlapping!");
//
//    move_result.choose_tasks(chosen_seq.front(), candidate_seq.front());
//
//    const int chosen_route = ns.route_id[chosen_seq.front()];
//    const int candidate_route = ns.route_id[candidate_seq.front()];
//
//    if (chosen_route == candidate_route) {
//        //flip operator here
//        auto chosen_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), chosen_seq.front());
//        auto candidate_ite = find(ns.delimiter_coding_sol.begin(), ns.delimiter_coding_sol.end(), candidate_seq.front());
//        My_Assert(chosen_ite != ns.delimiter_coding_sol.end() && candidate_ite != ns.delimiter_coding_sol.end(),
//                  "Can't find task!");
//        My_Assert(chosen_ite != candidate_ite, "two ends can't be same!");
//
//        int start_task;
//        int end_task;
//        if (chosen_ite < candidate_ite) {
//            start_task = chosen_seq.front();
//            end_task = candidate_seq.back();
//        }
//        else {
//            start_task = candidate_seq.front();
//            end_task = chosen_seq.back();
//        }
//
//
//        if (Flip::considerable_move(ns, mcgrp, start_task, end_task)
//            && ns.policy.check_move(mcgrp, Flip::move_result)) {
//            move_result = Flip::move_result;
//            return true;
//        }
//        else {
//            move_result.reset();
//            move_result.move_type = NeighborOperator::TWO_OPT;
//            return false;
//        }
//
//    }
//    else {
//        //swap ends here
//        if (SwapEnds::considerable_move(ns, mcgrp, chosen_seq, candidate_seq)
//            && ns.policy.check_move(mcgrp, SwapEnds::move_result)) {
//            move_result = SwapEnds::move_result;
//            return true;
//        }
//        else {
//            move_result.reset();
//            move_result.move_type = NeighborOperator::TWO_OPT;
//            return false;
//        }
//    }
//}
//
//void TwoOpt::move(NeighBorSearch &ns, const MCGRP &mcgrp)
//{
//    My_Assert(move_result.move_type == NeighborOperator::FLIP || move_result.move_type == NeighborOperator::SWAP_ENDS,
//              "Unknown move type!");
//
//    if (move_result.move_type == NeighborOperator::FLIP) {
//        Flip::move_result = move_result;
//        Flip::move(ns, mcgrp);
//    }
//    else if (move_result.move_type == NeighborOperator::SWAP_ENDS) {
//        SwapEnds::move_result = move_result;
//        SwapEnds::move(ns, mcgrp);
//    }
//
//    mcgrp.check_best_solution(ns.cur_solution_cost, ns.negative_coding_sol);
//
//    move_result.reset();
//    move_result.move_type = NeighborOperator::TWO_OPT;
//}
//
//void TwoOpt::unit_test(NeighBorSearch &ns, const MCGRP &mcgrp)
//{
//    vector<int> task_set(mcgrp.actual_task_num);
//    std::generate(task_set.begin(), task_set.end(), Generator());
//    mcgrp._rng.RandPerm(task_set);    //shuffle tasks
//
//    ns.policy.set(BEST_ACCEPT | DOWNHILL | DELTA_ONLY);
//    ns.policy.beta = 0.5;
////    ns.policy.tolerance = 0.003;
//    ns.neigh_size = mcgrp.neigh_size;
//
//    int chosen_task = -1;
//    for (int i = 0; i < mcgrp.actual_task_num; i++) {
//        chosen_task = task_set[i];
//
//        if (ns.next_array[chosen_task] == std::numeric_limits<identity<decltype(ns
//            .next_array)>::type::value_type>::max()) {    //保证进行扰动的任务位于当前解中(可行空间内扰动)
//            if (!mcgrp.is_edge(chosen_task)) {    //非边任务一定在解序列中
//                cerr << "A non edge task has been missed!\n";
//                abort();
//            }
//            else {
//                chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
//                My_Assert(ns.next_array[chosen_task]
//                              != std::numeric_limits<identity<decltype(ns.next_array)>::type::value_type>::max(),
//                          "An edge task has been missed");
//            }
//        }
//
//        search(ns, mcgrp, chosen_task);
//    }
//
//    ns.neigh_size = 0;
//}


bool NewTwoOpt::search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task){
    My_Assert(chosen_task != DUMMY, "Chosen task can't be dummy");

    flip_times = 0;
    swapends_times = 0;

    MCGRPMOVE BestM;

    // Create the search_space
    ns.create_search_neighborhood(mcgrp, chosen_task);

    int b = chosen_task;
    int a = max(ns.pred_array[b], DUMMY);
    int c = max(ns.next_array[b], DUMMY);

    for(auto neighbor_task : ns.search_space){
        My_Assert(neighbor_task != b,"Neighbor task can't be itself at all!");

        if(neighbor_task != DUMMY){
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;
            int i = max(ns.pred_array[j], 0);
            int k = max(ns.next_array[j], 0);

            //total having four cases:
            //(a,b)<->(i,j)
            //(a,b)<->(j,k)
            //(b,c)<->(i,j)
            //(b,c)<->(j,k)
            if (considerable_move(ns,mcgrp, a, b, i, j)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns,mcgrp, a, b, j, k)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns, mcgrp, b, c, i, j)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns,mcgrp, b, c, j, k)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }
        }
        else{
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here and b can't be dummy neither
            //each start and end location of each route will be considered
            //total 4 x route_nums cases
            int current_start = abs(ns.next_array[DUMMY]);

            while (current_start != DUMMY){
                // Consider the start location
                int j = current_start;

                //case: (a,b)<->(dummy,j)
                if (considerable_move(ns, mcgrp, a, b, DUMMY, j)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                //case: (b,c)<->(dummy,j)
                if (considerable_move(ns, mcgrp, b, c, DUMMY, j)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                // Consider the end location
                int current_route = ns.route_id[current_start];
                int current_end = ns.routes[current_route].end;
                j = current_end;

                //case: (a,b)<->(j,dummy)
                if (considerable_move(ns, mcgrp, a, b, j, DUMMY)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                //case: (b,c)<->(j,dummy)
                if (considerable_move(ns, mcgrp, b, c, j, DUMMY)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                // advance to next route
                current_start = abs(ns.next_array[current_end]);
            }
        }
    }

    if (ns.policy.has_rule(FIRST_ACCEPT))
    {
        DEBUG_PRINT("No actual move: First Accept Rule");
        return false;
    }
    else if (ns.policy.has_rule(BEST_ACCEPT)){
        if(BestM.considerable == false){
            DEBUG_PRINT("No actual move: Best Accept Rule");
            return false;
        }
        else{
            move_result = BestM;
            move(ns,mcgrp);
            return true;
        }
    }
    else{
        My_Assert(false,"Unknown accept rule");
    }

    DEBUG_PRINT("2-opt Flip times:" + to_string(flip_times));
    DEBUG_PRINT("2-opt SwapEnds times:" + to_string(swapends_times));
}

bool NewTwoOpt::before(const int a,const int b,const NeighBorSearch &ns){
    /// This function returns TRUE if a comes before b in their route
    /// and FALSE if b is before a. An error is reported if a and b are in different routes.
    /// Should be used sparingly as it loops and can be slow for large routes.

    My_Assert(a != DUMMY  && b != DUMMY,"You can't judge a dummy task!");
    My_Assert(ns.route_id[a] == ns.route_id[b],"Two tasks are not in the same route!");

    if (ns.next_array[a] == b)
        return true;
    if (ns.next_array[b] == a)
        return false;

    int i;
    i = a;
    while (i > 0 && i != b)
        i = ns.next_array[i];

    // At the end of this loop, if i<=0, then we're at the end of a route
    // and haven't encountered b ==> must have a after b in the route
    // If i==b, then we know that b follows a
    return (i == b);
}


bool NewTwoOpt::considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int a, int b, int c, int d){
    // A easy prediction for overlapping
    if ((a == c) || (a == d) || (b == c) || (b == d)) {
        return false;
    }

    // At most one DUMMY task in the procedure
    int num_dummy = 0;
    if (a == DUMMY) num_dummy++;
    if (b == DUMMY) num_dummy++;
    if (c == DUMMY) num_dummy++;
    if (d == DUMMY) num_dummy++;
    if (num_dummy > 1){
        DEBUG_PRINT("Too much dummy tasks");
        return false;
    }

    int a_route, c_route;

    if (a != DUMMY)
        a_route = ns.route_id[a];
    else
    {
        My_Assert(b != DUMMY,"task A is dummy and task B is dummy too? Come on! ");
        a_route = ns.route_id[b];
    }

    if (c != DUMMY)
        c_route = ns.route_id[c];
    else
    {
        My_Assert(b != DUMMY,"task C is dummy and task D is dummy too? Come on! ");
        c_route = ns.route_id[d];
    }


    if (a_route == c_route) {   //This is flip operation
        //Definitely feasible
        // same route: the 2opt move here corresponds to reversing the sequence of
        // the sub route that is between (a|b) and (c|d), open interval
        DEBUG_PRINT("Flip operator in 2-opt");
        flip_times++;
        if (a != DUMMY && b != DUMMY && c != DUMMY && d != DUMMY){
            if(before(a, c, ns)){      //...ab...cd...
                if(flip.considerable_move(ns, mcgrp, a, d) && ns.policy.check_move(flip.move_result)){
                    move_result = flip.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
            else{                   //...cd...ab...
                if(flip.considerable_move(ns,mcgrp,c,b) && ns.policy.check_move(flip.move_result)){
                    move_result = flip.move_result;
                    return true;
                }
                else{
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
        }
        else{
            if (a == DUMMY)			//ab...cd...
            {
                DEBUG_PRINT("a is dummy");
                DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

                // Put a sentinel task between a and b and predict
                ns.presert_sentinel(mcgrp,b);

                if (flip.considerable_move(ns,mcgrp, mcgrp.sentinel, d) && ns.policy.check_move(flip.move_result))
                {
                    ns.remove_sentinel(mcgrp);
                    move_result = flip.move_result;
                    return true;
                }
                else
                {
                    ns.remove_sentinel(mcgrp);
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
            else if (b == DUMMY)			//...cd...ab
            {
                DEBUG_PRINT("b is dummy");
                DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

                ns.postsert_sentinel(mcgrp,a);
                if (flip.considerable_move(ns,mcgrp, c, mcgrp.sentinel) && ns.policy.check_move(flip.move_result))
                {
                    ns.remove_sentinel(mcgrp);
                    move_result = flip.move_result;
                    return true;
                }
                else
                {
                    ns.remove_sentinel(mcgrp);
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
            else if (c == DUMMY)			//cd...ab...
            {
                DEBUG_PRINT("c is dummy");
                DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

                ns.presert_sentinel(mcgrp,d);
                if (flip.considerable_move(ns,mcgrp, mcgrp.sentinel, b) && ns.policy.check_move(flip.move_result))
                {
                    ns.remove_sentinel(mcgrp);
                    move_result = flip.move_result;
                    return true;
                }
                else
                {
                    ns.remove_sentinel(mcgrp);
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
            else if (d == DUMMY)			//...ab...cd
            {
                DEBUG_PRINT("d is dummy");
                DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));


                ns.postsert_sentinel(mcgrp, c);
                if (flip.considerable_move(ns,mcgrp, a, mcgrp.sentinel) && ns.policy.check_move( flip.move_result))
                {
                    ns.remove_sentinel(mcgrp);
                    move_result = flip.move_result;
                    return true;
                }
                else
                {
                    ns.remove_sentinel(mcgrp);
                    move_result.reset();
                    move_result.move_type = NeighborOperator::TWO_OPT;
                    return false;
                }
            }
        }
    }
    else //This is SwapEnds operation
    {
        //Cross routes, feasibility need checked
        //different routes: the 2opt move here corresponds to swap the sequence of
        // the sub route that is after a and c, open interval

        /// Example: ( a & v input): DEPOT-i-a-b-j-k-l-DEPOT and DEPOT-t-u-v-w-x-y-z-DEPOT becomes
        /// DEPOT-i-a-w-x-y-z-DEPOT and DEPOT-t-u-v-b-j-k-l-DEPOT
        DEBUG_PRINT("SwapEnds operator in 2-opt");
        swapends_times++;

        if (a != DUMMY && b != DUMMY && c != DUMMY && d != DUMMY)
        //...ab...
        //...cd...
        {
            if (swap_ends.considerable_move(ns,mcgrp, a, c) && ns.policy.check_move(swap_ends.move_result)){
                move_result = swap_ends.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
        else if (a == DUMMY){
        //ab...
        //...cd...
            DEBUG_PRINT("a is dummy");
            DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

            // Put a sentinel task between a and b and predict
            ns.presert_sentinel(mcgrp,b);
            if (swap_ends.considerable_move(ns,mcgrp, mcgrp.sentinel, c) && ns.policy.check_move(swap_ends.move_result)){
                ns.remove_sentinel(mcgrp);
                move_result = swap_ends.move_result;
                return true;
            }
            else{
                ns.remove_sentinel(mcgrp);
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
        else if (b == DUMMY){
        //...ab
        //...cd...
            DEBUG_PRINT("b is dummy");
            DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

            ns.postsert_sentinel(mcgrp,a);
            if (swap_ends.considerable_move(ns, mcgrp, a, c) && ns.policy.check_move(swap_ends.move_result)){
                ns.remove_sentinel(mcgrp);
                move_result = swap_ends.move_result;
                return true;
            }
            else{
                ns.remove_sentinel(mcgrp);
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
        else if (c == DUMMY){
        //...ab...
        //cd...
            DEBUG_PRINT("c is dummy");
            DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

            ns.presert_sentinel(mcgrp,d);
            if (swap_ends.considerable_move(ns,mcgrp, a, mcgrp.sentinel) && ns.policy.check_move(swap_ends.move_result)){
                ns.remove_sentinel(mcgrp);
                move_result = swap_ends.move_result;
                return true;
            }
            else{
                ns.remove_sentinel(mcgrp);
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
        else if (d == DUMMY){
            //...ab...
            //...cd
            DEBUG_PRINT("d is dummy");
            DEBUG_PRINT("a is: " + to_string(a) + "\nb is: " + to_string(b) \
                + "\nc is: " + to_string(c) + "\nd is: " + to_string(d));

            ns.postsert_sentinel(mcgrp,c);
            if (swap_ends.considerable_move(ns, mcgrp, a, c) && ns.policy.check_move(swap_ends.move_result)){
                ns.remove_sentinel(mcgrp);
                move_result = swap_ends.move_result;
                return true;
            }
            else{
                ns.remove_sentinel(mcgrp);
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }

        }

    }

    My_Assert(false,"Can't reach here!");
}


void NewTwoOpt::move(NeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt move");

    My_Assert(move_result.considerable,"Invalid predictions");

    if(move_result.move_type == NeighborOperator::FLIP){
        flip.move_result = move_result;
        flip.move(ns,mcgrp);
    }
    else if (move_result.move_type == NeighborOperator::SWAP_ENDS){
        swap_ends.move_result = move_result;
        swap_ends.move(ns,mcgrp);
    }
    else{
        My_Assert(false,"Unknown operator");
    }

    if (ns.total_vio_load == 0) {
        mcgrp.check_best_solution(ns.cur_solution_cost, ns.negative_coding_sol);
    }

    move_result.reset();
    move_result.move_type = NeighborOperator::TWO_OPT;
}

void NewTwoOpt::unit_test(NeighBorSearch &ns, const MCGRP &mcgrp)
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

bool NewTwoOpt::before(const int a,const int b, HighSpeedNeighBorSearch &ns){
    /// This function returns TRUE if a comes before b in their route
    /// and FALSE if b is before a.

    if(ns.solution.very_start == ns.solution[a]){
        return true;
    }

    if(ns.solution.very_end == ns.solution[a]){
        return false;
    }


    if(a < 0 && b < 0){
        int next_a = ns.solution[a]->next->ID;

        while(next_a > 0){
            next_a = ns.solution[next_a]->next->ID;
        }

        return next_a == b;
    }
    else if (a < 0)
    {
        int next_a = ns.solution[a]->next->ID;

        while(next_a > 0){
            if(next_a == b)
                return true;
            next_a = ns.solution[next_a]->next->ID;
        }

        return false;
    }
    else if (b < 0)
    {
        int next_a = ns.solution[a]->next->ID;

        while(next_a > 0){
            next_a = ns.solution[next_a]->next->ID;
        }

        return next_a == b;
    }
    else
    {
        int next_a = ns.solution[a]->next->ID;

        while(next_a > 0){
            if(next_a == b)
                return true;
            next_a = ns.solution[next_a]->next->ID;
        }

        return false;
    }

    My_Assert(false,"Can't reach here!");
}


bool NewTwoOpt::search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task){
    My_Assert(chosen_task >= 1 && chosen_task <= mcgrp.actual_task_num,"Wrong task");

    flip_times = 0;
    swapends_times = 0;

    MCGRPMOVE BestM;

    ns.create_search_neighborhood(mcgrp, chosen_task);

    int b = chosen_task;
    int a = ns.solution[b]->pre->ID;
    int c = ns.solution[b]->next->ID;

    for(auto neighbor_task : ns.search_space){
        My_Assert(neighbor_task != b,"Neighbor task can't be itself at all!");

        if(neighbor_task != DUMMY){
            //j can't be dummy and b can't be dummy neither here
            int j = neighbor_task;
            My_Assert(j >= 1 && j <= mcgrp.actual_task_num,"Wrong task");

            int i = ns.solution[j]->pre->ID;
            int k = ns.solution[j]->next->ID;

            //total having four cases:
            //(a,b)<->(i,j)
            //(a,b)<->(j,k)
            //(b,c)<->(i,j)
            //(b,c)<->(j,k)
            if (considerable_move(ns,mcgrp, a, b, i, j)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns,mcgrp, a, b, j, k)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns, mcgrp, b, c, i, j)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }

            if (considerable_move(ns,mcgrp, b, c, j, k)){
                if(ns.policy.has_rule(FIRST_ACCEPT)){
                    move(ns,mcgrp);
                    return true;
                }
                else if (ns.policy.has_rule(BEST_ACCEPT)){
                    if (ns.policy.check_result(move_result,BestM))
                        BestM = move_result;
                }
                else{
                    My_Assert(false,"Unknown accept rule!");
                }
            }
        }
        else{
            DEBUG_PRINT("Neighbor task is dummy task");
            //j is dummy here and b can't be dummy neither
            //each start and end location of each route will be considered
            //total 4 x route_nums cases
            int current_start = ns.solution.very_start->next->ID;

            while (current_start != DUMMY){
                // Consider the start location
                int j = current_start;
                My_Assert(ns.solution[j]->pre->ID < 0,"Wrong task");

                //case: (a,b)<->(dummy,j)
                if (considerable_move(ns, mcgrp, a, b, ns.solution[j]->pre->ID, j)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                //case: (b,c)<->(dummy,j)
                if (considerable_move(ns, mcgrp, b, c, ns.solution[j]->pre->ID, j)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                // Consider the end location
                int current_route = ns.solution[current_start]->route_id;
                int current_end = ns.routes[current_route]->end;
                j = current_end;

                My_Assert(ns.solution[j]->next->ID < 0,"Wrong task");

                //case: (a,b)<->(j,dummy)
                if (considerable_move(ns, mcgrp, a, b, j, ns.solution[j]->next->ID)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                //case: (b,c)<->(j,dummy)
                if (considerable_move(ns, mcgrp, b, c, j, ns.solution[j]->next->ID)){
                    if(ns.policy.has_rule(FIRST_ACCEPT)){
                        move(ns,mcgrp);
                        return true;
                    }
                    else if (ns.policy.has_rule(BEST_ACCEPT)){
                        if (ns.policy.check_result(move_result,BestM))
                            BestM = move_result;
                    }
                    else{
                        My_Assert(false,"Unknown accept rule!");
                    }
                }

                // advance to next route
                current_start = ns.solution[current_end]->next->next->ID;
            }
        }
    }

    if (ns.policy.has_rule(FIRST_ACCEPT))
    {
        DEBUG_PRINT("No actual move: First Accept Rule");
        return false;
    }
    else if (ns.policy.has_rule(BEST_ACCEPT)){
        if(BestM.considerable == false){
            DEBUG_PRINT("No actual move: Best Accept Rule");
            return false;
        }
        else{
            move_result = BestM;
            move(ns,mcgrp);
            return true;
        }
    }
    else{
        My_Assert(false,"Unknown accept rule");
    }

    DEBUG_PRINT("2-opt Flip times:" + to_string(flip_times));
    DEBUG_PRINT("2-opt SwapEnds times:" + to_string(swapends_times));
}


bool NewTwoOpt::considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int a, int b, int c, int d){
    // A easy prediction for overlapping
    if ((a == c) || (a == d) || (b == c) || (b == d)) {
        return false;
    }


    int a_route, c_route;

    if (a > 0)
        a_route = ns.solution[a]->route_id;
    else
    {
        My_Assert(b > 0,"task A is dummy and task B is dummy too? Come on! ");
        a_route = ns.solution[b]->route_id;
    }


    if (c > 0)
        c_route = ns.solution[c]->route_id;
    else
    {
        My_Assert(d > 0,"task C is dummy and task D is dummy too? Come on! ");
        c_route = ns.solution[d]->route_id;
    }


    if (a_route == c_route) {   //This is flip operation
        //Definitely feasible
        // same route: the 2opt move here corresponds to reversing the sequence of
        // the sub route that is between (a|b) and (c|d), open interval
        DEBUG_PRINT("Flip operator in 2-opt");
        flip_times++;

        if(before(a, c, ns)){      //...ab...cd...
            if(flip.considerable_move(ns, mcgrp, a, d) && ns.policy.check_move(flip.move_result)){
                move_result = flip.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
        else{        //...cd...ab...
            if(flip.considerable_move(ns,mcgrp,c,b) && ns.policy.check_move(flip.move_result)){
                move_result = flip.move_result;
                return true;
            }
            else{
                move_result.reset();
                move_result.move_type = NeighborOperator::TWO_OPT;
                return false;
            }
        }
    }
    else //This is SwapEnds operation
    {
        //Cross routes, feasibility need checked
        //different routes: the 2opt move here corresponds to swap the sequence of
        // the sub route that is after a and c, open interval

        /// Example: ( a & v input): DEPOT-i-a-b-j-k-l-DEPOT and DEPOT-t-u-v-w-x-y-z-DEPOT becomes
        /// DEPOT-i-a-w-x-y-z-DEPOT and DEPOT-t-u-v-b-j-k-l-DEPOT
        DEBUG_PRINT("SwapEnds operator in 2-opt");
        swapends_times++;

        if(swap_ends.considerable_move(ns, mcgrp, a, c, a_route, c_route) && ns.policy.check_move(swap_ends.move_result)){
            move_result = swap_ends.move_result;
            return true;
        }
        else{
            move_result.reset();
            move_result.move_type = NeighborOperator::TWO_OPT;
            return false;
        }

    }

    My_Assert(false,"Can't reach here!");
}


void NewTwoOpt::move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp){
    DEBUG_PRINT("execute a 2-opt move");

    My_Assert(move_result.considerable,"Invalid predictions");

    if(move_result.move_type == NeighborOperator::FLIP){
        flip.move_result = move_result;
        flip.move(ns,mcgrp);
    }
    else if (move_result.move_type == NeighborOperator::SWAP_ENDS){
        swap_ends.move_result = move_result;
        swap_ends.move(ns,mcgrp);
    }
    else{
        My_Assert(false,"Unknown operator");
    }

    ns.trace(mcgrp);

    //    mcgrp.check_best_infeasible_solution(ns.cur_solution_cost,ns.policy.beta,ns.total_vio_load,ns.negative_coding_sol);

    move_result.reset();
    move_result.move_type = NeighborOperator::TWO_OPT;
}


void NewTwoOpt::unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    vector<int> task_set(mcgrp.actual_task_num);
    std::generate(task_set.begin(), task_set.end(), Generator());
    mcgrp._rng.RandPerm(task_set);    //shuffle tasks

    auto original_policy = ns.policy.get();
    ns.policy.set(FIRST_ACCEPT | DOWNHILL | FITNESS_ONLY);
    ns.policy.beta = 0.5;
    ns.policy.tolerance = 0.003;
    ns.neigh_size = mcgrp.neigh_size;

    int chosen_task = -1;
    for (int i = 0; i < mcgrp.actual_task_num; i++) {
        chosen_task = task_set[i];

        if (ns.solution[chosen_task]->next == nullptr){
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
