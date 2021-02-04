//
// Created by luke on 2021/2/2.
//

#include "biobj.h"
#include "SingleInsert.h"
#include "DoubleInsert.h"
#include "TwoOpt.h"
#include "Swap.h"
#include "Invert.h"

bool sortbysec(const pair<int,double> &a,
               const pair<int,double> &b)
{
    return (a.second < b.second);
}

void BIOBJ::init_population(const MCGRP &mcgrp)
{
    cout << "Initialization pool...\n";

    for (auto try_count = 0; try_count < 2 * upper_bound && members.size() < upper_bound;) {
        Individual buffer;
        nearest_scanning(mcgrp, buffer);
        UNIT new_member(buffer.sequence, {buffer.total_cost,buffer.balance});
        update_bound(new_member.objectives);

        bool repeat = false;
        for(const auto& member : members) {
            if(member.solution == new_member.solution) {
                repeat = true;
                break;
            }
        }

        if(!repeat){
            members.push_back(new_member);
            cout << members.size() << "th individual has been created!\n";
        }

        try_count++;
    }
}



void BIOBJ::search(const MCGRP &mcgrp)
{
    // Initialize local search
    HighSpeedNeighBorSearch ns(mcgrp);
    ns.neigh_size = mcgrp.neigh_size;
    ns.policy.set(FIRST_ACCEPT | DOWNHILL | DELTA_ONLY);

    int max_Iter = 10000;
    for(int iter = 0;iter < max_Iter;iter++){
        int idx = select();

        // no more member can improve population
        if(idx == -1){
            cout<<"no more member can be improved"<<endl;
            return;
        }

        // try to get a new member
        auto new_member = local_search(members[idx].solution, ns, mcgrp);

        if(!new_member.solution.empty()) {
            // update the population

            update_population(new_member);
        }
        else{
            members[idx].tried = true;
        }
    }

}

BIOBJ::UNIT BIOBJ::local_search(const vector<int>& sol, HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp)
{
    ns.clear();
    ns.unpack_seq(sol, mcgrp);

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
        mcgrp._rng.RandPerm(ns.task_set);

        for (int i = 0; i < mcgrp.actual_task_num; i++) {
            chosen_task = ns.task_set[i];
            if (ns.solution[chosen_task]->next == nullptr) {
                if (mcgrp.is_edge(chosen_task)) {
                    chosen_task = mcgrp.inst_tasks[chosen_task].inverse;
                    My_Assert(local_search.solution[chosen_task]->next != nullptr,"An edge task has been missed");
                }
                else {
                    My_Assert(false,"A non edge task has been missed!");
                }
            }

            switch (cur_operator){
                case SINGLE_INSERT:
                    if(ns.single_insert->bi_search(ns, mcgrp, chosen_task)){
                        auto solution = ns.get_solution();
                        auto objectives = pair<double,double>{ns.get_cur_cost(), ns.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case DOUBLE_INSERT:
                    if(ns.double_insert->bi_search(ns, mcgrp, chosen_task)){
                        auto solution = ns.get_solution();
                        auto objectives = pair<double,double>{ns.get_cur_cost(), ns.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case SWAP:
                    if(ns.swap->bi_search(ns, mcgrp, chosen_task)){
                        auto solution = ns.get_solution();
                        auto objectives = pair<double,double>{ns.get_cur_cost(), ns.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case INVERT:
                    if(mcgrp.req_edge_num != 0){
                        if(ns.invert->bi_search(ns, mcgrp, chosen_task)){
                            auto solution = ns.get_solution();
                            auto objectives = pair<double,double>{ns.get_cur_cost(), ns.get_balance()};
                            return UNIT(solution,objectives);
                        }
                    }
                    break;
                case TWO_OPT:
                    if(ns.two_opt->bi_search(ns, mcgrp, chosen_task)){
                        auto solution = ns.get_solution();
                        auto objectives = pair<double,double>{ns.get_cur_cost(), ns.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                default:
                    My_Assert(false,"unknown operator!");
            }
        }
    }

    vector<int> dump{};
    return UNIT(dump,{0,0});
}

int BIOBJ::select()
{
    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(member_ids), std::end(member_ids), rng);
    int idx = -1;
    // select a member
    for(auto id : member_ids){
        if(members[id].tried == false){
            idx = id;
            break;
        }
    }

    return idx;
}
