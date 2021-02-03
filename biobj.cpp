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

bool try_to_replace(BIOBJ &biobj, pair<double, double> new_objectives){
    biobj.update_bound(new_objectives);

    BIOBJ::UNIT unit({},new_objectives);
    biobj.normalize(unit);

    unit.fitness = 0;
    for(const auto & member : biobj.members){
        unit.fitness += max(unit.normalized_objectives.first - member.normalized_objectives.first,
                            unit.normalized_objectives.second - member.normalized_objectives.second);
    }

    vector<double> new_fitness;
    for(const auto & member:biobj.members){
        new_fitness.emplace_back(member.fitness + max(member.normalized_objectives.first - unit.normalized_objectives.first,
                                                      member.normalized_objectives.second - unit.normalized_objectives.second));

    }

    sort(new_fitness.begin(),new_fitness.end());

    if(unit.fitness > new_fitness[0]){
        return true;
    }
    else{
        return false;
    }

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

    initialize_fitness();
}

void BIOBJ::search(const MCGRP &mcgrp)
{
    // Initialize local search
    HighSpeedNeighBorSearch local_search(mcgrp);
    local_search.neigh_size = mcgrp.neigh_size;
    local_search.policy.set(FIRST_ACCEPT | DOWNHILL | DELTA_ONLY);

    int idx = -1;
    while(true){
        idx = -1;
        // select a member
        for(int i = 0; i< members.size(); i++){
            if(members[i].tried == false){
                idx = i;
                break;
            }
        }

        // no member can improve population
        if(idx == -1){
            break;
        }

        // try to get a new member
        auto new_member = descent_search(members[idx].solution,local_search,mcgrp);

        if(!new_member.solution.empty()) {
            // update the population
            try_to_replace(new_member);
        }
        else{
            members[idx].tried = true;
        }
    }
}

BIOBJ::UNIT BIOBJ::descent_search(const vector<int>& sol, HighSpeedNeighBorSearch &local_search, const MCGRP &mcgrp)
{
    local_search.clear();
    local_search.unpack_seq(sol, mcgrp);

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
        mcgrp._rng.RandPerm(local_search.task_set);

        for (int i = 0; i < mcgrp.actual_task_num; i++) {
            chosen_task = local_search.task_set[i];
            if (local_search.solution[chosen_task]->next == nullptr) {
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
                    if(local_search.single_insert->bi_search(local_search, mcgrp, chosen_task)){
                        auto solution = local_search.get_solution();
                        auto objectives = pair<double,double>{local_search.get_cur_cost(),local_search.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case DOUBLE_INSERT:
                    if(local_search.double_insert->bi_search(local_search, mcgrp, chosen_task)){
                        auto solution = local_search.get_solution();
                        auto objectives = pair<double,double>{local_search.get_cur_cost(),local_search.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case SWAP:
                    if(local_search.swap->bi_search(local_search, mcgrp, chosen_task)){
                        auto solution = local_search.get_solution();
                        auto objectives = pair<double,double>{local_search.get_cur_cost(),local_search.get_balance()};
                        return UNIT(solution,objectives);
                    }
                    break;
                case INVERT:
                    if(mcgrp.req_edge_num != 0){
                        if(local_search.invert->bi_search(local_search, mcgrp, chosen_task)){
                            auto solution = local_search.get_solution();
                            auto objectives = pair<double,double>{local_search.get_cur_cost(),local_search.get_balance()};
                            return UNIT(solution,objectives);
                        }
                    }
                    break;
                case TWO_OPT:
                    if(local_search.two_opt->bi_search(local_search, mcgrp, chosen_task)){
                        auto solution = local_search.get_solution();
                        auto objectives = pair<double,double>{local_search.get_cur_cost(),local_search.get_balance()};
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
