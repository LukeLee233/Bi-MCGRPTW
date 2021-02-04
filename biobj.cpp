//
// Created by luke on 2021/2/2.
//

#include "biobj.h"
#include "SingleInsert.h"
#include "DoubleInsert.h"
#include "TwoOpt.h"
#include "Swap.h"
#include "Invert.h"

bool sortbyfirst(const pair<int,double> &a,
               const pair<int,double> &b)
{
    return (a.first < b.first);
}

void BIOBJ::init_population(const MCGRP &mcgrp)
{
    cout << "Initialization pool...\n";

    for (auto try_count = 0; try_count < 2 * max_members_size && members.size() < max_members_size;) {
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


void BIOBJ::collect_dominating_point(const pair<double, double> &point)
{
    points_list.clear();

    for(const auto &member : members){
        if(member.belong_to_dominating){
            if(member.normalized_objectives.first <= point.first && member.normalized_objectives.second <= point.second){
                points_list.push_back(member.normalized_objectives);
            }
        }
    }
}

double BIOBJ::hyper_volume(const pair<double, double> &lb, const pair<double, double> &ru)
{
    return (ru.first - lb.first) * (ru.second - lb.second);
}

void BIOBJ::collect_non_dominated_neighbor(const pair<double, double> &point)
{
    points_list.clear();

    double lower_bound = -1;
    double upper_bound = DBL_MAX;
    pair<double, double> left_neighbor{-1,-1};
    pair<double, double> right_neighbor{-1,-1};

    for(const UNIT& member:members){
        if(member.normalized_objectives == point){
            continue;
        }

        if(member.normalized_objectives.first < point.first && member.normalized_objectives.first > lower_bound){
            lower_bound = member.normalized_objectives.first;
            left_neighbor = member.normalized_objectives;
        }

        if(member.normalized_objectives.first > point.first && member.normalized_objectives.first < upper_bound){
            upper_bound = member.normalized_objectives.first;
            right_neighbor = member.normalized_objectives;
        }
    }

    if(left_neighbor == pair<double,double>{-1,-1}){
        left_neighbor = pair<double,double>{point.first, reference_point.second};
    }

    if(right_neighbor == pair<double,double>{-1,-1}){
        right_neighbor = pair<double,double>{reference_point.first,point.second};
    }

    points_list = vector<pair<double,double>>{left_neighbor,right_neighbor};

    return;
}

bool BIOBJ::update_population(BIOBJ::UNIT &unit)
{
    update_bound(unit.objectives);
    normalize(unit);

    for(auto member: members){
        normalize(member);
    }

    //determine dominate situation
    // 1. dominate to other
    // 2. belong to dominate
    // 3. dominated by other
    int dominated_flag = -2;

    if(unit.normalized_objectives.first < dominated_benchmark.first && unit.normalized_objectives.second < dominated_benchmark.second){
        dominated_flag = 1;
    }
    else if(unit.normalized_objectives.first > dominated_benchmark.second && unit.normalized_objectives.second > dominated_benchmark.second){
        dominated_flag = 3;
    }else{
        dominated_flag = 2;
    }

    // special case
    if(dominated_flag == 1){
        unit.fitness = hyper_volume(unit.normalized_objectives,reference_point);
        unit.belong_to_dominating = true;

        pair<double, int> min_member{DBL_MAX,-1};
        for(int i = 0 ; i < members.size();i++){
            members[i].belong_to_dominating = false;
            members[i].fitness = -hyper_volume(unit.normalized_objectives,members[i].normalized_objectives);

            if(members[i].fitness < min_member.first){
                min_member.first = members[i].fitness;
                min_member.second = i;
            }
        }

        members[min_member.second] = unit;

        return true;
    }

    else if(dominated_flag == 2){
        collect_non_dominated_neighbor(unit.normalized_objectives);
        unit.fitness = hyper_volume(unit.normalized_objectives,{points_list[1].first,points_list[0].second});

        unit.belong_to_dominating = true;


        // modify related non dominated member
        for(auto& member:members){
            if(!member.belong_to_dominating){
                member.fitness = min(member.fitness, hyper_volume(unit.normalized_objectives,member.normalized_objectives));
            }
        }

        // find out the member with minimum fitness
        pair<double, int> min_member{DBL_MAX,-1};
        for(int i = 0;i < members.size();i++){
            if(members[i].fitness < min_member.first){
                min_member.first = members[i].fitness;
                min_member.second = i;
            }
        }

        // try to replace the member with minimum fitness
        if(unit.fitness < min_member.first){
            return false;
        }
        else{
            members[min_member.second] = unit;

            for(auto &member:members){
                if(member.belong_to_dominating){
                    collect_non_dominated_neighbor(member.normalized_objectives);
                    member.fitness = hyper_volume(member.normalized_objectives,{points_list[1].first,points_list[0].second});
                }
            }


//            collect_non_dominated_neighbor(unit.normalized_objectives);

//            vector<int> left_neighbor_id = locate_member(points_list[0]);
//            vector<int> right_neighbor_id = locate_member(points_list[1]);
//
//            for(auto id : left_neighbor_id){
//                if(id != -1){
//                    collect_non_dominated_neighbor(members[id].normalized_objectives);
//                    members[id].fitness = hyper_volume(members[id].normalized_objectives,{unit.normalized_objectives.first,points_list[0].second});
//                }
//            }
//
//            for(auto id : right_neighbor_id){
//                if(id != -1){
//                    collect_non_dominated_neighbor(members[id].normalized_objectives);
//                    members[id].fitness = hyper_volume(members[id].normalized_objectives,{points_list[1].first, unit.normalized_objectives.second});
//                }
//            }

            return true;
        }

    }

    else{
        unit.belong_to_dominating = false;
        collect_dominating_point(unit.normalized_objectives);
        double max_val = 0;
        for(const auto& point : points_list){
            max_val = max(max_val,hyper_volume(point,unit.normalized_objectives));
        }

        unit.fitness = - max_val;

        pair<double, int> min_non_dominate{DBL_MAX,-1};
        for(int i = 0;i < members.size();i++){
            if(members[i].fitness < min_non_dominate.first){
                min_non_dominate.first = members[i].fitness;
                min_non_dominate.second = i;
            }
        }

        if(unit.fitness < min_non_dominate.first){
            members[min_non_dominate.second] = unit;
            return true;
        }else{
            return false;
        }
    }


}

