//
// Created by luke on 2021/2/2.
//

#ifndef BIOBJ_H
#define BIOBJ_H

#include <bits/stdc++.h>
#include "NeighborSearch.h"

using namespace std;

bool sortbyfirst(const pair<int,double> &a,
               const pair<int,double> &b);


class BIOBJ{
public:
    struct UNIT{
        bool tried = false;
        vector<int> solution;   // zero as delimiter
        double fitness = DBL_MAX;
        bool belong_to_dominating = false;
        pair<double,double> objectives;
        pair<double,double> normalized_objectives;

        UNIT(){}

        UNIT(const vector<int> &solution, const pair<double, double> &objectives)
            : solution(solution), objectives(objectives)
        {}
    };

    int max_members_size;
    vector<UNIT> members;
    vector<int> member_ids;
    pair<double, double> reference_point{1.1,1.1};
    pair<double, double> dominated_benchmark{DBL_MAX,DBL_MAX};

    static double hyper_volume(const pair<double,double>& lb, const pair<double,double>& ru);

    double min_o1 = DBL_MAX;
    double min_o2 = DBL_MAX;

    double max_o1 = DBL_MIN;
    double max_o2 = DBL_MIN;

    vector<pair<double, double>> points_list;

    void collect_dominating_point(const pair<double,double>& point);
    void collect_non_dominated_neighbor(const pair<double,double>& point);

    void update_bound(pair<double, double> objective){
        update_o1_bound(objective.first);
        update_o2_bound(objective.second);
    }

    void update_o1_bound(double o1){
        min_o1 = min(min_o1,o1);
        max_o1 = max(max_o1,o1);
    }

    void update_o2_bound(double o2){
        min_o2 = min(min_o2,o2);
        max_o2 = max(max_o2,o2);
    }

    void normalize(UNIT& unit) const{
        double normalized_o1 = (unit.objectives.first - min_o1) / (max_o1 - min_o1);
        double normalized_o2 = (unit.objectives.second - min_o2) / (max_o2 - min_o2);
        unit.normalized_objectives = {normalized_o1,normalized_o2};
    }

    void init_dominated_info(){
        dominated_benchmark = {DBL_MAX,DBL_MAX};
        for(const auto &member: members){
            if(member.normalized_objectives.first < dominated_benchmark.first && member.normalized_objectives.second < dominated_benchmark.second){
                dominated_benchmark = member.normalized_objectives;
            }
        }

        for(auto& member: members){
            if(member.normalized_objectives.first == dominated_benchmark.first && member.normalized_objectives.second == dominated_benchmark.second){
                member.belong_to_dominating = true;
            }
            else if(member.normalized_objectives.first >= dominated_benchmark.first && member.normalized_objectives.second >= dominated_benchmark.second){
                member.belong_to_dominating = false;
            }else{
                member.belong_to_dominating = true;
            }
        }
    }

    vector<int> locate_member(const pair<double, double>& point){
        if(point == reference_point){
            return {-1};
        }

        vector<int> res;

        for(int i = 0 ; i< members.size();i++){
            if(point == members[i].normalized_objectives){
                res.push_back(i);
            }
        }

        return res;
    }

    void initialize_fitness(){

        for(auto& member: members)
            normalize(member);

        init_dominated_info();

        for(auto& member:members) {
            // two cases
            if(member.belong_to_dominating){
                collect_non_dominated_neighbor(member.normalized_objectives);
                member.fitness = hyper_volume(member.normalized_objectives,{points_list[1].first,points_list[0].second});
            }
            else{
                collect_dominating_point(member.normalized_objectives);
                double max_val = 0;
                for(const auto& point : points_list){
                    max_val = max(max_val,hyper_volume(point,member.normalized_objectives));
                }

                member.fitness = - max_val;
            }
        }
    }

    bool update_population(UNIT& unit);


    void init_population(const MCGRP &mcgrp);

    static UNIT local_search(const vector<int>& sol, HighSpeedNeighBorSearch& ns, const MCGRP &mcgrp);

    int select();

    void search(const MCGRP &mcgrp);

    explicit BIOBJ(int upperBound)
        : max_members_size(upperBound)
    {
        for(int i = 0; i < max_members_size; i++){
            member_ids.push_back(i);
        }
    }


    void clear(){
        members.clear();
        min_o1 = DBL_MAX;
        min_o2 = DBL_MAX;

        max_o1 = DBL_MIN;
        max_o2 = DBL_MIN;
    }
};


#endif //BIOBJ_H
