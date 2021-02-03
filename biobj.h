//
// Created by luke on 2021/2/2.
//

#ifndef BIOBJ_H
#define BIOBJ_H

#include <bits/stdc++.h>
#include "NeighborSearch.h"

using namespace std;

bool sortbysec(const pair<int,double> &a,
               const pair<int,double> &b);


class BIOBJ{
public:
    struct UNIT{
        bool tried = false;
        vector<int> solution;   // zero as delimiter
        double fitness = DBL_MAX;
        pair<double,double> objectives;
        pair<double,double> normalized_objectives;

        UNIT(){}

        UNIT(const vector<int> &solution, const pair<double, double> &objectives)
            : solution(solution), objectives(objectives)
        {}
    };

    int upper_bound;
    vector<UNIT> members;
    double min_o1 = DBL_MAX;
    double min_o2 = DBL_MAX;

    double max_o1 = DBL_MIN;
    double max_o2 = DBL_MIN;

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

    void normalize(UNIT& unit){
        double normalized_o1 = (unit.objectives.first - min_o1) / (max_o1 - min_o1);
        double normalized_o2 = (unit.objectives.second - min_o2) / (max_o2 - min_o2);
        unit.normalized_objectives = {normalized_o1,normalized_o2};
    }

    void initialize_fitness(){
        for(auto& member: members){
            normalize(member);
        }

        for(int i = 0;i < members.size();i++) {
            members[i].fitness = 0;
            for (int j = 0; j < members.size(); j++) {
                if (i == j) continue;

                members[i].fitness += max(members[i].normalized_objectives.first - members[j].normalized_objectives.second,
                                          members[i].normalized_objectives.first - members[j].normalized_objectives.second);
            }

            members[i].fitness;
        }
    }

    bool try_to_replace(UNIT& unit){
        update_bound(unit.objectives);
        normalize(unit);

        unit.fitness = 0;
        for(const auto & member : members){
            unit.fitness += max(unit.normalized_objectives.first - member.normalized_objectives.first,
                                unit.normalized_objectives.second - member.normalized_objectives.second);
        }

        vector<pair<int,double>> new_fitness;
        int idx = 0;
        for(auto & member:members){
            new_fitness.emplace_back(idx,member.fitness + max(member.normalized_objectives.first - unit.normalized_objectives.first,
                                                              member.normalized_objectives.second - unit.normalized_objectives.second));

            idx++;
        }

        sort(new_fitness.begin(),new_fitness.end(),sortbysec);

        if(unit.fitness > new_fitness[0].second){
            for(auto & fitness: new_fitness){
                members[fitness.first].fitness = fitness.second;
            }

            members[new_fitness[0].first] = unit;
            return true;
        }
        else{
            return false;
        }

    }



    void init_population(const MCGRP &mcgrp);

    static UNIT descent_search(const vector<int>& sol, HighSpeedNeighBorSearch& ns, const MCGRP &mcgrp);

    void search(const MCGRP &mcgrp);

    explicit BIOBJ(int upperBound)
        : upper_bound(upperBound)
    {}


    void clear(){
        members.clear();
        min_o1 = DBL_MAX;
        min_o2 = DBL_MAX;

        max_o1 = DBL_MIN;
        max_o2 = DBL_MIN;
    }
};

bool try_to_replace(BIOBJ& biobj, pair<double,double> new_objectives);


#endif //BIOBJ_H
