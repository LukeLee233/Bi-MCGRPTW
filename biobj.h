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
    vector<int> member_ids;
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

    void normalize(UNIT& unit) const{
        double normalized_o1 = (unit.objectives.first - min_o1) / (max_o1 - min_o1);
        double normalized_o2 = (unit.objectives.second - min_o2) / (max_o2 - min_o2);
        unit.normalized_objectives = {normalized_o1,normalized_o2};
    }

    static double epsilon_I(const UNIT& m1,const UNIT& m2){
        return max(m1.normalized_objectives.first - m2.normalized_objectives.first,
                   m1.normalized_objectives.second - m2.normalized_objectives.second);
    }

    void initialize_fitness(){

        for(auto& member: members)
            normalize(member);

        for(int i = 0;i < members.size();i++) {

            members[i].fitness = 0;
            for (int j = 0; j < members.size(); j++) {
                if (i == j) continue;

                members[i].fitness += epsilon_I(members[j],members[i]);
            }
        }

    }

    bool update_population(UNIT& unit){
        update_bound(unit.objectives);
        normalize(unit);

        for(auto member: members){
            normalize(member);
        }

        // calculate fitness
        unit.fitness = 0;
        for(const auto & member : members){
            unit.fitness += epsilon_I(member,unit);
        }

        pair<int,double> minimum_member{-1,DBL_MAX};
        for(int i = 0; i< members.size();i++){
            members[i].fitness += epsilon_I(unit,members[i]);
            if(members[i].fitness < minimum_member.second){
                minimum_member.first = i;
                minimum_member.second = members[i].fitness;
            }
        }

        // preserve population
        if(unit.fitness < minimum_member.second){
            for(auto& member:members){
                member.fitness -= epsilon_I(unit,member);
            }

            return false;
        }
        else{
            unit.fitness -= epsilon_I(members[minimum_member.first],unit);
            for(auto & member: members){
                member.fitness -= epsilon_I(members[minimum_member.first],member);
            }

            members[minimum_member.first] = unit;
            return true;
        }

    }



    void init_population(const MCGRP &mcgrp);

    static UNIT local_search(const vector<int>& sol, HighSpeedNeighBorSearch& ns, const MCGRP &mcgrp);

    int select();

    void search(const MCGRP &mcgrp);

    explicit BIOBJ(int upperBound)
        : upper_bound(upperBound)
    {
        for(int i = 0; i < upper_bound;i++){
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
