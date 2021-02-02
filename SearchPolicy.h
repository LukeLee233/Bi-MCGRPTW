//
// Created by luke on 2019/12/31.
//

#ifndef SEARCHPOLICY_H
#define SEARCHPOLICY_H

#include <bitset>
#include "MCGRP.h"
#include "config.h"

enum search_sequence
{
    RTRIDP, IDPRTR
};

//the number of rules
const int RULES = 8;

using std::bitset;

//Search Rule
const bitset<RULES> UNDEFINE("00000000");                                //undefined policy

const bitset<RULES> DOWNHILL("00000001");
const bitset<RULES> TOLERANCE("00000010");

const bitset<RULES> DELTA_ONLY("10000000");                 //This means feasible search
const bitset<RULES> FITNESS_ONLY("01000000");              //This means infeasible search

//Accept Rule
const bitset<RULES> FIRST_ACCEPT("00010000");
const bitset<RULES> BEST_ACCEPT("00100000");



class Policy{
    bitset<RULES> current_policy;    //the policy used by improvement search

public:
    double beta = std::numeric_limits<decltype(beta)>::max();
    double benchmark = std::numeric_limits<decltype(benchmark)>::max();
    double nearest_feasible_cost = std::numeric_limits<decltype(benchmark)>::max();

    const int tabu_step_threshold = tabu_step;

    //infeasible search penalty coefficient
    double tolerance = 0;


    Policy(){current_policy = UNDEFINE;};
    inline bitset<RULES> get(){return current_policy;};

    inline void set(bitset<RULES> _policy){current_policy = _policy;};


    /*!
 * test policy whether has a given rule
 * @param policy
 * @param rule
 * @return
 */
   inline bool has_rule(const bitset<RULES> &rule){
        return current_policy == (current_policy | rule);}

    /*!
    * @details check whether the given move is permissible based on the given policy
    * @param mcgrp
    * @param move_result
    * @param policy
    * @return
    */
    bool check_move(const MCGRPMOVE &move_result);

    /*!
     * @details check if moveresult M1 is better than moveresult M2
     * @param ns
     * @param M1
     * @param M2
     * @return
     */
    bool check_result(const MCGRPMOVE &M1, const MCGRPMOVE &M2);
};

#endif //SEARCHPOLICY_H
