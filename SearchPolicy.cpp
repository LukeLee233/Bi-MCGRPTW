//
// Created by luke on 2019/12/31.
//

#include "SearchPolicy.h"


bool Policy::check_move(const MCGRPMOVE &move_result)
{
    //To control the frequency
    static int count = 0;
    static bool flag = true;

    /*----------------------------Feasible search policy------------------------------------*/
    if (has_rule(DELTA_ONLY)) {
        if ((has_rule(DOWNHILL)))
            return move_result.delta <= 0;
        else if (has_rule(TOLERANCE)) {
            if(move_result.delta < 0){
                return true;
            }

            else if (flag && move_result.new_total_route_length <= (1.0 + tolerance) * benchmark) {
                    flag = false;
                    return true;
                }
            else{
                count++;
                if (count == tabu_step_threshold) {
                    flag = true;
                    count = 0;
                }
                return false;
            }
        }
    }

    /*----------------------------Infeasible search policy------------------------------------*/
    else if (has_rule(FITNESS_ONLY)) {
        My_Assert(beta != std::numeric_limits<decltype(beta)>::max(), "beta is undefined!");
        double delta_fitness;

        delta_fitness = move_result.delta + beta * move_result.vio_load_delta;
        if ((has_rule(DOWNHILL)))
            return delta_fitness <= 0;
        else if (has_rule(TOLERANCE)) {
            if (tolerance != 0 && delta_fitness <= tolerance * benchmark) {
                if (flag) {
                    flag = false;
                    return true;
                }
                else {
                    count++;
                    if (count == tabu_step_threshold) {
                        flag = true;
                        count = 0;
                    }
                    return false;
                }
            }
            else {
                return false;
            }
        }
    }
}

bool Policy::check_result(const MCGRPMOVE &M1, const MCGRPMOVE &M2)
{
    // We are only concerned about the savings (increase/decrese) in total length
    // This is the default approach
    // Decide in terms of total length only
    if (has_rule(DELTA_ONLY)) {
        return M1.new_total_route_length < M2.new_total_route_length;
    }
    else if (has_rule(FITNESS_ONLY)) {
        if(M2.considerable == false){
            return true;
        }
        My_Assert(beta != std::numeric_limits<decltype(beta)>::max(), "beta is undefined!");

        double M1_fitness = M1.delta + beta * M1.vio_load_delta;
        double M2_fitness = M2.delta + beta * M2.vio_load_delta;
        return M1_fitness < M2_fitness;
    }
}