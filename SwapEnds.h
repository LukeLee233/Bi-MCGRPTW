#pragma once
#include "utils.h"
#include "MCGRP.h"
#include "NeighborSearch.h"


//class SwapEnds
//{
//public:
//    static MCGRPMOVE move_result;
//public:
//    static bool
//    considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, vector<int> chosen_seq, vector<int> candidate_seq);
//
//    static void move(NeighBorSearch &ns, const MCGRP &mcgrp);
//};


class NewSwapEnds
{
public:
    MCGRPMOVE move_result;
public:
    NewSwapEnds():move_result(MCGRPMOVE(NeighborOperator::SWAP_ENDS)){}

    bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, const int chosen_task, const int neighbor_task);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int chosen_task, const int neighbor_task,const int chosen_route,const int neighbor_route);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};