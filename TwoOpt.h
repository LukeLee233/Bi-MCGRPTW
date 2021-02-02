#pragma once

#include "NeighborSearch.h"
#include "MCGRP.h"
#include "Flip.h"
#include "SwapEnds.h"
#include <vector>

//high level operator
//class TwoOpt
//{
//public:
//    static const int length = 2;
//    static MCGRPMOVE move_result;
//public:
//    /*!
//     * @details get the successor of the chosen task within the same route,exclude dummy task
//     * @param ns
//     * @param chosen_task
//     * @param length
//     * @return
//     */
//    static vector<int> get_successor_tasks(NeighBorSearch &ns, const int chosen_task, const int _length);
//
//    static bool overlap(const vector<int> &chosen_seq, const vector<int> &candidate_seq);
//
//    static bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);
//
//    static bool considerable_move(NeighBorSearch &ns,
//                                  const MCGRP &mcgrp,
//                                  vector<int> chosen_seq,
//                                  vector<int> candidate_seq);
//
//    static void move(NeighBorSearch &ns, const MCGRP &mcgrp);
//
//    static void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);
//};



class NewTwoOpt{
//    static const int length = 2;
    MCGRPMOVE move_result;
    int flip_times;
    int swapends_times;

    NewFlip flip;
    NewSwapEnds swap_ends;

    bool before(const int a,const int b, const NeighBorSearch &ns);
    bool before(const int a,const int b, HighSpeedNeighBorSearch &ns);
public:

    NewTwoOpt():flip(NewFlip())
    ,swap_ends(NewSwapEnds())
    ,move_result(MCGRPMOVE(NeighborOperator::TWO_OPT))
    ,flip_times(0)
    ,swapends_times(0){};

    bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    bool considerable_move(NeighBorSearch &ns,
                                  const MCGRP &mcgrp,
                                  int a, int b, int c, int d);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);


    bool search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    bool considerable_move(HighSpeedNeighBorSearch &ns,
                                  const MCGRP &mcgrp,
                                  int a, int b, int c, int d);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};


