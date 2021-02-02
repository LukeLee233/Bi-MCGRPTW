#pragma once
#include "utils.h"
#include "NeighborSearch.h"
#include "MCGRP.h"

//low level operator
class Flip
{
    //This will flip the whole tasks within two ends
public:
    static MCGRPMOVE move_result;
public:
    /*!
     * @details get the entire sequence needed to be fliped
     * @param ns
     * @param start
     * @param end
     * @return
     */
    static vector<int> get_sequence(NeighBorSearch &ns, const int start, const int end);

    static bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task);

    static void move(NeighBorSearch &ns, const MCGRP &mcgrp);
};


//low level operator
class NewFlip
{
    //This will flip the whole tasks within two ends
public:
    MCGRPMOVE move_result;

    /*!
     * @details get the entire sequence needed to be fliped
     * @param ns
     * @param start
     * @param end
     * @return
     */
    vector<int> get_sequence(NeighBorSearch &ns, const int start, const int end);

    vector<int> get_sequence(HighSpeedNeighBorSearch &ns, const int start, const int end);
public:
    NewFlip():move_result(MCGRPMOVE(NeighborOperator::FLIP)){};

    bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int start_task, int end_task);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};
