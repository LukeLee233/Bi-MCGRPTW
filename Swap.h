#pragma once
#include "utils.h"
#include "MCGRP.h"
#include "NeighborSearch.h"

//low level operator
class Swap
{
public:
    static MCGRPMOVE move_result;
public:
    static int distance(const NeighBorSearch &ns, const int task1, const int task2);

    static bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details swap task j and task b
     * @param ns
     * @param C
     * @param j
     * @param b
     * @param rules
     * @return
     */
    static bool considerable_move(NeighBorSearch &ns, const MCGRP &C, int i,int u);

    static void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    static void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);
};


//low level operator
class NewSwap
{
public:
    MCGRPMOVE move_result;
public:

    NewSwap() : move_result(MCGRPMOVE(NeighborOperator::SWAP)){};

    int distance(const NeighBorSearch &ns, const int task1, const int task2);

    bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details swap task j and task b
     * @param ns
     * @param C
     * @param j
     * @param b
     * @param rules
     * @return
     */
    bool considerable_move(NeighBorSearch &ns, const MCGRP &C, int i,int u);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);




    bool search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details swap task j and task b
     * @param ns
     * @param C
     * @param j
     * @param b
     * @param rules
     * @return
     */
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &C, int i,int u);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};