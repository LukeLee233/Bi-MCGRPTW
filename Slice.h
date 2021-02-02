//
// Created by luke on 2020/1/26.
//

#ifndef SLICE_H
#define SLICE_H

#include "MCGRP.h"
#include "NeighborSearch.h"
#include "utils.h"

//low level operator
class Preslice
{
public:
    MCGRPMOVE move_result;
public:

    Preslice(): move_result(MCGRPMOVE(NeighborOperator::PRE_SLICE)){};

    /*!
 * @details presert from task u to task i
 * @param ns
 * @param mcgrp
 * @param u
 * @param i
 * @param move_result
 * @return
 */
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int b);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};

//low level operator
class Postslice
{
public:
    MCGRPMOVE move_result;
public:

    Postslice(): move_result(MCGRPMOVE(NeighborOperator::POST_SLICE)){};

    /*!
 * prosert from task u to task i
 * @param ns
 * @param mcgrp
 * @param u
 * @param i
 * @param move_result
 * @return
 */
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, const int b);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};


//high level operator
class Slice
{
    MCGRPMOVE move_result;
    int pre_slice_times;
    int post_slice_times;
    Preslice preslice;
    Postslice postslice;
public:
    Slice(): preslice(Preslice()), postslice(Postslice())
    ,move_result(MCGRPMOVE(NeighborOperator::SLICE))
    ,pre_slice_times(0)
    ,post_slice_times(0){};

    /*----------------High speed neighbor search---------------------*/
    bool search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details insert task b to task j
     * @param ns
     * @param mcgrp
     * @param j
     * @param b
     * @param M
     * @param policy
     * @return
     */
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp,const int b);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};


#endif //SLICE_H
