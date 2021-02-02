#pragma once

#include "utils.h"
#include "NeighborSearch.h"


//low level operator
class Postsert
{
public:
    MCGRPMOVE move_result;
public:

    Postsert(): move_result(MCGRPMOVE(NeighborOperator::POSTSERT)){};

    /*!
     * prosert from task u to task i
     * @param ns
     * @param mcgrp
     * @param u
     * @param i
     * @param move_result
     * @return
     */
    bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int u, const int i);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    /*!
 * prosert from task u to task i
 * @param ns
 * @param mcgrp
 * @param u
 * @param i
 * @param move_result
 * @return
 */
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int u, const int i);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};