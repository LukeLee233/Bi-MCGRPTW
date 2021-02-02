//
// Created by luke on 2020/1/27.
//

#ifndef EXTRACTION_H
#define EXTRACTION_H

#include "MCGRP.h"
#include "NeighborSearch.h"
#include "utils.h"

//low level operator
class Extraction
{
    MCGRPMOVE move_result;
public:
    Extraction():move_result(MCGRPMOVE(NeighborOperator::EXTRACTION)){};

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

#endif //EXTRACTION_H
