#pragma once

#include "MCGRP.h"
#include "NeighborSearch.h"
#include "utils.h"
#include "Presert.h"
#include "PostSert.h"

//high level operator
class SingleInsert
{
    MCGRPMOVE move_result;
    int presert_times;
    int postsert_times;
    Presert presert;
    Postsert postsert;
public:
    SingleInsert() : presert(Presert()), postsert(Postsert()),
    move_result(MCGRPMOVE(NeighborOperator::SINGLE_INSERT)),
    presert_times(0),postsert_times(0){};

    bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

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
    bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int b, const int j);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);


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
    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int b, const int j);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};
