#pragma once
#include "utils.h"
#include "MCGRP.h"
#include "NeighborSearch.h"


//low level operator
class Invert
{
public:
    MCGRPMOVE move_result;

    Invert() : move_result(MCGRPMOVE(NeighborOperator::INVERT)){};

    bool search(NeighBorSearch &ns, const class MCGRP &mcgrp, int chosen_task);

    bool considerable_move(NeighBorSearch &ns, const MCGRP &mcgrp, int u);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);

    /***********************************************************/

    bool search(HighSpeedNeighBorSearch &ns, const class MCGRP &mcgrp, int chosen_task);

    bool considerable_move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int u);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};