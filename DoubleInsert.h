#pragma once
#include "utils.h"
#include "NeighborSearch.h"
#include "MoveString.h"


//high level operator
class DoubleInsert
{
    // Here you can set a given length to achieve m-sert
    const int length = 2;
    MCGRPMOVE move_result;

    int presert_times;
    int postsert_times;

    MoveString move_string;
    PreMoveString pre_move_string;
    PostMoveString post_move_string;

public:
    DoubleInsert(): move_string(MoveString())
    ,pre_move_string(PreMoveString())
    ,post_move_string(PostMoveString())
    ,move_result(MCGRPMOVE(NeighborOperator::DOUBLE_INSERT))
    ,presert_times(0),postsert_times(0){};

    /*!
     * @details get the seccessor of the chosen task within the same route,exclude dummy task
     * @param ns
     * @param chosen_task
     * @param length
     * @return
     */
    vector<int> get_successor_tasks(NeighBorSearch &ns, const int chosen_task);

    bool search(NeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details double sert from disturbance_seq to candidate_task
     * @param ns
     * @param mcgrp
     * @param disturbance_seq
     * @param j
     * @param M
     * @param policy
     * @return
     */
    bool considerable_move(NeighBorSearch &ns,
                                  const MCGRP &mcgrp,
                                  vector<int> disturbance_seq,
                                  const int j);

    void move(NeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(NeighBorSearch &ns, const MCGRP &mcgrp);



    /*!
 * @details get the seccessor of the chosen task within the same route,exclude dummy task
 * @param ns
 * @param chosen_task
 * @param length
 * @return
 */
    vector<int> get_successor_tasks(HighSpeedNeighBorSearch &ns, const int chosen_task);

    bool search(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp, int chosen_task);

    /*!
     * @details double sert from disturbance_seq to candidate_task
     * @param ns
     * @param mcgrp
     * @param disturbance_seq
     * @param j
     * @param M
     * @param policy
     * @return
     */
    bool considerable_move(HighSpeedNeighBorSearch &ns,
                                  const MCGRP &mcgrp,
                                  vector<int> disturbance_seq,
                                  const int j);

    void move(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);

    void unit_test(HighSpeedNeighBorSearch &ns, const MCGRP &mcgrp);
};