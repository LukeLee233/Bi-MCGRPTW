//
// Created by luke on 2020/1/30.
//

#include "Similarity.h"
#include <algorithm>

double hamming_dist(const MCGRP &mcgrp, const vector<int> &a, const vector<int> &b)
{
    vector<vector<int>> atom_link;
    //include offset 1
    atom_link.resize(mcgrp.node_num + 1);

    My_Assert(atom_link[0].empty(), "Incorrect initializetion!");

    int pre_task, cur_task, next_task;
    int source_node, target_node;

    //parse solution a
    for (int i = 0; i < a.size(); ++i) {
        cur_task = abs(a[i]);

        //Special case: start of the route
        if (a[i] < 0) {
            //Inside & outside will be considered meantime
            pre_task = DUMMY;
            source_node = mcgrp.inst_tasks[pre_task].head_node;
            target_node = mcgrp.inst_tasks[cur_task].tail_node;
            atom_link[source_node].push_back(target_node);

            source_node = mcgrp.inst_tasks[pre_task].tail_node;
            target_node = mcgrp.inst_tasks[cur_task].head_node;
            atom_link[source_node].push_back(target_node);

        }

        if (i == a.size() - 1)
            next_task = DUMMY;
        else
            next_task = max(DUMMY, a[i + 1]);

        //Inside & outside will be considered meantime
        source_node = mcgrp.inst_tasks[cur_task].head_node;
        target_node = mcgrp.inst_tasks[next_task].tail_node;
        atom_link[source_node].push_back(target_node);

        source_node = mcgrp.inst_tasks[cur_task].tail_node;
        target_node = mcgrp.inst_tasks[next_task].head_node;
        atom_link[source_node].push_back(target_node);
    }


    int same_count = 0, num_rts = 0;


    //parse route b
    for (int i = 0; i < b.size(); ++i) {
        cur_task = abs(b[i]);

        //Special case: start of the route
        if (b[i] < 0) {
            num_rts++;
            pre_task = DUMMY;
            source_node = mcgrp.inst_tasks[pre_task].head_node;
            target_node = mcgrp.inst_tasks[cur_task].tail_node;

            auto ite = find(atom_link[source_node].begin(), atom_link[source_node].end(), target_node);
            if (ite != atom_link[source_node].end()) {
                atom_link[source_node].erase(ite);
                same_count++;
            }

            source_node = mcgrp.inst_tasks[pre_task].tail_node;
            target_node = mcgrp.inst_tasks[cur_task].head_node;

            ite = find(atom_link[source_node].begin(), atom_link[source_node].end(), target_node);
            if (ite != atom_link[source_node].end()) {
                atom_link[source_node].erase(ite);
                same_count++;
            }
        }


        if (i == b.size() - 1)
            next_task = DUMMY;
        else
            next_task = max(DUMMY, b[i + 1]);

        source_node = mcgrp.inst_tasks[cur_task].head_node;
        target_node = mcgrp.inst_tasks[next_task].tail_node;

        auto ite = find(atom_link[source_node].begin(), atom_link[source_node].end(), target_node);
        if (ite != atom_link[source_node].end()) {
            atom_link[source_node].erase(ite);
            same_count++;
        }

        source_node = mcgrp.inst_tasks[cur_task].tail_node;
        target_node = mcgrp.inst_tasks[next_task].head_node;

        ite = find(atom_link[source_node].begin(), atom_link[source_node].end(), target_node);
        if (ite != atom_link[source_node].end()) {
            atom_link[source_node].erase(ite);
            same_count++;
        }
    }

    double dist = 2 * (mcgrp.req_node_num + mcgrp.req_arc_num + mcgrp.req_edge_num + num_rts) - same_count;

    return dist;
}