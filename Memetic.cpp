#include "Memetic.h"
#include <iostream>
#include <vector>
#include "ConstructPolicy.h"
#include <algorithm>
#include <sys/timeb.h>
#include "Similarity.h"
#include "config.h"

using namespace std;

int POPOrderAscent(const POPOrder &a, const POPOrder &b)
{
    if (a.val > b.val)
        return 1;
    else
        return -1;
}

int POPOrderDescent(const POPOrder &a, const POPOrder &b)
{
    if (a.val < b.val)
        return 1;
    else
        return -1;
}

Memetic::Memetic(NeighBorSearch &local_search, const int pool_size, const int evolve_steps, int _QNDF_weight)
    : ns(local_search)
{
    QNDF_weight = _QNDF_weight;
    popSize = pool_size;
    evolve_num = evolve_steps;
}

void Memetic::reset()
{
    population.clear();
    distMatrix.clear();
    indi_obj.clear();
}

void Memetic::memetic_search(const MCGRP &mcgrp)
{
    struct timeb search_start_time;
    struct timeb cur_time;
    ftime(&search_start_time);
    ftime(&cur_time);

    //start initializing the pool
    init_population(mcgrp);

    if (popSize == 1) {
        cout << "Pop Size is 1\n"
             << "Toggle to Local Search...\n";
        for (auto iter = 1; iter <= evolve_num
        && get_time_difference(search_start_time,cur_time)< search_time ; iter++) {
            ftime(&cur_time);
            cout << iter << "th times local search\n";

            ns.unpack_seq(get_delimiter_coding(population.front()), mcgrp);
//            ns.create_individual(mcgrp, ns.ns_indi);
            ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

            ns.neighbor_search(mcgrp);
            population.front() = ns.negative_coding_sol;
            indi_obj.front() = ns.cur_solution_cost;
        }
    }
    else {
        cout << "Pop Size is " << popSize << endl;
        cout << "Evolution Search...\n";

        //initialize the hamming distance matrix of the pooling solution
        initDistMatrix(mcgrp);

        int randNo1, randNo2;    //The ID of parents who involve in crossovering

        for (auto iter = 1; iter <= evolve_num
        && get_time_difference(search_start_time,cur_time)< search_time ; iter++) {
            cout << iter << "th times evolution\n";
            ftime(&cur_time);
            /* choose parent randomly */
            randNo1 = mcgrp._rng.Randint(0, popSize - 1);
            randNo2 = mcgrp._rng.Randint(0, popSize - 1);
            while (randNo2 == randNo1)
                randNo2 = mcgrp._rng.Randint(0, popSize - 1);


            Individual parent1, parent2, xed_child;

            parse_solution(mcgrp, parent1, population[randNo1], indi_obj[randNo1]);
            parse_solution(mcgrp, parent2, population[randNo2], indi_obj[randNo2]);


            route_based_crossover(xed_child, parent1, parent2, mcgrp);        //crossover motion


            /* local search */
            ns.unpack_seq(xed_child.sequence, mcgrp);
//            ns.create_individual(mcgrp, ns.ns_indi);
            ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);

            ns.neighbor_search(mcgrp);

            auto search_result = ns.negative_coding_sol;
            auto search_result_obj = ns.cur_solution_cost;

            mcgrp.check_best_solution(search_result_obj, search_result);

            poolUpdate(mcgrp, search_result, search_result_obj);
        }
    }
}


bool Memetic::poolUpdate(const MCGRP &mcgrp, const vector<int> new_sol, const double new_obj)
{
    vector<double> distance_to_other_sol;
    for (int i = 0; i < popSize; ++i)
        distance_to_other_sol.push_back(hamming_dist(mcgrp, new_sol, population[i]));

    //consider new_sol too
    vector<POPOrder> objVec(popSize + 1);
    vector<POPOrder> distVec(popSize + 1);
    for (int i = 0; i < popSize; ++i) {
        objVec[i].posi = i;
        objVec[i].val = indi_obj[i];
    }
    objVec[popSize].posi = popSize;
    objVec[popSize].val = new_obj;
    sort(objVec.begin(), objVec.end(), POPOrderAscent);//from small to large


    for (int i = 0; i < popSize; ++i) {
        distVec[i].posi = i;
        distVec[i].val = 0;
        for (int j = 0; j < i; ++j)
            distVec[i].val += distMatrix[j][i];
        for (int j = i + 1; j < popSize; ++j)
            distVec[i].val += distMatrix[i][j];
        distVec[i].val = distVec[i].val * 1.0 / popSize;
    }

    distVec[popSize].posi = popSize;
    distVec[popSize].val = 0;
    for (auto dist : distance_to_other_sol) {
        distVec[popSize].val += dist;
    }
    distVec[popSize].val = distVec[popSize].val * 1.0 / popSize;

    sort(distVec.begin(), distVec.end(), POPOrderDescent);


    vector<double> qdf(popSize + 1);

    for (int i = 0; i <= popSize; ++i)        //calculate each solution's qdf value
    {
        qdf[objVec[i].posi] += QNDF_weight * i;
        qdf[distVec[i].posi] += (1 - QNDF_weight) * i;
    }

    double max_qdf = numeric_limits<decltype(max_qdf)>::min();
    int max_qdf_id = -1;
    for (int i = 0; i <= popSize; ++i) {
        if (qdf[i] > max_qdf) {
            max_qdf = qdf[i];
            max_qdf_id = i;
        }
    }
    My_Assert(max_qdf_id != -1, "Cannot find the maximum qdf!");

    if (max_qdf_id != popSize) {
        population[max_qdf_id] = new_sol;
        indi_obj[max_qdf_id] = new_obj;
        for (int i = 0; i < popSize; ++i) {
            if (max_qdf_id == i) {
                distMatrix[max_qdf_id][i] = 0;
            }
            else {
                distMatrix[max_qdf_id][i] = distance_to_other_sol[i];
                distMatrix[i][max_qdf_id] = distMatrix[max_qdf_id][i];
            }
        }
        return true;
    }
    else {
        return false;
    }
}

void Memetic::remove_empty_routes(vector<vector<int>> &XRoutes)
{
    vector<vector<int>> buffer;

    for (int i = 0; i < XRoutes.size(); i++) {   //each route
        if (XRoutes[i].size() > 2) {
            buffer.push_back(XRoutes[i]);
        }
    }
    XRoutes = buffer;
}

vector<int> Memetic::cal_loads(const MCGRP &mcgrp, const vector<vector<int>> &XRoutes)
{
    vector<int> load_buffer;
    for (int i = 0; i < XRoutes.size(); ++i)   //each route
    {
        int culmulative_load = 0;
        for (int j = 1; j < XRoutes[i].size() - 1; ++j)
            culmulative_load += mcgrp.inst_tasks[XRoutes[i][j]].demand;
        load_buffer.push_back(culmulative_load);
    }
    return load_buffer;
}

void
Memetic::route_based_crossover(Individual &xed_child, const Individual &p1, const Individual &p2, const MCGRP &mcgrp)
{
    vector<int> seq1;
    vector<int> seq2;

    if (p1.total_cost <= p2.total_cost) {
        seq1 = p1.sequence;
        seq2 = p2.sequence;
    }
    else {
        seq1 = p2.sequence;
        seq2 = p1.sequence;
    }


    vector<vector<int>> Routes1;
    vector<vector<int>> Routes2;
    vector<int> Positions;

    //Copy the information of solution 1 to route 1;
    Routes1.clear();
    Positions = find_ele_positions(seq1, DUMMY);
    for (int i = 0; i < Positions.size() - 1; i++) {  //extract each route sequence based on delimiter
        Routes1.push_back(vector<int>(seq1.begin() + Positions[i], seq1.begin() + Positions[i + 1]));
        Routes1.back().push_back(DUMMY);
    }


    //Copy the information of solution 2 to route 2;
    Routes2.clear();
    Positions = find_ele_positions(seq2, DUMMY);
    for (int i = 0; i < Positions.size() - 1; i++) {
        Routes2.push_back(vector<int>(seq2.begin() + Positions[i], seq2.begin() + Positions[i + 1]));
        Routes2.back().push_back(DUMMY);
    }

    int cross_num;
    if (cross_route_num <= 0) {
        cross_num = 1;
        cout << "Attention, Single route crossover!\n";
    }
    else {
        cross_num = min(cross_route_num, int(min(Routes1.size(), Routes2.size())));
        cout << "cross routes: " << cross_num << endl;
    }


    int rd1 = (int) mcgrp._rng.Randint(0, Routes1.size() - 1);    //choose a route from parent1 randomly
    int rd2 = (int) mcgrp._rng.Randint(0, Routes2.size() - 1);    //choose a route from parent2 randomly

    Routes1[rd1] = Routes2[rd2];        //begin crossover

//    cross over other route, avoid crossover same route(one-time step promise)
    for (int i = 2; i <= cross_num; i++) {
        int another_route;
        another_route = (int) mcgrp._rng.Randint(0, Routes1.size() - 1);
        while (rd1 == another_route)
            another_route = (int) mcgrp._rng.Randint(1, Routes1.size() - 1);
        rd1 = another_route;

        another_route = (int) mcgrp._rng.Randint(1, Routes2.size() - 1);
        while (rd2 == another_route)
            another_route = (int) mcgrp._rng.Randint(1, Routes2.size() - 1);
        rd2 = another_route;


        Routes1[rd1] = Routes2[rd2];        //begin crossover
    }


    /* repair part */
    vector<int> Omitted_task;
    remove_duplicates(mcgrp, Routes1, Omitted_task);


    vector<int> Routes_load;

    remove_empty_routes(Routes1);
    Routes_load = cal_loads(mcgrp, Routes1);        //calculate route's load

    My_Assert(Routes1.size() == Routes_load.size(), "Inconsistency between Route and route load!");


    mcgrp._rng.RandPerm(Omitted_task);

    struct Insert
    {
        int InsertedTask;        //待插入任务
        int InsertRouteID;        //插入路径ID
        int InsertPos;            //插入位置
        int InsertCost;
    };

    vector<Insert> CandInsertions;


    //Now we find the best place to insert the omitted tasks
    for (auto omitted_task : Omitted_task) {
        CandInsertions.clear();
        double best_delta = numeric_limits<decltype(best_delta)>::max();


        for (int route_id = 0; route_id < Routes1.size(); route_id++) {
            // check feasibility
            double delta;
            if (Routes_load[route_id] + mcgrp.inst_tasks[omitted_task].demand > mcgrp.capacity)
                continue;
            for (int cursor = 1; cursor < Routes1[route_id].size(); cursor++) {

                delta =
                    mcgrp.min_cost[mcgrp.inst_tasks[Routes1[route_id][cursor - 1]].tail_node][mcgrp
                        .inst_tasks[omitted_task].head_node] +
                        mcgrp.inst_tasks[omitted_task].serv_cost +
                        mcgrp.min_cost[mcgrp.inst_tasks[omitted_task].tail_node][mcgrp
                            .inst_tasks[Routes1[route_id][cursor]].head_node] -
                        mcgrp.min_cost[mcgrp.inst_tasks[Routes1[route_id][cursor - 1]].tail_node][mcgrp
                            .inst_tasks[Routes1[route_id][cursor]].head_node];

                if (delta < best_delta) {
                    best_delta = delta;
                    CandInsertions.clear();

                    Insert temp;
                    temp.InsertedTask = omitted_task;
                    temp.InsertRouteID = route_id;
                    temp.InsertPos = cursor;
                    temp.InsertCost = delta;
                    CandInsertions.push_back(temp);
                }
                else if (almost_equal(delta, best_delta)) {
                    Insert temp;
                    temp.InsertedTask = omitted_task;
                    temp.InsertRouteID = route_id;
                    temp.InsertPos = cursor;
                    temp.InsertCost = delta;
                    CandInsertions.push_back(temp);
                }

                if (mcgrp.is_edge(omitted_task))    //边任务
                {
                    int inverse_id = mcgrp.inst_tasks[omitted_task].inverse;
                    delta =
                        mcgrp.min_cost[mcgrp.inst_tasks[Routes1[route_id][cursor - 1]].tail_node][mcgrp
                            .inst_tasks[inverse_id].head_node]
                            + mcgrp.inst_tasks[inverse_id].serv_cost
                            + mcgrp.min_cost[mcgrp.inst_tasks[inverse_id].tail_node][mcgrp
                                .inst_tasks[Routes1[route_id][cursor]].head_node]
                            - mcgrp.min_cost[mcgrp.inst_tasks[Routes1[route_id][cursor - 1]].tail_node][mcgrp
                                .inst_tasks[Routes1[route_id][cursor]].head_node];

                    if (delta < best_delta) {
                        best_delta = delta;
                        CandInsertions.clear();

                        Insert temp;
                        temp.InsertedTask = inverse_id;
                        temp.InsertRouteID = route_id;
                        temp.InsertPos = cursor;
                        temp.InsertCost = delta;
                        CandInsertions.push_back(temp);
                    }
                    else if (almost_equal(delta, best_delta)) {
                        Insert temp;
                        temp.InsertedTask = inverse_id;
                        temp.InsertRouteID = route_id;
                        temp.InsertPos = cursor;
                        temp.InsertCost = delta;
                        CandInsertions.push_back(temp);
                    }
                }
            }
        }

        if (CandInsertions.empty()) {   //create a new route
            vector<int> new_route{DUMMY, DUMMY};
            double delta;

            if (mcgrp.is_edge(omitted_task)) {
                delta = mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[omitted_task].head_node] +
                    +mcgrp.inst_tasks[omitted_task].serv_cost +
                    mcgrp.min_cost[mcgrp.inst_tasks[omitted_task].tail_node][mcgrp.inst_tasks[DUMMY].head_node]
                    - mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

                double delta_tilde;
                int omitted_task_tilde = mcgrp.inst_tasks[omitted_task].inverse;
                delta_tilde =
                    mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[omitted_task_tilde].head_node] +
                        +mcgrp.inst_tasks[omitted_task_tilde].serv_cost +
                        mcgrp.min_cost[mcgrp.inst_tasks[omitted_task_tilde].tail_node][mcgrp.inst_tasks[DUMMY]
                            .head_node]
                        - mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

                int actual_chosen_task = delta < delta_tilde ? omitted_task : omitted_task_tilde;

                new_route.insert(new_route.begin() + 1, actual_chosen_task);
            }
            else {
                new_route.insert(new_route.begin() + 1, omitted_task);
            }

            Routes1.push_back(new_route);
            Routes_load.push_back(mcgrp.inst_tasks[omitted_task].demand);
        }
        else {

            int k = mcgrp._rng.Randint(0, CandInsertions.size() - 1);
            int route_ID = CandInsertions[k].InsertRouteID;
            int insert_pos = CandInsertions[k].InsertPos;
            int inserted_task = CandInsertions[k].InsertedTask;

            Routes1[route_ID].insert(Routes1[route_ID].begin() + insert_pos, inserted_task);
            Routes_load[route_ID] += mcgrp.inst_tasks[inserted_task].demand;
        }

        //check consistency
        My_Assert(cal_loads(mcgrp, Routes1) == Routes_load, "Inconsistent load info!");
    }


    for (int i = 0; i < Routes1.size(); i++) {
        if (Routes1[i].size() <= 2) {
            cerr << "Fata Error: generate empty routes!\n";
            abort();
        }
        xed_child.sequence.insert(xed_child.sequence.end(), Routes1[i].begin(), Routes1[i].end() - 1);
    }
    xed_child.sequence.push_back(DUMMY);


    xed_child.total_cost = mcgrp.get_task_seq_total_cost(xed_child.sequence);
    xed_child.route_seg_load = Routes_load;

    for (auto load : xed_child.route_seg_load) {
        if (load <= 0 || load > mcgrp.capacity) {
            cerr << "Fata Error! Infeasible load exists!";
            abort();
        }
    }

    xed_child.total_vio_load = 0;

}

void Memetic::parse_solution(const MCGRP &mcgrp, Individual &p, const vector<int> &sol, const double val)
{
    int route_number = 0;
    int route_load = 0;

    p.sequence = get_delimiter_coding(sol);
    p.route_seg_load.clear();
    for (int i = 0; i < sol.size(); ++i) {
        if (sol[i] < 0) {
            route_number++;

            if (route_number > 1) {
                p.route_seg_load.push_back(route_load);
                route_load = 0;
            }

        }
        route_load += mcgrp.inst_tasks[abs(sol[i])].demand;
    }
    if (route_load > 0) {
        p.route_seg_load.push_back(route_load);
    }

    My_Assert(p.route_seg_load.size() == route_number, "Incorrect parse!");

    p.total_cost = val;
    p.total_vio_load = 0;
}

void Memetic::remove_duplicates(const MCGRP &mcgrp, vector<vector<int>> &XRoutes, vector<int> &Omitted_task)
{
    //statisticize the distribution of tasks, exclude dummy task
    Omitted_task.clear();
    vector<vector<int>> task_distribution(mcgrp.actual_task_num + 1);

    //get distribution
    for (int i = 0; i < XRoutes.size(); ++i)    //each route
    {
        for (int j = 1; j < XRoutes[i].size() - 1; ++j)   //each customer,exclude dummy customer
        {
            int task_id = XRoutes[i][j];
            // partial order preserves for edge task.(smaller will be recorded)
            if (mcgrp.is_edge(task_id)) {
                if (task_id > mcgrp.req_edge_num)
                    task_id = mcgrp.inst_tasks[task_id].inverse;
            }

            task_distribution[task_id].push_back(i);    //record appear in which route and only appear once
            My_Assert(task_distribution[task_id].size() < 2
                          || task_distribution[task_id][task_distribution[task_id].size() - 2]
                              != task_distribution[task_id].back(),
                      "duplicate task must occur only once in the same route!");
        }
    }



    //calculate all task
    for (int i = 1; i <= mcgrp.actual_task_num; ++i) {
        if (mcgrp.is_edge(i)) {
            if (i > mcgrp.req_edge_num)
                continue;
        }

        struct Reserve_Info
        {
            int pos = -1;
            int route = -1;
            double delta = numeric_limits<decltype(delta)>::max();
        };
//		task which never appears
        if (task_distribution[i].empty())
            Omitted_task.push_back(i);
        else {
            Reserve_Info reserve_info;
            int pre;
            int cur;
            int post;
            double delta;
            for (auto route_id : task_distribution[i]) {
                for (int j = 1; j < XRoutes[route_id].size() - 1; ++j) {
                    pre = XRoutes[route_id][j - 1];
                    post = XRoutes[route_id][j + 1];
                    cur = XRoutes[route_id][j];

                    if (mcgrp.is_edge(i)) {    //edge task
                        if (XRoutes[route_id][j] == i || XRoutes[route_id][j] == mcgrp.inst_tasks[i].inverse) {
                            delta = -mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[cur].head_node] -
                                mcgrp.min_cost[mcgrp.inst_tasks[cur].tail_node][mcgrp.inst_tasks[post].head_node] -
                                mcgrp.inst_tasks[cur].serv_cost +
                                mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[post].head_node];

                            if (delta < reserve_info.delta) {
                                reserve_info.route = route_id;
                                reserve_info.pos = j;
                                reserve_info.delta = delta;
                            }
                        }
                    }
                    else {
                        if (XRoutes[route_id][j] == i) {
                            delta = -mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[cur].head_node] -
                                mcgrp.min_cost[mcgrp.inst_tasks[cur].tail_node][mcgrp.inst_tasks[post].head_node] -
                                mcgrp.inst_tasks[cur].serv_cost +
                                mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[post].head_node];

                            if (delta < reserve_info.delta) {
                                reserve_info.route = route_id;
                                reserve_info.pos = j;
                                reserve_info.delta = delta;
                            }
                        }
                    }
                }
            }

            if (reserve_info.route == -1) {
                cout << "After route crosserover, no duplicate task!\n";
            }
            else {
                for (auto route_id : task_distribution[i]) {
                    if (route_id != reserve_info.route) {
                        if (mcgrp.is_edge(i)) {
                            auto ite = find(XRoutes[route_id].begin(), XRoutes[route_id].end(), i);
                            if (ite == XRoutes[route_id].end()) {
                                ite = find(XRoutes[route_id].begin(),
                                           XRoutes[route_id].end(),
                                           mcgrp.inst_tasks[i].inverse);
                            }
                            My_Assert(ite != XRoutes[route_id].end(), "Can't locate the task!");
                            XRoutes[route_id].erase(ite);
                        }
                        else {
                            auto ite = find(XRoutes[route_id].begin(), XRoutes[route_id].end(), i);
                            My_Assert(ite != XRoutes[route_id].end(), "Can't locate the task!");
                            XRoutes[route_id].erase(ite);
                        }
                    }
                }
            }
        }

    }

}

void Memetic::init_population(const MCGRP &mcgrp)
{
    cout << "Initialization pool...\n";
    for (auto try_count = 0; try_count < 3 * popSize && population.size() < popSize; try_count++) {
        //initialize a solution sequence for local search
        Individual buffer;
        nearest_scanning(mcgrp, buffer);

        /* local search on construct solution */
        ns.unpack_seq(buffer.sequence, mcgrp);
        ns.delimiter_coding_sol = get_delimiter_coding(ns.negative_coding_sol);
        ns.neighbor_search(mcgrp);

        if (!is_used(ns.cur_solution_cost, SIMILARITY::COST_DISTANCE)) {
            population.push_back(ns.negative_coding_sol);
            indi_obj.push_back(ns.cur_solution_cost);

            cout << population.size() << "th individual has been created!\n";
        }
    }

    popSize = population.size();
    cout << "Pool has been generated successfully!\n";
}

bool Memetic::is_used(const double tested_value, SIMILARITY sim)
{
    My_Assert(sim == SIMILARITY::COST_DISTANCE, "Uknown similarity metric!");
    for (auto obj : indi_obj) {
        if (almost_equal(tested_value, obj)) {
            return true;
        }
    }
    return false;
}

void Memetic::initDistMatrix(const MCGRP &mcgrp)
{
    distMatrix.resize(popSize);
    for (auto i = 0; i < distMatrix.size(); i++) {
        distMatrix[i].resize(popSize);
    }

    double par_dist;
    for (auto row = 0; row < distMatrix.size(); ++row) {
        for (auto col = row; col < distMatrix[row].size(); ++col) {
            if (col == row) {
                distMatrix[row][col] = 0;
            }
            par_dist = hamming_dist(mcgrp, population[row], population[col]);

            //This is a symmetric matrix
            distMatrix[row][col] = par_dist;
            distMatrix[col][row] = par_dist;
        }
    }
}

double Memetic::hamming_dist(const MCGRP &mcgrp, const vector<int> &a, const vector<int> &b)
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



/*
 * High Speed Memetic
 */

HighSpeedMemetic::HighSpeedMemetic(HighSpeedNeighBorSearch &local_search, const int pool_size, const int evolve_steps, int _QNDF_weight)
    : hardware_threads(thread::hardware_concurrency()), thread_pool(hardware_threads - 1)
{
    QNDF_weight = _QNDF_weight;
    popSize = pool_size;
    evolve_num = evolve_steps;
}


void HighSpeedMemetic::memetic_search(const MCGRP &mcgrp)
{
    struct timeb search_start_time;
    struct timeb cur_time;
    ftime(&search_start_time);
    ftime(&cur_time);

    //start initializing the pool
    init_population(mcgrp);

    if (population.size() == 1) {
        DEBUG_PRINT("Pop Size is 1\nToggle to Local Search...");
        HighSpeedNeighBorSearch ns(mcgrp);
        for (auto iter = 1; iter <= evolve_num
        && get_time_difference(search_start_time,cur_time)< search_time ; iter++) {
            ftime(&cur_time);
            cout << iter << "th times local search\n";
            ns.clear();
            ns.unpack_seq(get_delimiter_coding(population.front().solution), mcgrp);

            ns.neighbor_search(mcgrp);

            population.front().solution = get_negative_coding(ns.get_solution());
            population.front().obj = ns.get_cur_cost();
        }
    }
    else {
        DEBUG_PRINT("Pop Size is " + to_string(population.size()));
        DEBUG_PRINT("Evolution Search...");

        //initialize the hamming distance matrix of the pooling solution
        initDistMatrix(mcgrp,HAMMING_DISTANCE);

        clear_buffer();

        for (auto iter = 0; get_time_difference(search_start_time,cur_time) < search_time ;) {
            ftime(&cur_time);

            unique_lock<mutex> citizen_buffer_lock(buffer_lock);

            if(thread_pool.activate_thread_num + iter < evolve_num){
                //spawn several threads if needed, lock meantime avoiding deadlocking
                auto left_allowed_thread = min(thread_pool.max_allowed_activated_thread,evolve_num - iter);

                while(thread_pool.activate_thread_num < left_allowed_thread){
                    /* choose parent randomly */
                    auto parents_id = choose_parents(mcgrp);
                    auto parent1_id = parents_id.first;
                    auto parent2_id = parents_id.second;

                    if(population[parent2_id].obj < population[parent1_id].obj){
                        auto tmp = parent1_id;
                        parent1_id = parent2_id;
                        parent2_id = tmp;
                    }

                    auto child = route_based_crossover(mcgrp,population[parent1_id].solution,population[parent2_id].solution);
                    My_Assert(child.size() == mcgrp.actual_task_num - mcgrp.req_edge_num,"Wrong served number!");

                    //spawn a new thread
                    thread new_thread(bind(&HighSpeedMemetic::search_child, this,ref(mcgrp),child));
                    new_thread.detach();
                    thread_pool.total_threads_num++;
                    thread_pool.activate_thread_num++;
                }
            }

            data_cond.wait(citizen_buffer_lock,[this] {return !citizen_buffer.empty();});

            while (!citizen_buffer.empty()){
                UNIT immigrant = citizen_buffer.front();
                citizen_buffer.pop();

                cout << iter + 1 << "th times evolution\n";

                poolUpdate(mcgrp, immigrant.solution, immigrant.obj);
                iter++;
            }

            citizen_buffer_lock.unlock();

        }

        unique_lock<mutex> citizen_buffer_lock(buffer_lock);
        data_cond.wait(citizen_buffer_lock,[this] {return thread_pool.activate_thread_num == 0;});
    }
}

pair<int,int> HighSpeedMemetic::choose_parents(const MCGRP &mcgrp){
    int randNo1 = mcgrp._rng.Randint(0, population.size() - 1);
    int randNo2 = mcgrp._rng.Randint(0, population.size() - 1);
    while (randNo2 == randNo1)
        randNo2 = mcgrp._rng.Randint(0, population.size() - 1);

    return make_pair(randNo1,randNo2);
}


bool HighSpeedMemetic::poolUpdate(const MCGRP &mcgrp, const vector<int> new_sol, const double new_obj)
{
    vector<double> distance_to_other_sol;
    for (int i = 0; i < population.size(); ++i)
        distance_to_other_sol.push_back(hamming_dist(mcgrp, new_sol, population[i].solution));

    //consider new_sol too
    vector<POPOrder> objVec(population.size() + 1);
    vector<POPOrder> distVec(population.size() + 1);
    for (int i = 0; i < population.size(); ++i) {
        objVec[i].posi = i;
        objVec[i].val = population[i].obj;
    }

    //child info
    objVec[population.size()].posi = population.size();
    objVec[population.size()].val = new_obj;

    sort(objVec.begin(), objVec.end(), POPOrderAscent);


    for (int i = 0; i < population.size(); ++i) {
        distVec[i].posi = i;
        distVec[i].val = 0;
        for (int j = 0; j < i; ++j)
            distVec[i].val += distMatrix[j][i];
        for (int j = i + 1; j < population.size(); ++j)
            distVec[i].val += distMatrix[i][j];
        distVec[i].val = distVec[i].val * 1.0 / population.size();
    }

    //child info
    distVec[population.size()].posi = population.size();
    distVec[population.size()].val = 0;
    for (auto dist : distance_to_other_sol) {
        distVec[population.size()].val += dist;
    }
    distVec[population.size()].val = distVec[population.size()].val * 1.0 / population.size();


    sort(distVec.begin(), distVec.end(), POPOrderDescent);


    vector<double> quality_distance_fitness(population.size() + 1);

    for (int i = 0; i <= population.size(); ++i)
    {
        quality_distance_fitness[objVec[i].posi] += QNDF_weight * i;
        quality_distance_fitness[distVec[i].posi] += (1 - QNDF_weight) * i;
    }

    double max_qdf = numeric_limits<decltype(max_qdf)>::min();
    int max_qdf_id = -1;
    for (int i = 0; i <= population.size(); ++i) {
        if (quality_distance_fitness[i] > max_qdf) {
            max_qdf = quality_distance_fitness[i];
            max_qdf_id = i;
        }
    }

    My_Assert(max_qdf_id != -1, "Cannot find the maximum qdf!");

    if (max_qdf_id != population.size()) {
        population[max_qdf_id].solution = new_sol;
        population[max_qdf_id].obj = new_obj;
        for (int i = 0; i < population.size(); ++i) {
            if (max_qdf_id == i) {
                distMatrix[max_qdf_id][i] = 0;
            }
            else {
                distMatrix[max_qdf_id][i] = distance_to_other_sol[i];
                distMatrix[i][max_qdf_id] = distMatrix[max_qdf_id][i];
            }
        }
        DEBUG_PRINT("child replace ancestor "+to_string(max_qdf_id));
        return true;
    }
    else {
        DEBUG_PRINT("child cannot replace ancestors. :(");
        return false;
    }
}

void HighSpeedMemetic::merge_split_repair(const MCGRP &mcgrp,SOLUTION& solution, vector<int> Omitted_task){
    vector<int> task_set;
    for(auto task : Omitted_task){
        if(mcgrp.is_edge(task) && task > mcgrp.inst_tasks[task].inverse){
            continue;
        }

        task_set.push_back(task);
    }

    vector<int> merge_sequence;
    Individual res;
    double fitness;
    double buffer;


    /*-----------------Nearest L2 distance merge policy--------------------------*/
    merge_sequence = nearest_growing(mcgrp, task_set, mcgrp.capacity);
    Individual nearest_L2_indi;
    nearest_L2_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    res = nearest_L2_indi;
    fitness = res.total_cost;
    /*-----------------Nearest L2 distance merge policy--------------------------*/

    /*-----------------Furthest L2 distance merge policy--------------------------*/
    merge_sequence = nearest_depot_growing(mcgrp, task_set,mcgrp.capacity);
    Individual nearest_depot_indi;
    nearest_depot_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = nearest_depot_indi.total_cost;
    if (buffer < fitness) {
        res = nearest_depot_indi;
        fitness = buffer;
    }
    /*-----------------Furthest L2 distance merge policy--------------------------*/

    /*-----------------Max yield merge policy--------------------------*/
    merge_sequence = maximum_yield_growing(mcgrp, task_set,mcgrp.capacity);
    Individual maximum_yield_indi;
    maximum_yield_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = maximum_yield_indi.total_cost;
    if (buffer < fitness) {
        res = maximum_yield_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/

    /*-----------------Min yield merge policy--------------------------*/
    merge_sequence = minimum_yield_growing(mcgrp, task_set,mcgrp.capacity);
    Individual minimum_yield_indi;
    minimum_yield_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = minimum_yield_indi.total_cost;
    if (buffer < fitness) {
        res = minimum_yield_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/

    /*-----------------mixtured merge policy--------------------------*/
    merge_sequence = mixtured_growing(mcgrp, task_set,mcgrp.capacity);
    Individual mixtured_indi;
    mixtured_indi = mcgrp.parse_delimiter_seq(merge_sequence);
    buffer = mixtured_indi.total_cost;
    if (buffer < fitness) {
        res = mixtured_indi;
        fitness = buffer;
    }
    /*-----------------Max yield merge policy--------------------------*/


    if(!res.sequence.empty()){
        My_Assert(res.sequence.front() == DUMMY,"Wrong res");
        double load = 0;
        double cost = 0;

        ROUTE first_new_route;
        first_new_route.seq = {DUMMY};
        solution.routes.push_back(first_new_route);

        for(auto i = 1;i<res.sequence.size();i++){
            if(res.sequence[i] == DUMMY){
                //create a new route
                solution.routes.back().length += mcgrp.min_cost[mcgrp.inst_tasks[solution.routes.back().seq.back()].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
                solution.routes.back().seq.push_back(DUMMY);
                solution.length += solution.routes.back().length;
                solution.load += solution.routes.back().load;

                if(i == res.sequence.size() -1){
                    ROUTE new_route;
                    new_route.seq = {DUMMY};
                    solution.routes.push_back(new_route);
                }
            }
            else{
                solution.routes.back().length += mcgrp.min_cost[mcgrp.inst_tasks[solution.routes.back().seq.back()].tail_node][mcgrp.inst_tasks[res.sequence[i]].head_node];
                solution.routes.back().seq.push_back(res.sequence[i]);
                solution.routes.back().load += mcgrp.inst_tasks[res.sequence[i]].demand;
            }
        }
    }

    My_Assert(valid(mcgrp,solution),"Wrong result!");

}

void HighSpeedMemetic::best_insert_repair(const MCGRP &mcgrp,SOLUTION &solution, vector<int> Omitted_task){
    struct Insert
    {
        int InsertedTask;
        int InsertRouteID;
        int InsertPos;
        int InsertDelta;
    };


    //Now we find the best place to insert the omitted tasks
    for (auto omitted_task : Omitted_task) {
        if (!mcgrp.is_edge(omitted_task) || (omitted_task < mcgrp.inst_tasks[omitted_task].inverse)) {
            vector<Insert> CandInsertions;

            double best_delta = numeric_limits<decltype(best_delta)>::max();

            for (int route_id = 0; route_id < solution.routes.size(); route_id++) {
                // check feasibility
                if (solution.routes[route_id].load + mcgrp.inst_tasks[omitted_task].demand > mcgrp.capacity)
                    continue;

                for (int cursor = 1; cursor < solution.routes[route_id].seq.size(); cursor++) {
                    const auto a = solution.routes[route_id].seq[cursor - 1];
                    const auto b = solution.routes[route_id].seq[cursor];
                    const auto ab = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[b].head_node];
                    const auto
                        ao = mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[omitted_task].head_node];
                    const auto
                        ob = mcgrp.min_cost[mcgrp.inst_tasks[omitted_task].tail_node][mcgrp.inst_tasks[b].head_node];

                    double delta = -ab + ao + mcgrp.inst_tasks[omitted_task].serv_cost + ob;

                    if (delta < best_delta) {
                        best_delta = delta;
                        CandInsertions.clear();

                        Insert temp;
                        temp.InsertedTask = omitted_task;
                        temp.InsertRouteID = route_id;
                        temp.InsertPos = cursor;
                        temp.InsertDelta = delta;
                        CandInsertions.push_back(temp);
                    }
                    else if (almost_equal(delta, best_delta)) {
                        Insert temp;
                        temp.InsertedTask = omitted_task;
                        temp.InsertRouteID = route_id;
                        temp.InsertPos = cursor;
                        temp.InsertDelta = delta;
                        CandInsertions.push_back(temp);
                    }

                    if (mcgrp.is_edge(omitted_task)) {
                        int inverse_id = mcgrp.inst_tasks[omitted_task].inverse;
                        const auto ao_tilde =
                            mcgrp.min_cost[mcgrp.inst_tasks[a].tail_node][mcgrp.inst_tasks[inverse_id].head_node];
                        const auto o_tildeb =
                            mcgrp.min_cost[mcgrp.inst_tasks[inverse_id].tail_node][mcgrp.inst_tasks[b].head_node];

                        delta = -ab + ao_tilde + mcgrp.inst_tasks[inverse_id].serv_cost + o_tildeb;

                        if (delta < best_delta) {
                            best_delta = delta;
                            CandInsertions.clear();

                            Insert temp;
                            temp.InsertedTask = inverse_id;
                            temp.InsertRouteID = route_id;
                            temp.InsertPos = cursor;
                            temp.InsertDelta = delta;
                            CandInsertions.push_back(temp);
                        }
                        else if (almost_equal(delta, best_delta)) {
                            Insert temp;
                            temp.InsertedTask = inverse_id;
                            temp.InsertRouteID = route_id;
                            temp.InsertPos = cursor;
                            temp.InsertDelta = delta;
                            CandInsertions.push_back(temp);
                        }
                    }
                }
            }


            if (CandInsertions.empty()) {
                //create a new route
                ROUTE new_route;
                new_route.seq = {DUMMY, omitted_task, DUMMY};

                new_route.length =
                    mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[omitted_task].head_node]
                        + mcgrp.inst_tasks[omitted_task].serv_cost
                        + mcgrp.min_cost[mcgrp.inst_tasks[omitted_task].tail_node][mcgrp.inst_tasks[DUMMY].head_node];

                new_route.load = mcgrp.inst_tasks[omitted_task].demand;

                solution.length += new_route.length;
                solution.load += new_route.load;
                solution.routes.push_back(new_route);

            }
            else {
                //no need to create new route
                int k = mcgrp._rng.Randint(0, CandInsertions.size() - 1);
                int route_ID = CandInsertions[k].InsertRouteID;
                int insert_pos = CandInsertions[k].InsertPos;
                int inserted_task = CandInsertions[k].InsertedTask;
                auto delta = CandInsertions[k].InsertDelta;
                auto demand = mcgrp.inst_tasks[inserted_task].demand;

                solution.routes[route_ID].seq.insert(solution.routes[route_ID].seq.begin() + insert_pos, inserted_task);
                solution.routes[route_ID].load += demand;
                My_Assert(solution.routes[route_ID].load <= mcgrp.capacity, "Wrong state");
                solution.routes[route_ID].length += delta;

                solution.load += demand;
                solution.length += delta;
            }

            My_Assert(check_empty_route(mcgrp,solution), "Wrong insert!");
            My_Assert(valid(mcgrp, solution), "Wrong insert!");
        }

    }
}


HighSpeedMemetic::SOLUTION HighSpeedMemetic::unpack_seq(const MCGRP &mcgrp, const vector<int> &neg_seq){
    My_Assert(neg_seq.size() > 2, "You can't unpack an empty sequence!");

    SOLUTION sol;
    vector<vector<int>> seg;
    vector<int> buffer;
    My_Assert(neg_seq.front() < 0,"Wrong arguments");
    buffer.push_back(-neg_seq.front());
    for (int cursor = 1; cursor < neg_seq.size(); cursor++) {
        if (neg_seq[cursor] < 0) {
            seg.push_back(buffer);
            buffer.clear();
            buffer.push_back(-neg_seq[cursor]);
        }
        else {
            buffer.push_back(neg_seq[cursor]);
        }
    }

    if(!buffer.empty()){
        seg.push_back(buffer);
    }

    for(int i = 0;i<seg.size();i++){
        sol.routes.push_back(ROUTE());
        sol.routes.back().seq.push_back(DUMMY);
        sol.routes.back().seq.insert(sol.routes.back().seq.end(),seg[i].begin(),seg[i].end());
        sol.routes.back().seq.push_back(DUMMY);

        //0-a...
        sol.routes.back().length += mcgrp.inst_tasks[DUMMY].serv_cost;
        sol.routes.back().length += mcgrp.min_cost[mcgrp.inst_tasks[DUMMY].tail_node][mcgrp.inst_tasks[seg[i].front()].head_node];

        for(int j = 0;j<seg[i].size() - 1;j++){
            //...b-c...
            sol.routes.back().length += mcgrp.inst_tasks[seg[i][j]].serv_cost;
            sol.routes.back().length +=
                mcgrp.min_cost[mcgrp.inst_tasks[seg[i][j]].tail_node][mcgrp.inst_tasks[seg[i][j + 1]].head_node];
            sol.routes.back().load += mcgrp.inst_tasks[seg[i][j]].demand;
        }

        //...d-0
        sol.routes.back().length += mcgrp.inst_tasks[seg[i].back()].serv_cost;
        sol.routes.back().length +=
            mcgrp.min_cost[mcgrp.inst_tasks[seg[i].back()].tail_node][mcgrp.inst_tasks[DUMMY].head_node];
        sol.routes.back().length += mcgrp.inst_tasks[DUMMY].serv_cost;
        sol.routes.back().load += mcgrp.inst_tasks[seg[i].back()].demand;

        sol.length += sol.routes.back().length;
        sol.load += sol.routes.back().load;
    }

    My_Assert(valid(mcgrp,sol),"Wrong unpack");
    return sol;
}

bool HighSpeedMemetic::valid(const MCGRP& mcgrp, const SOLUTION &solution){
    double cost = 0;
    double total_cost = 0;
    double load = 0;
    double total_load = 0;
    for(auto route : solution.routes){
        My_Assert(route.seq.front() == DUMMY && route.seq.back() == DUMMY,"Wrong tasks");
        for(auto cur = 0;cur<route.seq.size()-1;cur++){
            cost += mcgrp.inst_tasks[route.seq[cur]].serv_cost;
            cost += mcgrp.min_cost[mcgrp.inst_tasks[route.seq[cur]].tail_node][mcgrp.inst_tasks[route.seq[cur+1]].head_node];
            load += mcgrp.inst_tasks[route.seq[cur]].demand;
        }
        cost += mcgrp.inst_tasks[route.seq.back()].serv_cost;
        load += mcgrp.inst_tasks[route.seq.back()].demand;

        if(load != route.load)
            return false;
        if(load > mcgrp.capacity)
            return false;
        if(cost != route.length)
            return false;

        total_cost += cost;
        total_load += load;
        cost = 0;
        load = 0;
    }

    return total_load == solution.load && total_cost == solution.length;
}

bool HighSpeedMemetic::check_integrity(const MCGRP &mcgrp, const SOLUTION &solution, const vector<int>& ommited){
    vector<bool> tasks(mcgrp.actual_task_num + 1, false);

    tasks.front() = true;

    for(auto route:solution.routes){
        My_Assert(route.seq.front() == DUMMY && route.seq.back() == DUMMY && route.seq.size() >= 2,"Wrong route");
        for(int j = 1;j < route.seq.size() - 1; j++){
            My_Assert(route.seq[j] != DUMMY,"Wrong state");
            if(mcgrp.is_edge(route.seq[j])){
                auto inverse = mcgrp.inst_tasks[route.seq[j]].inverse;
                if(tasks[route.seq[j]] == true || tasks[inverse] == true){
                    return false;
                }

                tasks[route.seq[j]] = true;
                tasks[inverse] = true;
            }
            else{
                if(tasks[route.seq[j]] == true){
                    return false;
                }

                tasks[route.seq[j]] = true;
            }

        }
    }


    for(auto task : ommited){
        My_Assert(task != DUMMY,"Wrong state");
        if(tasks[task] == true){
            return false;
        }

        tasks[task] = true;
    }

    for(auto flag : tasks){
        if(!flag)
            return false;
    }

    return true;
}


vector<int> HighSpeedMemetic::route_based_crossover(const MCGRP &mcgrp, const vector<int>& parent_a_seq,const vector<int>& parent_b_seq)
{
    SOLUTION parent_a = unpack_seq(mcgrp, parent_a_seq);
    SOLUTION parent_b = unpack_seq(mcgrp, parent_b_seq);

    SOLUTION child = parent_a;

    int actual_cross_num = min(cross_route_num, int(min(parent_a.routes.size(), parent_b.routes.size())));
    DEBUG_PRINT("cross routes: " + to_string(actual_cross_num));

    /* generate corresponding routes_id */
    // random policy,avoiding crossover the same route repeatly
    vector<int> parent_a_route_id(parent_a.routes.size());
    std::generate(parent_a_route_id.begin(), parent_a_route_id.end(), Generator(-1));
    mcgrp._rng.RandPerm(parent_a_route_id);

    vector<int> parent_b_route_id(parent_b.routes.size());
    std::generate(parent_b_route_id.begin(), parent_b_route_id.end(), Generator(-1));
    mcgrp._rng.RandPerm(parent_b_route_id);

    vector<int> chosen_parent_a_route_id(parent_a_route_id.begin(),parent_a_route_id.begin()+ actual_cross_num);
    vector<int> chosen_parent_b_route_id(parent_b_route_id.begin(),parent_b_route_id.begin()+ actual_cross_num);

    My_Assert(chosen_parent_a_route_id.size() == chosen_parent_b_route_id.size(),"Different size!");


    //crossover routes
    for (int i = 0;i<chosen_parent_a_route_id.size();i++) {
        child.load -= child.routes[chosen_parent_a_route_id[i]].load;
        child.load += parent_b.routes[chosen_parent_b_route_id[i]].load;

        child.length -= child.routes[chosen_parent_a_route_id[i]].length;
        child.length += parent_b.routes[chosen_parent_b_route_id[i]].length;

        child.routes[chosen_parent_a_route_id[i]] = parent_b.routes[chosen_parent_b_route_id[i]];

        My_Assert(valid(mcgrp, child),"Wrong state!");
    }


    /* repair part */
    repair_solution(mcgrp,child);
    My_Assert(valid(mcgrp, child),"Wrong state!");


    return child.get_whole_sequence();
}

void HighSpeedMemetic::repair_solution(const MCGRP &mcgrp, SOLUTION & solution){

    vector<int> Omitted_task = remove_duplicates(mcgrp, solution);
    My_Assert(check_integrity(mcgrp,solution,Omitted_task),"Wrong remove");

    mcgrp._rng.RandPerm(Omitted_task);

    best_insert_repair(mcgrp,solution, Omitted_task);

    Omitted_task.clear();
    My_Assert(check_integrity(mcgrp,solution,Omitted_task),"Wrong remove");

//    merge_split_repair(mcgrp,solution, Omitted_task);

    My_Assert(valid(mcgrp,solution),"Wrong repair");

    return;
}

vector<int> HighSpeedMemetic::remove_duplicates(const MCGRP &mcgrp, SOLUTION &solution)
{
    struct Position{
        int route = -1;
        int pos = -1;

        Position(int route_, int pos_):route(route_),pos(pos_){}
    };

    struct Reserve_Info
    {
        int route = -1;
        int pos = -1;
        // A bug will happen when you use numeric_limits<double>::min() as the minimum :(
        double delta = -10000000;
    };


    vector<Position> distribution;
    vector<int> Omitted_task;
    for (int i = 1; i <= mcgrp.actual_task_num; ++i) {
        distribution.clear();

        if(mcgrp.is_edge(i)) {
            if(i < mcgrp.inst_tasks[i].inverse){
                //avoid handle twice
                auto i_tilde = mcgrp.inst_tasks[i].inverse;

                for (int ii = 0; ii < solution.routes.size(); ++ii) {
                    for (int j = 1; j < solution.routes[ii].seq.size() - 1; ++j) {
                        auto task_id = solution.routes[ii].seq[j];
                        if(i == task_id || i_tilde == task_id){
                            distribution.push_back(Position(ii,j));
                        }
                    }
                }

                if (distribution.empty()){
                    //ommited task
                    Omitted_task.push_back(i);
                    Omitted_task.push_back(i_tilde);
                }
                else if(distribution.size() != 1){
                    My_Assert(distribution.size()>1,"Wrong task");

                    Reserve_Info reserve_info;
                    vector<double> deltas;
                    for(auto pos : distribution)
                    {
                        auto pre = solution.routes[pos.route].seq[pos.pos - 1];
                        auto cur = solution.routes[pos.route].seq[pos.pos];
                        auto post = solution.routes[pos.route].seq[pos.pos + 1];
                        My_Assert(solution.routes[pos.route].seq[pos.pos] == i || solution.routes[pos.route].seq[pos.pos] == mcgrp.inst_tasks[i].inverse,"Wrong tasks");

                        double delta =
                            - mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[cur].head_node]
                            - mcgrp.inst_tasks[cur].serv_cost
                            - mcgrp.min_cost[mcgrp.inst_tasks[cur].tail_node][mcgrp.inst_tasks[post].head_node]
                            + mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[post].head_node];

                        if (delta > reserve_info.delta)
                        {
                            reserve_info.route = pos.route;
                            reserve_info.pos = pos.pos;
                            reserve_info.delta = delta;
                        }

                        deltas.push_back(delta);
                    }

                    My_Assert(reserve_info.route != -1,"Wrong state");
                    My_Assert(deltas.size() == distribution.size(),"Wrong state");
                    for(int ii = 0;ii<distribution.size(); ii++) {
                        if (distribution[ii].route != reserve_info.route) {
                            auto ite = solution.routes[distribution[ii].route].seq.begin() + distribution[ii].pos;
                            solution.routes[distribution[ii].route].load -= mcgrp.inst_tasks[*ite].demand;
                            solution.load -= mcgrp.inst_tasks[*ite].demand;
                            solution.routes[distribution[ii].route].length += deltas[ii];
                            solution.length += deltas[ii];
                            solution.routes[distribution[ii].route].seq.erase(ite);
                            My_Assert(valid(mcgrp,solution),"Wrong remove");
                        }
                    }
                }
                else {
                    //means existing only once in the solution
                    continue;
                }
            }
        }
        else{
            for (int ii = 0; ii < solution.routes.size(); ++ii) {
                for (int j = 1; j < solution.routes[ii].seq.size() - 1; ++j) {
                    auto task_id = solution.routes[ii].seq[j];
                    if(i == task_id){
                        distribution.push_back(Position(ii,j));
                    }
                }
            }

            if (distribution.empty()){
                //ommited task
                Omitted_task.push_back(i);
            }
            else if(distribution.size() != 1){
                //duplicated tasks
                My_Assert(distribution.size()>1,"Wrong task");

                Reserve_Info reserve_info;
                vector<double> deltas;
                for(auto pos : distribution)
                {
                    auto pre = solution.routes[pos.route].seq[pos.pos - 1];
                    auto cur = solution.routes[pos.route].seq[pos.pos];
                    auto post = solution.routes[pos.route].seq[pos.pos + 1];
                    My_Assert(solution.routes[pos.route].seq[pos.pos] == i ,"Wrong tasks");

                    double delta =
                        - mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[cur].head_node]
                        - mcgrp.inst_tasks[cur].serv_cost
                        - mcgrp.min_cost[mcgrp.inst_tasks[cur].tail_node][mcgrp.inst_tasks[post].head_node]
                        + mcgrp.min_cost[mcgrp.inst_tasks[pre].tail_node][mcgrp.inst_tasks[post].head_node];

                    if (delta > reserve_info.delta) {
                        reserve_info.route = pos.route;
                        reserve_info.pos = pos.pos;
                        reserve_info.delta = delta;
                    }

                    deltas.push_back(delta);
                }

                My_Assert(reserve_info.route != -1,"Wrong state");
                My_Assert(deltas.size() == distribution.size(),"Wrong state");

                for(int ii = 0;ii<distribution.size(); ii++) {
                    if (distribution[ii].route != reserve_info.route) {
                        auto ite = solution.routes[distribution[ii].route].seq.begin() + distribution[ii].pos;
                        solution.routes[distribution[ii].route].load -= mcgrp.inst_tasks[*ite].demand;
                        solution.load -= mcgrp.inst_tasks[*ite].demand;
                        solution.routes[distribution[ii].route].length += deltas[ii];
                        solution.length += deltas[ii];
                        solution.routes[distribution[ii].route].seq.erase(ite);
                        My_Assert(valid(mcgrp,solution),"Wrong remove");
                    }
                }
            }
            else{
                //means existing only once in the solution
                continue;
            }
        }
    }

    My_Assert(valid(mcgrp,solution),"Wrong state");
    //remove empty routes
    int cursor = 0;
    while(cursor < solution.routes.size()){
        if(solution.routes[cursor].seq.size() == 2){
            solution.routes.erase(solution.routes.begin() + cursor);
        }
        else{
            cursor++;
        }
    }
    My_Assert(valid(mcgrp,solution),"Wrong state");

    return Omitted_task;
}

bool HighSpeedMemetic::check_empty_route(const MCGRP &mcgrp,const SOLUTION &solution){
    for(auto route : solution.routes){
        if(route.seq.size() <= 2)
            return false;
    }

    return true;
}

void HighSpeedMemetic::create_citizen(const MCGRP& mcgrp){
    DEBUG_PRINT("A new thread has been created!\n");
    cout<<this_thread::get_id()<<endl;

    Individual buffer;
    nearest_scanning(mcgrp, buffer);

    HighSpeedNeighBorSearch  ns_(mcgrp);
    ns_.unpack_seq(buffer.sequence, mcgrp);
    ns_.trace(mcgrp);

//    ns_.infeasible_exploration(mcgrp);
    ns_.neighbor_search(mcgrp);
    UNIT tmp;
    tmp.obj = ns_.get_cur_cost();
    tmp.solution = get_negative_coding(ns_.get_solution());

    lock_guard<mutex> thread_pool_lock(buffer_lock);
    citizen_buffer.push(tmp);
    thread_pool.activate_thread_num--;
    thread_pool.dead_thread_num++;
    data_cond.notify_one();
}

void HighSpeedMemetic::search_child(const MCGRP &mcgrp, vector<int> child) {
    DEBUG_PRINT("A new thread has been created!\n");
    cout<<this_thread::get_id()<<endl;

    HighSpeedNeighBorSearch  ns_(mcgrp);
    ns_.unpack_seq(get_delimiter_coding(child), mcgrp);
    ns_.trace(mcgrp);

//    ns_.infeasible_exploration(mcgrp);
    ns_.neighbor_search(mcgrp);
    UNIT tmp;
    tmp.obj = ns_.get_cur_cost();
    tmp.solution = get_negative_coding(ns_.get_solution());


    lock_guard<mutex> thread_pool_lock(buffer_lock);
    citizen_buffer.push(tmp);
    thread_pool.activate_thread_num--;
    thread_pool.dead_thread_num++;
    data_cond.notify_one();
}


void HighSpeedMemetic::init_population(const MCGRP &mcgrp)
{
    cout << "Initialization pool...\n";
    struct timeb start_time;
    ftime(&start_time);

    int max_allowed_thread = min(thread_pool.max_allowed_activated_thread, popSize) ;

    for (auto try_count = 0; try_count < 2 * popSize && population.size() < popSize;) {

        unique_lock<mutex> citizen_buffer_lock(buffer_lock);
        if(citizen_buffer.empty() && thread_pool.activate_thread_num + population.size() < popSize){
            //spawn several threads if needed, lock meantime avoiding deadlocking

//            lock_guard<mutex> thread_pool_lock(thread_pool.thread_info_lock);

            while(thread_pool.activate_thread_num < max_allowed_thread){
                thread new_thread(bind(&HighSpeedMemetic::create_citizen, this,ref(mcgrp)));
                new_thread.detach();
                thread_pool.total_threads_num++;
                thread_pool.activate_thread_num++;
            }

//            citizen_buffer_lock.unlock();

        }

        data_cond.wait(citizen_buffer_lock,[this] {return !citizen_buffer.empty();});

        while (!citizen_buffer.empty()){
            UNIT immigrant = citizen_buffer.front();
            citizen_buffer.pop();

            if (!repeated(mcgrp, immigrant.obj, immigrant.solution, SIMILARITY::COST_DISTANCE)) {
                population.push_back(immigrant);
                cout << population.size() << "th individual has been created!\n";
            }
            try_count++;
        }
        citizen_buffer_lock.unlock();

        /*use future may not be a good choice
//        packaged_task<vector<int>(const MCGRP&)> spawn_citizen(bind(&HighSpeedMemetic::create_citizen, this,std::placeholders::_1));
//        future<vector<int>> fu = spawn_citizen.get_future();

//        thread t1(std::move(spawn_citizen), ref(mcgrp));
//        t1.detach();
//        auto immigrant = fu.get();
        /*-----------------------------------*/

        /* local search on construct solution
        //single thread way
        //initialize a solution sequence for local search
//        Individual buffer;
//        nearest_scanning(mcgrp, buffer);
//        ns.clear();
//        ns.unpack_seq(buffer.sequence, mcgrp);
//        ns.trace(mcgrp);
//        ns.infeasible_exploration(mcgrp);
//        ns.neighbor_search(mcgrp);
//        auto immigrant = get_negative_coding(ns.get_solution());
        /*------------------------------------*/
    }



    struct timeb end_time;
    ftime(&end_time);
    cout << "Pool has been generated successfully!\n Spent: "
         <<fixed << (end_time.time - start_time.time)
             + ((end_time.millitm - start_time.millitm) * 1.0 / 1000) << 's' << endl;
}

bool HighSpeedMemetic::repeated(const MCGRP &mcgrp, const double tested_value, const vector<int> &immigrant, SIMILARITY sim)
{
    if(sim == SIMILARITY::COST_DISTANCE){
        for (auto citizen : population) {
            if (almost_equal(tested_value, citizen.obj)) {
                return true;
            }
        }
        return false;
    }
    else if (sim == SIMILARITY::HAMMING_DISTANCE){
        for (auto citizen : population) {
            if(hamming_dist(mcgrp,immigrant, citizen.solution) < 10)
                return true;
        }
        return false;
    }
    else{
        My_Assert(false, "Uknown similarity metric!");
    }

    My_Assert(false, "Cannot reach here");
}

void HighSpeedMemetic::initDistMatrix(const MCGRP &mcgrp, SIMILARITY sim)
{
    distMatrix.resize(population.size());
    for (auto i = 0; i < distMatrix.size(); i++) {
        distMatrix[i].resize(population.size());
    }

    for (auto row = 0; row < distMatrix.size(); ++row) {
        for (auto col = row; col < distMatrix[row].size(); ++col) {
            if (col == row) {
                distMatrix[row][col] = 0;
            }

            double par_dist = numeric_limits<double>::max();
            switch (sim){
                case HAMMING_DISTANCE:
                    par_dist = hamming_dist(mcgrp, population[row].solution, population[col].solution);
                    break;
                case COST_DISTANCE:
                    par_dist = abs(population[row].obj - population[col].obj);
                    break;
                default:
                    My_Assert(false,"Unknown Similarity!");
            }

            //This is a symmetric matrix
            distMatrix[row][col] = par_dist;
            distMatrix[col][row] = par_dist;
        }
    }
}

