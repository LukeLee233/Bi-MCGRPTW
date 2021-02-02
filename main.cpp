#include <iostream>
#include <sys/timeb.h>
#include "file.h"
#include "utils.h"
#include "MCGRP.h"
#include "RNG.h"
#include "NeighborSearch.h"
#include "Memetic.h"
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include "ConstructPolicy.h"
#include "SingleInsert.h"
#include "DoubleInsert.h"
#include "Slice.h"
#include "Extraction.h"
#include "Swap.h"
#include "Invert.h"
#include "TwoOpt.h"
#include <boost/program_options.hpp>
#include <algorithm>
#include <numeric>
#include "json.hpp"
#include "config.h"

using namespace std;
namespace bpo = boost::program_options;
using json = nlohmann::json;

struct timeb phase_start_time;
struct timeb cur_time;


int main(int argc, char *argv[])
{
    /*-------------------------parse command line----------------------------*/
    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
        ("help", "MCGRP program")
        ("directory,dir", bpo::value<std::string>(&instance_directory), "the instance directory")
        ("config_file,config", bpo::value<string>(&config_file)->default_value(""), "parameter_file")
        ("freeze_config", bpo::value<bool>(&freeze_config)->default_value(false), "whether to record the parameters")
        ("search_time", bpo::value<int>(&search_time)->default_value(60), "total search time")
        ("neighbor_search_mode", bpo::value<std::string>(&neighbor_search_mode)->default_value("random"), "select mode in neighbor search")
        ("significant_search", bpo::value<int>(&significant_search)->default_value(5), "if valid move steps doesn't less than this, descent search will be terminated")
        ("local_ratio", bpo::value<double>(&local_ratio)->default_value(0.8), "if the percentage of equal move larger than this number, descent search will be terminated")
        ("max_RTR_search_cycle", bpo::value<int>(&max_RTR_search_cycle)->default_value(50), "max allowed search time within a RTR search")
        ("local_minimum_threshold", bpo::value<int>(&local_minimum_threshold)->default_value(40), "determine whether we arrived at a local optimal")
        ("tabu_step", bpo::value<int>(&tabu_step)->default_value(500), "tabu steps in the ascent search")
        ("infeasible_distance", bpo::value<double>(&infeasible_distance)->default_value(0.3), "the parameter determine the distance between infeasible region and feasible region")
        ("pool_size,ps", bpo::value<int>(&pool_size)->default_value(5), "size of population")
        ("evolve_step,es", bpo::value<int>(&evolve_steps)->default_value(5), "number of evolving step")
        ("phase_number,np", bpo::value<int>(&phase_number)->default_value(1))
        ("random_seed,rs", bpo::value<int>(&random_seed)->default_value(0), "number of evolving step")
        ("neighbor_size,ns", bpo::value<int>(&neighbor_size)->default_value(25), "neighbor size of the local search")
        ("qndf_weights,qs", bpo::value<double>(&QNDF_weights)->default_value(0.6), "weight of solution distance");

    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
        std::cout << opts << std::endl;
        return 0;
    }

    if (instance_directory.empty()) {
        std::cerr << "you need to specify a directory!\n";
        return -1;
    }

    if (!config_file.empty()) {
        json j;
        ifstream fin(config_file);
        fin >> j;

        j.at("pool_size").get_to(pool_size);
        j.at("evolve_steps").get_to(evolve_steps);
        j.at("phase_number").get_to(phase_number);
        j.at("random_seed").get_to(random_seed);
        j.at("neighbor_size").get_to(neighbor_size);
        j.at("search_time").get_to(search_time);
        j.at("QNDF_weights").get_to(QNDF_weights);
        j.at("neighbor_search_mode").get_to(neighbor_search_mode);
        j.at("significant_search").get_to(significant_search);
        j.at("local_ratio").get_to(local_ratio);
        j.at("max_RTR_search_cycle").get_to(max_RTR_search_cycle);
        j.at("local_minimum_threshold").get_to(local_minimum_threshold);
        j.at("tabu_step").get_to(tabu_step);
        j.at("infeasible_distance").get_to(infeasible_distance);
    }

    if (random_seed < 0 || random_seed > 100) {
        std::cerr << "Incorrect random seed base address!\n";
        return -1;
    }

    if (freeze_config) {
        json j;
        j["pool_size"] = pool_size;
        j["evolve_steps"] = evolve_steps;
        j["phase_number"] = phase_number;
        j["random_seed"] = random_seed;
        j["neighbor_size"] = neighbor_size;
        j["QNDF_weights"] = QNDF_weights;

        config_file = config_file.empty() ? "demo.json" : config_file;
        ofstream fout(config_file);
        fout << setw(4) << j << endl;
    }

    phase_number = (phase_number == -1) ? 1e6 : phase_number;
    evolve_steps = (evolve_steps == -1) ? 1e6 : evolve_steps;
    /*----------------------------------------------------------------*/

    /*----------------------Create search info log----------------------*/
    //create log folder
    string log_dir = instance_directory + "/log";
    boost::filesystem::path log_dir_path(log_dir);
    if (!(boost::filesystem::exists(log_dir_path))) {
        cout << "Create log directory..." << endl;
        if (boost::filesystem::create_directory(log_dir_path)) {
            cout << "Log Directory created successfully!\n\n";
        }
    }
    else {
        cout << "Log directory already exists!\n\n";
    }

    // create result folder
    boost::posix_time::ptime time_stamp = boost::posix_time::second_clock::local_time();
    stringstream buffer;
    string date_;
    buffer << time_stamp;
    date_ = buffer.str();
    string date_folder = instance_directory + "/log/" + date_;
    boost::filesystem::path date_dir_path(date_folder);
    if (!(boost::filesystem::exists(date_dir_path))) {
        cout << "Create" << date_ << " directory..." << endl;
        if (boost::filesystem::create_directory(date_dir_path)) {
            cout << "Directory created successfully!\n\n";
        }
    }
    else {
        cout << "Result directory already exists!\n\n";
    }

    // create result folder
    string result_dir = date_folder + "/result";
    boost::filesystem::path result_dir_path(result_dir);
    if (!(boost::filesystem::exists(result_dir_path))) {
        cout << "Create result directory..." << endl;
        if (boost::filesystem::create_directory(result_dir_path)) {
            cout << "Directory created successfully!\n\n";
        }
    }
    else {
        cout << "Result directory already exists!\n\n";
    }

/*----------------------Create info recorder----------------------*/
//  record searching parameters
    log_out.open(date_folder + "/parameters.txt", ios::out);
    print(cout, log_out, "Parameters settings:");
    print(cout, log_out, "directory " + instance_directory);
    print(cout, log_out, "base address of seeds: " + to_string(random_seed));
    print(cout, log_out, "search times: " + to_string(phase_number));
    print(cout, log_out, "evolution times: " + to_string(evolve_steps));
    print(cout, log_out, "pool size: " + to_string(pool_size));
    print(cout, log_out, "neighbor_size: " + to_string(neighbor_size));
    print(cout, log_out, "QNDF weights: " + to_string(QNDF_weights));
    log_out.close();
/*----------------------------------------------------------------*/


    vector<string> file_set = read_directory(instance_directory);
    for (auto file_name : file_set) {
        cout << string(2,'\n') << string (24,'-')
             << "Start instance: "
             << file_name
             << string (24,'-') << string(2,'\n')
             << flush;

        /* global info */
        double bestobj = numeric_limits<decltype(bestobj)>::max();
        double best_solution_time = numeric_limits<decltype(best_solution_time)>::max();
        int best_phase = -1;
        vector<int> best_buffer;
        vector<double> FitnessVec;
        vector<double> BestTimeVec;
        vector<double> SearchTimeVec;
        vector<vector<int>> SolutionVec;

        /************************************************************************************************/
        /* initialize the instance object */
        instance_num_information instance_info;
        GetTasksNum(instance_directory + '/' + file_name, instance_info);

        RNG rng = RNG();
        MCGRP Mixed_Instance(instance_info, rng);

        Mixed_Instance.load_file_info(instance_directory + '/' + file_name, instance_info);
        Mixed_Instance.create_neighbor_lists(neighbor_size);

#ifdef DEBUG
        log_out.open(date_folder + '/' + file_name + ".log", ios::out);
#endif
        struct timeb search_start_time;
        ftime(&search_start_time);
        ftime(&cur_time);
        for (int start_seed = random_seed; start_seed < random_seed + phase_number
        && get_time_difference(search_start_time,cur_time)<search_time; start_seed++) {
            cout << string (24,'-') << "Start "
                 << start_seed - random_seed + 1
                 << "th times search" << string (24,'-') << endl;

            Mixed_Instance._rng.change(seed[start_seed % seed_size]);
            Mixed_Instance.best_total_route_length = numeric_limits<double>::max();
            Mixed_Instance.best_sol_time = numeric_limits<double>::max();

            HighSpeedNeighBorSearch NBS(Mixed_Instance);

            /*----------------------------------------------------------*/
            cout << "Begin Memetic search..." << endl;
            HighSpeedMemetic MA(NBS, pool_size, evolve_steps, QNDF_weights);
            ftime(&phase_start_time);
            MA.memetic_search(Mixed_Instance);
            ftime(&cur_time);
            cout << "Finish " << start_seed - random_seed << "th search, spent: "
                 << get_time_difference(phase_start_time,cur_time) << 's' << endl;

            /*----------------------------------------------------------*/

            /* solution record */
            ftime(&cur_time);
            SearchTimeVec.push_back(get_time_difference(phase_start_time,cur_time));

            /* record the best info of each epoch */
            FitnessVec.push_back(Mixed_Instance.best_total_route_length);
            BestTimeVec.push_back(Mixed_Instance.best_sol_time);
            SolutionVec.push_back(Mixed_Instance.best_sol_buff);

            cout << "Finish " << start_seed - random_seed + 1 << "th times\n";

            /* record the best solution during the whole searching process*/
            if (Mixed_Instance.best_total_route_length < bestobj) {
                best_solution_time = Mixed_Instance.best_sol_time;
                bestobj = Mixed_Instance.best_total_route_length;
                best_buffer = Mixed_Instance.best_sol_buff;
                best_phase = start_seed - random_seed + 1;
            }

        }

#ifdef DEBUG
        log_out.close();
#endif

        /***********************************output part********************************************/
        if (bestobj == numeric_limits<decltype(bestobj)>::max()) {
            cerr << "ERROR! Can't find a solution\n";
            abort();
        }
        double total_cost = accumulate(FitnessVec.begin(), FitnessVec.end(), 0);
        double best_total_time = accumulate(BestTimeVec.begin(), BestTimeVec.end(), 0);
        double total_time = accumulate(SearchTimeVec.begin(), SearchTimeVec.end(), 0);
        double average_cost = total_cost / FitnessVec.size();
        double average_best_time = best_total_time / BestTimeVec.size();
        double average_search_time = total_time / SearchTimeVec.size();

        cout << string(2,'\n') << string(24,'*')
             << "Searching Finish!" << string(24,'*') << endl;
        cout << "Searching result:"<<string(2,'\n') << flush;


        result_out.open(result_dir + '/' + file_name + ".res", ios::out);
        result_out << setprecision(2) << fixed;

        print(cout, result_out, "Instance: " + file_name);
        print(cout, result_out, "Best Cost: " + to_string(bestobj));
        print(cout, result_out, "Best Cost Epoch: " + to_string(best_phase));
        print(cout, result_out, "Best Solution Search Time: " + to_string(best_solution_time) + 's');
        print(cout, result_out, "Best Solution: ");

        string buff = "";
        for (int i = 0; i < best_buffer.size(); i++) {
            if (i == best_buffer.size() - 1)
                buff += to_string(best_buffer[i]);
            else
                buff += (to_string(best_buffer[i]) + "->");
        }
        print(cout, result_out, buff);

        print(cout, result_out, "\n\nSearch time: " + to_string(FitnessVec.size()));


        print(cout, result_out, "\n\nAverage cost: " + to_string(average_cost));
        print(cout, result_out, "Average best solution search time: " + to_string(average_best_time) + 's');
        print(cout, result_out, "Average search time: " + to_string(average_search_time) + 's');
        result_out.close();
    }

    return 0;
}