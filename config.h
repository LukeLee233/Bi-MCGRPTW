//
// Created by luke on 2020/9/4.
//

#ifndef CONFIG_H
#define CONFIG_H
#include <string>

extern std::string instance_directory;
extern std::string config_file;
extern bool freeze_config;

extern int search_time;

extern int pool_size;
extern int evolve_steps;
extern int phase_number;
extern int random_seed;
extern int neighbor_size;
extern double QNDF_weights;

extern std::string neighbor_search_mode;

// Descent Local Search(determine whether we can do down further)
extern int significant_search;
extern double local_ratio;

// Determine whether we arrive at a local minimum
extern int max_RTR_search_cycle;
extern int local_minimum_threshold;

extern int tabu_step;

// Infeasible Local Search
extern double infeasible_distance;


#endif //CONFIG_H
