#include "file.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

void GetTasksNum(std::string filename, instance_num_information &number_info)
{
    FILE *infile;

    infile = fopen(filename.c_str(), "r");
    if (infile == NULL) {
        fprintf(stderr, "Unable to open %s for reading\n", filename.c_str());
        exit(-1);
    }
    if (feof(infile)) {
        fprintf(stderr, "It's an empty file %s\nExiting...\n", filename.c_str());
        exit(-1);
    }

    char dummy_string[50];
    int count = 0;

    while (true) {
        fscanf(infile, "%s", dummy_string);
        if (strcmp(dummy_string, "#Required") == 0) {
            fscanf(infile, "%s", dummy_string);
            if (strcmp(dummy_string, "E:") == 0) {
                fscanf(infile, "%d", &number_info.req_edge_num);
                count++;
            }
            else if (strcmp(dummy_string, "N:") == 0) {
                fscanf(infile, "%d", &number_info.req_node_num);
                count++;
            }
            else if (strcmp(dummy_string, "A:") == 0) {
                fscanf(infile, "%d", &number_info.req_arc_num);
                count++;
            }
        }

        if (strcmp(dummy_string, "#Nodes:") == 0) {
            fscanf(infile, "%d", &number_info.node_num);
            count++;
        }
        if (strcmp(dummy_string, "#Edges:") == 0) {
            fscanf(infile, "%d", &number_info.edge_num);
            count++;
        }
        if (strcmp(dummy_string, "#Arcs:") == 0) {
            fscanf(infile, "%d", &number_info.arc_num);
            count++;
        }
        if (count == 6)
            break;
    }

    fclose(infile);
}