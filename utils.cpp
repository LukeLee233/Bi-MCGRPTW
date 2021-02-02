#include "utils.h"
#include <iostream>
#include <array>
#include "NeighborSearch.h"
#include <boost/filesystem.hpp>

using namespace std;

ofstream result_out;
ofstream log_out;

void __My_Assert(const char *expr_str, bool expr, const char *file, int line, const char *msg)
{
    if (!expr) {
        std::cerr << "Assert failed:\t" << msg << "\n"
                  << "Expected:\t" << expr_str << "\n"
                  << "Source:\t\t" << file << ", line " << line << "\n";
        abort();
    }
}

vector<int> get_negative_coding(const vector<int> &sequence)
{
    int sign = 1;
    vector<int> negative_coding_sol;
    negative_coding_sol.clear();

    for (auto task : sequence) {
        if (task == DUMMY) {
            sign = -1;
        }
        else {
            negative_coding_sol.push_back(sign * task);
            sign = 1;
        }
    }

    return negative_coding_sol;
}

vector<int> get_delimiter_coding(const vector<int> &negative_coding)
{
    vector<int> delimiter_coding;
    delimiter_coding.clear();
    for (auto task:negative_coding) {
        if (task < 0) {
            delimiter_coding.push_back(DUMMY);
        }
        delimiter_coding.push_back(abs(task));
    }
    delimiter_coding.push_back(DUMMY);

    return delimiter_coding;
}

std::vector<std::string> read_directory(const std::string dir_path)
{
    cout<<"Reading directory...\n";
    using namespace boost::filesystem;
    std::vector<std::string> buffer;

    path p(dir_path);
    std::vector<directory_entry> v;
    if(is_directory(p)){
        copy(directory_iterator(p),directory_iterator(),back_inserter(v));
        for(std::vector<directory_entry>::const_iterator it = v.begin(); it != v.end(); ++it){
            if(is_regular_file(*it))
            {
                buffer.push_back((*it).path().filename().string());
            }
        }
    }

    return buffer;
}

bool print(std::ostream &os1, std::ostream &os2, const std::string &str)
{
    if(os1 << str << endl && os2 << str << endl){
        return true;
    }

    return false;
}

bool print(std::ostream &os1, const std::string &str)
{
    if(os1 << str << endl){
        return true;
    }

    return false;
}

//double sum(const vector<double> &vec)
//{
//    double tmp = 0;
//    for (auto &n: vec) {
//        tmp += n;
//    }
//    return tmp;
//}

vector<int> find_ele_positions(const std::vector<int> &seq, int target)
{
    vector<int> buffer;
    for (int i = 0; i < seq.size(); i++) {
        if (seq[i] == target) {
            buffer.push_back(i);
        }
    }

    return buffer;
}

double sel_ratio(const vector<double> &_prob, const vector<double> &ratio, RNG &rng)
{
    double rd = rng.Randfloat(0, 1);
    double fsum = 0;
    for (int i = 0; i < _prob.size(); ++i) {
        fsum += _prob[i];
        if (rd < fsum) {
            return ratio[i];
        }
    }

    My_Assert(false, "ERROR! Cannot select a ratio!");
}
