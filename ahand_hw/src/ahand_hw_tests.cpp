#include "ahand_hw/ahand_filters.h"
#include <iostream>
#include <random>

void print_buffer(std::vector<double>& v){
    for(std::size_t i = 0; i < v.size();i++){
        std::cout<< v[i] << " ";
    }
    std::cout<<std::endl;
}

int main(int argc, char** argv){

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0,1};


    filters::SavitzkyGolay sgfilter(3, 2);
    double x = 0;
    for(int i = 0; i < 100; i++){
        x = std::sin(static_cast<double>(i)/50.0);
        std::cout<< "x: " << x << std::endl;
        sgfilter.update(x);
        std::cout<< "c: " << sgfilter.c << std::endl;
    }



    return 0;
}