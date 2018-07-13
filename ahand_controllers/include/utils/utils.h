#ifndef AHAND_UTILS__FILTERS_H
#define AHAND_UTILS__FILTERS_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <regex>
#include <boost/lexical_cast.hpp>

namespace ahand{


std::vector<std::string> split(const std::string& input, const std::string& regex) {
    std::regex re(regex);
    std::sregex_token_iterator
        first{input.begin(), input.end(), re, -1},
        last;
    return {first, last};
}

template<typename T>
static std::vector<std::vector<T>> load(std::string filename, std::string delimiter=","){
    std::ifstream in;
    in.open(filename);
    std::vector<std::vector<T>> data;
    if(!in.is_open()){
        std::cerr<< "failed to open file: " << filename <<std::endl;
        return data;
    }
    std::string line;
    while(std::getline(in, line)){
        std::vector<std::string> items = split(line, delimiter);
        std::vector<T> values;
        std::transform(items.begin(), items.end(), std::back_inserter(values), [](const std::string& one_item) { return boost::lexical_cast<T>(one_item);});
        data.push_back(values);
    }
    return data;
}



}

#endif
