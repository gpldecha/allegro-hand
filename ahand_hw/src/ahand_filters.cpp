#include "ahand_hw/ahand_filters.h"
#include <algorithm>
#include <iostream>
#include <Eigen/Eigen>

namespace filters {

    Median::Median(std::size_t window_size) {
        if (window_size < 2)
            window_size = 3;
        if (window_size % 2 == 0)
            window_size++;
        this->window_size = window_size;
        median_idx = static_cast<std::size_t >((window_size - 1) / 2);
        cbuffer_ = boost::circular_buffer<double>(window_size);
        buffer_= std::vector<double>(window_size);
    }

    double Median::get(const double x) {
        cbuffer_.push_back(x);
        if(cbuffer_.size() != cbuffer_.capacity()){
            return x;
        }
        for(std::size_t i = 0; i < cbuffer_.size(); i++){
            buffer_[i] = cbuffer_[i];
        }
        std::sort(buffer_.begin(), buffer_.end());
        return buffer_[median_idx];
    }


    SavitzkyGolay::SavitzkyGolay(unsigned int window_size, unsigned int num_coeff){
        if (window_size < 2)
            window_size = 3;
        if (window_size % 2 == 0)
            window_size++;

        position=0.0;
        velocity=0.0;

        if(num_coeff < 2) {
            num_coeff = 2;
        }

        cbuffer_ = boost::circular_buffer<double>(window_size);

        Eigen::MatrixXd A(window_size, num_coeff);
        Eigen::VectorXd col(window_size, 1);

        double entry = -(static_cast<double>(window_size)-1)/2.0;
        for(std::size_t i = 0; i < window_size; i++){
            col(i) = entry; entry++;
        }
        for(std::size_t j = 0; j < A.cols(); j++){
            A.col(j) = col.array().pow(static_cast<double>(j));
        }
        C = (A.transpose()*A).inverse()*A.transpose();
        y.resize(window_size);
        c.resize(num_coeff);
        c = c.setZero();
    }

    void SavitzkyGolay::update(double x){
        cbuffer_.push_back(x);
        if(cbuffer_.size() != cbuffer_.capacity()) {
            position = x;
            velocity = 0;
            return;
        }
        for(std::size_t i = 0; i < cbuffer_.size(); i++){
            y[i] = cbuffer_[i];
        }
        c = C*y;
        position = c(0);
        velocity = c(1);
    }

}

