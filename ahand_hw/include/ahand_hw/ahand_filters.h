//
// Created by guillaume on 20/08/18.
//

#ifndef AHAND_HW_FILTERS_H
#define AHAND_HW_FILTERS_H

#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

namespace filters {

    class Median {

        public:

            explicit Median(std::size_t window_size);

            double get(double x);

        public:

            std::size_t median_idx;
            std::size_t window_size;
            boost::circular_buffer<double> cbuffer_;
            std::vector<double> buffer_;

    };


    class SavitzkyGolay{

        public:

            SavitzkyGolay(unsigned int window_size, unsigned int num_coeff);

            void update(double x);

        private:

            boost::circular_buffer<double> cbuffer_;
            Eigen::MatrixXd C;
            Eigen::VectorXd y;

        public:

            Eigen::VectorXd c;
            double position;
            double velocity;

    };


}


#endif