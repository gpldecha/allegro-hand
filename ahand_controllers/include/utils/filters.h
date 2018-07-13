#ifndef AHAND_CONTROLLERS__FILTERS_H
#define AHAND_CONTROLLERS__FILTERS_H

namespace ahand_controllers{


static inline double exponentialSmoothing(double current_raw_value, double last_smoothed_value, double alpha){
    return alpha*current_raw_value + (1.0-alpha)*last_smoothed_value;
}

}


#endif
