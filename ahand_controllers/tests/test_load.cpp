#include "utils/utils.h"

int main(){
  std::string filename = "/home/guillaume/data_positions.txt";
  std::vector<std::vector<double>> data = ahand::load<double>(filename);

  for(auto item : data){
    for(auto value : item){
      std::cout<< value << " ";
    }
    std::cout<<std::endl;
  }

  return 0;
}
