#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "include/Astar_algorithm.h"
#include "include/grid_input.h"

int main() {
  int map_num = 0;
  while (1) {
    std::cout << "Please input the map index (1~20): ";
    std::cin >> map_num;
    if (map_num < 1 || map_num > 20) {
      std::cout << "Input wrong, Please input again !!" << std::endl;
      continue;
    }
    std::cout << "/*******************************************/" << std::endl;
    std::cout << "/**********file——" << map_num << "**********/" << std::endl;
    SearchOneMap(map_num - 1);
    std::cout << "/*******************************************/" << std::endl;
    std::cout << std::endl;
  }

  return 0;
}
