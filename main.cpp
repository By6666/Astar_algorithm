#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "include/Astar_algorithm.h"
#include "include/grid_input.h"

void SearchOneMap(int map_num_);

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

void SearchOneMap(int map_num_) {
  //获得map信息
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  //数据传入，构造类对象
  Astar Astar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                        map_info.get_start_pos(), map_info.get_goal_pos(),
                        map_info.get_obstacle_pos());

  int search_count = 0;  //搜索计数

  while (1) {
    //规划路径
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << ++search_count << std::endl;
    Astar_algorithm.AstarGetPath();  //以当前起点为起点进行一次路径规划

    if (Astar_algorithm.get_current_path().size() == 0) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      break;
    }

    while (!Astar_algorithm.get_current_path().empty()) {
      //更新当前点各临近点的信息
      Astar_algorithm.UpdataMapInfo();

      if (Astar_algorithm.NextStepIsInObstacleList()) {
        break;  //当前点要移动到的下一个点是obstacle
      } else {
       Astar_algorithm.StartMove();
      }
    }

    if (Astar_algorithm.ArriveGoal()) {  //走到了终点
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      std::cout << "|final result: get goal successflly!!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
      break;
    }
  }
}

// void SearchOneMap(int map_num_) {
//   //获得map信息
//   GrideInput map_info(map_num_);
//   map_info.GetOneGrid();
//   map_info.PrintMap();  //打印原始map

//   int row = map_info.get_grid_rows();          //获得map行数
//   int column = map_info.get_grid_columns();    //获得map列数
//   Points start_pos(map_info.get_start_pos());  //获得起点坐标
//   Points goal_pos(map_info.get_goal_pos());    //获得终点坐标

//   std::vector<Points> map_obstacle_list(
//       map_info.get_obstacle_pos());  //所有障碍物list

//   std::vector<Points> current_path;  //当前path

//   std::vector<Points> current_obstacle_list;  //当前障碍物list

//   // int row = 10, columns = 10;
//   // Points start_pos(0, 0), goal_pos(9, 9);  //获得起点坐标
//   // std::vector<Points> current_path;        //当前path
//   // std::vector<Points> map_obstacle_list;   //所有障碍物list
//   // map_obstacle_list.push_back(Points(1, 0));
//   // map_obstacle_list.push_back(Points(0, 1));
//   // std::vector<Points> current_obstacle_list;  //当前障碍物list
//   // Points current_start(start_pos);

//   Points current_start(start_pos);  //赋值当前点
//   int search_count = 0;

//   while (1) {
//     //规划路径
//     std::cout << "**********" << std::endl;
//     std::cout << "search num : " << ++search_count << std::endl;
//     current_path = Astar_Algorithm(row, column, current_start, goal_pos,
//                                    current_obstacle_list);

//     if (current_path.size() == 0) {
//       std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
//       std::cout << "|final result : no path to goal !!|" << std::endl;
//       std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
//       break;
//     }

//     while (!current_path.empty()) {
//       //更新当前点各临近点的信息
//       UpdataMapInfo(row, column, current_start, map_obstacle_list,
//                     current_obstacle_list);

//       if (IsInObstacleList(current_path.back(), current_obstacle_list)) {
//         break;  //当前点要移动到的下一个点是obstacle
//       } else {
//         current_start = current_path.back();
//         current_path.pop_back();
//       }
//     }

//     if (current_start == goal_pos) {  //走到了终点
//       std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
//       std::cout << "|final result: get goal successflly!!|" << std::endl;
//       std::cout << "-——-——-——-——-——-——-——-———-——-——-——-——-" << std::endl;
//       break;
//     }
//   }
// }