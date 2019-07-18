#ifndef PLANNER_ASTAR_ALGORITHM_H
#define PLANNER_ASTAR_ALGORITHM_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "grid_input.h"

void SearchOneMap(int map_num_);  //对一张地图进行算法搜索
void PrintSumResult();            //打印结果统计

//每一个cell的信息，使用结构体定义
struct CellInfo {
  double all_cost_;       //总花费
  double cost_to_start_;  // g-value
  Points xoy_;            //坐标点
  bool operator<(const CellInfo& pos) const {
    // if f-vaule 相同，比较 g-vaule
    if (all_cost_ == pos.all_cost_)
      return cost_to_start_ > pos.cost_to_start_;
    else
      return all_cost_ > pos.all_cost_;
  }
};

class Astar {
 public:
  Astar(int row_, int columns_, Points statr_, Points goal_,
        std::vector<Points> obstacle_list_);  //构造函数
  Astar(const Astar& as) = delete;            //不使用复制构造函数

  //******下面函数用于一次A*规划算法中*******//
  void AstarGetPath();       // 一次A*算法函数
  void SearchResultPrint();  //打印搜索结果
  //计算当前点与终点距离的函数
  inline double DistenceToGoal(const Points& current) {
    //曼哈顿距离
    return fabs(current.first - goal_pos_.first) +
           fabs(current.second - goal_pos_.second);
  }

  //获得当前节点的neighbors
  std::vector<CellInfo> GetNeighbors(const Points& current_pos);
  //**************************************//

  //**********下面函数用于整体算法中**********//
  void UpdataMapInfo();  //获得当前节点的neighbors，但仅仅包含障碍信息

  // move one step along the shortest path
  //沿着当前最短路径走一步
  void StartMove() {
    current_start_ = current_path_.back();
    current_path_.pop_back();
    ++move_step_nums_;
  }

  //判断是否走到了终点
  bool ArriveGoal() { return current_start_ == goal_pos_; }

  //判断要移动的下一个点是否为障碍物
  inline bool NextStepIsInObstacleList() {
    return IsInList(current_path_.back(), current_obstacle_list_);
  }
  //打印计数结果
  void PrintCountResult();
  //**************************************//

  //**********获取类内私有成员函数**********//
  inline int get_row() const { return row_; }
  inline int get_column() const { return column_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline Points get_current_start() const { return current_start_; }
  inline std::vector<Points> get_map_obstacle_list() const {
    return map_obstacle_list_;
  }
  inline std::vector<Points> get_current_obstacle_list() const {
    return current_obstacle_list_;
  }
  inline std::vector<Points> get_current_path() const { return current_path_; }
  inline int get_all_expand_nums() const { return all_expand_points_count_; }
  inline int get_search_nums() const { return search_nums_count_; }
  inline int get_move_step_nums() const { return move_step_nums_; }
  //************************************//

  //**********类内私有成员赋值函数**********//
  inline int& set_row() { return row_; }
  inline int& set_column() { return column_; }
  inline Points& set_start_pos() { return start_pos_; }
  inline Points& set_goal_pos() { return goal_pos_; }
  inline Points& set_current_start() { return current_start_; }
  inline std::vector<Points>& set_map_obstacle_list() {
    return map_obstacle_list_;
  }
  inline std::vector<Points>& set_current_obstacle_list() {
    return current_obstacle_list_;
  }
  inline std::vector<Points>& set_current_path() { return current_path_; }
  //************************************//

 private:
  int row_, column_;                           // map的行、列数
  Points start_pos_, goal_pos_;                // map的起点与终点坐标
  Points current_start_;                       //当前起点
  std::vector<Points> map_obstacle_list_;      //所有障碍物list
  std::vector<Points> current_path_;           //当前路径
  std::vector<Points> current_obstacle_list_;  //当前已知的障碍物list
  int current_expand_points_count_,            //一次算法中的expand计数
      all_expand_points_count_,                //整体算法中的expand计数
      search_nums_count_,                      //搜索次数计数
      move_step_nums_;                         //移动步数

  //判断点是否在list中
  inline bool IsInList(const Points& point, const std::vector<Points>& list) {
    return std::find(list.begin(), list.end(), point) != list.end();
  }
};

#endif