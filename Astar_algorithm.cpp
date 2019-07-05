#include "include/Astar_algorithm.h"

double DistenceToGoal(const Points& current, const Points& goal) {
  //曼哈顿距离
  return fabs(current.first - goal.first) + fabs(current.second - goal.second);

  // //欧式距离平方
  // return pow((current.first - goal.first), 2) +
  //             pow((current.second - goal.second), 2);

  // //欧式距离
  // return sqrt(pow((current.first - goal.first), 2) +
  //             pow((current.second - goal.second), 2));
}

std::vector<Points> Astar_Algorithm(
    const int row_args, const int columns_args, const Points& start_args,
    const Points& goal_args, const std::vector<Points>& close_list_args) {
  std::priority_queue<CellInfo> open_list;
  std::vector<Points> close_list(close_list_args);
  std::map<Points, Points> save_path_hash;
  std::vector<Points> path_result_list;  //存放结果

  CellInfo start_info = {0, 0, start_args};  //初始化起点的信息
  int search_successful_flg = 1, expand_points_count = 0;

  open_list.push(start_info);  //起点入队列
  while (!open_list.empty()) {
    CellInfo current_cell_pos = open_list.top();
    open_list.pop();
    // std::cout << "(" << current_cell_pos.xoy_.first << " , "
    //           << current_cell_pos.xoy_.second << ")"
    //           << "  ";

    if (current_cell_pos.xoy_ == goal_args) {  //找到终点，一次算法结束
      search_successful_flg = 0;
      break;
    }

    if (std::find(close_list.begin(), close_list.end(),
                  current_cell_pos.xoy_) == close_list.end()) {
      close_list.push_back(current_cell_pos.xoy_);
      ++expand_points_count;
      // std::cout<<"("<<current_cell_pos.xoy_.first<<" ,
      // "<<current_cell_pos.xoy_.second<<")"<<"  ";

      std::vector<CellInfo> neighbors = GetNeighbors(
          row_args, columns_args, start_args, goal_args, current_cell_pos.xoy_);

      for (int i = 0; i < neighbors.size(); ++i) {
        if (std::find(close_list.begin(), close_list.end(),
                      neighbors[i].xoy_) == close_list.end()) {
          // g(n)
          neighbors[i].cost_to_start_ += current_cell_pos.cost_to_start_ + 1;
          // f(n)=g(n)+h(n)
          neighbors[i].all_cost_ = neighbors[i].cost_to_start_ +
                                   DistenceToGoal(neighbors[i].xoy_, goal_args);
          open_list.push(neighbors[i]);
          save_path_hash[neighbors[i].xoy_] = current_cell_pos.xoy_;
        }
      }
    }
  }
  //   std::cout << std::endl << std::endl;
  //   std::cout << "----------" << close_list.size() << "----------" <<
  //   std::endl; for (int i = 0; i < close_list.size(); ++i) {
  //     std::cout << "(" << close_list[i].first << " , " <<
  //     close_list[i].second
  //               << ")"
  //               << "  ";
  //   }
  //   std::cout << std::endl << std::endl;

  // there is one shortest path to goal
  if (search_successful_flg) {
    std::cout << "search fail !!" << std::endl;
  }
  // nope path to goal
  else {
    std::cout << "search successfully !!" << std::endl;
    Points node = goal_args;
    //得到最短路径的坐标向量
    while (node != start_info.xoy_) {
      path_result_list.push_back(node);
      node = save_path_hash[node];
      // std::cout<<"("<<node.first<<" , "<<node.second<<")"<<std::endl;
    }
  }

  // map output
  for (int i = 0; i < row_args; ++i) {
    for (int j = 0; j < columns_args; ++j) {
      if (start_args.first == i && start_args.second == j)
        std::cout << "s ";
      else if (goal_args.first == i && goal_args.second == j)
        std::cout << "g ";
      else if (std::find(close_list_args.begin(), close_list_args.end(),
                         Points(i, j)) != close_list_args.end())
        std::cout << "x ";
      else if (std::find(path_result_list.begin(), path_result_list.end(),
                         Points(i, j)) != path_result_list.end())
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << path_result_list.size()
            << "    expand point nums : " << expand_points_count << std::endl;

  std::cout << std::endl << std::endl;

  // for (int i = 0; i < path_result_list.size(); i++) {
  //   std::cout << "(" << path_result_list[i].first << ","
  //             << path_result_list[i].second << ")" << std::endl;
  // }
  return path_result_list;

  // return path_result_list;
}

// 找出当前点的neighbors
// 对neighbors进行初始化时，根据起点与终点的方向，在neighbors的cost_to_start_参数进行方向性的初始化，
// 及添加起点至终点向量的0.01倍作为其初值，使其轨迹方向性更好
std::vector<CellInfo> GetNeighbors(const int row_args, const int columns_args,
                                   const Points& start_args,
                                   const Points& goal_args,
                                   const Points& current_pos) {
  std::vector<CellInfo> neighbors;
  // Up
  if ((current_pos.first - 1) >= 0) {
    neighbors.push_back({0, (goal_args.first - start_args.first) * 0.01,
                         Points(current_pos.first - 1, current_pos.second)});
  }
  // Down
  if ((current_pos.first + 1) < row_args) {
    neighbors.push_back({0, -(goal_args.first - start_args.first) * 0.01,
                         Points(current_pos.first + 1, current_pos.second)});
  }
  // Left
  if ((current_pos.second - 1) >= 0) {
    neighbors.push_back({0, -(goal_args.second - start_args.second) * 0.01,
                         Points(current_pos.first, current_pos.second - 1)});
  }
  // Right
  if ((current_pos.second + 1) < columns_args) {
    neighbors.push_back({0, (goal_args.second - start_args.second) * 0.01,
                         Points(current_pos.first, current_pos.second + 1)});
  }

  return neighbors;
}

//获得当前节点的neighbors，但仅仅包含障碍信息
void UpdataMapInfo(const int row_args, const int columns_args,
                   const Points& start_args,
                   const std::vector<Points>& map_obstacle_list_,
                   std::vector<Points>& current_obstacle_list_) {
  if ((start_args.first - 1) >= 0) {
    if (IsInObstacleList(Points(start_args.first - 1, start_args.second),
                         map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(start_args.first - 1, start_args.second));
  }
  // Down
  if ((start_args.first + 1) < row_args) {
    if (IsInObstacleList(Points(start_args.first + 1, start_args.second),
                         map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(start_args.first + 1, start_args.second));
  }
  // Left
  if ((start_args.second - 1) >= 0) {
    if (IsInObstacleList(Points(start_args.first, start_args.second - 1),
                         map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(start_args.first, start_args.second - 1));
  }
  // Right
  if ((start_args.second + 1) < columns_args) {
    if (IsInObstacleList(Points(start_args.first, start_args.second + 1),
                         map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(start_args.first, start_args.second + 1));
  }
}

//判断当前点是否在障碍list中
// return 1:on  0:nop
bool IsInObstacleList(const Points& point,
                      const std::vector<Points>& obstacle_list) {
  return std::find(obstacle_list.begin(), obstacle_list.end(), point) !=
         obstacle_list.end();
}

//*********************************************************************
Astar::Astar(int row, int column, Points statr, Points goal,
             std::vector<Points> obstacle_list_)
    : row_(row),
      column_(column),
      start_pos_(statr),
      goal_pos_(goal),
      map_obstacle_list_(obstacle_list_) {
  current_start_ = start_pos_;
}

//获得当前节点的neighbors,一次A*算法中
std::vector<CellInfo> Astar::GetNeighbors(const Points& current_pos) {
  std::vector<CellInfo> neighbors;
  // Up
  if ((current_pos.first - 1) >= 0) {
    neighbors.push_back({0, (goal_pos_.first - current_pos.first) * 0.01,
                         Points(current_pos.first - 1, current_pos.second)});
  }
  // Down
  if ((current_pos.first + 1) < row_) {
    neighbors.push_back({0, -(goal_pos_.first - current_pos.first) * 0.01,
                         Points(current_pos.first + 1, current_pos.second)});
  }
  // Left
  if ((current_pos.second - 1) >= 0) {
    neighbors.push_back({0, -(goal_pos_.second - current_pos.second) * 0.01,
                         Points(current_pos.first, current_pos.second - 1)});
  }
  // Right
  if ((current_pos.second + 1) < column_) {
    neighbors.push_back({0, (goal_pos_.second - current_pos.second) * 0.01,
                         Points(current_pos.first, current_pos.second + 1)});
  }
  return neighbors;
}

//获得当前节点的neighbors，但仅仅包含障碍信息,整体A*算法中
void Astar::UpdataMapInfo() {
  if ((current_start_.first - 1) >= 0) {
    if (IsInObstacleList(
            Points(current_start_.first - 1, current_start_.second),
            map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    if (IsInObstacleList(
            Points(current_start_.first + 1, current_start_.second),
            map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    if (IsInObstacleList(
            Points(current_start_.first, current_start_.second - 1),
            map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    if (IsInObstacleList(
            Points(current_start_.first, current_start_.second + 1),
            map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
}

//一次A*算法，以当前起点为起点
void Astar::AstarGetPath() {
  {
    std::priority_queue<CellInfo> open_list;  //存放将要遍历的点
    std::vector<Points> close_list(
        current_obstacle_list_);  //存放已经遍历的点以及已知的地图障碍点
    std::map<Points, Points> save_path_hash;  //用于路径回溯
    std::vector<Points> path_result_list;     //存放结果

    CellInfo start_info = {0, 0, current_start_};  //初始化起点的信息
    int search_successful_flg = 1, expand_points_count = 0;

    open_list.push(start_info);  //起点入队列

    while (!open_list.empty()) {
      CellInfo current_cell_pos = open_list.top();
      open_list.pop();
      //找到终点，一次算法结束
      if (current_cell_pos.xoy_ == goal_pos_) {
        search_successful_flg = 0;  //搜索成功
        break;
      }

      //如果不在closelist中，可以expand
      if (!IsInList(current_cell_pos.xoy_, close_list)) {
        close_list.push_back(current_cell_pos.xoy_);
        ++expand_points_count;
        // std::cout<<"("<<current_cell_pos.xoy_.first<<" ,
        // "<<current_cell_pos.xoy_.second<<")"<<"  ";

        std::vector<CellInfo> neighbors = GetNeighbors(current_cell_pos.xoy_);

        for (int i = 0; i < neighbors.size(); ++i) {
          if (!IsInList(neighbors[i].xoy_, close_list)) {
            // g(n)
            neighbors[i].cost_to_start_ += current_cell_pos.cost_to_start_ + 1;
            // f(n)=g(n)+h(n)
            neighbors[i].all_cost_ =
                neighbors[i].cost_to_start_ + DistenceToGoal(neighbors[i].xoy_);
            open_list.push(neighbors[i]);
            save_path_hash[neighbors[i].xoy_] = current_cell_pos.xoy_;
          }
        }
      }
    }
    //   std::cout << std::endl << std::endl;
    //   std::cout << "----------" << close_list.size() << "----------" <<
    //   std::endl; for (int i = 0; i < close_list.size(); ++i) {
    //     std::cout << "(" << close_list[i].first << " , " <<
    //     close_list[i].second
    //               << ")"
    //               << "  ";
    //   }
    //   std::cout << std::endl << std::endl;

    // there is one shortest path to goal
    if (search_successful_flg) {
      std::cout << "search fail !!" << std::endl;
    }
    // nope path to goal
    else {
      std::cout << "search successfully !!" << std::endl;
      Points node = goal_pos_;
      //得到最短路径的坐标向量
      while (node != start_info.xoy_) {
        path_result_list.push_back(node);
        node = save_path_hash[node];
        // std::cout<<"("<<node.first<<" , "<<node.second<<")"<<std::endl;
      }
    }

    // map output
    for (int i = 0; i < row_; ++i) {
      for (int j = 0; j < column_; ++j) {
        if (current_start_.first == i && current_start_.second == j)
          std::cout << "s ";

        else if (goal_pos_.first == i && goal_pos_.second == j)
          std::cout << "g ";

        else if (IsInList(Points(i, j), current_obstacle_list_))
          std::cout << "x ";

        else if (IsInList(Points(i, j), path_result_list))
          std::cout << "o ";
        else
          std::cout << "_ ";
      }
      std::cout << std::endl;
    }
    std::cout << "shortest path step nums : " << path_result_list.size()
              << "    expand point nums : " << expand_points_count << std::endl;

    std::cout << std::endl << std::endl;

    // for (int i = 0; i < path_result_list.size(); i++) {
    //   std::cout << "(" << path_result_list[i].first << ","
    //             << path_result_list[i].second << ")" << std::endl;
    // }
    current_path_.clear();
    current_path_ = path_result_list;
  }
}