#include "Astar_algorithm.h"

std::vector<std::string> sum_result;  //总结输出结果

/* 进行一次总体的A*算法
 * 输入：文件序号
 * 输出：无
 *  */
void SearchOneMap(int map_num_) {
  //获得map信息
  GrideInput map_info(map_num_);
  map_info.GetOneGrid();
  map_info.PrintMap();  //打印原始map

  //数据传入，构造类对象
  Astar Astar_algorithm(map_info.get_grid_rows(), map_info.get_grid_columns(),
                        map_info.get_start_pos(), map_info.get_goal_pos(),
                        map_info.get_obstacle_pos());

  while (1) {
    //规划路径
    std::cout << "**********" << std::endl;
    std::cout << "search num : " << Astar_algorithm.get_search_nums() + 1
              << std::endl;
    Astar_algorithm.AstarGetPath();  //以当前起点为起点进行一次路径规划

    if (Astar_algorithm.get_current_path().size() == 0) {
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      std::cout << "|final result : no path to goal !!|" << std::endl;
      std::cout << "-——-——-——-——-——-——-——-———-——-——-——-" << std::endl;
      Astar_algorithm.PrintCountResult();
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
      Astar_algorithm.PrintCountResult();
      break;
    }
  }
  sum_result.push_back(
      std::to_string(map_num_ + 1) + "          " +
      std::to_string(Astar_algorithm.get_search_nums()) + "          " +
      std::to_string(Astar_algorithm.get_all_expand_nums()) + "          " +
      std::to_string(Astar_algorithm.get_move_step_nums()));
}

/* 构造函数
 * 输入：地图的行数、列数、起点、终点、障碍物点
 * 输出：无
 * */
Astar::Astar(int row, int column, Points statr, Points goal,
             std::vector<Points> obstacle_list_)
    : row_(row),
      column_(column),
      start_pos_(statr),
      goal_pos_(goal),
      map_obstacle_list_(obstacle_list_) {
  current_start_ = start_pos_;
  all_expand_points_count_ = 0;
  search_nums_count_ = 0;
  move_step_nums_ = 0;
}

/* 在一次A*算法中获得当前点的四个临近点的坐标
 * 输入：当前点的坐标
 * 输出：四个临近点的坐标信息
 * */
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

/* 在整体ARA*算法中获得当前起点的四个临近点的信息
 * 输入：无
 * 输出：无
 * */
void Astar::UpdataMapInfo() {
  if ((current_start_.first - 1) >= 0) {
    if (IsInList(Points(current_start_.first - 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first - 1, current_start_.second));
  }
  // Down
  if ((current_start_.first + 1) < row_) {
    if (IsInList(Points(current_start_.first + 1, current_start_.second),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first + 1, current_start_.second));
  }
  // Left
  if ((current_start_.second - 1) >= 0) {
    if (IsInList(Points(current_start_.first, current_start_.second - 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second - 1));
  }
  // Right
  if ((current_start_.second + 1) < column_) {
    if (IsInList(Points(current_start_.first, current_start_.second + 1),
                 map_obstacle_list_))
      current_obstacle_list_.push_back(
          Points(current_start_.first, current_start_.second + 1));
  }
}

/* 将openlist中的最小元素放到末尾
 * 输入：无
 * 输出：openlist中的最小元素
 * */
CellInfo Astar::OpenLIstPopMinElem(std::vector<CellInfo>& open_list) {
  if (open_list.size() < 2) return open_list.back();

  std::vector<CellInfo>::iterator itr =
      std::min_element(open_list.begin(), open_list.end());

  CellInfo temp = *itr;
  *itr = open_list.back();
  open_list.back() = temp;

  return temp;
}

/* 执行一次A*算法
 * 输入：无
 * 输出：无
 * */
void Astar::AstarGetPath() {
  {
    std::vector<CellInfo> open_list;  //存放将要遍历的点
    std::vector<Points> close_list(
        current_obstacle_list_);  //存放已经遍历的点以及已知的地图障碍点
    std::map<Points, Points> save_path_hash;  //用于路径回溯
    std::vector<Points> path_result_list;     //存放结果

    CellInfo start_info = {0, 0, current_start_};  //初始化起点的信息
    int search_successful_flg = 1;
    current_expand_points_count_ = 0;

    open_list.push_back(start_info);  //起点入队列

    while (!open_list.empty()) {
      CellInfo current_cell_pos = OpenLIstPopMinElem(open_list);
      open_list.pop_back();
      //找到终点，一次算法结束
      if (current_cell_pos.xoy_ == goal_pos_) {
        search_successful_flg = 0;  //搜索成功
        break;
      }

      //如果不在closelist中，可以expand
      if (!IsInList(current_cell_pos.xoy_, close_list)) {
        close_list.push_back(current_cell_pos.xoy_);

        std::vector<CellInfo> neighbors = GetNeighbors(current_cell_pos.xoy_);
        int8_t neighbor_expand_cnt = 0;
        for (int i = 0; i < neighbors.size(); ++i) {
          if (!IsInList(neighbors[i].xoy_, close_list)) {
            ++neighbor_expand_cnt;
            // g(n)
            neighbors[i].cost_to_start_ += current_cell_pos.cost_to_start_ + 1;
            // f(n)=g(n)+h(n)
            neighbors[i].all_cost_ =
                neighbors[i].cost_to_start_ + DistenceToGoal(neighbors[i].xoy_);
            open_list.push_back(neighbors[i]);
            save_path_hash[neighbors[i].xoy_] = current_cell_pos.xoy_;
          }
        }
        if (neighbor_expand_cnt) ++current_expand_points_count_;  //扩展点自曾
      }
    }

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

    all_expand_points_count_ += current_expand_points_count_;  // expand计数累加

    current_path_.clear();
    current_path_ = path_result_list;

    // SearchResultPrint();

    ++search_nums_count_;  //搜索次数自增1
  }
}

/* 打印一次搜索的结果
 * 输入：无
 * 输出：无
 * */
void Astar::SearchResultPrint() {
  for (int i = 0; i < row_; ++i) {
    for (int j = 0; j < column_; ++j) {
      if (current_start_.first == i && current_start_.second == j)
        std::cout << "s ";

      else if (goal_pos_.first == i && goal_pos_.second == j)
        std::cout << "g ";

      else if (IsInList(Points(i, j), current_obstacle_list_))
        std::cout << "x ";

      else if (IsInList(Points(i, j), current_path_))
        std::cout << "o ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << "shortest path step nums : " << current_path_.size()
            << "    expand point nums : " << current_expand_points_count_
            << std::endl;

  std::cout << std::endl << std::endl;
}

/* 打印计数结果
 * 输入：无
 * 输出：无
 *  */
void Astar::PrintCountResult() {
  std::cout << std::endl
            << "The nums of search : " << search_nums_count_
            << "  total expanded nums : " << all_expand_points_count_
            << std::endl
            << std::endl;
}

/* 打印统计结果
 * 输入：无
 * 输出：无
 *  */
void PrintSumResult() {
  std::cout << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl
            << "-——                   Sum  Result                    ——-"
            << std::endl
            << "-——-——-——-——-——-——-——-——-***-——-——-——-——-——-——-——-——-——-"
            << std::endl;
  std::cout << "| map num | search nums | expand nums | move_step nums |"
            << std::endl;
  for (int16_t i = 0; i < sum_result.size(); ++i) {
    std::cout << sum_result[i] << std::endl;
  }
}