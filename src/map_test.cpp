#include "scanmatch/map.h"
#include <iostream>
#include <iomanip>
using namespace std;

void PrintCostMap(const vector<vector<float>>& cost_map, int row_start, 
    int row_end, int col_start, int col_end) {
  cout << "cost map:" << endl;
  for (int j = row_start; j < row_end; ++j) {
    for (int i = col_start; i < col_end; ++i) {
      cout << fixed << setprecision(2) << cost_map[j][i] << " ";
    }
    cout << "\n";
  }
}

void PrintProb(const Map& map, int row_start, int row_end, int col_start, 
    int col_end) {
  cout << "prob map:" << endl;
  for (int j = 0; j < row_end - row_start; ++j) {
    for (int i = 0; i < col_end - col_start; ++i) {
      cout << fixed << setprecision(2) << map.GetProb(
          Point(i*map.Resolution(), j*map.Resolution())) << " ";
    }
    cout << "\n";
  }
}

int main() {
  int row_start = 150; 
  int row_end = 165;
  int col_start = 150;
  int col_end = 165;
  Map map;
  auto& cost_map = map.CostMap();
  PrintCostMap(cost_map, row_start, row_end, col_start, col_end);
  Pose pose(0, 0, 0);
  vector<Point> point_cloud{{0.5, 0.5}, {0.5, 0.3}};
  for (int i = 0; i < 20; ++i) {
    cout << i << "\n";
    map.Update(pose, point_cloud);
    PrintCostMap(cost_map, row_start, row_end, col_start, col_end);
    PrintProb(map, row_start, row_end, col_start, col_end);
  }
  
}