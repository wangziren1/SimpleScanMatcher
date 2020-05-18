#include "scanmatch/map.h"
#include <iostream>
using namespace std;

int main() {
  GridPoint start(426, 551);
  cout << "start:" << start.x_ << "," << start.y_ << endl;
  GridPoint end(382, 594);
  cout << "end:" << end.x_ << "," << end.y_ << endl;
  vector<GridPoint> result;
  result = Bresenham(start, end);
  for (auto& point : result) {
    cout << "(" << point.x_ << "," << point.y_ << ") ";
  }
  cout << endl;
  {
    GridPoint start(382, 594);
    cout << "start:" << start.x_ << "," << start.y_ << endl;
    GridPoint end(426, 551);
    cout << "end:" << end.x_ << "," << end.y_ << endl;
    vector<GridPoint> result;
    result = Bresenham(start, end);
    for (auto& point : result) {
      cout << "(" << point.x_ << "," << point.y_ << ") ";
    }
    cout << endl;
  }
  
}