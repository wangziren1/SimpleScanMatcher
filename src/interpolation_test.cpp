#include <iostream>
#include <iomanip>
using namespace std;

#include "ceres/cubic_interpolation.h"

struct Grid2D {
  enum { DATA_DIMENSION = 1 };
  Grid2D(const double data[], int row_num, int col_num)
    : data_(data), row_num(row_num), col_num(col_num){}

  void GetValue(int row, int col, double* f) const {
    *f = data_[row * col_num + col];
  }
  
  int row_num;
  int col_num;
  const double* data_;
};

int main() {
  // const double data[] = {1.0, 3.0, -1.0, 4.0,
  //                        3.6, 2.1,  4.2, 2.0,
  //                        2.0, 1.0,  3.1, 5.2,
  //                        2.0, 1.0,  3.1, 5.2};
  const double data[] = {
   0.900, 0.100, 0.104, 0.450, 
 0.900, 0.867, 0.104, 0.100 ,
 0.900, 0.900, 0.636, 0.897 ,
 0.900, 0.900, 0.900, 0.900
  };
  Grid2D array(data, 4, 4);
  ceres::BiCubicInterpolator<Grid2D> interpolator(array);
  double f, dfdr, dfdc;
  interpolator.Evaluate(1.106, 1.648, &f, &dfdr, &dfdc);
  
  cout << std::fixed << setprecision(3);
  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 4; ++i) {
      cout << data[j * 4 + i] << " ";
    }
    cout << "\n";
  }
  cout << f << endl; 
  return 0;
}