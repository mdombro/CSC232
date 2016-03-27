#include <math.h>
#include <vector>

typedef vector<float> row;
typedef vector<row> matrix;

class Matrix {
    int row, col;
    matrix data(1,row(1));
public:
    Matrix(int m, int n) {
        row = m;
        col = n;
        data.resize(row);
        for (int i = 0; i < row; i++) {
            data[i].resize(col);
        }
    }

    void insert(int row, int col, float dat) {
        data[row][col] = dat;
    }

    Matrix& operator+ (Matrix& l, const matrix& m1) {
        return (*this += m1);
    }

    Matrix& operator* (const matrix& m1) {
        return (*this *= m1);
    }

    Matrix& operator+= (const matrix& rhs) {
        for (int i = 0; i < row; i++) {
            for (int o = 0; o < col; o++) {
                data[i][o] += rhs.data[i][o];
            }
        }
        return *this;
    }

    Matrix& operator*= (const matrix& T) {
        if(col == T.row) {
            for(int i = 0; i < T.row; ++i) {
                for(int k = 0; k < col; ++k) {
                    data[i][k] *= T.data[k][i];
                }
            }
        }
        return *this;
    }

    Matrix& operator= (const matrix& T) {
        p = T.p;
        n = T.n;
        m = T.m;
        return *this;
    }

}
