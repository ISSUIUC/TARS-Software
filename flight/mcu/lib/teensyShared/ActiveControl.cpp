#include "ActiveControl.h"

void ActiveControl::acTickFunction(float AV_X) {
    Matrix<float, 2, 3> lengths = k_oct * AV_X;
}