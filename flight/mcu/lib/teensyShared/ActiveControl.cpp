#include "ActiveControl.h"

void ActiveControl::acTickFunction(float AV_X) {
    Matrix<float, 2, 1> lengths = k_oct * AV_X;
}
