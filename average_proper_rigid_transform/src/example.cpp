#include "average_proper_rigid_transform/average_proper_rigid_transform.h"
#include <iostream>
using namespace average_proper_rigid_transform;

template <class Scalar=float>
void print_mat(const char* label, const Scalar* m)
{
    std::cout
        << label << "\n"
        << m[0] << ", " << m[1] << ", " << m[2] << ", " << m[3] << "\n"
        << m[4] << ", " << m[5] << ", " << m[6] << ", " << m[7] << "\n"
        << m[8] << ", " << m[9] << ", " << m[10] << ", " << m[11] << "\n"
        << m[12] << ", " << m[13] << ", " << m[14] << ", " << m[15] << "\n"
        << "\n";
}

template <class Scalar=float>
void print_quat(const char* label, const Scalar* q)
{
    std::cout
        << label << "\n"
        << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "\n"
        << "\n";
}

int main() {
    float id[16] = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };
    float rotZ90[16] = {
        0,-1,0,0,
        1,0,0,0,
        0,0,1,0,
        0,0,0,1
    };
    float quat_id[4];
    float quat_rotZ90[4];
    mat_to_quat(quat_id, id);
    mat_to_quat(quat_rotZ90, rotZ90);
    float quat_rotZ45[4];
    mix_quat(quat_rotZ45, quat_id, quat_rotZ90, 0.5f);
    float rotZ45[16];
    quat_to_mat(rotZ45, quat_rotZ45);

    print_mat ("id",          id);           
    print_quat("quat_id",     quat_id);     
    print_mat ("rotZ90",      rotZ90);       
    print_quat("quat_rotZ90", quat_rotZ90); 
    print_mat ("rotZ45",      rotZ45);       
    print_quat("quat_rotZ45", quat_rotZ45); 
}
// Output:
// 
// id
// 1, 0, 0, 0
// 0, 1, 0, 0
// 0, 0, 1, 0
// 0, 0, 0, 1

// quat_id
// 0, 0, 0, 1

// rotZ90
// 0, -1, 0, 0
// 1, 0, 0, 0
// 0, 0, 1, 0
// 0, 0, 0, 1

// quat_rotZ90
// 0, 0, 0.707107, 0.707107

// rotZ45
// 0.707107, -0.707107, 0, 0
// 0.707107, 0.707107, 0, 0
// 0, 0, 1, 0
// 0, 0, 0, 1

// quat_rotZ45
// 0, 0, 0.382683, 0.92388
