
#include "average_affine_transform_mat/average_affine_transform_mat.h"
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <string>

using namespace average_affine_transform_mat;

const float pi = 3.14159265358979f;
const float d2r = pi / 180.0f;
const float r2d = pi / 180.0f;

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

void rot_x(float* mat, float angle)
{
    float cs = cos(angle);
    float sn = sin(angle);
    mat[0*4 + 0] = 1;
    mat[1*4 + 0] = 0;
    mat[2*4 + 0] = 0;
    mat[3*4 + 0] = 0;
    mat[0*4 + 1] = 0;
    mat[1*4 + 1] = cs;
    mat[2*4 + 1] = sn;
    mat[3*4 + 1] = 0;
    mat[0*4 + 2] = 0;
    mat[1*4 + 2] = -sn;
    mat[2*4 + 2] = cs;
    mat[3*4 + 2] = 0;
    mat[0*4 + 3] = 0;
    mat[1*4 + 3] = 0;
    mat[2*4 + 3] = 0;
    mat[3*4 + 3] = 1;
}

void rot_y(float* mat, float angle)
{
    float cs = cos(angle);
    float sn = sin(angle);
    mat[0*4 + 0] = cs;
    mat[1*4 + 0] = 0;
    mat[2*4 + 0] = -sn;
    mat[3*4 + 0] = 0;
    mat[0*4 + 1] = 0;
    mat[1*4 + 1] = 1;
    mat[2*4 + 1] = 0;
    mat[3*4 + 1] = 0;
    mat[0*4 + 2] = sn;
    mat[1*4 + 2] = 0;
    mat[2*4 + 2] = cs;
    mat[3*4 + 2] = 0;
    mat[0*4 + 3] = 0;
    mat[1*4 + 3] = 0;
    mat[2*4 + 3] = 0;
    mat[3*4 + 3] = 1;
}

void rot_z(float* mat, float angle)
{
    float cs = cos(angle);
    float sn = sin(angle);
    mat[0*4 + 0] = cs;
    mat[1*4 + 0] = sn;
    mat[2*4 + 0] = 0;
    mat[3*4 + 0] = 0;
    mat[0*4 + 1] = -sn;
    mat[1*4 + 1] = cs;
    mat[2*4 + 1] = 0;
    mat[3*4 + 1] = 0;
    mat[0*4 + 2] = 0;
    mat[1*4 + 2] = 0;
    mat[2*4 + 2] = 1;
    mat[3*4 + 2] = 0;
    mat[0*4 + 3] = 0;
    mat[1*4 + 3] = 0;
    mat[2*4 + 3] = 0;
    mat[3*4 + 3] = 1;
}

void rot(float* mat, float angle, int axis)
{
    if (axis == 0) rot_x(mat, angle);
    if (axis == 1) rot_y(mat, angle);
    if (axis == 2) rot_z(mat, angle);
}

void test_equality(const char* label, int* error, const float *a, const float *b, int n=16, float eps=1e-6)
{
    for (int i=0; i<n; i++)
    {
        bool is_equal = (abs(a[i] - b[i]) < eps);
        //assert(is_equal);
        if (!is_equal) throw label;
        error[0] += is_equal ? 0 : 1;
    }
}

template <class T=int>
struct ABC_ { T a; T b; T c; };
using ABC = ABC_<int>;
using ABCf = ABC_<float>;

int test()
{
    int error = 0;
    // circle around axis with 45 degrees interval (45==360/8)
    float r[16*8];
    #define ROT(N) (r+16*(((N)%360)/45))
    float m[16];
    int s = 16;

    int num = 8;
    ABC abc[8] = {
        {0,45,90},{90,135,180},{180,225,270},{270,315,360},
        {45,90,135},{135,180,225},{225,270,315},{315,0,45},
    };
    for (int axis=0; axis<3; axis++)
    {
        for (int i=0; i<8; ++i)
        {
            rot(ROT(i*45), i*45*d2r, axis);
        }
        for (int i=0; i<num; ++i)
        {
            int a = abc[i].a;
            int b = abc[i].b;
            int c = abc[i].c;
            try
            {
                average_mat(m, ROT(a), ROT(c));    test_equality("test_0", &error, m, ROT(b));
                mix_mat(m, ROT(a), ROT(c), 0.5f);  test_equality("test_1", &error, m, ROT(b));
                mix_mat(m, ROT(a), ROT(c), 0.0f);  test_equality("test_2", &error, m, ROT(a));
                mix_mat(m, ROT(a), ROT(c), 1.0f);  test_equality("test_3", &error, m, ROT(c));
            }
            catch (const char* label)
            {
                std::string str_a = "ROT(" + std::to_string(a) + ")";
                std::string str_b = "ROT(" + std::to_string(b) + ")";
                std::string str_c = "ROT(" + std::to_string(c) + ")";
                std::string str_m = "m";
                std::cout << label << "\n";
                print_mat(str_a.c_str(), ROT(a));
                print_mat(str_b.c_str(), ROT(b));
                print_mat(str_c.c_str(), ROT(c));
                print_mat(str_m.c_str(), m);
                throw label;
            }
        }

        try
        {
            // m = average(r[0],r[2])
            average_mat<float,16*2>(m, r, 2); test_equality("test_4", &error, m, ROT(45));
            
            // m = average(r[0],r[1],r[2])
            average_mat(m, r, 3); test_equality("test_5", &error, m, ROT(45));

            // m = average(r[1],r[2],r[3])
            average_mat(m, r+16*1, 3); test_equality("test_6", &error, m, ROT(90));

        }
        catch (const char* label)
        {
            std::cout << label << "\n";
            for (int i=0; i<8; ++i)
            {
                std::string str_i = "ROT(" + std::to_string(i*45) + ")";
                print_mat(str_i.c_str(), ROT(i*45));
            }
            std::string str_m = "m";
            print_mat(str_m.c_str(), m);
            throw label;
        } 

        int num_angles = 7;
        ABCf angles[7] = {
            {0,45,90},
            {45,90,135},
            {90,135,180},
            {135,180,225},
            {180,225,270},
            {225,270,315},
            {270,315,360}
        };
        
        int num_weights = 10;
        ABCf weights[10] = {
            {1,1,1},

            {1,0,0},
            {0,1,0},
            {0,0,1},

            {1,1,0},
            {1,0,1},
            {0,1,1},

            {1,1,2},
            {1,2,1},
            {2,1,1}
        };
        float mats[3*16];
        float mat_mean[16];
        for (int i=0; i<num_angles; ++i)
        for (int k=0; k<num_weights; ++k)
        {
            float wsum = weights[k].a + weights[k].b + weights[k].c;

            float angle_mean = (1/wsum) * (
                angles[i].a * weights[k].a
              + angles[i].b * weights[k].b
              + angles[i].c * weights[k].c
            );
            rot(mats + 16*0, angles[i].a * d2r, axis);
            rot(mats + 16*1, angles[i].b * d2r, axis);
            rot(mats + 16*2, angles[i].c * d2r, axis);
            rot(mat_mean, angle_mean * d2r, axis);

            average_mat(m, mats, 3, &(weights[k].a)); test_equality("test_7", &error, m, mat_mean);
        }


    } // for (int axis=0; axis<3; axis++)
    #undef ROT


    return error;
}

int main() {

    int error = 0;
    error += test();
    return error;
}
