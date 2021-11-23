
#include "average_proper_rigid_transform/average_proper_rigid_transform.h"
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <string>
#include <chrono>
#include <sstream>
#include <vector>


using namespace average_proper_rigid_transform;

const float pi = 3.14159265358979f;
const float d2r = pi / 180.0f;
const float r2d = pi / 180.0f;

template <class Scalar=float, int StrideY = 4, int StrideX = 1>
void print_tabular(const char* label, const Scalar* data, int cols=1, int rows=1)
{
    static std::vector<std::stringstream> cells;
    static std::vector<int> col_widths;
    if (cols * rows > cells.size())
        cells.resize(cols*rows);
    col_widths.clear();
    col_widths.resize(cols);

    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            cells[y*cols+x].str("");
            cells[y*cols+x] << data[y*StrideY + x*StrideX];
            int col_width = cells[y*cols+x].str().size();
            if (col_width > col_widths[x]) 
                col_widths[x] = col_width;
        }
    }
    if (label != nullptr)
    {
        std::cout << label << "\n";    
    }

    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            const auto& str = cells[y*cols+x].str();
            int col_width = str.size();
            for (int k=0; k<col_widths[x]-col_width; ++k)
            {
                std::cout << " ";
            }
            std::cout << str;
            if (x < cols-1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "\n";
    }   
    std::cout << "\n";
}

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

void test_equality(const char* label, int* error, const float *a, const float *b, int n=16, float eps=1e-4)
{
    for (int i=0; i<n; i++)
    {
        bool is_equal = (abs(a[i] - b[i]) < eps);
        //assert(is_equal);
        if (!is_equal) throw label;
        error[0] += is_equal ? 0 : 1;
    }
}

template <class TimeT  = std::chrono::duration<double, std::milli>,
          class ClockT = std::chrono::steady_clock>
struct MeasureTime
{
    std::chrono::time_point<ClockT> time_begin;
    std::chrono::time_point<ClockT> time_end;
    TimeT duration;
    void begin()
    {
        time_begin = ClockT::now();
    }
    double end()
    {
        time_end = ClockT::now();
        duration = time_end - time_begin;
        return duration.count();
    }
    double end(const char* label, int num_it)
    {
        end();
        std::cout << "duration " << label << ": " << (duration.count()/num_it) << " milli seconds\n";
        return duration.count();
    }
};


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

        int num_angles = 10;
        ABCf angles[10] = {
            {0,45,90},
            {45,90,135},
            {90,135,180},
            {135,180,225},
            {180,225,270},
            {225,270,315},
            {270,315,360},
            {315,270,225},
            {-45,0,+45},
            {+45,0,-45}
        };
        
        int num_weights = 16;
        ABCf weights[16] = {
            {1,1,1},

            {1,0,0},
            {0,1,0},
            {0,0,1},

            {1,1,0},
            {1,0,1},
            {0,1,1},

            {2,1,0},
            {2,0,1},
            {0,2,1},

            {1,2,0},
            {1,0,2},
            {0,1,2},


            {1,1,2},
            {1,2,1},
            {2,1,1}
        };
        float mats[3*16];
        float mat_mean[16];
        float quat_mean[4];
        float quats[300*4];
        float qqt_weights[300];
        float qqt1[16];
        // float qqt2[16];
        MeasureTime<> mtime;
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
            mat_to_quat(quat_mean, mat_mean);

            int num_repeat = 1;
            int num_items = 3*num_repeat;
            for (int j=0; j<num_repeat; ++j)
            {
                mat_to_quat(quats + 4*(0+j*num_items), mats + 16*0);
                mat_to_quat(quats + 4*(1+j*num_items), mats + 16*1);
                mat_to_quat(quats + 4*(2+j*num_items), mats + 16*2);
                qqt_weights[0+j*num_items] = weights[k].a;
                qqt_weights[1+j*num_items] = weights[k].b;
                qqt_weights[2+j*num_items] = weights[k].c;
            }
            int num_iter;
            num_iter = 10;
            // mtime.begin();
            for (int j=0; j<num_iter;++j)
            {
                compute_qqt(qqt1, quats, num_items, qqt_weights);
            }
            // mtime.end("compute_qqt", num_iter);

            float quats_avg[4];
            float quats_avg_eig[4];
            average_quat(quats_avg, quats, num_items, qqt_weights);
            average_quat_eig(quats_avg_eig, quats, num_items, qqt_weights);

            num_iter = 100;
            // mtime.begin();
            for (int j=0; j<num_iter;++j)
            {
                average_quat(quats_avg, quats, num_items, qqt_weights);
            }
            // mtime.end("average_quat", num_iter);

            // mtime.begin();
            for (int j=0; j<num_iter;++j)
            {
                average_quat_eig(quats_avg_eig, quats, num_items, qqt_weights);
            }
            // mtime.end("average_quat_eig", num_iter);

            // print_quat("quats_avg", quats_avg);
            // print_quat("quats_avg_eig", quats_avg_eig);
            bool different = false;
            for (int j = 0; j < 4; ++j)
            {
                if (abs(abs(quats_avg[j])-abs(quats_avg_eig[j])) > 1e-6)
                {
                    different = true;
                    break;
                }
            }
            bool weights_different = false;
            for (int j = 1; j < num_items; ++j)
            {
                if (abs(qqt_weights[j] - qqt_weights[j-1]) > 1e-6)
                {
                    weights_different = true;
                    break;
                }
            }
            // expected:
            // when weights are different the resulting averages may also be different.
            // but when weights are the same, the results should also be the same.

            if (different && !weights_different)
            {
                error += 1;
            }
            if (different && !weights_different)
            {
                std::cout << "weights_different " << weights_different << "\n";
                print_tabular("angles", &angles[i].a, 3);
                print_tabular("weights", &weights[k].a, 3);
                print_tabular("angle_mean", &angle_mean, 1);
                // std::cout << "angles:     " << angles[i].a << ", " << angles[i].b << angles[i].c << "\n";
                // std::cout << "weights:    " << weights[k].a << weights[k].b << weights[k].c << "\n";
                std::cout << "angle_mean: " << angle_mean << "\n";
                print_quat("quat_mean", quat_mean);
                print_quat("quats_avg", quats_avg);
                print_quat("quats_avg_eig", quats_avg_eig);
            }

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
