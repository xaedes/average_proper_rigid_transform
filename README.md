# average_affine_transform_mat
Small zero dependency C++ library to correctly average affine transformation matrices using quaternion SLERP.
Supports weighted and non-weighted average of two or more matrices/quaternions.

```cpp
int main() {
    float id[16] = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1
    };
    float rotZ_90[16] = {
        0,-1,0,0,
        1,0,0,0,
        0,0,1,0,
        0,0,0,1
    };
    float rotZ_45[16];
    average_mat(rotZ_45, id, rotZ_90);
}
```


```cpp
void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b);
void average_mat(Scalar* mat_average, const Scalar* mats, int num);
void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights);

void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b);
void average_quat(Scalar* quat_average, const Scalar* quats, int num);
void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);

void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);
void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);

void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);
void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);

void mat_to_quat(Scalar* quat, const Scalar* mat);
void quat_to_mat(Scalar* mat, const Scalar* quat);
```


Derived from https://github.com/g-truc/glm
