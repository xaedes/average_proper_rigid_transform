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
// eigen analysis-based averaging
void average_quat_eig(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);

void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
void mix_quat(Scalar* quats_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

void mat_to_quat(Scalar* quat, const Scalar* mat);
void quat_to_mat(Scalar* mat, const Scalar* quat);

unsigned int find_eigenvalues_sym_real(Scalar* eigenvalues, Scalar* eigenvectors, const Scalar* mat);
```


Derived from https://github.com/g-truc/glm


---

# Comparision between slerp and eigen analysis based quaternion average

Both methods give different results for weighted averages, but the slerp based method is probably what you want.

Consider the averaging of two quaternions A and B corresponding to rotations around X-Axis by 0° and by 90° degree with weights w_A = 2 and w_B = 1.
The expected weighted average should correspond to a rotation around X-Axis by 30°.
Slerp-based weighted average will return the expected value.
Eigen analysis-based weighted average will return a rotation by 26.56°.

Eigen analysis-based method will return the quaternion which minimized Frobenius norm of the corresponding rotation matrices. The slerp based method will instead compute the average in 3D rotational space in terms of quaternions.

```python
import math
import numpy as np
import quaternion # (pip install numpy-quaternion)
d2r = math.pi/180
r2d = 180/math.pi

def recover_angle(mat):
    return np.arctan2(mat[1,0], mat[0,0])

# ground truth
angles = np.array([0,90])
weights = np.array([2,1])
mean_angle = np.sum(angles*(weights/weights.sum()))
quaternion_mean = quaternion.from_euler_angles(mean_angle*d2r,0,0)

# eigen analysis
Q = quaternion.as_float_array(
    [
        (weight**0.5) * quaternion.from_euler_angles(0,0,angle*d2r) 
        for angle,weight 
        in zip(angles,weights)
    ]
).T
QQT = Q @ Q.T
eigval,eigvec = np.linalg.eig(QQT)
quaternion_mean_eig = quaternion.from_float_array( eigvec[:,np.argmax(eigval)] )

# slerp based
def slerp_avg(quaternions, weights):
    # welford's mean in terms of linear mix operations:
    toqt = quaternion.from_float_array
    mix = lambda a,b,k: quaternion.slerp(toqt(a),toqt(b),0,1,k)
    wmean, wsum, num = quaternions[0], weights[0], len(weights)
    for i in range(1,num):
        wsum += weights[i]
        k = (weights[i]/wsum)
        # wmean := wmean*(1-k) + quaternions[i]*k
        wmean = mix(wmean,quaternions[i],k) 
    return wmean

quaternion_mean_slerp = slerp_avg(quaternion.as_float_array(
    [quaternion.from_euler_angles(0,0,angle*d2r) for angle in angles]
), weights)

mat_mean = quaternion.as_rotation_matrix(quaternion_mean)
mat_mean_eig = quaternion.as_rotation_matrix(quaternion_mean_eig)
mat_mean_slerp = quaternion.as_rotation_matrix(quaternion_mean_slerp)

print("expected", recover_angle(mat_mean)*r2d)
print("eigen", recover_angle(mat_mean_eig)*r2d)
print("slerp", recover_angle(mat_mean_slerp)*r2d)
```

Outputs:
```
expected 29.999999999999996
eigen 26.565051177077994
slerp 30.00000000000001
```

