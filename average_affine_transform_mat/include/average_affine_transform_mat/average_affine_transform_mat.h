#pragma once

#include <limits>
#include <cassert>
#include <cmath>


namespace average_affine_transform_mat {

    template<class Scalar=float> Scalar dot(const Scalar* a, const Scalar* b);

    // void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b);
    // void average_mat(Scalar* mat_average, const Scalar* mats, int num);
    // void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights);

    // void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b);
    // void average_quat(Scalar* quat_average, const Scalar* quats, int num);
    // void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);
    // void average_quat_eig(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);

    // void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
    // void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
    // void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
    // void mix_quat(Scalar* quats_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    // void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
    // void slerp_quat(Scalar* quats_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    // void mat_to_quat(Scalar* quat, const Scalar* mat);
    // void quat_to_mat(Scalar* mat, const Scalar* quat);
    // 
    // unsigned int find_eigenvalues_sym_real(Scalar* eigenvalues, Scalar* eigenvectors, const Scalar* mat);
    
    // Function declarations short line
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           
    void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>  
    void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                       
    void average_mat(Scalar* mat_average, const Scalar* mats, int num);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             
    void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>                                                        
    void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             
    void average_quat(Scalar* quat_average, const Scalar* quats, int num);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>
    void average_quat_eig(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                      
    void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>  
    void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void mix_quat(Scalar* quats_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void slerp_quat(Scalar* quats_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           
    void quat_to_mat(Scalar* mat, const Scalar* quat);
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           
    void mat_to_quat(Scalar* quat, const Scalar* mat);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1>
    unsigned int find_eigenvalues_sym_real(Scalar* eigenvalues, Scalar* eigenvectors, const Scalar* mat);

    // Function declarations long line (scroll to the right)
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>  void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                       void average_mat(Scalar* mat_average, const Scalar* mats, int num);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>                                                        void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             void average_quat(Scalar* quat_average, const Scalar* quats, int num);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>                                                        void average_quat_eig(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                      void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>  void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void mix_quat(Scalar* quats_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void slerp_quat(Scalar* quats_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void quat_to_mat(Scalar* mat, const Scalar* quat);
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void mat_to_quat(Scalar* quat, const Scalar* mat);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1>                                                                           unsigned int find_eigenvalues_sym_real(Scalar* eigenvalues, Scalar* eigenvectors, const Scalar* mat);


    // Function definitions

    template<class Scalar=float> Scalar dot(const Scalar* a, const Scalar* b)
    {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    }

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>
    void quat_to_mat(Scalar* mat, const Scalar* quat)
    {
        // glm/gtc/quaternion.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER mat<3, 3, T, Q> mat3_cast(qua<T, Q> const& q)
        // see GLM.LICENSE file 
        Scalar qx = quat[QuatXYZ+0];
        Scalar qy = quat[QuatXYZ+1];
        Scalar qz = quat[QuatXYZ+2];
        Scalar qw = quat[QuatW];
        Scalar qxx(qx * qx);
        Scalar qyy(qy * qy);
        Scalar qzz(qz * qz);
        Scalar qxz(qx * qz);
        Scalar qxy(qx * qy);
        Scalar qyz(qy * qz);
        Scalar qwx(qw * qx);
        Scalar qwy(qw * qy);
        Scalar qwz(qw * qz);

        mat[StrideX*0+StrideY*0] = Scalar(1) - Scalar(2) * (qyy +  qzz);
        mat[StrideX*0+StrideY*1] = Scalar(2) * (qxy + qwz);
        mat[StrideX*0+StrideY*2] = Scalar(2) * (qxz - qwy);

        mat[StrideX*1+StrideY*0] = Scalar(2) * (qxy - qwz);
        mat[StrideX*1+StrideY*1] = Scalar(1) - Scalar(2) * (qxx +  qzz);
        mat[StrideX*1+StrideY*2] = Scalar(2) * (qyz + qwx);

        mat[StrideX*2+StrideY*0] = Scalar(2) * (qxz + qwy);
        mat[StrideX*2+StrideY*1] = Scalar(2) * (qyz - qwx);
        mat[StrideX*2+StrideY*2] = Scalar(1) - Scalar(2) * (qxx +  qyy);

        if ((StrideY == 4) || (StrideX == 4))
        {
            mat[StrideX*0+StrideY*3] = static_cast<Scalar>(0);
            mat[StrideX*1+StrideY*3] = static_cast<Scalar>(0);
            mat[StrideX*2+StrideY*3] = static_cast<Scalar>(0);

            mat[StrideX*3+StrideY*0] = static_cast<Scalar>(0);
            mat[StrideX*3+StrideY*1] = static_cast<Scalar>(0);
            mat[StrideX*3+StrideY*2] = static_cast<Scalar>(0);
            mat[StrideX*3+StrideY*3] = static_cast<Scalar>(1);
        }
    }

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>
    void mat_to_quat(Scalar* quat, const Scalar* mat)
    {
        // glm/glm/gtc/quaternion.inl template<typename Scalar, qualifier Q> GLM_FUNC_QUALIFIER qua<Scalar, Q> quat_cast(mat<4, 4, Scalar, Q> const& m)
        // see GLM.LICENSE file 
        
        Scalar m00 = mat[StrideX*0+StrideY*0];
        Scalar m11 = mat[StrideX*1+StrideY*1];
        Scalar m22 = mat[StrideX*2+StrideY*2];        
        Scalar fourX2Minus1 = m00 - m11 - m22;
        Scalar fourY2Minus1 = m11 - m00 - m22;
        Scalar fourZ2Minus1 = m22 - m00 - m11;
        Scalar fourW2Minus1 = m00 + m11 + m22;

        int biggestIndex = 0;
        Scalar fourBiggest2Minus1 = fourW2Minus1;
        if(fourX2Minus1 > fourBiggest2Minus1)
        {
            fourBiggest2Minus1 = fourX2Minus1;
            biggestIndex = 1;
        }
        if(fourY2Minus1 > fourBiggest2Minus1)
        {
            fourBiggest2Minus1 = fourY2Minus1;
            biggestIndex = 2;
        }
        if(fourZ2Minus1 > fourBiggest2Minus1)
        {
            fourBiggest2Minus1 = fourZ2Minus1;
            biggestIndex = 3;
        }

        Scalar biggestVal = sqrt(fourBiggest2Minus1 + static_cast<Scalar>(1)) * static_cast<Scalar>(0.5);
        Scalar mult = static_cast<Scalar>(0.25) / biggestVal;

        switch(biggestIndex)
        {
        case 0:
            quat[QuatXYZ+0] = (mat[StrideX*1+StrideY*2] - mat[StrideX*2+StrideY*1]) * mult;
            quat[QuatXYZ+1] = (mat[StrideX*2+StrideY*0] - mat[StrideX*0+StrideY*2]) * mult;
            quat[QuatXYZ+2] = (mat[StrideX*0+StrideY*1] - mat[StrideX*1+StrideY*0]) * mult;
            quat[QuatW] = biggestVal; 
            break;
        case 1:
            quat[QuatXYZ+0] = biggestVal;
            quat[QuatXYZ+1] = (mat[StrideX*0+StrideY*1] + mat[StrideX*1+StrideY*0]) * mult;
            quat[QuatXYZ+2] = (mat[StrideX*2+StrideY*0] + mat[StrideX*0+StrideY*2]) * mult;
            quat[QuatW] = (mat[StrideX*1+StrideY*2] - mat[StrideX*2+StrideY*1]) * mult;
            break;
        case 2:
            quat[QuatXYZ+0] = (mat[StrideX*0+StrideY*1] + mat[StrideX*1+StrideY*0]) * mult;
            quat[QuatXYZ+1] = biggestVal;
            quat[QuatXYZ+2] = (mat[StrideX*1+StrideY*2] + mat[StrideX*2+StrideY*1]) * mult;
            quat[QuatW] = (mat[StrideX*2+StrideY*0] - mat[StrideX*0+StrideY*2]) * mult;
            break;
        case 3:
            quat[QuatXYZ+0] = (mat[StrideX*2+StrideY*0] + mat[StrideX*0+StrideY*2]) * mult;
            quat[QuatXYZ+1] = (mat[StrideX*1+StrideY*2] + mat[StrideX*2+StrideY*1]) * mult;
            quat[QuatXYZ+2] = biggestVal;
            quat[QuatW] = (mat[StrideX*0+StrideY*1] - mat[StrideX*1+StrideY*0]) * mult;
            break;
        default: // Silence a -Wswitch-default warning in GCC. Should never actually get here. Assert is just for sanity.
            assert(false);
            quat[QuatXYZ+0] = 0;
            quat[QuatXYZ+1] = 0;
            quat[QuatXYZ+2] = 0;
            quat[QuatW] = 1;
        }
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_)
    {
        // glm/ext/quaternion_common.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER qua<T, Q> slerp(qua<T, Q> const& x, qua<T, Q> const& y, T a)
        // see GLM.LICENSE file 

        Scalar cosTheta = dot(quat_a, quat_b);
        // If cosTheta < 0, the interpolation will take the long way around the sphere.
        // To fix this, one quat must be negated.
        Scalar negate = (cosTheta < static_cast<Scalar>(0)) ? -static_cast<Scalar>(1) : +static_cast<Scalar>(1);
        Scalar z[4];
        z[QuatXYZ+0] = negate * quat_b[QuatXYZ+0];
        z[QuatXYZ+1] = negate * quat_b[QuatXYZ+1];
        z[QuatXYZ+2] = negate * quat_b[QuatXYZ+2];
        z[QuatW] = negate * quat_b[QuatW];
        cosTheta *= negate;

        Scalar k = static_cast<Scalar>(k_);
        // Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
        if(cosTheta > static_cast<Scalar>(1) - std::numeric_limits<Scalar>::epsilon())
        {
            // Linear interpolation
            Scalar j = static_cast<Scalar>(1)-k;
            quat_slerp[0] = quat_a[0] * j + z[0] * k;
            quat_slerp[1] = quat_a[1] * j + z[1] * k;
            quat_slerp[2] = quat_a[2] * j + z[2] * k;
            quat_slerp[3] = quat_a[3] * j + z[3] * k;
        }
        else
        {
            // Essential Mathematics, page 467
            Scalar j = static_cast<Scalar>(1)-k;
            Scalar angle = acos(cosTheta);
            Scalar r_s_angle = static_cast<Scalar>(1) / sin(angle);
            Scalar sj = sin(j * angle) * r_s_angle;
            Scalar sk = sin(k * angle) * r_s_angle;
            quat_slerp[0] = quat_a[0] * sj + z[0] * sk;
            quat_slerp[1] = quat_a[1] * sj + z[1] * sk;
            quat_slerp[2] = quat_a[2] * sj + z[2] * sk;
            quat_slerp[3] = quat_a[3] * sj + z[3] * sk;
        }
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k)
    {
        // glm/ext/quaternion_common.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER qua<T, Q> slerp(qua<T, Q> const& x, qua<T, Q> const& y, T a)
        // see GLM.LICENSE file 

        Scalar cosTheta = dot(quat_a, quat_b);
        // If cosTheta < 0, the interpolation will take the long way around the sphere.
        // To fix this, one quat must be negated.
        Scalar negate = (cosTheta < static_cast<Scalar>(0)) ? -static_cast<Scalar>(1) : +static_cast<Scalar>(1);
        Scalar z[4];
        z[QuatXYZ+0] = negate * quat_b[QuatXYZ+0];
        z[QuatXYZ+1] = negate * quat_b[QuatXYZ+1];
        z[QuatXYZ+2] = negate * quat_b[QuatXYZ+2];
        z[QuatW] = negate * quat_b[QuatW];
        cosTheta *= negate;

        // Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
        if(cosTheta > static_cast<Scalar>(1) - std::numeric_limits<Scalar>::epsilon())
        {
            // Linear interpolation
            for (int i=0; i<num_k; ++i)
            {
                Scalar k = static_cast<Scalar>(ks[i]);
                Scalar j = static_cast<Scalar>(1)-k;
                quat_slerp[i*4 + 0] = quat_a[0] * j + z[0] * k;
                quat_slerp[i*4 + 1] = quat_a[1] * j + z[1] * k;
                quat_slerp[i*4 + 2] = quat_a[2] * j + z[2] * k;
                quat_slerp[i*4 + 3] = quat_a[3] * j + z[3] * k;
            }
        }
        else
        {
            // Essential Mathematics, page 467
            Scalar angle = acos(cosTheta);
            Scalar r_s_angle = static_cast<Scalar>(1) / sin(angle);
            for (int i=0; i<num_k; ++i)
            {
                Scalar k = static_cast<Scalar>(ks[i]);
                Scalar j = static_cast<Scalar>(1)-k;
                Scalar sj = sin(j * angle) * r_s_angle;
                Scalar sk = sin(k * angle) * r_s_angle;
                quat_slerp[i*4 + 0] = quat_a[0] * sj + z[0] * sk;
                quat_slerp[i*4 + 1] = quat_a[1] * sj + z[1] * sk;
                quat_slerp[i*4 + 2] = quat_a[2] * sj + z[2] * sk;
                quat_slerp[i*4 + 3] = quat_a[3] * sj + z[3] * sk;
            }
        }
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k)
    {
        slerp_quat<Scalar,QuatW,QuatXYZ,ScalarK>(quat_mix, quat_a, quat_b, k);
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k)
    {
        slerp_quat<Scalar,QuatW,QuatXYZ,ScalarK>(quat_mix, quat_a, quat_b, ks, num_k);
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>
    void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b)
    {
        slerp_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_average, quat_a, quat_b, static_cast<Scalar>(0.5));
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>
    void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights)
    {
        // use welford's algorithm to calculate moving average. reformulate to
        // mix(x,y,k) which can be computed for quaternions via slerp.

        // welford's mean:
        // wmean = 0
        // wsum = 0
        // for i in [0..num-1]:
        //   wsum += w[i]
        //   delta = val[i] - wmean
        //   wmean := wmean + delta * (w[i]/wsum)
        //   
        // wmean assignmend transformed into mix formulation x*j+y*k, so we can replace it by mix(x,y,k) afterwards:
        // 
        // k = (w[i]/wsum)
        // j = 1-k
        // wmean := wmean + delta * (w[i]/wsum)
        //        = wmean + (val[i] - wmean) * k
        //        = wmean + val[i]*k - wmean*k
        //        = wmean*(1-k) + val[i]*k
        //        = wmean*j + val[i]*k
        // 
        // `wsum = 0` is unclear for wsum of type quaternion.
        // Avoid zero initialization:
        // wmean = val[0]
        // wsum = w[0]
        // for i in [1..num-1]:
        //   wsum += w[i]
        //   k = (w[i]/wsum)
        //   j = 1-k
        //   wmean := wmean*j + val[i]*k
        //          = mix(wmean,val[i],k)
        
        assert(num > 0);
        if (num <= 0) return;

        Scalar wsum = static_cast<Scalar>(weights[0]);
        quat_average[0] = quats[0];
        quat_average[1] = quats[1];
        quat_average[2] = quats[2];
        quat_average[3] = quats[3];
        for (int i=1; i<num; ++i)
        {
            Scalar w = static_cast<Scalar>(weights[i]);
            wsum += w;
            Scalar k = (wsum==0) ? 1 : (w/wsum);
            mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_average, quat_average, &quats[i*4], k);
        }
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>
    void average_quat(Scalar* quat_average, const Scalar* quats, int num)
    {
        assert(num > 0);
        if (num <= 0) return;

        quat_average[0] = quats[0];
        quat_average[1] = quats[1];
        quat_average[2] = quats[2];
        quat_average[3] = quats[3];
        for (int i=1; i<num; ++i)
        {
            Scalar k = 1/static_cast<Scalar>(1+i);
            mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_average, quat_average, &quats[i*4], k);
        }
    }

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_)
    {
        Scalar quat_a[4];
        Scalar quat_b[4];
        Scalar quat_mix[4];

        Scalar k = static_cast<Scalar>(k_);
        Scalar j = static_cast<Scalar>(1) - k;

        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_a, mat_a);
        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_b, mat_b);
        mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_mix, quat_a, quat_b, k);
        quat_to_mat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(mat_mix, quat_mix);
        for (int y=0; y<3; ++y)
        {
            int idx = StrideX*3+StrideY*y;
            mat_mix[idx] = mat_a[idx] * j + mat_b[idx] * k;
        }
    }

    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>
    void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k)
    {
        Scalar quat_a[4];
        Scalar quat_b[4];
        Scalar quat_mix[4];

        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_a, mat_a);
        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_b, mat_b);
        for (int i = 0; i < num_k; ++i)
        {
            Scalar k = static_cast<Scalar>(ks[i]);
            Scalar j = static_cast<Scalar>(1) - k;
            Scalar* mat = mats_mix + i*StrideMat;
            mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_mix, quat_a, quat_b, k);
            quat_to_mat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(mat , quat_mix);
            for (int y=0; y<3; ++y)
            {
                int idx = StrideX*3+StrideY*y;
                mat[idx] = mat_a[idx] * j + mat_b[idx] * k;
            }
        }
    }

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>
    void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b)
    {
        Scalar quat_a[4];
        Scalar quat_b[4];
        Scalar quat_average[4];

        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_a, mat_a);
        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_b, mat_b);
        average_quat<Scalar,QuatW,QuatXYZ>(quat_average, quat_a, quat_b);
        quat_to_mat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(mat_average, quat_average);
        for (int y=0; y<3; ++y)
        {
            int idx = StrideX*3+StrideY*y;
            mat_average[idx] = (mat_a[idx] + mat_b[idx]) * static_cast<Scalar>(0.5);
        }
    }
    
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>
    void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights)
    {
        assert(num > 0);
        if (num <= 0) return;
        Scalar quat_average[4];
        Scalar pos_average[3];
        Scalar quat[4];
        Scalar wsum = static_cast<Scalar>(weights[0]);
        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_average, mats);
        pos_average[0] = mats[StrideX*3+StrideY*0];
        pos_average[1] = mats[StrideX*3+StrideY*1];
        pos_average[2] = mats[StrideX*3+StrideY*2];
        for (int i=1; i<num; ++i)
        {
            Scalar w = static_cast<Scalar>(weights[i]);
            wsum += w;
            Scalar k = (wsum==0) ? 1 : (w/wsum);
            Scalar j = static_cast<Scalar>(1) - k;
            const Scalar* mat = mats + i*StrideMat;
            mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat, mat);
            mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_average, quat_average, quat, k);
            pos_average[0] = pos_average[0] * j + mat[StrideX*3+StrideY*0] * k;
            pos_average[1] = pos_average[1] * j + mat[StrideX*3+StrideY*1] * k;
            pos_average[2] = pos_average[2] * j + mat[StrideX*3+StrideY*2] * k;
        }
        quat_to_mat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(mat_average, quat_average);
        mat_average[StrideX*3+StrideY*0] = pos_average[0];
        mat_average[StrideX*3+StrideY*1] = pos_average[1];
        mat_average[StrideX*3+StrideY*2] = pos_average[2];
    }
    
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>
    void average_mat(Scalar* mat_average, const Scalar* mats, int num)
    {
        assert(num > 0);
        if (num <= 0) return;
        Scalar quat_average[4];
        Scalar pos_average[3];
        Scalar quat[4];
        mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat_average, mats);
        pos_average[0] = mats[StrideX*3+StrideY*0];
        pos_average[1] = mats[StrideX*3+StrideY*1];
        pos_average[2] = mats[StrideX*3+StrideY*2];
        for (int i=1; i<num; ++i)
        {
            Scalar k = 1/static_cast<Scalar>(1+i);
            Scalar j = static_cast<Scalar>(1) - k;
            const Scalar* mat = mats + i*StrideMat;
            mat_to_quat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(quat, mat);
            mix_quat<Scalar,QuatW,QuatXYZ,Scalar>(quat_average, quat_average, quat, k);
            pos_average[0] = pos_average[0] * j + mat[StrideX*3+StrideY*0] * k;
            pos_average[1] = pos_average[1] * j + mat[StrideX*3+StrideY*1] * k;
            pos_average[2] = pos_average[2] * j + mat[StrideX*3+StrideY*2] * k;
        }
        quat_to_mat<Scalar,StrideY,StrideX,QuatW,QuatXYZ>(mat_average, quat_average);
        mat_average[StrideX*3+StrideY*0] = pos_average[0];
        mat_average[StrideX*3+StrideY*1] = pos_average[1];
        mat_average[StrideX*3+StrideY*2] = pos_average[2];
    }

    // eigen analysis based averaging:

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>
    void compute_qqt(Scalar* qqt, const Scalar* quats, int num, const ScalarW* weights)
    {
        Scalar wsum = 0;
        for (int k=0; k<num; ++k)
        {
            wsum += static_cast<Scalar>(weights[k]);
        }
        if (wsum == 0) wsum = 1;
        for (int y = 0; y < 4; ++y)
        {
            for (int x = 0; x < 4; ++x)
            {
                int xy = StrideX*x + StrideY*y;
                qqt[xy] = 0;
            }
        }
        for (int k=0; k<num; ++k)
        {
            Scalar w = static_cast<Scalar>(weights[k]) / wsum;
            Scalar w2 = w;//*w;
            const Scalar* quat = quats + k*4;
            for (int y = 0; y < 4; ++y)
            {
                // only compute values of upper matrix elements
                for (int x = y; x < 4; ++x)
                {
                    int xy = StrideX*x + StrideY*y;
                    qqt[xy] += w2 * quat[y] * quat[x];
                }
            }
        }
        // mirror elements in lower matrix from upper matrix elements
        for (int y = 0; y < 4; ++y)
        {
            for (int x = 0; x < y; ++x)
            {
                int xy = StrideX*x + StrideY*y;
                int yx = StrideX*y + StrideY*x;
                qqt[xy] = qqt[yx];
            }
        }

    }

    template<class Scalar=float>
    int argmin(const Scalar* values, int num)
    {
        if (num <= 0) return num;

        int best_idx = 0;
        Scalar best = values[best_idx];
        for (int i=1; i<num; ++i)
        {
            Scalar v = values[i];
            if (v < best)
            {
                best_idx = i;
                best = v;
            }
        }
        return best_idx;
    }

    template<class Scalar=float>
    int argmax(const Scalar* values, int num)
    {
        if (num <= 0) return num;

        int best_idx = 0;
        Scalar best = values[best_idx];
        for (int i=1; i<num; ++i)
        {
            Scalar v = values[i];
            if (v > best)
            {
                best_idx = i;
                best = v;
            }
        }
        return best_idx;
    }

    template<class Scalar=float>
    bool approx_equal(Scalar a, Scalar b, Scalar eps)
    {
        return abs(a-b) < eps;
    }

    template<class Scalar=float>
    Scalar transfer_sign(Scalar v, Scalar s)
    {
        return ((s) >= 0 ? abs(v) : -abs(v));
    }

    template<class Scalar=float>
    Scalar pythag(Scalar a, Scalar b) 
    {
        // glm/gtx/pca.inl pythag
        // see GLM.LICENSE file 
        static const Scalar epsilon = static_cast<Scalar>(0.0000001);
        Scalar absa = abs(a);
        Scalar absb = abs(b);
        if(absa > absb) 
        {
            absb /= absa;
            absb *= absb;
            return absa * sqrt(static_cast<Scalar>(1) + absb);
        }
        if(approx_equal<Scalar>(absb,0,epsilon)) return static_cast<Scalar>(0);
        absa /= absb;
        absa *= absa;
        return absb * sqrt(static_cast<Scalar>(1) + absa);
    }

    /// Assuming the provided covariance matrix `covarMat` is symmetric and real-valued, this function find the `D` Eigenvalues of the matrix, and also provides the corresponding Eigenvectors.
    /// Note: the data in `outEigenvalues` and `outEigenvectors` are in matching order, i.e. `outEigenvector[i]` is the Eigenvector of the Eigenvalue `outEigenvalue[i]`.
    /// This is a numeric implementation to find the Eigenvalues, using 'QL decomposition` (variant of QR decomposition: https://en.wikipedia.org/wiki/QR_decomposition).
    /// @param covarMat A symmetric, real-valued covariance matrix, e.g. computed from `computeCovarianceMatrix`.
    /// @param outEigenvalues Vector to receive the found eigenvalues
    /// @param outEigenvectors Matrix to receive the found eigenvectors corresponding to the found eigenvalues, as column vectors
    /// @return The number of eigenvalues found, usually D if the precondition of the covariance matrix is met.
    template<class Scalar=float, int StrideY = 4, int StrideX = 1>
    unsigned int find_eigenvalues_sym_real(Scalar* eigenvalues, Scalar* eigenvectors, const Scalar* mat)
    {
        // glm/gtx/pca.inl findEigenvaluesSymReal(...)
        // see GLM.LICENSE file 
        constexpr int D = 4;

        Scalar a[D * D]; // matrix -- input and workspace for algorithm (will be changed inplace)
        Scalar d[D]; // diagonal elements
        Scalar e[D]; // off-diagonal elements

        for(int r = 0; r < D; r++)
            for(int c = 0; c < D; c++)
                a[(r) * D + (c)] = mat[c*StrideX + r*StrideY];

        // 1. Householder reduction.
        int l, k, j, i;
        Scalar scale, hh, h, g, f;
        static const Scalar epsilon = static_cast<Scalar>(0.0000001);

        for(i = D; i >= 2; i--)
        {
            l = i - 1;
            h = scale = 0;
            if(l > 1)
            {
                for(k = 1; k <= l; k++)
                {
                    scale += abs(a[(i - 1) * D + (k - 1)]);
                }
                if(approx_equal<Scalar>(scale, 0, epsilon))
                {
                    e[i - 1] = a[(i - 1) * D + (l - 1)];
                }
                else
                {
                    for(k = 1; k <= l; k++)
                    {
                        a[(i - 1) * D + (k - 1)] /= scale;
                        h += a[(i - 1) * D + (k - 1)] * a[(i - 1) * D + (k - 1)];
                    }
                    f = a[(i - 1) * D + (l - 1)];
                    g = ((f >= 0) ? -sqrt(h) : sqrt(h));
                    e[i - 1] = scale * g;
                    h -= f * g;
                    a[(i - 1) * D + (l - 1)] = f - g;
                    f = 0;
                    for(j = 1; j <= l; j++)
                    {
                        a[(j - 1) * D + (i - 1)] = a[(i - 1) * D + (j - 1)] / h;
                        g = 0;
                        for(k = 1; k <= j; k++)
                        {
                            g += a[(j - 1) * D + (k - 1)] * a[(i - 1) * D + (k - 1)];
                        }
                        for(k = j + 1; k <= l; k++)
                        {
                            g += a[(k - 1) * D + (j - 1)] * a[(i - 1) * D + (k - 1)];
                        }
                        e[j - 1] = g / h;
                        f += e[j - 1] * a[(i - 1) * D + (j - 1)];
                    }
                    hh = f / (h + h);
                    for(j = 1; j <= l; j++)
                    {
                        f = a[(i - 1) * D + (j - 1)];
                        e[j - 1] = g = e[j - 1] - hh * f;
                        for(k = 1; k <= j; k++)
                        {
                            a[(j - 1) * D + (k - 1)] -= (f * e[k - 1] + g * a[(i - 1) * D + (k - 1)]);
                        }
                    }
                }
            }
            else
            {
                e[i - 1] = a[(i - 1) * D + (l - 1)];
            }
            d[i - 1] = h;
        }
        d[0] = 0;
        e[0] = 0;
        for(i = 1; i <= D; i++)
        {
            l = i - 1;
            if(!approx_equal<Scalar>(d[i - 1], 0, epsilon))
            {
                for(j = 1; j <= l; j++)
                {
                    g = 0;
                    for(k = 1; k <= l; k++)
                    {
                        g += a[(i - 1) * D + (k - 1)] * a[(k - 1) * D + (j - 1)];
                    }
                    for(k = 1; k <= l; k++)
                    {
                        a[(k - 1) * D + (j - 1)] -= g * a[(k - 1) * D + (i - 1)];
                    }
                }
            }
            d[i - 1] = a[(i - 1) * D + (i - 1)];
            a[(i - 1) * D + (i - 1)] = 1;
            for(j = 1; j <= l; j++)
            {
                a[(j - 1) * D + (i - 1)] = a[(i - 1) * D + (j - 1)] = 0;
            }
        }

        // 2. Calculation of eigenvalues and eigenvectors (QL algorithm)
        int m, iter;
        Scalar s, r, p, dd, c, b;
        const int MAX_ITER = 30;

        for(i = 2; i <= D; i++)
        {
            e[i - 2] = e[i - 1];
        }
        e[D - 1] = 0;

        for(l = 1; l <= D; l++)
        {
            iter = 0;
            do
            {
                for(m = l; m <= D - 1; m++)
                {
                    dd = abs(d[m - 1]) + abs(d[m - 1 + 1]);
                    if(approx_equal<Scalar>(abs(e[m - 1]) + dd, dd, epsilon))
                        break;
                }
                if(m != l)
                {
                    if(iter++ == MAX_ITER)
                    {
                        return 0; // Too many iterations in FindEigenvalues
                    }
                    g = (d[l - 1 + 1] - d[l - 1]) / (2 * e[l - 1]);
                    r = pythag<Scalar>(g, 1);
                    g = d[m - 1] - d[l - 1] + e[l - 1] / (g + transfer_sign(r, g));
                    s = c = 1;
                    p = 0;
                    for(i = m - 1; i >= l; i--)
                    {
                        f = s * e[i - 1];
                        b = c * e[i - 1];
                        e[i - 1 + 1] = r = pythag(f, g);
                        if(approx_equal<Scalar>(r, 0, epsilon))
                        {
                            d[i - 1 + 1] -= p;
                            e[m - 1] = 0;
                            break;
                        }
                        s = f / r;
                        c = g / r;
                        g = d[i - 1 + 1] - p;
                        r = (d[i - 1] - g) * s + 2 * c * b;
                        d[i - 1 + 1] = g + (p = s * r);
                        g = c * r - b;
                        for(k = 1; k <= D; k++)
                        {
                            f = a[(k - 1) * D + (i - 1 + 1)];
                            a[(k - 1) * D + (i - 1 + 1)] = s * a[(k - 1) * D + (i - 1)] + c * f;
                            a[(k - 1) * D + (i - 1)] = c * a[(k - 1) * D + (i - 1)] - s * f;
                        }
                    }
                    if(approx_equal<Scalar>(r, 0, epsilon) && (i >= l))
                        continue;
                    d[l - 1] -= p;
                    e[l - 1] = g;
                    e[m - 1] = 0;
                }
            } while(m != l);
        }

        // 3. output
        for(i = 0; i < D; i++)
            eigenvalues[i] = d[i];
        for(i = 0; i < D; i++)
            for(j = 0; j < D; j++)
                eigenvectors[i*StrideX+j*StrideY] = a[(j) * D + (i)];

        return D;
    }

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>
    void average_quat_eig(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights)
    {
        // Markley, F.L., Cheng, Y., Crassidis, J.L. and Oshman, Y., 2007. Averaging quaternions. Journal of Guidance, Control, and Dynamics, 30(4), pp.1193-1197.
        
        // https://stackoverflow.com/a/27410865/798588
        //
        // let Q = [w_1 * q_1, w_2 * q_2, ..., w_n * q_n]
        //
        // where w_i is the weight of the i-th quaternion and q_i is the i-tz
        // quaternion as column vector. Q is therefore a 4xN matrix. The
        // normalized eigenvector corresponding to the largest eigenvalue of
        // Q*Q^T is the weighted average. Since Q*Q^T is self-adjoint and at
        // least positive semi-definite, fast and robust methods of solving that
        // eigenproblem are available. Computing the matrix-matrix product is
        // the only step that grows with the number of elements being averaged.
        // Q*Q^T is a symmetric 4x4 matrix.
        
        constexpr int StrideX = 1;
        constexpr int StrideY = 4;
        Scalar qqt[4*4];
        Scalar qqt_eigval[4];
        Scalar qqt_eigvec[4*4];
        compute_qqt<Scalar,StrideY,StrideX,QuatW,QuatXYZ,ScalarW>(qqt, quats, num, weights);
        find_eigenvalues_sym_real<Scalar,StrideY,StrideX>(qqt_eigval, qqt_eigvec, qqt);
        int biggest_eigval = argmax(qqt_eigval, 4);
        for (int i = 0; i < 4; ++i)
        {
            quat_average[i] = qqt_eigvec[i*StrideY+biggest_eigval*StrideX];
        }
    }



} // namespace average_affine_transform_mat
