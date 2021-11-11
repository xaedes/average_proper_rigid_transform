#pragma once

#include <limits>
#include <cassert>
#include <cmath>


namespace average_affine_transform_mat {

    template<class Scalar=float> Scalar dot(const Scalar* a, const Scalar* b);

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

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                      
    void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>  
    void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        
    void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           
    void quat_to_mat(Scalar* mat, const Scalar* quat);
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           
    void mat_to_quat(Scalar* quat, const Scalar* mat);


    // Function declarations long line
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void average_mat(Scalar* mat_average, const Scalar* mat_a, const Scalar* mat_b);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>  void average_mat(Scalar* mat_average, const Scalar* mats, int num, const ScalarW* weights);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                       void average_mat(Scalar* mat_average, const Scalar* mats, int num);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             void average_quat(Scalar* quat_average, const Scalar* quat_a, const Scalar* quat_b);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarW=float>                                                        void average_quat(Scalar* quat_average, const Scalar* quats, int num, const ScalarW* weights);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0>                                                                             void average_quat(Scalar* quat_average, const Scalar* quats, int num);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                      void mix_mat(Scalar* mat_mix, const Scalar* mat_a, const Scalar* mat_b, ScalarK k_);
    template<class Scalar=float, int StrideMat = 16, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>  void mix_mat(Scalar* mats_mix, const Scalar* mat_a, const Scalar* mat_b, const ScalarK* ks, int num_k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, ScalarK k);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void mix_quat(Scalar* quat_mix, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, ScalarK k_);
    template<class Scalar=float, int QuatW = 3, int QuatXYZ = 0, class ScalarK=float>                                                        void slerp_quat(Scalar* quat_slerp, const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k);

    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void quat_to_mat(Scalar* mat, const Scalar* quat);
    template<class Scalar=float, int StrideY = 4, int StrideX = 1, int QuatW = 3, int QuatXYZ = 0>                                           void mat_to_quat(Scalar* quat, const Scalar* mat);

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
            Scalar sj = sin(j * angle) / sin(angle);
            Scalar sk = sin(k * angle) / sin(angle);
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


} // namespace average_affine_transform_mat
