#pragma once

#include <limits>
#include <cassert>
#include <cmath>


namespace average_affine_transform_mat {

    template<class Scalar=float, int StrideX = 1, int StrideY = 4, int QuatW = 3, int QuatXYZ = 0>
    void quat_to_mat(const Scalar* quat, Scalar* mat)
    {
        // glm/gtc/quaternion.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER mat<3, 3, T, Q> mat3_cast(qua<T, Q> const& q)
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

    template<class Scalar=float, int StrideX = 1, int StrideY = 4, int QuatW = 3, int QuatXYZ = 0>
    void mat_to_quat(const Scalar* mat, Scalar* quat)
    {
        // glm/glm/gtc/quaternion.inl template<typename Scalar, qualifier Q> GLM_FUNC_QUALIFIER qua<Scalar, Q> quat_cast(mat<4, 4, Scalar, Q> const& m)
        
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

    template<class Scalar=float> Scalar dot(const Scalar* a, const Scalar* b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }


    template<class Scalar=float, class ScalarK=float, int QuatW = 3, int QuatXYZ = 0>
    void quat_slerp(const Scalar* quat_a, const Scalar* quat_b, ScalarK k, Scalar* quat_slerp)
    {
        // glm/ext/quaternion_common.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER qua<T, Q> slerp(qua<T, Q> const& x, qua<T, Q> const& y, T a)

        Scalar cosTheta = dot(quat_a, quat_b);
        // If cosTheta < 0, the interpolation will take the long way around the sphere.
        // To fix this, one quat must be negated.
        Scalar sign = (cosTheta < static_cast<Scalar>(0)) ? -static_cast<Scalar>(1) : +static_cast<Scalar>(1);
        Scalar z[4];
        z[0] = sign * quat_b[0];
        z[1] = sign * quat_b[1];
        z[2] = sign * quat_b[2];
        z[3] = sign * quat_b[3];

        // Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
        if(cosTheta > static_cast<Scalar>(1) - std::numeric_limits<Scalar>::epsilon())
        {
            // Linear interpolation
            ScalarK j = static_cast<ScalarK>(1)-k;
            quat_slerp[0] = quat_a[0] * j + z[0] * k;
            quat_slerp[1] = quat_a[1] * j + z[1] * k;
            quat_slerp[2] = quat_a[2] * j + z[2] * k;
            quat_slerp[3] = quat_a[3] * j + z[3] * k;
        }
        else
        {
            // Essential Mathematics, page 467
            ScalarK j = static_cast<ScalarK>(1)-k;
            Scalar angle = acos(cosTheta);
            Scalar sj = sin(j * angle) / sin(angle);
            Scalar sk = sin(k * angle) / sin(angle);
            quat_slerp[0] = quat_a[0] * sj + z[0] * sk;
            quat_slerp[1] = quat_a[1] * sj + z[1] * sk;
            quat_slerp[2] = quat_a[2] * sj + z[2] * sk;
            quat_slerp[3] = quat_a[3] * sj + z[3] * sk;
        }
    }

    template<class Scalar=float, class ScalarK=float, int QuatW = 3, int QuatXYZ = 0>
    void quat_slerp(const Scalar* quat_a, const Scalar* quat_b, const ScalarK* ks, int num_k, Scalar* quat_slerp)
    {
        // glm/ext/quaternion_common.inl template<typename T, qualifier Q> GLM_FUNC_QUALIFIER qua<T, Q> slerp(qua<T, Q> const& x, qua<T, Q> const& y, T a)

        Scalar cosTheta = dot(quat_a, quat_b);
        // If cosTheta < 0, the interpolation will take the long way around the sphere.
        // To fix this, one quat must be negated.
        Scalar sign = (cosTheta < static_cast<Scalar>(0)) ? -static_cast<Scalar>(1) : +static_cast<Scalar>(1);
        Scalar z[4];
        z[0] = sign * quat_b[0];
        z[1] = sign * quat_b[1];
        z[2] = sign * quat_b[2];
        z[3] = sign * quat_b[3];

        // Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
        if(cosTheta > static_cast<Scalar>(1) - std::numeric_limits<Scalar>::epsilon())
        {
            // Linear interpolation
            for (int i=0; i<num_k; ++i)
            {
                ScalarK k = ks[i];
                ScalarK j = static_cast<ScalarK>(1)-k;
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
            Scalar r_angle = static_cast<Scalar>(1) / angle;
            for (int i=0; i<num_k; ++i)
            {
                ScalarK k = ks[i];
                ScalarK j = static_cast<ScalarK>(1)-k;
                Scalar sj = sin(j * angle) * r_angle;
                Scalar sk = sin(k * angle) * r_angle;
                quat_slerp[i*4 + 0] = quat_a[0] * sj + z[0] * sk;
                quat_slerp[i*4 + 1] = quat_a[1] * sj + z[1] * sk;
                quat_slerp[i*4 + 2] = quat_a[2] * sj + z[2] * sk;
                quat_slerp[i*4 + 3] = quat_a[3] * sj + z[3] * sk;
            }
        }
    }

} // namespace average_affine_transform_mat
