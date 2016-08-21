#include "3dmath.h"


/*based on:
    http://www.cprogramming.com/tutorial/3d/quaternions.html
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    https://en.wikipedia.org/wiki/Quaternion
*/

float Q_rsqrt(float number) {

    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (threehalfs - (x2 * y * y));   // 1st iteration
    y = y * (threehalfs - (x2 * y * y));   // 2nd iteration, this can be removed
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 3nd iteration, this can be removed

    return y;
}


void _v3d_muladd(float *retu, const float *v, float m, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] += m*v[i];
    }
}
void _v3d_add(float *retu, const float *v, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] += v[i];
    }
}
void _v3d_sub(float *retu, const float *v, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] -= v[i];
    }
}

void _v3d_mulf(float *retu, const float m, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] *= m;
    }
}
void _v3d_zero(float *retu, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] = 0.0f;
    }
}

void _v3d_copy(float *retu, const float *v, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] = v[i];
    }
}

float _v3d_square_sum(const float *v, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += v[i] * v[i];
    }
    return sum;
}


//quaterions

void quaternion_product(float *retu, const float * r, const float *q) {
    retu[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
    retu[1] = r[0] * q[1] + r[1] * q[0] + r[2] * q[3] - r[3] * q[2];
    retu[2] = r[0] * q[2] - r[1] * q[3] + r[2] * q[0] + r[3] * q[1];
    retu[3] = r[0] * q[3] + r[1] * q[2] - r[2] * q[1] + r[3] * q[0];
}

#define v3d_sin(val) (val)
#define v3d_cos(val) 1.0f


void quaternion_rotationX2(float *retu, const float * rv) {
    float c2x = v3d_cos(rv[0]);
    float c2y = v3d_cos(rv[1]);
    float c2z = v3d_cos(rv[2]);

    float s2x = v3d_sin(rv[0]);
    float s2y = v3d_sin(rv[1]);
    float s2z = v3d_sin(rv[2]);

    float q[4] ,q2[4];
    q[0] = c2x*c2y; q[1] = s2x*c2y; q[2] = c2x*s2y; q[3] = s2x*s2y;
    quaternion_copy(retu, q);
    quaternion_mulf(retu, c2z);
    quaternion_mulf(q, s2z);
    quaternion_mul_k(q2, q);
    quaternion_add(retu, q2);
}


void quaternion_conjugate(float *retu) {
    v3d_mulf(retu + 1, -1.0f);
}

float quaternion_inv_magnitude(const float *r) {
    return Q_rsqrt(_v3d_square_sum(r, 4));
}

float quaternion_magnitude(const float *r) {
    return 1.0f / quaternion_inv_magnitude(r);
}

void quaternion_normalize(float *r) {
    float m = quaternion_inv_magnitude(r);
    quaternion_mulf(r, m);
}
void quaternion_normalize_1(float *r) {
    float m = _v3d_square_sum(r+1, 3);
    r[0] = 1.0f/Q_rsqrt(1.0f-m);
//    r[0] = 1.0f-m;
}


void quaternion_normalize_t(float *r, float t) {
    float m = quaternion_inv_magnitude(r);
    quaternion_mulf(r, m*t);
}


void quaternion_mulf(float *retu, float m) {
    _v3d_mulf(retu, m, 4);
}

void quaternion_mul_k(float *retu, const float *q) {
    retu[0] = -q[3];
    retu[1] = q[2];
    retu[2] = -q[1];
    retu[3] = q[0];
}

void quaternion_add(float *retu, const float *q) {
    _v3d_add(retu, q, 4);
}

void quaternion_copy(float *retu, const float *v) {
    _v3d_copy(retu, v, 4);
}

void quaternion_rotate_k(float *retu, const float *q) {
    //https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    retu[0] = 0.0f;
    retu[1] = q[1]*q[3]+q[0]*q[2];
    retu[1] += retu[1]; //*2
    retu[2] = q[2]*q[3]-q[0]*q[1];
    retu[2] += retu[2]; //*2
    retu[3] = q[1]*q[1]+q[2]*q[2];
    retu[3] += retu[3]; //*2
    retu[3] = 1.0f - retu[3];
}


//v3d: 3d vector

float v3d_inv_magnitude(const float *r) {
    return Q_rsqrt(_v3d_square_sum(r, 3));
}

float v3d_magnitude(const float *r) {
    return 1.0f / v3d_inv_magnitude(r);
}

void v3d_normalize(float *r) {
    float m = v3d_inv_magnitude(r);
    v3d_mulf(r, m);
}

void v3d_normalize_t(float *r, float t) {
    float m = v3d_inv_magnitude(r);
    v3d_mulf(r, m*t);
}

void v3d_to_quaternion(float *q, const float *v3d) {
    q[0] = 0.0f;
    v3d_copy(q + 1, v3d);
}

void v3d_rotate(float *retu, const float *q) {
    float p[4], t2[4], t3[4];

    v3d_to_quaternion(p, retu);
    quaternion_product(t2, q, p);
    quaternion_copy(p, q);
    quaternion_conjugate(p);
    quaternion_product(t3, t2, p);

    v3d_copy(retu, t3 + 1);
}

void v3d_set(float *retu, const uint8_t *data) {
    for(int i=0;i<3;i++) {
        retu[i] = (int16_t) (((uint16_t)data[i*2] << 8) + data[i*2+1]);
    }
}

void v3d_mulf(float *retu, float m) {
    _v3d_mulf(retu, m, 3);
}

void v3d_add(float *retu, const float *v3d) {
    _v3d_muladd(retu, v3d, 1.0f, 3);
}

void v3d_sub(float *retu, const float *v3d) {
    _v3d_muladd(retu, v3d, -1.0f, 3);
}

void v3d_zero(float *retu) {
    _v3d_zero(retu, 3);
}

void v3d_copy(float *retu, const float *v3d) {
    _v3d_copy(retu, v3d, 3);
}



void v3d_rotate_90(float * v) {
    float t = v[1];
    v[1] = -v[0];
    v[0] = t;
}
void v3d_rotate_180(float * v) {
    v[1] = -v[1];
    v[0] = -v[0];
}
void v3d_rotate_270(float * v) {
    float t = v[1];
    v[1] = v[0];
    v[0] = -t;
}

