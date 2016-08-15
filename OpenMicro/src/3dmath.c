#include "3dmath.h"

//TODO: add
float Q_rsqrt(float number);

void _v3d_add(float *retu, const float *v, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] += v[i];
    }
}
void _v3d_mulf(float *retu, const float m, int n) {
    for (int i = 0; i < n; i++) {
        retu[i] *= m;
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

void quaternion_mulf(float *retu, float m) {
    _v3d_mulf(retu, m, 4);
}

void quaternion_copy(float *retu, const float *v) {
    _v3d_copy(retu, v, 4);
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

void v3d_rotate(float *retu, const float *q) {
    float t1[4], t2[4], t3[4];
    t1[0] = 0.0f;
    v3d_copy(t1 + 1, retu);

    quaternion_product(t2, q, t1);
    quaternion_copy(t1, q);
    quaternion_product(t3, t2, t1);

    v3d_copy(retu, t3 + 1);
}

void v3d_set(float *retu, const uint8_t *data) {
    retu[0] = (int16_t) ((data[0] << 8) + data[1]);
    retu[1] = (int16_t) ((data[2] << 8) + data[3]);
    retu[2] = (int16_t) ((data[4] << 8) + data[5]);
}

void v3d_mulf(float *retu, float m) {
    _v3d_mulf(retu, m, 3);
}

void v3d_add(float *retu, const float *v3d) {
    _v3d_add(retu, v3d, 3);
}

void v3d_copy(float *retu, const float *v3d) {
    _v3d_copy(retu, v3d, 3);
}
