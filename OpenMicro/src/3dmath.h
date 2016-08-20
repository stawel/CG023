#ifndef _3DMATH_H_
#define _3DMATH_H_

#include <stdint.h>

// Quaternion: float[4]
// Vector3d:   float[3]

void quaternion_product(float * retu, const float *r, const float *q);
void quaternion_conjugate(float *retu);
float quaternion_magnitude(const float *q);
float quaternion_inv_magnitude(const float *q);
void quaternion_normalize(float *retu);
void quaternion_copy(float *retu, const float *q);
void quaternion_mulf(float *retu, float m);
void quaternion_mul_k(float *retu);
void quaternion_add(float *retu, const float *q);
void quaternion_rotationX2(float *retu, const float * rvector);

float v3d_magnitude(const float *v3d);
float v3d_inv_magnitude(const float *v3d);
void v3d_normalize();
void v3d_rotate(float *retu, const float *q);
void v3d_set(float *retu, const uint8_t *data);
void v3d_zero(float *retu);
void v3d_mulf(float *retu, float a);
void v3d_add(float *retu, const float *v3d);
void v3d_sub(float *retu, const float *v3d);
void v3d_copy(float *retu, const float *v3d);


void v3d_rotate_90(float * v);
void v3d_rotate_180(float * v);
void v3d_rotate_270(float * v);

#define v3d_static_rotate2(v, a) v3d_rotate_ ## a (v)
#define v3d_static_rotate(v, a)  v3d_static_rotate2(v, a)

#endif /* _3DMATH_H_ */
