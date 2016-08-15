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

#endif /* _3DMATH_H_ */
