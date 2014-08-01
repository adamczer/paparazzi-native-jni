/*
 * Copyright (C) 2008-2011 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file pprz_algebra_float.h
 *   @brief Paparazzi floating point algebra.
 *
 *   This is the more detailed description of this file.
 *
 */

#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H

#include "pprz_algebra.h"

#include <math.h>
#include <float.h> // for FLT_MIN

/* this seems to be missing for some arch */
#ifndef M_SQRT2
#define M_SQRT2         1.41421356237309504880
#endif

struct FloatVect2 {
  float x;
  float y;
};

struct FloatVect3 {
  float x;
  float y;
  float z;
};

/**
 * @brief Roation quaternion
 */
struct FloatQuat {
  float qi;
  float qx;
  float qy;
  float qz;
};

struct FloatMat33 {
  float m[3*3];
};

/**
 * @brief rotation matrix
 */
struct FloatRMat {
  float m[3*3];
};

/**
 * @brief euler angles
 * @details Units: radians */
struct FloatEulers {
  float phi; ///< in radians
  float theta; ///< in radians
  float psi; ///< in radians
};

/**
 * @brief angular rates
 * @details Units: rad/s */
struct FloatRates {
  float p; ///< in rad/s
  float q; ///< in rad/s
  float r; ///< in rad/s
};

#define FLOAT_ANGLE_NORMALIZE(_a) {             \
    while (_a >  M_PI) _a -= (2.*M_PI);             \
    while (_a < -M_PI) _a += (2.*M_PI);             \
  }

//
//
// Vector algebra
//
//


/*
 * Dimension 2 Vectors
 */

#define FLOAT_VECT2_ZERO(_v) VECT2_ASSIGN(_v, 0., 0.)

/* a =  {x, y} */
#define FLOAT_VECT2_ASSIGN(_a, _x, _y) VECT2_ASSIGN(_a, _x, _y)

/* a = b */
#define FLOAT_VECT2_COPY(_a, _b) VECT2_COPY(_a, _b)

/* a += b */
#define FLOAT_VECT2_ADD(_a, _b)  VECT2_ADD(_a, _b)

/* c = a + b */
#define FLOAT_VECT2_SUM(_c, _a, _b) VECT2_SUM(_c, _a, _b)

/* c = a - b */
#define FLOAT_VECT2_DIFF(_c, _a, _b) VECT2_DIFF(_c, _a, _b)

/* a -= b */
#define FLOAT_VECT2_SUB(_a, _b) VECT2_SUB(_a, _b)

/* _vo = _vi * _s */
#define FLOAT_VECT2_SMUL(_vo, _vi, _s) VECT2_SMUL(_vo, _vi, _s)

#define FLOAT_VECT2_NORM2(_v) ((_v).x*(_v).x + (_v).y*(_v).y)

#define FLOAT_VECT2_NORM(_n, _v) {               \
    _n = sqrtf(FLOAT_VECT2_NORM2(_v));           \
  }

#define FLOAT_VECT2_NORMALIZE(_v) {             \
    const float n = sqrtf(FLOAT_VECT2_NORM2(_v)); \
    FLOAT_VECT2_SMUL(_v, _v, 1./n);             \
  }

#define FLOAT_VECT2_DOT_PRODUCT(_v1, _v2) ((_v1).x*(_v2).x + (_v1).y*(_v2).y)

/*
 * Dimension 3 Vectors
 */

#define FLOAT_VECT3_SUM(_a, _b, _c) VECT3_SUM(_a, _b, _c)
#define FLOAT_VECT3_SDIV(_a, _b, _s) VECT3_SDIV(_a, _b, _s)
#define FLOAT_VECT3_COPY(_a, _b) VECT3_COPY(_a, _b)

#define FLOAT_VECT3_ZERO(_v) VECT3_ASSIGN(_v, 0., 0., 0.)

#define FLOAT_VECT3_ASSIGN(_a, _x, _y, _z) VECT3_ASSIGN(_a, _x, _y, _z)

/* c = a - b */
#define FLOAT_VECT3_DIFF(_c, _a, _b) VECT3_DIFF(_c, _a, _b)

/* a -= b */
#define FLOAT_VECT3_SUB(_a, _b) VECT3_SUB(_a, _b)

#define FLOAT_VECT3_ADD(_a, _b) VECT3_ADD(_a, _b)

/* _vo = _vi * _s */
#define FLOAT_VECT3_SMUL(_vo, _vi, _s) VECT3_SMUL(_vo, _vi, _s)

#define FLOAT_VECT3_MUL(_v1, _v2) VECT3_MUL(_v1, _v2)

#define FLOAT_VECT3_NORM2(_v) ((_v).x*(_v).x + (_v).y*(_v).y + (_v).z*(_v).z)

#define FLOAT_VECT3_NORM(_v) (sqrtf(FLOAT_VECT3_NORM2(_v)))

#define FLOAT_VECT3_DOT_PRODUCT( _v1, _v2) ((_v1).x*(_v2).x + (_v1).y*(_v2).y + (_v1).z*(_v2).z)

#define FLOAT_VECT3_CROSS_PRODUCT(_vo, _v1, _v2) {          \
    (_vo).x = (_v1).y*(_v2).z - (_v1).z*(_v2).y;            \
    (_vo).y = (_v1).z*(_v2).x - (_v1).x*(_v2).z;            \
    (_vo).z = (_v1).x*(_v2).y - (_v1).y*(_v2).x;            \
  }

#define FLOAT_VECT3_INTEGRATE_FI(_vo, _dv, _dt) {           \
  (_vo).x += (_dv).x * (_dt);                       \
  (_vo).y += (_dv).y * (_dt);                       \
  (_vo).z += (_dv).z * (_dt);                       \
  }



#define FLOAT_VECT3_NORMALIZE(_v) {     \
    const float n = FLOAT_VECT3_NORM(_v);   \
    FLOAT_VECT3_SMUL(_v, _v, 1./n);     \
  }

#define FLOAT_RATES_ZERO(_r) {          \
    RATES_ASSIGN(_r, 0., 0., 0.);       \
  }

#define FLOAT_RATES_NORM(_v) (sqrtf((_v).p*(_v).p + (_v).q*(_v).q + (_v).r*(_v).r))

#define FLOAT_RATES_ADD_SCALED_VECT(_ro, _v, _s) {  \
    _ro.p += _v.x * _s;                 \
    _ro.q += _v.y * _s;                 \
    _ro.r += _v.z * _s;                 \
  }


#define FLOAT_RATES_LIN_CMB(_ro, _r1, _s1, _r2, _s2) {          \
    _ro.p = _s1 * _r1.p + _s2 * _r2.p;                  \
    _ro.q = _s1 * _r1.q + _s2 * _r2.q;                  \
    _ro.r = _s1 * _r1.r + _s2 * _r2.r;                  \
  }


#define FLOAT_RATES_SCALE(_ro,_s) {         \
    _ro.p *= _s;                    \
    _ro.q *= _s;                    \
    _ro.r *= _s;                    \
  }

#define FLOAT_RATES_INTEGRATE_FI(_ra, _racc, _dt) {         \
  (_ra).p += (_racc).p * (_dt);                     \
  (_ra).q += (_racc).q * (_dt);                     \
  (_ra).r += (_racc).r * (_dt);                     \
  }

#define FLOAT_RATES_OF_EULER_DOT(_ra, _e, _ed) {            \
    _ra.p =  _ed.phi                         - sinf(_e.theta)             *_ed.psi; \
    _ra.q =           cosf(_e.phi)*_ed.theta + sinf(_e.phi)*cosf(_e.theta)*_ed.psi; \
    _ra.r =          -sinf(_e.phi)*_ed.theta + cosf(_e.phi)*cosf(_e.theta)*_ed.psi; \
  }



/*
 * 3x3 matrices
 */
#define FLOAT_MAT33_ZERO(_m) {                      \
    MAT33_ELMT(_m, 0, 0) = 0.;                      \
    MAT33_ELMT(_m, 0, 1) = 0.;                      \
    MAT33_ELMT(_m, 0, 2) = 0.;                                          \
    MAT33_ELMT(_m, 1, 0) = 0.;                      \
    MAT33_ELMT(_m, 1, 1) = 0.;                      \
    MAT33_ELMT(_m, 1, 2) = 0.;                      \
    MAT33_ELMT(_m, 2, 0) = 0.;                      \
    MAT33_ELMT(_m, 2, 1) = 0.;                      \
    MAT33_ELMT(_m, 2, 2) = 0.;                      \
  }

#define FLOAT_MAT33_DIAG(_m, _d00, _d11, _d22) {            \
    MAT33_ELMT(_m, 0, 0) = _d00;                    \
    MAT33_ELMT(_m, 0, 1) = 0.;                      \
    MAT33_ELMT(_m, 0, 2) = 0.;                      \
    MAT33_ELMT(_m, 1, 0) = 0.;                      \
    MAT33_ELMT(_m, 1, 1) = _d11;                    \
    MAT33_ELMT(_m, 1, 2) = 0.;                      \
    MAT33_ELMT(_m, 2, 0) = 0.;                      \
    MAT33_ELMT(_m, 2, 1) = 0.;                      \
    MAT33_ELMT(_m, 2, 2) = _d22;                    \
  }


//
//
// Rotation Matrices
//
//


/** initialises a matrix to identity */
#define FLOAT_RMAT_ZERO(_rm) FLOAT_MAT33_DIAG(_rm, 1., 1., 1.)

#define FLOAT_RMAT_OF_AXIS_ANGLE(_rm, _uv, _an) float_rmat_of_axis_angle(&(_rm), &(_uv), _an)

/** initialises a rotation matrix from unit vector axis and angle */
static inline void float_rmat_of_axis_angle(struct FloatRMat *rm, struct FloatVect3 *uv, float angle) {

  const float ux2  = uv->x * uv->x;
  const float uy2  = uv->y * uv->y;
  const float uz2  = uv->z * uv->z;
  const float uxuy = uv->x * uv->y;
  const float uyuz = uv->y * uv->z;
  const float uxuz = uv->x * uv->z;
  const float can  = cosf(angle);
  const float san  = sinf(angle);
  const float one_m_can = (1. - can);

  RMAT_ELMT(*rm, 0, 0) = ux2 + (1.-ux2)*can;
  RMAT_ELMT(*rm, 0, 1) = uxuy*one_m_can + uv->z*san;
  RMAT_ELMT(*rm, 0, 2) = uxuz*one_m_can - uv->y*san;
  RMAT_ELMT(*rm, 1, 0) = RMAT_ELMT(*rm, 0, 1);
  RMAT_ELMT(*rm, 1, 1) = uy2 + (1.-uy2)*can;
  RMAT_ELMT(*rm, 1, 2) = uyuz*one_m_can + uv->x*san;
  RMAT_ELMT(*rm, 2, 0) = RMAT_ELMT(*rm, 0, 2);
  RMAT_ELMT(*rm, 2, 1) = RMAT_ELMT(*rm, 1, 2);
  RMAT_ELMT(*rm, 2, 2) = uz2 + (1.-uz2)*can;
  }

/* multiply _vin by _rmat, store in _vout */
#define FLOAT_RMAT_VECT3_MUL(_vout, _rmat, _vin) RMAT_VECT3_MUL(_vout, _rmat, _vin)
#define FLOAT_RMAT_VECT3_TRANSP_MUL(_vout, _rmat, _vin) RMAT_VECT3_TRANSP_MUL(_vout, _rmat, _vin)

#define FLOAT_RMAT_TRANSP_RATEMULT(_vb, _m_b2a, _va) {          \
    (_vb).p = ( (_m_b2a).m[0]*(_va).p + (_m_b2a).m[3]*(_va).q + (_m_b2a).m[6]*(_va).r); \
    (_vb).q = ( (_m_b2a).m[1]*(_va).p + (_m_b2a).m[4]*(_va).q + (_m_b2a).m[7]*(_va).r); \
    (_vb).r = ( (_m_b2a).m[2]*(_va).p + (_m_b2a).m[5]*(_va).q + (_m_b2a).m[8]*(_va).r); \
  }

#define FLOAT_RMAT_RATEMULT(_vb, _m_a2b, _va) {             \
    (_vb).p = ( (_m_a2b).m[0]*(_va).p + (_m_a2b).m[1]*(_va).q + (_m_a2b).m[2]*(_va).r); \
    (_vb).q = ( (_m_a2b).m[3]*(_va).p + (_m_a2b).m[4]*(_va).q + (_m_a2b).m[5]*(_va).r); \
    (_vb).r = ( (_m_a2b).m[6]*(_va).p + (_m_a2b).m[7]*(_va).q + (_m_a2b).m[8]*(_va).r); \
  }

#define FLOAT_RMAT_COMP(_m_a2c, _m_a2b, _m_b2c) float_rmat_comp(&(_m_a2c), &(_m_a2b), &(_m_b2c))
/* _m_a2c = _m_a2b comp _m_b2c , aka  _m_a2c = _m_b2c * _m_a2b */
static inline void float_rmat_comp(struct FloatRMat *m_a2c, struct FloatRMat *m_a2b, struct FloatRMat *m_b2c) {
  m_a2c->m[0] = m_b2c->m[0] * m_a2b->m[0] + m_b2c->m[1] * m_a2b->m[3] + m_b2c->m[2] * m_a2b->m[6];
  m_a2c->m[1] = m_b2c->m[0] * m_a2b->m[1] + m_b2c->m[1] * m_a2b->m[4] + m_b2c->m[2] * m_a2b->m[7];
  m_a2c->m[2] = m_b2c->m[0] * m_a2b->m[2] + m_b2c->m[1] * m_a2b->m[5] + m_b2c->m[2] * m_a2b->m[8];
  m_a2c->m[3] = m_b2c->m[3] * m_a2b->m[0] + m_b2c->m[4] * m_a2b->m[3] + m_b2c->m[5] * m_a2b->m[6];
  m_a2c->m[4] = m_b2c->m[3] * m_a2b->m[1] + m_b2c->m[4] * m_a2b->m[4] + m_b2c->m[5] * m_a2b->m[7];
  m_a2c->m[5] = m_b2c->m[3] * m_a2b->m[2] + m_b2c->m[4] * m_a2b->m[5] + m_b2c->m[5] * m_a2b->m[8];
  m_a2c->m[6] = m_b2c->m[6] * m_a2b->m[0] + m_b2c->m[7] * m_a2b->m[3] + m_b2c->m[8] * m_a2b->m[6];
  m_a2c->m[7] = m_b2c->m[6] * m_a2b->m[1] + m_b2c->m[7] * m_a2b->m[4] + m_b2c->m[8] * m_a2b->m[7];
  m_a2c->m[8] = m_b2c->m[6] * m_a2b->m[2] + m_b2c->m[7] * m_a2b->m[5] + m_b2c->m[8] * m_a2b->m[8];
  }


#define FLOAT_RMAT_COMP_INV(_m_a2b, _m_a2c, _m_b2c) float_rmat_comp_inv(&(_m_a2b), &(_m_a2c), &(_m_b2c))
/* _m_a2b = _m_a2c comp_inv _m_b2c , aka  _m_a2b = inv(_m_b2c) * _m_a2c */
static inline void float_rmat_comp_inv(struct FloatRMat *m_a2b, struct FloatRMat *m_a2c, struct FloatRMat *m_b2c) {
  m_a2b->m[0] = m_b2c->m[0] * m_a2c->m[0] + m_b2c->m[3] * m_a2c->m[3] + m_b2c->m[6] * m_a2c->m[6];
  m_a2b->m[1] = m_b2c->m[0] * m_a2c->m[1] + m_b2c->m[3] * m_a2c->m[4] + m_b2c->m[6] * m_a2c->m[7];
  m_a2b->m[2] = m_b2c->m[0] * m_a2c->m[2] + m_b2c->m[3] * m_a2c->m[5] + m_b2c->m[6] * m_a2c->m[8];
  m_a2b->m[3] = m_b2c->m[1] * m_a2c->m[0] + m_b2c->m[4] * m_a2c->m[3] + m_b2c->m[7] * m_a2c->m[6];
  m_a2b->m[4] = m_b2c->m[1] * m_a2c->m[1] + m_b2c->m[4] * m_a2c->m[4] + m_b2c->m[7] * m_a2c->m[7];
  m_a2b->m[5] = m_b2c->m[1] * m_a2c->m[2] + m_b2c->m[4] * m_a2c->m[5] + m_b2c->m[7] * m_a2c->m[8];
  m_a2b->m[6] = m_b2c->m[2] * m_a2c->m[0] + m_b2c->m[5] * m_a2c->m[3] + m_b2c->m[8] * m_a2c->m[6];
  m_a2b->m[7] = m_b2c->m[2] * m_a2c->m[1] + m_b2c->m[5] * m_a2c->m[4] + m_b2c->m[8] * m_a2c->m[7];
  m_a2b->m[8] = m_b2c->m[2] * m_a2c->m[2] + m_b2c->m[5] * m_a2c->m[5] + m_b2c->m[8] * m_a2c->m[8];
  }


/* _m_b2a = inv(_m_a2b) = transp(_m_a2b) */
#define FLOAT_RMAT_INV(_m_b2a, _m_a2b) {            \
    RMAT_ELMT(_m_b2a, 0, 0) = RMAT_ELMT(_m_a2b, 0, 0);      \
    RMAT_ELMT(_m_b2a, 0, 1) = RMAT_ELMT(_m_a2b, 1, 0);      \
    RMAT_ELMT(_m_b2a, 0, 2) = RMAT_ELMT(_m_a2b, 2, 0);      \
    RMAT_ELMT(_m_b2a, 1, 0) = RMAT_ELMT(_m_a2b, 0, 1);      \
    RMAT_ELMT(_m_b2a, 1, 1) = RMAT_ELMT(_m_a2b, 1, 1);      \
    RMAT_ELMT(_m_b2a, 1, 2) = RMAT_ELMT(_m_a2b, 2, 1);      \
    RMAT_ELMT(_m_b2a, 2, 0) = RMAT_ELMT(_m_a2b, 0, 2);      \
    RMAT_ELMT(_m_b2a, 2, 1) = RMAT_ELMT(_m_a2b, 1, 2);      \
    RMAT_ELMT(_m_b2a, 2, 2) = RMAT_ELMT(_m_a2b, 2, 2);      \
  }

#define FLOAT_RMAT_NORM(_m) (                       \
    sqrtf(SQUARE((_m).m[0])+ SQUARE((_m).m[1])+ SQUARE((_m).m[2])+  \
      SQUARE((_m).m[3])+ SQUARE((_m).m[4])+ SQUARE((_m).m[5])+  \
      SQUARE((_m).m[6])+ SQUARE((_m).m[7])+ SQUARE((_m).m[8]))  \
    )

#define FLOAT_RMAT_OF_EULERS(_rm, _e)     float_rmat_of_eulers_321(&(_rm), &(_e))
#define FLOAT_RMAT_OF_EULERS_321(_rm, _e) float_rmat_of_eulers_321(&(_rm), &(_e))
#define FLOAT_RMAT_OF_EULERS_312(_rm, _e) float_rmat_of_eulers_312(&(_rm), &(_e))
#define FLOAT_RMAT_OF_QUAT(_rm, _q)       float_rmat_of_quat(&(_rm), &(_q))

/* C n->b rotation matrix */
static inline void float_rmat_of_eulers_321(struct FloatRMat *rm, struct FloatEulers *e) {

  const float sphi   = sinf(e->phi);
  const float cphi   = cosf(e->phi);
  const float stheta = sinf(e->theta);
  const float ctheta = cosf(e->theta);
  const float spsi   = sinf(e->psi);
  const float cpsi   = cosf(e->psi);

  RMAT_ELMT(*rm, 0, 0) = ctheta*cpsi;
  RMAT_ELMT(*rm, 0, 1) = ctheta*spsi;
  RMAT_ELMT(*rm, 0, 2) = -stheta;
  RMAT_ELMT(*rm, 1, 0) = sphi*stheta*cpsi - cphi*spsi;
  RMAT_ELMT(*rm, 1, 1) = sphi*stheta*spsi + cphi*cpsi;
  RMAT_ELMT(*rm, 1, 2) = sphi*ctheta;
  RMAT_ELMT(*rm, 2, 0) = cphi*stheta*cpsi + sphi*spsi;
  RMAT_ELMT(*rm, 2, 1) = cphi*stheta*spsi - sphi*cpsi;
  RMAT_ELMT(*rm, 2, 2) = cphi*ctheta;
  }

static inline void float_rmat_of_eulers_312(struct FloatRMat *rm, struct FloatEulers *e) {

  const float sphi   = sinf(e->phi);
  const float cphi   = cosf(e->phi);
  const float stheta = sinf(e->theta);
  const float ctheta = cosf(e->theta);
  const float spsi   = sinf(e->psi);
  const float cpsi   = cosf(e->psi);

  RMAT_ELMT(*rm, 0, 0) =  ctheta*cpsi - sphi * stheta * spsi;
  RMAT_ELMT(*rm, 0, 1) =  ctheta*spsi + sphi * stheta * cpsi;
  RMAT_ELMT(*rm, 0, 2) = -cphi * stheta;
  RMAT_ELMT(*rm, 1, 0) = -cphi * spsi;
  RMAT_ELMT(*rm, 1, 1) =  cphi * cpsi;
  RMAT_ELMT(*rm, 1, 2) =  sphi;
  RMAT_ELMT(*rm, 2, 0) =  stheta*cpsi + sphi*ctheta*spsi;
  RMAT_ELMT(*rm, 2, 1) =  stheta*spsi - sphi*ctheta*cpsi;
  RMAT_ELMT(*rm, 2, 2) =  cphi*ctheta;
  }


/* C n->b rotation matrix */
static inline void float_rmat_of_quat(struct FloatRMat *rm, struct FloatQuat *q) {
  const float _a = M_SQRT2 * q->qi;
  const float _b = M_SQRT2 * q->qx;
  const float _c = M_SQRT2 * q->qy;
  const float _d = M_SQRT2 * q->qz;
  const float a2_1 = _a*_a-1;
  const float ab = _a*_b;
  const float ac = _a*_c;
  const float ad = _a*_d;
  const float bc = _b*_c;
  const float bd = _b*_d;
  const float cd = _c*_d;
  RMAT_ELMT(*rm, 0, 0) = a2_1+_b*_b;
  RMAT_ELMT(*rm, 0, 1) = bc+ad;
  RMAT_ELMT(*rm, 0, 2) = bd-ac;
  RMAT_ELMT(*rm, 1, 0) = bc-ad;
  RMAT_ELMT(*rm, 1, 1) = a2_1+_c*_c;
  RMAT_ELMT(*rm, 1, 2) = cd+ab;
  RMAT_ELMT(*rm, 2, 0) = bd+ac;
  RMAT_ELMT(*rm, 2, 1) = cd-ab;
  RMAT_ELMT(*rm, 2, 2) = a2_1+_d*_d;
  }

#define FLOAT_RMAT_INTEGRATE_FI(_rm, _omega, _dt) float_rmat_integrate_fi(&(_rm), &(_omega), &(_dt))
/** in place first order integration of a rotation matrix */
static inline void float_rmat_integrate_fi(struct FloatRMat *rm, struct FloatRates *omega, float dt) {
  struct FloatRMat exp_omega_dt = {
    { 1.        ,  dt*omega->r, -dt*omega->q,
     -dt*omega->r,  1.        ,  dt*omega->p,
      dt*omega->q, -dt*omega->p,  1.          }};
  struct FloatRMat R_tdt;
  FLOAT_RMAT_COMP(R_tdt, *rm, exp_omega_dt);
  memcpy(rm, &R_tdt, sizeof(R_tdt));
  }


static inline float renorm_factor(float n) {
  if (n < 1.5625f && n > 0.64f)
    return .5 * (3-n);
  else if (n < 100.0f && n > 0.01f)
    return  1. / sqrtf(n);
  else
    return 0.;
}

static inline float float_rmat_reorthogonalize(struct FloatRMat* rm) {

  const struct FloatVect3 r0 = {RMAT_ELMT(*rm, 0,0),
                                RMAT_ELMT(*rm, 0,1),
                                RMAT_ELMT(*rm, 0,2)};
  const struct FloatVect3 r1 = {RMAT_ELMT(*rm, 1,0),
                                RMAT_ELMT(*rm, 1,1),
                                RMAT_ELMT(*rm, 1,2)};
  float _err = -0.5*FLOAT_VECT3_DOT_PRODUCT(r0, r1);
  struct FloatVect3 r0_t;
  VECT3_SUM_SCALED(r0_t, r0, r1, _err);
  struct FloatVect3 r1_t;
  VECT3_SUM_SCALED(r1_t,  r1, r0, _err);
  struct FloatVect3 r2_t;
  FLOAT_VECT3_CROSS_PRODUCT(r2_t, r0_t, r1_t);
  float s = renorm_factor(FLOAT_VECT3_NORM2(r0_t));
  MAT33_ROW_VECT3_SMUL(*rm, 0, r0_t, s);
  s = renorm_factor(FLOAT_VECT3_NORM2(r1_t));
  MAT33_ROW_VECT3_SMUL(*rm, 1, r1_t, s);
  s = renorm_factor(FLOAT_VECT3_NORM2(r2_t));
  MAT33_ROW_VECT3_SMUL(*rm, 2, r2_t, s);

  return _err;

}

//
//
// Quaternion algebras
//
//

#define FLOAT_QUAT_ZERO(_q) QUAT_ASSIGN(_q, 1., 0., 0., 0.)

#define FLOAT_QUAT_ASSIGN(_qi, _i, _x, _y, _z) QUAT_ASSIGN(_qi, _i, _x, _y, _z)

/* _q += _qa */
#define FLOAT_QUAT_ADD(_qo, _qi) QUAT_ADD(_qo, _qi)

/* _qo = _qi * _s */
#define FLOAT_QUAT_SMUL(_qo, _qi, _s) QUAT_SMUL(_qo, _qi, _s)

/* _qo = _qo / _s */
#define FLOAT_QUAT_SDIV( _qo, _qi, _s) QUAT_SDIV(_qo, _qi, _s)

/*  */
#define FLOAT_QUAT_EXPLEMENTARY(b,a) QUAT_EXPLEMENTARY(b,a)

/*  */
#define FLOAT_QUAT_COPY(_qo, _qi) QUAT_COPY(_qo, _qi)

#define FLOAT_QUAT_NORM(_q) float_quat_norm(&(_q))
#define FLOAT_QUAT_NORMALIZE(_q) float_quat_normalize(&(_q))

static inline float float_quat_norm(struct FloatQuat *q) {
  return sqrtf(SQUARE(q->qi) + SQUARE(q->qx)+  SQUARE(q->qy) + SQUARE(q->qz));
}

static inline void float_quat_normalize(struct FloatQuat *q) {
  float qnorm = float_quat_norm(q);
  if (qnorm > FLT_MIN) {
    q->qi = q->qi / qnorm;
    q->qx = q->qx / qnorm;
    q->qy = q->qy / qnorm;
    q->qz = q->qz / qnorm;
  }
  }

/*   */
#define FLOAT_QUAT_EXTRACT(_vo, _qi) QUAT_EXTRACT_Q(_vo, _qi)

/* Be careful : after invert make a normalization */
#define FLOAT_QUAT_INVERT(_qo, _qi) QUAT_INVERT(_qo, _qi)

#define FLOAT_QUAT_WRAP_SHORTEST(_q) float_quat_wrap_shortest(&(_q))
static inline void float_quat_wrap_shortest(struct FloatQuat *q) {
  if (q->qi < 0.)
    QUAT_EXPLEMENTARY(*q,*q);
  }

/*
 *
 * Rotation Matrix using quaternions
 *
 */

 /*
  * The (non commutative) quaternion product * then reads
  *
  *         [    p0.q0 - p.q      ]
  * p * q = [                     ]
  *         [ p0.q + q0.p + p x q ]
  *
  */

  /* (qi)-1 * vi * qi represents R_q of n->b on vectors vi
   *
   *  "FLOAT_QUAT_EXTRACT : Extracted of the vector part"
   */

#define FLOAT_QUAT_RMAT_N2B(_n2b, _qi, _vi){    \
                                                \
  struct FloatQuat quatinv;                     \
  struct FloatVect3 quat3, v1, v2;              \
  float qi;                                     \
                                                \
  FLOAT_QUAT_INVERT(quatinv, _qi);              \
  FLOAT_QUAT_NORMALIZE(quatinv);                \
                                                \
  FLOAT_QUAT_EXTRACT(quat3, quatinv);           \
  qi = - FLOAT_VECT3_DOT_PRODUCT(quat3, _vi);   \
  FLOAT_VECT3_CROSS_PRODUCT(v1, quat3, _vi);    \
  FLOAT_VECT3_SMUL(v2, _vi, (quatinv.qi)) ;     \
  FLOAT_VECT3_ADD(v2, v1);                      \
                                                \
  FLOAT_QUAT_EXTRACT(quat3, _qi);               \
  FLOAT_VECT3_CROSS_PRODUCT(_n2b, v2, quat3);   \
  FLOAT_VECT3_SMUL(v1, v2, (_qi).qi);           \
  FLOAT_VECT3_ADD(_n2b,v1);                     \
  FLOAT_VECT3_SMUL(v1, quat3, qi);              \
  FLOAT_VECT3_ADD(_n2b,v1);                     \
}

  /*
   * qi * vi * (qi)-1 represents R_q of b->n on vectors vi
   */
#define FLOAT_QUAT_RMAT_B2N(_b2n,_qi,_vi){  \
                                            \
  struct FloatQuat _quatinv;                \
                                            \
                                            \
  FLOAT_QUAT_INVERT(_quatinv, _qi);         \
  FLOAT_QUAT_NORMALIZE(_quatinv);           \
                                            \
  FLOAT_QUAT_RMAT_N2B(_b2n, _quatinv, _vi); \
}

  /* Right multiplication by a quaternion
   *
   * vi * qi
   *
   */
#define FLOAT_QUAT_VMUL_RIGHT(_mright,_qi,_vi){ \
                                                \
  struct FloatVect3 quat3, v1, v2;              \
  float qi;                                     \
                                                \
  FLOAT_QUAT_EXTRACT(quat3, _qi);               \
  qi = - FLOAT_VECT3_DOT_PRODUCT(_vi, quat3);   \
  FLOAT_VECT3_CROSS_PRODUCT(v1, _vi, quat3);    \
  FLOAT_VECT3_SMUL(v2, _vi, (_qi.qi));          \
  FLOAT_VECT3_ADD(v2, v1);                      \
  FLOAT_QUAT_ASSIGN(_mright, qi, v2.x, v2.y, v2.z);\
}


  /* Left multiplication by a quaternion
  *
  * qi * vi
  *
  */
#define FLOAT_QUAT_VMUL_LEFT(_mleft,_qi,_vi){ \
                                              \
  struct FloatVect3 quat3, v1, v2;            \
  float qi;                                   \
                                              \
  FLOAT_QUAT_EXTRACT(quat3, _qi);             \
  qi = - FLOAT_VECT3_DOT_PRODUCT(quat3, _vi); \
  FLOAT_VECT3_CROSS_PRODUCT(v1, quat3, _vi);  \
  FLOAT_VECT3_SMUL(v2, _vi, (_qi.qi));        \
  FLOAT_VECT3_ADD(v2, v1);                    \
  FLOAT_QUAT_ASSIGN(_mleft, qi, v2.x, v2.y, v2.z);\
}


#define FLOAT_QUAT_COMP(_a2c, _a2b, _b2c) float_quat_comp(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_MULT(_a2c, _a2b, _b2c) float_quat_comp(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_INV_COMP(_b2c, _a2b, _a2c) float_quat_inv_comp(&(_b2c), &(_a2b), &(_a2c))
#define FLOAT_QUAT_COMP_INV(_a2b, _a2c, _b2c) float_quat_comp_inv(&(_a2b), &(_a2c), &(_b2c))
#define FLOAT_QUAT_COMP_NORM_SHORTEST(_a2c, _a2b, _b2c) float_quat_comp_norm_shortest(&(_a2c), &(_a2b), &(_b2c))
#define FLOAT_QUAT_COMP_INV_NORM_SHORTEST(_a2b, _a2c, _b2c) float_quat_comp_inv_norm_shortest(&(_a2b), &(_a2c), &(_b2c))
#define FLOAT_QUAT_INV_COMP_NORM_SHORTEST(_b2c, _a2b, _a2c) float_quat_inv_comp_norm_shortest(&(_b2c), &(_a2b), &(_a2c))
/* _a2c = _a2b comp _b2c , aka  _a2c = _a2b * _b2c */
static inline void float_quat_comp(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c) {
  a2c->qi = a2b->qi*b2c->qi - a2b->qx*b2c->qx - a2b->qy*b2c->qy - a2b->qz*b2c->qz;
  a2c->qx = a2b->qi*b2c->qx + a2b->qx*b2c->qi + a2b->qy*b2c->qz - a2b->qz*b2c->qy;
  a2c->qy = a2b->qi*b2c->qy - a2b->qx*b2c->qz + a2b->qy*b2c->qi + a2b->qz*b2c->qx;
  a2c->qz = a2b->qi*b2c->qz + a2b->qx*b2c->qy - a2b->qy*b2c->qx + a2b->qz*b2c->qi;
  }

/* _a2b = _a2c comp_inv _b2c , aka  _a2b = _a2c * inv(_b2c) */
static inline void float_quat_comp_inv(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c) {
  a2b->qi =  a2c->qi*b2c->qi + a2c->qx*b2c->qx + a2c->qy*b2c->qy + a2c->qz*b2c->qz;
  a2b->qx = -a2c->qi*b2c->qx + a2c->qx*b2c->qi - a2c->qy*b2c->qz + a2c->qz*b2c->qy;
  a2b->qy = -a2c->qi*b2c->qy + a2c->qx*b2c->qz + a2c->qy*b2c->qi - a2c->qz*b2c->qx;
  a2b->qz = -a2c->qi*b2c->qz - a2c->qx*b2c->qy + a2c->qy*b2c->qx + a2c->qz*b2c->qi;
  }

/* _b2c = _a2b inv_comp _a2c , aka  _b2c = inv(_a2b) * _a2c */
static inline void float_quat_inv_comp(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c) {
  b2c->qi = a2b->qi*a2c->qi + a2b->qx*a2c->qx + a2b->qy*a2c->qy + a2b->qz*a2c->qz;
  b2c->qx = a2b->qi*a2c->qx - a2b->qx*a2c->qi - a2b->qy*a2c->qz + a2b->qz*a2c->qy;
  b2c->qy = a2b->qi*a2c->qy + a2b->qx*a2c->qz - a2b->qy*a2c->qi - a2b->qz*a2c->qx;
  b2c->qz = a2b->qi*a2c->qz - a2b->qx*a2c->qy + a2b->qy*a2c->qx - a2b->qz*a2c->qi;
}

/* _a2c = _a2b comp _b2c , aka  _a2c = _a2b * _b2c */
static inline void float_quat_comp_norm_shortest(struct FloatQuat *a2c, struct FloatQuat *a2b, struct FloatQuat *b2c) {
  float_quat_comp(a2c, a2b, b2c);
  float_quat_wrap_shortest(a2c);
  float_quat_normalize(a2c);
  }

/* _a2b = _a2c comp_inv _b2c , aka  _a2b = _a2c * inv(_b2c) */
static inline void float_quat_comp_inv_norm_shortest(struct FloatQuat *a2b, struct FloatQuat *a2c, struct FloatQuat *b2c) {
  float_quat_comp_inv(a2b, a2c, b2c);
  float_quat_wrap_shortest(a2b);
  float_quat_normalize(a2b);
  }

/* _b2c = _a2b inv_comp _a2c , aka  _b2c = inv(_a2b) * _a2c */
static inline void float_quat_inv_comp_norm_shortest(struct FloatQuat *b2c, struct FloatQuat *a2b, struct FloatQuat *a2c) {
  float_quat_inv_comp(b2c, a2b, a2c);
  float_quat_wrap_shortest(b2c);
  float_quat_normalize(b2c);
  }


#define FLOAT_QUAT_DIFFERENTIAL float_quat_differential(&(q_out), &(w), dt)
static inline void float_quat_differential(struct FloatQuat *q_out, struct FloatRates *w, float dt) {
  const float v_norm = sqrtf(w->p*w->p + w->q*w->q + w->r*w->r);
  const float c2 = cos(dt*v_norm/2.0);
  const float s2 = sin(dt*v_norm/2.0);
  if (v_norm < 1e-8) {
    q_out->qi = 1;
    q_out->qx = 0;
    q_out->qy = 0;
    q_out->qz = 0;
  } else {
    q_out->qi = c2;
    q_out->qx = w->p / v_norm * s2;
    q_out->qy = w->q / v_norm * s2;
    q_out->qz = w->r / v_norm * s2;
  }
  }

#define FLOAT_QUAT_INTEGRATE(_q, _omega, _dt) float_quat_integrate(&(_q), &(_omega), _dt)
/** in place quaternion integration with constant rotational velocity */
static inline void float_quat_integrate(struct FloatQuat *q, struct FloatRates *omega, float dt) {
  const float no = FLOAT_RATES_NORM(*omega);
  if (no > FLT_MIN) {
    const float a  = 0.5*no*dt;
    const float ca = cosf(a);
    const float sa_ov_no = sinf(a)/no;
    const float dp = sa_ov_no * omega->p;
    const float dq = sa_ov_no * omega->q;
    const float dr = sa_ov_no * omega->r;
    const float qi = q->qi;
    const float qx = q->qx;
    const float qy = q->qy;
    const float qz = q->qz;
    q->qi = ca*qi - dp*qx - dq*qy - dr*qz;
    q->qx = dp*qi + ca*qx + dr*qy - dq*qz;
    q->qy = dq*qi - dr*qx + ca*qy + dp*qz;
    q->qz = dr*qi + dq*qx - dp*qy + ca*qz;
  }
  }

#define FLOAT_QUAT_VMULT(v_out, q, v_in) float_quat_vmult(&(v_out), &(q), &(v_in))
static inline void float_quat_vmult(struct FloatVect3 *v_out, struct FloatQuat *q,struct FloatVect3 * v_in) {
  const float qi2_M1_2  = q->qi*q->qi - 0.5;
  const float qiqx = q->qi*q->qx;
  const float qiqy = q->qi*q->qy;
  const float qiqz = q->qi*q->qz;
  float m01  = q->qx*q->qy;   /* aka qxqy */
  float m02  = q->qx*q->qz;   /* aka qxqz */
  float m12  = q->qy*q->qz;   /* aka qyqz */

  const float m00  = qi2_M1_2 + q->qx*q->qx;
  const float m10  = m01 - qiqz;
  const float m20  = m02 + qiqy;
  const float m21  = m12 - qiqx;
  m01 += qiqz;
  m02 -= qiqy;
  m12 += qiqx;
  const float m11  = qi2_M1_2 + q->qy*q->qy;
  const float m22  = qi2_M1_2 + q->qz*q->qz;
  v_out->x = 2*(m00 * v_in->x + m01 * v_in->y + m02 * v_in->z);
  v_out->y = 2*(m10 * v_in->x + m11 * v_in->y + m12 * v_in->z);
  v_out->z = 2*(m20 * v_in->x + m21 * v_in->y + m22 * v_in->z);
  }

#define FLOAT_QUAT_DERIVATIVE(_qd, _r, _q) float_quat_derivative(&(_qd), &(_r), &(_q))
#define FLOAT_QUAT_DERIVATIVE_LAGRANGE(_qd, _r, _q) float_quat_derivative_lagrange(&(_qd), &(_r), &(_q))

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 */
static inline void float_quat_derivative(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q) {
  qd->qi = -0.5*( r->p*q->qx + r->q*q->qy + r->r*q->qz);
  qd->qx = -0.5*(-r->p*q->qi - r->r*q->qy + r->q*q->qz);
  qd->qy = -0.5*(-r->q*q->qi + r->r*q->qx - r->p*q->qz);
  qd->qz = -0.5*(-r->r*q->qi - r->q*q->qx + r->p*q->qy);
  }

/** Quaternion derivative from rotational velocity.
 * qd = -0.5*omega(r) * q
 */
static inline void float_quat_derivative_lagrange(struct FloatQuat *qd, struct FloatRates *r, struct FloatQuat *q) {
  const float K_LAGRANGE = 1.;
  const float c = K_LAGRANGE * ( 1 - float_quat_norm(q)) / -0.5;
  qd->qi = -0.5*(      c*q->qi + r->p*q->qx + r->q*q->qy + r->r*q->qz);
  qd->qx = -0.5*(-r->p*q->qi +      c*q->qx - r->r*q->qy + r->q*q->qz);
  qd->qy = -0.5*(-r->q*q->qi + r->r*q->qx +      c*q->qy - r->p*q->qz);
  qd->qz = -0.5*(-r->r*q->qi - r->q*q->qx + r->p*q->qy +      c*q->qz);
  }

#define FLOAT_QUAT_OF_EULERS(_q, _e) float_quat_of_eulers(&(_q), &(_e))
#define FLOAT_QUAT_OF_AXIS_ANGLE(_q, _uv, _an) float_quat_of_axis_angle(&(_q), &(_uv), _an)
#define FLOAT_QUAT_OF_ORIENTATION_VECT(_q, _ov) float_quat_of_orientation_vect(&(_q), &(_ov))
#define FLOAT_QUAT_OF_RMAT(_q, _r) float_quat_of_rmat(&(_q), &(_r))

static inline void float_quat_of_eulers(struct FloatQuat *q, struct FloatEulers *e) {

  const float phi2   = e->phi/2.0;
  const float theta2 = e->theta/2.0;
  const float psi2   = e->psi/2.0;

  const float s_phi2   = sinf(phi2);
  const float c_phi2   = cosf(phi2);
  const float s_theta2 = sinf(theta2);
  const float c_theta2 = cosf(theta2);
  const float s_psi2   = sinf(psi2);
  const float c_psi2   = cosf(psi2);

  q->qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2;
  q->qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2;
  q->qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2;
  q->qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2;
  }

static inline void float_quat_of_axis_angle(struct FloatQuat *q, const struct FloatVect3 *uv, float angle) {
  const float san = sinf(angle/2.);
  q->qi = cosf(angle/2.);
  q->qx = san * uv->x;
  q->qy = san * uv->y;
  q->qz = san * uv->z;
  }

static inline void float_quat_of_orientation_vect(struct FloatQuat *q, const struct FloatVect3 *ov) {
  const float ov_norm = sqrtf(ov->x*ov->x + ov->y*ov->y + ov->z*ov->z);
  if (ov_norm < 1e-8) {
    q->qi = 1;
    q->qx = 0;
    q->qy = 0;
    q->qz = 0;
  } else {
    const float s2_normalized = sinf(ov_norm/2.0) / ov_norm;
    q->qi = cosf(ov_norm/2.0);
    q->qx = ov->x * s2_normalized;
    q->qy = ov->y * s2_normalized;
    q->qz = ov->z * s2_normalized;
  }
  }

static inline void float_quat_of_rmat(struct FloatQuat *q, struct FloatRMat *rm) {
  const float tr = RMAT_TRACE(*rm);
  if (tr > 0) {
    const float two_qi = sqrtf(1.+tr);
    const float four_qi = 2. * two_qi;
    q->qi = 0.5 * two_qi;
    q->qx =  (RMAT_ELMT(*rm, 1, 2)-RMAT_ELMT(*rm, 2, 1))/four_qi;
    q->qy =  (RMAT_ELMT(*rm, 2, 0)-RMAT_ELMT(*rm, 0, 2))/four_qi;
    q->qz =  (RMAT_ELMT(*rm, 0, 1)-RMAT_ELMT(*rm, 1, 0))/four_qi;
    /*printf("tr > 0\n");*/
  }
  else {
    if (RMAT_ELMT(*rm, 0, 0) > RMAT_ELMT(*rm, 1, 1) &&
        RMAT_ELMT(*rm, 0, 0) > RMAT_ELMT(*rm, 2, 2)) {
      const float two_qx = sqrtf(RMAT_ELMT(*rm, 0, 0) -RMAT_ELMT(*rm, 1, 1)
                                 -RMAT_ELMT(*rm, 2, 2) + 1);
      const float four_qx = 2. * two_qx;
      q->qi = (RMAT_ELMT(*rm, 1, 2)-RMAT_ELMT(*rm, 2, 1))/four_qx;
      q->qx = 0.5 * two_qx;
      q->qy = (RMAT_ELMT(*rm, 0, 1)+RMAT_ELMT(*rm, 1, 0))/four_qx;
      q->qz = (RMAT_ELMT(*rm, 2, 0)+RMAT_ELMT(*rm, 0, 2))/four_qx;
      /*printf("m00 largest\n");*/
    }
    else if (RMAT_ELMT(*rm, 1, 1) > RMAT_ELMT(*rm, 2, 2)) {
      const float two_qy =
        sqrtf(RMAT_ELMT(*rm, 1, 1) - RMAT_ELMT(*rm, 0, 0) - RMAT_ELMT(*rm, 2, 2) + 1);
      const float four_qy = 2. * two_qy;
      q->qi = (RMAT_ELMT(*rm, 2, 0) - RMAT_ELMT(*rm, 0, 2))/four_qy;
      q->qx = (RMAT_ELMT(*rm, 0, 1) + RMAT_ELMT(*rm, 1, 0))/four_qy;
      q->qy = 0.5 * two_qy;
      q->qz = (RMAT_ELMT(*rm, 1, 2) + RMAT_ELMT(*rm, 2, 1))/four_qy;
      /*printf("m11 largest\n");*/
    }
    else {
      const float two_qz =
        sqrtf(RMAT_ELMT(*rm, 2, 2) - RMAT_ELMT(*rm, 0, 0) - RMAT_ELMT(*rm, 1, 1) + 1);
      const float four_qz = 2. * two_qz;
      q->qi = (RMAT_ELMT(*rm, 0, 1)- RMAT_ELMT(*rm, 1, 0))/four_qz;
      q->qx = (RMAT_ELMT(*rm, 2, 0)+ RMAT_ELMT(*rm, 0, 2))/four_qz;
      q->qy = (RMAT_ELMT(*rm, 1, 2)+ RMAT_ELMT(*rm, 2, 1))/four_qz;
      q->qz = 0.5 * two_qz;
      /*printf("m22 largest\n");*/
    }
  }
  }



//
//
// Euler angles
//
//

#define FLOAT_EULERS_ZERO(_e) EULERS_ASSIGN(_e, 0., 0., 0.);

#define FLOAT_EULERS_NORM(_e) (sqrtf(SQUARE((_e).phi)+SQUARE((_e).theta)+SQUARE((_e).psi))) ;

#define FLOAT_EULERS_OF_RMAT(_e, _rm) float_eulers_of_rmat(&(_e), &(_rm))
#define FLOAT_EULERS_OF_QUAT(_e, _q) float_eulers_of_quat(&(_e), &(_q))

static inline void float_eulers_of_rmat(struct FloatEulers *e, struct FloatRMat *rm) {
  const float dcm00 = rm->m[0];
  const float dcm01 = rm->m[1];
  const float dcm02 = rm->m[2];
  const float dcm12 = rm->m[5];
  const float dcm22 = rm->m[8];
  e->phi   = atan2f(dcm12, dcm22);
  e->theta = -asinf(dcm02);
  e->psi   = atan2f(dcm01, dcm00);
  }

static inline void float_eulers_of_quat(struct FloatEulers *e, struct FloatQuat *q) {
  const float qx2  = q->qx * q->qx;
  const float qy2  = q->qy * q->qy;
  const float qz2  = q->qz * q->qz;
  const float qiqx = q->qi * q->qx;
  const float qiqy = q->qi * q->qy;
  const float qiqz = q->qi * q->qz;
  const float qxqy = q->qx * q->qy;
  const float qxqz = q->qx * q->qz;
  const float qyqz = q->qy * q->qz;
  const float dcm00 = 1.0 - 2.*(  qy2 +  qz2 );
  const float dcm01 =       2.*( qxqy + qiqz );
  const float dcm02 =       2.*( qxqz - qiqy );
  const float dcm12 =       2.*( qyqz + qiqx );
  const float dcm22 = 1.0 - 2.*(  qx2 +  qy2 );

  e->phi = atan2f(dcm12, dcm22);
  e->theta = -asinf(dcm02);
  e->psi = atan2f(dcm01, dcm00);
  }


//
//
// Generic vector algebra
//
//

/** a = 0 */
static inline void float_vect_zero(float * a, const int n) {
  int i;
  for (i = 0; i < n; i++) { a[i] = 0.; }
}

/** a = b */
static inline void float_vect_copy(float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { a[i] = b[i]; }
}

/** o = a + b */
static inline void float_vect_sum(float * o, const float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] + b[i]; }
}

/** o = a - b */
static inline void float_vect_diff(float * o, const float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] - b[i]; }
}

/** o = a * b (element wise) */
static inline void float_vect_mul(float * o, const float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * b[i]; }
}

/** a += b */
static inline void float_vect_add(float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { a[i] += b[i]; }
}

/** a -= b */
static inline void float_vect_sub(float * a, const float * b, const int n) {
  int i;
  for (i = 0; i < n; i++) { a[i] -= b[i]; }
}

/** o = a * s */
static inline void float_vect_smul(float * o, const float * a, const float s, const int n) {
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * s; }
}

/** o = a / s */
static inline void float_vect_sdiv(float * o, const float * a, const float s, const int n) {
  int i;
  if ( fabs(s) > 1e-5 ) {
    for (i = 0; i < n; i++) { o[i] = a[i] / s; }
  }
}

/** ||a|| */
static inline float float_vect_norm(const float * a, const int n) {
  int i;
  float sum = 0;
  for (i = 0; i < n; i++) { sum += a[i] * a[i]; }
  return sqrtf(sum);
}

//
//
// Generic matrix algebra
//
//

/** Make a pointer to a matrix of _rows lines */
#define MAKE_MATRIX_PTR(_ptr, _mat, _rows) \
  float * _ptr[_rows]; \
  { \
    int i; \
    for (i = 0; i < _rows; i++) { _ptr[i] = &_mat[i][0]; } \
  }

/** a = 0 */
static inline void float_mat_zero(float ** a, int m, int n) {
  int i,j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = 0.; }
  }
}

/** a = b */
static inline void float_mat_copy(float ** a, float ** b, int m, int n) {
  int i,j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = b[i][j]; }
  }
}

/** o = a + b */
static inline void float_mat_sum(float ** o, float ** a, float ** b, int m, int n) {
  int i,j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] + b[i][j]; }
  }
}

/** o = a - b */
static inline void float_mat_diff(float ** o, float ** a, float ** b, int m, int n) {
  int i,j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] - b[i][j]; }
  }
}

/** transpose square matrix */
static inline void float_mat_transpose(float ** a, int n) {
  int i,j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < i; j++) {
      float t = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = t;
    }
  }
}

/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 */
static inline void float_mat_mul(float ** o, float ** a, float ** b, int m, int n, int l) {
  int i,j,k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      o[i][j] = 0.;
      for (k = 0; k < n; k++) {
        o[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

/** matrix minor
 *
 * a: [m x n]
 * o: [I(d,d)     0     ]
 *    [  0    a(d,m:d,n)]
 */
static inline void float_mat_minor(float ** o, float ** a, int m, int n, int d) {
  int i,j;
  float_mat_zero(o, m, n);
  for (i = 0; i < d; i++) { o[i][i] = 1.0; }
  for (i = d; i < m; i++) {
    for (j = d; j < n; j++) {
      o[i][j] = a[i][j];
    }
  }
}

/** o = I - v v^T */
static inline void float_mat_vmul(float ** o, float * v, int n)
{
  int i,j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      o[i][j] = -2. *  v[i] * v[j];
    }
  }
  for (i = 0; i < n; i++) {
    o[i][i] += 1.;
  }
}

/** o = c-th column of matrix a[m x n] */
static inline void float_mat_col(float * o, float ** a, int m, int c) {
  int i;
  for (i = 0; i < m; i++) {
    o[i] = a[i][c];
  }
}

#endif /* PPRZ_ALGEBRA_FLOAT_H */
