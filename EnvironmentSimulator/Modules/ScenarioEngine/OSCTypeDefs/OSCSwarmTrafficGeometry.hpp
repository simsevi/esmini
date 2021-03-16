/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once
#include<cmath>
#include "RoadManager.hpp"

namespace STGeometry {

#define M(hdg) tan(hdg)
#define Q(x0, y0, hdg0) ((y0) - M(hdg) * (x0))
#define ZERO_T 0 // SMALL_NUMBER


    /*
     * Line considered in the form y = mx + q
     * @m: angular coefficient of the line
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    static inline double A(double m, double hdg, double SMjA, double SMnA) {
        double f1, f2, cos_, sin_;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1 = pow((m * sin_ + cos_) / SMjA, 2);
        f2 = pow((-m * cos_ + sin_) / SMnA, 2);
        return f1 + f2;
    }
    
    /*
     * Line considered in the form y = mx + q, where m = tan(theta)
     * @m: angular coefficient of the line
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMjA semi-major axes
     * @SMnA semi-minor axes
     */
    static inline double B(double m, double q, double h, double k, double hdg, double SMjA, double SMnA) {
        double f1, f2, cos_, sin_;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1   = 2 * (-h * cos_ + (q - k) * sin_) * (m * sin_ + cos_) / pow(SMjA, 2);
        f2   = 2 * (-h * sin_ - (q - k) * cos_) * (-m * cos_ + sin_) / pow(SMnA, 2);
        return f1 + f2;
    }

    /*
     * Line considered in the form y = mx + q, where m = tan(hdg)
     * @q: y intercept of the line
     * @h: x-position of the center of the ellipses
     * @k: y-position of the center of the ellipses
     * @hdg: orientation of ellipses
     * @SMnA: semi-minor axes
     * @SMjA semi-major axes
     */
    static inline double C(double q, double h, double k, double hdg, double SMjA, double SMnA) {
        double cos_, sin_, f1, f2;
        cos_ = cos(hdg);
        sin_ = sin(hdg);

        f1 = pow((-h * cos_ + (-k + q) * sin_) / SMjA, 2);
        f2 = pow((-h * sin_ - (-k + q) * cos_) / SMnA, 2);
        return f1 + f2 - 1;
    }

    /*
     * Checks whether the intersection points found belong to the segment of road
     */
    static char checkRange(roadmanager::Line *line, char solsN, double* x1, double* y1, double* x2, double* y2) {
        double xmin, ymin, xmax, ymax, hdg;

        xmin = line->GetX();
        xmax = line->GetY();
        line->EvaluateDS(line->GetLength(), &xmax, &ymax, &hdg);

        if (xmin > xmax) std::swap(xmin, xmax);
        if (ymin > ymax) std::swap(ymin, ymax);

        switch (solsN) {
            case 2:
                if (!(xmin <= *x1 && *x1 <= xmax && ymin <= *y1 && *y1 <= ymax)) {
                   *x1 = *x2;
                   *y1 = *y2;
                   *x2 = *y2 = 0;
                   solsN = 1;
                } else if (!(xmin <= *x2 && *x2 <= xmax && ymin <= *y2 && *y2 <= ymax)) {
                    *x2 = *y2 = 0;
                    return 1;
                }
            case 1:
                if (!(xmin <= *x1 && *x1 <= xmax && ymin <= *y1 && *y1 <= ymax)) {
                    *x1 = *y1 = 0;
                    return 0;
                } else 
                    break;
            default:
                break;
        }
        return solsN;
    }

    /* 
     * Finds the intersection points between an ellipses and a stright piece of the road
     * The equation to find the x points of intersection is assumed to be in the form
     *   A x² + Bx² + C = 0
     * while to find the y points the line equation is used:
     *   y = m*x + q
     * 
     * The ellipse is considered to have the form:
     *   [(x - h)cos(A) + (y - k)sin(A)]²     [(x  -h)sin(A) - (y - k)cos(A)]²
     *  ---------------------------------- + ----------------------------------
     *                  a²                                   b²
     * Where:
     *   * x, y: coordinates of a point of the ellipse
     *   * h, k: coordinates of the center of the ellipses
     *   * A: orientation angle
     *   * a, b: semi-major axes and semi-minor axes    
     */
    char lineIntersect(roadmanager::Position pos, roadmanager::Line *line, double SMjA, double SMnA, double* x1, double* y1, double* x2, double* y2) {
        double _A, _B, _C, delta, x0, y0, hdg, sqrt_delta, m, q, h, k;
        x0  = line->GetX();
        y0  = line->GetY();
        hdg = line->GetHdg();
        h   = pos.GetX();
        k   = pos.GetY();
        m   = M(hdg);
        q   = Q(x0, y0, hdg);
        
        hdg = pos.GetH();
        _A = A(m, hdg, SMjA, SMnA);
        _B = B(m, q, h, k, hdg, SMjA, SMnA);
        _C = C(q, h, k, hdg, SMjA, SMnA);

        *x1 = *x2 = *y1 = *y2 = 0;

        delta = pow(_B, 2) - 4 * _A * _C;
        if(delta > 0) {
            sqrt_delta = sqrt(delta);
            *x1 = (-_B - sqrt_delta) / (2 * _A);
            *x2 = (-_B + sqrt_delta) / (2 * _A);
            *y1 = m * (*x1) + q;
            *y2 = m * (*x2) + q;
            return checkRange(line, 2, x1, y1, x2, y2);
        }
        else if (delta == 0) {
            *x1 = -_B / (2.0 * _A);
            *y2 = m * *x1 + q;
            return checkRange(line, 1, x1, y1, x2, y2);
        }
        else {
            return 0;
        }
    }

    static inline double pA(double l_hdg, double e_hdg, double SMjA, double SMnA) {
        double cos_l, cos_e, sin_l, sin_e, f1, f2;
        cos_l = cos(l_hdg);
        sin_l = sin(l_hdg);
        cos_e = cos(e_hdg);
        sin_e = sin(e_hdg);

        f1 = pow((sin_l * sin_e + cos_l * cos_e) / SMjA, 2);
        f2 = pow((-sin_l * cos_e + cos_l * sin_e) / SMnA, 2);
        return f1 + f1;
    }

    static inline double pB(double x0, double y0, double h, double k, double l_hdg, double e_hdg, double SMjA, double SMnA) {
        double cos_l, cos_e, sin_l, sin_e, f1, f2;
        cos_l = cos(l_hdg);
        sin_l = sin(l_hdg);
        cos_e = cos(e_hdg);
        sin_e = sin(e_hdg);

        f1 = ((x0 - h) * cos_e + (y0 - k) * sin_e) * (sin_l * sin_e + cos_l * cos_e) / pow(SMjA, 2);
        f2 = ((x0 - h) * sin_e - (y0 - k) * cos_e) * (cos_l * sin_e - sin_l * cos_e) / pow(SMnA, 2);
        return 2 * (f1 + f2);
    }

    static inline double pC(double x0, double y0, double h, double k, double e_hdg, double SMjA, double SMnA) {
        double cos_e, sin_e, f1, f2;
        cos_e = cos(e_hdg);
        sin_e = sin(e_hdg);

        f1 = pow(((x0 - h) * cos_e + (y0 - k) * sin_e) / SMjA, 2);
        f2 = pow(((x0 - h) * sin_e - (y0 - k) * cos_e) / SMnA, 2);
        return f1 + f2 -1;
    }

    char polarLineIntersect(roadmanager::Position pos, roadmanager::Line *line, double SMjA, double SMnA, double* s1, double *s2) {
        double _pA, _pB, _pC, x0, y0, h, k, l_hdg, e_hdg, delta, sqrt_delta;

        x0    = line->GetX();
        y0    = line->GetY();
        l_hdg = line->GetHdg();
        h     = pos.GetX();
        k     = pos.GetY();  
        e_hdg = pos.GetH();

        _pA = pA(l_hdg, e_hdg, SMjA, SMnA);
        _pB = pB(x0, y0, h, k, l_hdg, e_hdg, SMjA, SMnA);
        _pC = pC(x0, y0, h, k, e_hdg, SMjA, SMnA);

        printf("A %.2f, B: %.2f, C: %.2f\n", _pA, _pB, _pC);
        
        *s1 = *s2 = 0;

        delta = pow(_pB, 2) - 4 * _pA * _pC;
        if(delta > 0) {
            sqrt_delta = sqrt(delta);
            *s1 = (-_pB - sqrt_delta) / (2 * _pA);
            *s2 = (-_pB + sqrt_delta) / (2 * _pA);
            return 2;
        }
        else if (delta == 0) {
            *s1 = -_pB / (2.0 * _pA);
            return 1;
        }
        else {
            return 0;
        }     
    }

    inline void polar2cartesian(double s, double x0, double y0, double theta, double *x, double *y) {
        *x = x0 + s * cos(theta);
        *y = y0 + s * sin(theta);
        printf("COMPUTED (%.2f, %.2f)\n", *x, *y);
    }

}