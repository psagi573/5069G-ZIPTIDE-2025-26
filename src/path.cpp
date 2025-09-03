#include "path.h"
#include <cassert>
#include <limits>
#include "utils.h"

void SplinePath::addPoint(double x, double y, double theta)
{
    controlPoints.push_back({x, y, theta});
}

void SplinePath::clearPoints()
{
    controlPoints.clear();
}

std::vector<SplinePoint> SplinePath::generatePath(int samplesPerSegment)
{
    std::vector<SplinePoint> path;
    if (controlPoints.size() < 2)
        return path;

    size_t n = controlPoints.size();
    std::vector<double> xs(n), ys(n);
    for (size_t i = 0; i < n; ++i)
    {
        xs[i] = controlPoints[i].x;
        ys[i] = controlPoints[i].y;
    }

    // cubic coefficients for x and y
    std::vector<double> ax, bx, cx, dx;
    std::vector<double> ay, by, cy, dy;

    computeNaturalCubic(xs, ax, bx, cx, dx);
    computeNaturalCubic(ys, ay, by, cy, dy);

    // Sample each segment
    for (size_t i = 0; i + 1 < n; ++i)
    {
        for (int j = 0; j < samplesPerSegment; ++j)
        {
            double t = (double)j / samplesPerSegment;
            double t2 = t * t;
            double t3 = t2 * t;

            double x = ax[i] + bx[i] * t + cx[i] * t2 + dx[i] * t3;
            double y = ay[i] + by[i] * t + cy[i] * t2 + dy[i] * t3;

            // tangent for heading
            double dxdt = bx[i] + 2 * cx[i] * t + 3 * dx[i] * t2;
            double dydt = by[i] + 2 * cy[i] * t + 3 * dy[i] * t2;
            double theta = rad2deg(atan2(dydt, dxdt));

            path.push_back({x, y, wrapDeg360(theta)});
        }
    }
    path.push_back(controlPoints.back());
    return path;
}

// Natural cubic spline solver using tridiagonal system
void SplinePath::computeNaturalCubic(const std::vector<double> &positions,
                                     std::vector<double> &a,
                                     std::vector<double> &b,
                                     std::vector<double> &c,
                                     std::vector<double> &d)
{
    size_t n = positions.size();
    assert(n >= 2);

    a.resize(n - 1);
    b.resize(n - 1);
    c.resize(n - 1);
    d.resize(n - 1);

    std::vector<double> h(n - 1);
    for (size_t i = 0; i < n - 1; ++i)
        h[i] = 1.0; // uniform parameterization

    std::vector<double> alpha(n - 1, 0.0);
    for (size_t i = 1; i < n - 1; ++i)
        alpha[i] = 3.0 * (positions[i + 1] - positions[i]) - 3.0 * (positions[i] - positions[i - 1]);

    std::vector<double> l(n, 0.0), mu(n, 0.0), z(n, 0.0);
    l[0] = 1.0;
    mu[0] = z[0] = 0.0;

    for (size_t i = 1; i < n - 1; ++i)
    {
        l[i] = 4.0 - mu[i - 1];
        mu[i] = 1.0 / l[i];
        z[i] = (alpha[i] - z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    std::vector<double> cCoeff(n, 0.0);

    for (int j = n - 2; j >= 0; --j)
    {
        cCoeff[j] = z[j] - mu[j] * cCoeff[j + 1];
        b[j] = positions[j + 1] - positions[j] - (2 * cCoeff[j] + cCoeff[j + 1]) / 3.0;
        d[j] = (cCoeff[j + 1] - cCoeff[j]) / 3.0;
        a[j] = positions[j];
        c[j] = cCoeff[j];
    }
}
