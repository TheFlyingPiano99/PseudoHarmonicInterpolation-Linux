#include "modifiedgordonwixomsurface.h"
#include <math.h>

Geometry::ModifiedGordonWixomSurface::ModifiedGordonWixomSurface(const std::function<Point2D(double)>& _curve,
                                                                 const std::function<double(Point2D)>& _height)
    : curve(_curve), height(_height)
{
    discretizeCurve();
}

double Geometry::ModifiedGordonWixomSurface::eval(const Point2D &x) const
{
    constexpr int n = 100;
    constexpr double delta_theta = 2.0 * M_PI / n;
    double integral_den = 0.0;
    double integral_div = 0.0;
    for (int i = 0; i < n; i++) {
        Vector2D direction(std::cos(i * delta_theta), std::sin(i * delta_theta));

        auto intersections = findLineCurveIntersections(x, direction);

        if (intersections.size() % 2 != 0) {
            continue;
        }

        inr idx_of_previous_intersect_to_section = 0;
        for (int j = 0; j < intersections.size() - 1; j += 2) {
            if ((intersections[j] - x).dot(intersections[j + 1] - x)) {
                idx_of_previous_intersect_to_section = j;
                break;
            }
        }

        double a = 0.0;
        double b = 0.0;
        double c = 1.0 / (intersections[0] - x).length();
        double d = 0.0;
        for (int j = 0; j < intersections.size(); j++) {
            double distance = (intersections[i] - x).length();
            a += ((j == 0 || j % 2 == 1)? 1.0 : -1.0) * height(intersections[i]) / distance;
            b += ((j == 0 || j % 2 == 1)? 1.0 : -1.0) / distance;
            if (0 != j) {
                d += ((j == 0 || j % 2 == 1)? 1.0 : -1.0) / distance;
            }
        }
        c *= d;
        integral_den += a / b * c * delta_theta;
        integral_div += c * delta_theta;
    }
    double u = integral_den / integral_div;
    if (u != u)
        u = height(x);
    return u;
}

void Geometry::ModifiedGordonWixomSurface::setCurve(const std::function<Point2D(double)> &_curve)
{
    curve = _curve;
    discretizeCurve();
}

void Geometry::ModifiedGordonWixomSurface::setHeight(const std::function<double(Point2D)>&_height)
{
    height = _height;
}

void Geometry::ModifiedGordonWixomSurface::discretizeCurve()
{
    discretizedCurve.clear();
    constexpr int n = 256;
    for (int i = 0; i < n; i++) {
        Point2D p = curve(i / (double)n);
        discretizedCurve.push_back(p);
        // Update min and max:
        if (0 == i) {
            boundingRectangleMin = p;
            boundingRectangleMax = p;
        }
        else {
            if (boundingRectangleMin[0] > p[0]) {
                boundingRectangleMin[0] = p[0];
            }
            if (boundingRectangleMin[1] > p[1]) {
                boundingRectangleMin[1] = p[1];
            }
            if (boundingRectangleMax[0] < p[0]) {
                boundingRectangleMax[0] = p[0];
            }
            if (boundingRectangleMax[1] < p[1]) {
                boundingRectangleMax[1] = p[1];
            }
        }
    }
}

std::vector<Geometry::Point2D> Geometry::ModifiedGordonWixomSurface::findLineCurveIntersections(
    const Point2D& x, const Vector2D& direction
) const
{
    std::vector<Point2D> intersection_points;
    for (int i = 0; i < discretizedCurve.size(); i++){
        Point2D p0 = discretizedCurve[i];
        Point2D p1 = (i == discretizedCurve.size() - 1)? discretizedCurve[0] : discretizedCurve[i + 1];
        Point2D sectionDiff = p1 - p0;
        double sectionLength = sectionDiff.length();
        if (sectionLength < std::numeric_limits<double>::min()) {
            continue;
        }
        Vector2D sectionDir = sectionDiff / sectionLength;
        double t = (p0[1] - x[1] - direction[1] * (p0[0] - x[0]) / direction[0])
                   / (direction[1] * sectionDir[0] / direction[0] - sectionDir[1]);
        if (t >= 0 && t < sectionLength) {
            intersection_points.push_back(p0 + sectionDir * t);
        }
    }
    return intersection_points;
}

Geometry::Point2D Geometry::ModifiedGordonWixomSurface::getBoundingRectangleMin() const
{
    return boundingRectangleMin;
}

Geometry::Point2D Geometry::ModifiedGordonWixomSurface::getBoundingRectangleMax() const
{
    return boundingRectangleMax;
}

const std::vector<Geometry::Point2D> &Geometry::ModifiedGordonWixomSurface::getDiscretizedCurve() const
{
    return discretizedCurve;
}
