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
    constexpr int n = 128;
    constexpr double delta_theta = M_PI / n;
    constexpr double offset = 0.1;
    double integral_den = 0.0;
    double integral_div = 0.0;
    for (int i = 0; i < n; i++) {
	Vector2D direction(std::cos(i * delta_theta + offset), std::sin(i * delta_theta + offset));
	if (direction[0] == 0.0 || direction[1] == 0.0) {
	    direction = Vector2D(std::cos((i + 0.5) * delta_theta + offset), std::sin((i + 0.5) * delta_theta + offset));
	    std::cout << "Recalculating direction." << std::endl;
	}

	auto intersections = findLineCurveIntersections(x, direction);
	

	// Calculate weights:
	double a = 0.0;
	double b = 0.0;
	double c = 1.0;
	double d = 0.0;
	for (int j = 0; j < intersections.first.size(); j++) {
	    if (intersections.first[j].second) {	// is hitting concave corner?
//		continue;
	    }
	    double distance = (intersections.first[j].first - x).length();
	    if (distance == 0) {
		    return height(x);
	    }
	    a += ((j % 2 == 0) ? 1.0 : -1.0) * height(intersections.first[j].first) / distance;
	    b += ((j % 2 == 0) ? 1.0 : -1.0) / distance;
	    d += ((j % 2 == 0) ? 1.0 : -1.0) / distance;
	}
	c *= d;
	d = 0.0;
	for (int j = 0; j < intersections.second.size(); j++) {
	    if (intersections.second[j].second) {	// is hitting concave corner?
//		continue;
	    }
	    double distance = (intersections.second[j].first - x).length();
	    a += ((j % 2 == 0) ? 1.0 : -1.0) * height(intersections.second[j].first) / distance;
	    if (distance == 0) {
		    return height(x);
	    }
	    b += ((j % 2 == 0) ? 1.0 : -1.0) / distance;
	    d += ((j % 2 == 0) ? 1.0 : -1.0) / distance;
	}
	c *= d;
	integral_den += a / b * c;
	integral_div += c;
    }
    double u = integral_den / integral_div;
    if (u != u) {
	std::cout << "u was NaN!" << std::endl;
	std::cout << "u = " << integral_den << " / " << integral_div << std::endl; 
	return height(x);
    }
    else {
	return u;
    }
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

    // Determine concave corners:
    isConcaveCorner.clear();
    isConcaveCorner.reserve(n);
    for (int i = 0; i < n; i++) {
	Point2D prev = discretizedCurve[(i > 0)? i - 1 : n - 1];
	Point2D current = discretizedCurve[i];
	Point2D next = discretizedCurve[(i < n - 1)? i + 1 : 0];
	Vector2D tangent = (next - prev).normalize();
	auto intersections = findLineCurveIntersections(current, tangent);
	isConcaveCorner.push_back(intersections.first.size() % 2 == 1);	// tangent ray from concave corner will cross the polygon odd times.
	
    }
}

 std::pair<std::vector<std::pair<Geometry::Point2D, bool>>, std::vector<std::pair<Geometry::Point2D, bool>>>
 Geometry::ModifiedGordonWixomSurface::findLineCurveIntersections(
    const Point2D& x, const Vector2D& direction) const
{
    std::pair<std::vector<std::pair<Geometry::Point2D, bool>>, std::vector<std::pair<Geometry::Point2D, bool>>> intersection_points;  // The first of the pair is on one side of the line and the second of the pair is on the other side of the line respectively to the x point.
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
        if (t == t && t >= 0 && t < sectionLength) {
            double tau = (p0[0] + t * sectionDir[0] - x[0]) / direction[0];
	    if (tau != tau) {
		std::cout << "Tau = NaN!" << std::endl;
	    }
	    constexpr double epsilon = 0.00000001;
	    if (tau < 0) {
	    intersection_points.first.push_back(
			std::make_pair(
				p0 + sectionDir * t,
				(isConcaveCorner[i] && t < epsilon) || (isConcaveCorner[(i == discretizedCurve.size() - 1)? 0 : i + 1] && sectionLength - t < epsilon)
			)
		);
            }
            else {
                intersection_points.second.push_back(
				std::make_pair(
					p0 + sectionDir * t,
					(isConcaveCorner[i] && t < epsilon) || (isConcaveCorner[(i == discretizedCurve.size() - 1)? 0 : i + 1] && sectionLength - t < epsilon)
				)
			);
            }
        }
    }
    // Sort the points:
    std::sort(intersection_points.first.begin(), intersection_points.first.end(), [x](std::pair<Geometry::Point2D, bool> p0, std::pair<Geometry::Point2D, bool> p1) { return (p0.first - x).length() < (p1.first - x).length();});
    std::sort(intersection_points.second.begin(), intersection_points.second.end(), [x](std::pair<Geometry::Point2D, bool> p0, std::pair<Geometry::Point2D, bool> p1) { return (p0.first - x).length() < (p1.first - x).length();});
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
