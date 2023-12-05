#pragma once

#include "geometry.hh"
#include <functional>
#include <utility>

namespace Geometry {

  class ModifiedGordonWixomSurface
  {
  public:
    /*
     * Receives a function: t in [0, 1] -> R^3 describing a closed curve
     * The surface will interpolated inside the closed curve
    */
    ModifiedGordonWixomSurface(const std::function<Point2D(double)>& curve, const std::function<double(Point2D)>& height);

    double eval(const Point2D& x) const;

    void setCurve(const std::function<Point2D(double)>& curve);

    void setHeight(const std::function<double(Point2D)>& curve);

    /*
     * Returns a pair of arrays of intersection points
     * Points in the first array of the pair are on the oposite side of the line related to the x point than the points in the second array of the pair.
    */
    std::pair<std::vector<std::pair<Geometry::Point2D, bool>>, std::vector<std::pair<Geometry::Point2D, bool>>> findLineCurveIntersections(const Point2D& x, const Vector2D& direction,
		    std::pair<std::vector<bool>, std::vector<bool>>& isHittingConcaveCorner) const;

    Point2D getBoundingRectangleMin() const;

    Point2D getBoundingRectangleMax() const;

    const std::vector<Point2D>& getDiscretizedCurve() const;

private:
    void discretizeCurve();

    Point2D boundingRectangleMin;
    Point2D boundingRectangleMax;
    std::function<Geometry::Point2D(double)> curve;
    std::function<double(Point2D)> height;
    std::vector<Point2D> discretizedCurve;
    std::vector<bool> isConcaveCorner;
  };
}
