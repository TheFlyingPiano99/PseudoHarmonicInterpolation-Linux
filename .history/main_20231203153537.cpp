#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <functional>
#define ANSI_DECLARATORS
#define REAL double
#define VOID void

#include "modifiedgordonwixomsurface.h"
extern "C" {
#include "triangle/triangle.h"
}

int main(int argc, char **argv) {


	// Create surface:
	Geometry::ModifiedGordonWixomSurface surface(
		std::function<Geometry::Point2D(double)>(
			[](double t){ double r = 2; return Geometry::Point2D(r * std::cos(t), r * std::sin(t)); }
		),
		std::function<double(Geometry::Point2D)>(
			[](Geometry::Point2D p){ return std::sin(p[0]); }
		)
	);

	std::vector<Geometry::Point2D> discretizedCurve = surface.getDiscretizedCurve();

	size_t n = discretizedCurve.size();	// # of points
	std::vector<double> points;
	points.reserve(n * 2);
	for (int i = 0; i < n; i++) {
		points.push_back(discretizedCurve[i][0]);
		points.push_back(discretizedCurve[i][1]);
	}
	double max_area = 0.611416847148; // nice number

  // Input segments : just a closed polygon
  std::vector<int> segments; segments.reserve(2 * n);
  for (size_t i = 0; i < n; ++i) {
    segments.push_back(i);
    segments.push_back(i + 1);
  }
  segments.back() = 0;

  // Setup output data structure
  struct triangulateio in, out;
  in.pointlist = &points[0];
  in.numberofpoints = n;
  in.numberofpointattributes = 0;
  in.pointmarkerlist = nullptr;
  in.segmentlist = &segments[0];
  in.numberofsegments = n;
  in.segmentmarkerlist = nullptr;
  in.numberofholes = 0;
  in.numberofregions = 0;

  // Setup output data structure
  out.pointlist = nullptr;
  out.pointattributelist = nullptr;
  out.pointmarkerlist = nullptr;
  out.trianglelist = nullptr;
  out.triangleattributelist = nullptr;
  out.segmentlist = nullptr;
  out.segmentmarkerlist = nullptr;

  // Call the library function
  // Look up all the switches to see what they do!
  std::ostringstream cmd;
  cmd << "pqa" << std::fixed << max_area << "DBPzQ";
  triangulate(const_cast<char *>(cmd.str().c_str()), &in, &out, (struct triangulateio *)nullptr);

	// Write an OBJ file as the output
	std::ofstream f("test.obj");
	for (int i = 0; i < out.numberofpoints; ++i)
		f << "v " << out.pointlist[2*i] << ' ' << out.pointlist[2*i+1] << " 0" << std::endl;
	for (int i = 0; i < out.numberoftriangles; ++i)
		f << "f " << out.trianglelist[3*i] + 1 << ' '
			<< out.trianglelist[3*i+1] + 1 << ' '
			<< out.trianglelist[3*i+2] + 1 << std::endl;

	trifree(out.pointlist);
	trifree(out.trianglelist);
}
