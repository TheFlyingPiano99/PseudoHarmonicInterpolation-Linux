#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
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
		[](double t){ double r = 2; return Geometry::Point2D(r * std::cos(t), r * std::sin(t)); },
		[](const Geometry::Point2D& p){ return std::sin(p[0]); },
	);

	std::vector<double> points = {
		0, 0,												//
		1, 0,												//       x
		2, 0,												//      / \
		3, 1,												//     x   x
		4, 1,												//    /     \
		4, 2,												//   x       x
		3, 3,												//   |       |
		2, 4,												//   x     x-x
		1, 3,												//   |    /
		0, 2,												//   x-x-x
		0, 1												// (0,0)
	};
	size_t n = points.size() / 2;	// # of points
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
