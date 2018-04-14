#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <iostream>
#include <fstream>
typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;
typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
typedef Kernel::Vector_3  Vector_3;
typedef Kernel::Aff_transformation_3  Aff_transformation_3;
using namespace std;

int main() {
  Polyhedron landscape;
  std::ifstream input_landscape("landscape.off");
  input_landscape >> landscape;
 
  Polyhedron bbox;
  std::ifstream input_bbox("bbox.off");
  input_bbox >> bbox;

  Polyhedron clipped_landscape;

  if(landscape.is_closed()) {
    Nef_polyhedron N1(landscape);
    if (bbox.is_closed()) {
      Nef_polyhedron N2(bbox);
      Nef_polyhedron result;
      result = N1*N2;

     if (result.is_simple()) {
        result.convert_to_polyhedron(clipped_landscape);
        std::ofstream output("clipped_landscape.off");
        output << clipped_landscape;
     }
     else cout << "is not simple" << endl;
    }
    else cout << "bbox is not closed" << endl;
  }
  else cout << "landscape is not closed" << endl;

  return 0;
}

