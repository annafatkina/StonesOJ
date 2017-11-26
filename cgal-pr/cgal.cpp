#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>

#include <utility> // defines std::pair
#include <list>
#include <fstream>
#include <vector>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


int main() {
    // Reads a .xyz point set file in points[].
    std::list<PointVectorPair> points;
    std::ifstream stream("kitten.xyz");
    if (!stream ||
        !CGAL::read_xyz_points(stream,
                               std::back_inserter(points),
                               CGAL::First_of_pair_property_map<PointVectorPair>()))
    {
      std::cerr << "Error: cannot read file" << std::endl;
        return EXIT_FAILURE;
    }
    // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals<Concurrency_tag>(points.begin(), points.end(),
                               CGAL::First_of_pair_property_map<PointVectorPair>(),
                               CGAL::Second_of_pair_property_map<PointVectorPair>(),
                               nb_neighbors);
    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    std::list<PointVectorPair>::iterator unoriented_points_begin =
      CGAL::mst_orient_normals(points.begin(), points.end(),
                                 CGAL::First_of_pair_property_map<PointVectorPair>(),
                                 CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                 nb_neighbors);
    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    points.erase(unoriented_points_begin, points.end());
    





    
    Polyhedron output_mesh;
  
    double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
      (points.begin(), points.end(), CGAL::First_of_pair_property_map<PointVectorPair>(), 6);
    if (CGAL::poisson_surface_reconstruction_delaunay
        (points.begin(), points.end(),
         CGAL::First_of_pair_property_map<PointVectorPair>(),
         CGAL::Second_of_pair_property_map<PointVectorPair>(),
         output_mesh, average_spacing))
      {
	  std::ofstream out("kitten_surface.off");
          out << output_mesh;
      }
    else
      return EXIT_FAILURE;


    return EXIT_SUCCESS;
}
