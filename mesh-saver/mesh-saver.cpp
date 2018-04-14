#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

#include <CGAL/boost/graph/graph_traits_PolyMesh_ArrayKernelT.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>

#include <CGAL/bounding_box.h>
#include <CGAL/Polygon_mesh_processing/internal/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include <utility> // defines std::pair
#include <list>
#include <fstream>
#include <vector>
#include <string>

namespace params = CGAL::Polygon_mesh_processing::parameters;
using namespace std;

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

typedef OpenMesh::PolyMesh_ArrayKernelT</* MyTraits*/> Surface_mesh;


int main(int argc, char* argv[]) {
    bool isReversed = false;
    string output_filename, input_filename;
    for (int i = 0; i < argc; i++) {

        if (argv[i][0] == '-' && argv[i][1] == 'i') {
            input_filename = argv[i+1];
	}

	if (argv[i][0] == '-' && argv[i][1] == 'o') {
	    output_filename = argv[i+1];
	}
        if (argv[i][0] == '-' && argv[i][1] == 'r') {
          isReversed = true;
        }

    }

    // Reads a .xyz point set file in points[].
    std::list<PointVectorPair> points;
    std::ifstream stream(input_filename);
    if (!stream ||
        !CGAL::read_xyz_points(stream,
                               std::back_inserter(points),
                               CGAL::First_of_pair_property_map<PointVectorPair>()))
    {
      std::cerr << "Error: cannot read file " <<  std::endl;
        return EXIT_FAILURE;
    }

    //Computes bounding box
    std::vector<Point> points_vector;
    for (auto const& i: points) {
      Point tempPoint = i.first;
      points_vector.push_back(tempPoint);
    }
    cout << points_vector.size() << endl;
    auto bbox = CGAL::bounding_box(points_vector.begin(), points_vector.end());

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
    std::cout << points.size() << std::endl;
    Polyhedron output_mesh;
  
    double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
      (points.begin(), points.end(), CGAL::First_of_pair_property_map<PointVectorPair>(), 6);
    if (CGAL::poisson_surface_reconstruction_delaunay
        (points.begin(), points.end(),
         CGAL::First_of_pair_property_map<PointVectorPair>(),
         CGAL::Second_of_pair_property_map<PointVectorPair>(),
         output_mesh, average_spacing))
      {
	  if (isReversed) {
          	output_mesh.inside_out();
                std::cout<<"Reverse applied"<<std::endl;
          }
          
          //Converts bounding box (K_Iso_Cuboid_3) to Polyhedron
          Polyhedron cuboid;
          cuboid.make_tetrahedron(bbox.vertex(0), bbox.vertex(1), bbox.vertex(2), bbox.vertex(3));
	  
          //Computes the intersection
	  Polyhedron output_final;
          CGAL::Polygon_mesh_processing::corefine_and_compute_intersection(output_mesh, cuboid, output_final,
          params::face_index_map(get(CGAL::face_external_index, output_mesh)).vertex_index_map(get(CGAL::vertex_external_index, output_mesh)),
          params::face_index_map(get(CGAL::face_external_index, cuboid)));
          
          //Saves polyhedrons in case we need them in future (optional)
	  std::ofstream output_landscape("landscape.off");
          output_landscape << output_mesh;
          std::ofstream output_bbox("bbox.off");
          output_bbox << cuboid;
          
          //Writing to obj file
          Surface_mesh mesh_out;
          CGAL::copy_face_graph(output_final, mesh_out);

	  if (!OpenMesh::IO::write_mesh(mesh_out, output_filename))
	  {
		std::cout  << "obj file write error" << std::endl;
	  }
      }
    else
      return EXIT_FAILURE;


    return EXIT_SUCCESS;
}
