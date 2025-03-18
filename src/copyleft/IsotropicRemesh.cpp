#include "IsotropicRemesh.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Surface_mesh.h>

#include <spdlog/spdlog.h>

using Kernel = CGAL::Epick;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace copyleft {
void isotropicRemesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                     const Eigen::VectorXi &C, const std::vector<int> &SC,
                     IsotropicRemeshParameters params, Eigen::MatrixXd &V_out,
                     Eigen::MatrixXi &F_out, Eigen::VectorXi &C_out) {
  assert(C.rows() == F.rows());
  spdlog::info("Isotropic remeshing parameters:\n\tTarget edge length: {}\n\t"
               "Number of iterations: {}",
               params.targetEdgeLength, params.numberIter);

  // Create a CGAL surface mesh from the input mesh
  Mesh mesh;

  auto face_patch_map =
      mesh.add_property_map<Mesh::face_index, int>("f:patch").first;
  for (int i = 0; i < V.rows(); ++i) {
    mesh.add_vertex(Kernel::Point_3(V(i, 0), V(i, 1), V(i, 2)));
  }
  for (int i = 0; i < F.rows(); ++i) {
    auto f = mesh.add_face(CGAL::SM_Vertex_index(F(i, 0)),
                           CGAL::SM_Vertex_index(F(i, 1)),
                           CGAL::SM_Vertex_index(F(i, 2)));
    face_patch_map[f] = C(i);
  }

  assert(V.rows() == mesh.number_of_vertices());

  std::vector<std::vector<Mesh::face_index>> patch_to_face;
  patch_to_face.resize(static_cast<size_t>(C.maxCoeff()) + 1);
  for (auto f : mesh.faces()) {
    // patch index is 1-based
    patch_to_face[face_patch_map[f] - 1].push_back(f);
  }

  double min_edge_length = std::numeric_limits<double>::infinity();
  for (auto h : mesh.halfedges()) {
    auto e = mesh.edge(h);
    min_edge_length = std::min(
        min_edge_length, CGAL::Polygon_mesh_processing::edge_length(e, mesh));
  }
  min_edge_length *= 5;
  spdlog::info("Min edge length: {}", min_edge_length);

  for (int i : SC) {
    if (i > patch_to_face.size() || i < 0 || patch_to_face[i].size() == 0)
      continue;
    // extract the border
    std::vector<Mesh::halfedge_index> border;
    PMP::border_halfedges(patch_to_face[i], mesh, std::back_inserter(border));

    spdlog::info("Remeshing patch {}: {} faces", i, patch_to_face[i].size());
    PMP::isotropic_remeshing(patch_to_face[i], min_edge_length, mesh,
                             CGAL::parameters::face_patch_map(face_patch_map)
                                 .number_of_iterations(params.numberIter)
                                 .protect_constraints(true));
  }

  mesh.collect_garbage();
  // Copy the output mesh to the output variables
  V_out.resize(mesh.number_of_vertices(), 3);
  F_out.resize(mesh.number_of_faces(), 3);
  C_out.resize(mesh.number_of_faces());

  for (const auto &v : mesh.vertices()) {
    const auto &p = mesh.point(v);
    V_out(v.id(), 0) = p.x();
    V_out(v.id(), 1) = p.y();
    V_out(v.id(), 2) = p.z();
  }
  for (const auto &f : mesh.faces()) {
    const auto &h = mesh.halfedge(f);
    C_out(f.id()) = face_patch_map[f];
    F_out(f.id(), 0) = mesh.source(h).id();
    F_out(f.id(), 1) = mesh.target(h).id();
    F_out(f.id(), 2) = mesh.target(mesh.next(h)).id();
  }
}
} // namespace copyleft
