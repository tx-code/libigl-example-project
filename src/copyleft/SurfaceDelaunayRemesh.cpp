#include "SurfaceDelaunayRemesh.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Surface_mesh.h>

#include <spdlog/spdlog.h>

using Kernel = CGAL::Epick;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace copyleft {

void surfaceDelaunayRemesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                           const Eigen::VectorXi &C,
                           const SurfaceDelaunayRemeshParameters &params,
                           Eigen::MatrixXd &V_out, Eigen::MatrixXi &F_out,
                           Eigen::VectorXi &C_out) {
  assert(C.rows() == F.rows());
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

  // each border of patch should be protected
  auto eif = get(CGAL::edge_is_feature, mesh);
  for (const auto &patch : patch_to_face) {
    // extract the border
    std::vector<Mesh::halfedge_index> border;
    PMP::border_halfedges(patch, mesh, std::back_inserter(border));
    for (const auto &h : border) {
      eif[mesh.edge(h)] = true;
    }
  }

  // the input mesh must be free of self-intersections
  if (PMP::does_self_intersect(mesh)) {
    spdlog::error("The input mesh has self-intersections");
    return;
  }

  // FIXME: find the suitable parameters
  auto remeshedMesh = PMP::surface_Delaunay_remeshing(
      mesh, CGAL::parameters::facet_size(0.1)
                .protect_constraints(true)
                .edge_is_constrained_map(eif)
                .face_patch_map(face_patch_map));
  remeshedMesh.collect_garbage();

  spdlog::info("Remeshed mesh has {} vertices and {} faces",
               remeshedMesh.number_of_vertices(),
               remeshedMesh.number_of_faces());

  // Copy the output mesh to the output variables
  V_out.resize(remeshedMesh.number_of_vertices(), 3);
  F_out.resize(remeshedMesh.number_of_faces(), 3);
  C_out.resize(remeshedMesh.number_of_faces());

  for (const auto &v : remeshedMesh.vertices()) {
    const auto &p = remeshedMesh.point(v);
    V_out(v.id(), 0) = p.x();
    V_out(v.id(), 1) = p.y();
    V_out(v.id(), 2) = p.z();
  }
  for (const auto &f : remeshedMesh.faces()) {
    const auto &h = remeshedMesh.halfedge(f);
    C_out(f.id()) = face_patch_map[f];
    F_out(f.id(), 0) = remeshedMesh.source(h).id();
    F_out(f.id(), 1) = remeshedMesh.target(h).id();
    F_out(f.id(), 2) = remeshedMesh.target(remeshedMesh.next(h)).id();
  }
}
} // namespace copyleft