#include "NetgenMesher.h"

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#pragma warning(push, 0)

#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif

#include <meshing/meshing.hpp>
#include <occ/occgeom.hpp>

namespace nglib {
#include <nglib.h>
}

#pragma warning(pop)

namespace {

std::string to_string(const nglib::Ng_Result &res) {
  switch (res) {
  case nglib::NG_ERROR:
    return "NG_ERROR";
  case nglib::NG_OK:
    return "NG_OK";
  case nglib::NG_SURFACE_INPUT_ERROR:
    return "NG_SURFACE_INPUT_ERROR";
  case nglib::NG_VOLUME_FAILURE:
    return "NG_VOLUME_FAILURE";
  case nglib::NG_STL_INPUT_ERROR:
    return "NG_STL_INPUT_ERROR";
  case nglib::NG_SURFACE_FAILURE:
    return "NG_SURFACE_FAILURE";
  case nglib::NG_FILE_NOT_FOUND:
    return "NG_FILE_NOT_FOUND";
  default:
    return "Unknown";
  }
}

} // namespace

NetgenMesher::NetgenMesher(Fineness fineness) : mFineness(fineness) {
  // Constructor implementation
}

bool NetgenMesher::generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                                Eigen::MatrixXi &F) {
  spdlog::stopwatch sw;

  std::unordered_map<int, std::unordered_set<int>> face_elems;
  bool result = doNetgenMesh(shape, V, F, face_elems);

  spdlog::info("NETGEN meshing completed in {:.3f}s", sw);
  return result;
}

bool NetgenMesher::doNetgenMesh(
    const TopoDS_Shape &shape, Eigen::MatrixXd &V, Eigen::MatrixXi &F,
    std::unordered_map<int, std::unordered_set<int>> &face_elems) {
  using namespace nglib;

  spdlog::info("================================================");
  spdlog::info("Do Netgen meshing...");
  spdlog::info("================================================");
  Ng_Init();

  // Create OCC geometry for Netgen
  auto occ_geometry =
      std::make_shared<netgen::OCCGeometry>(shape, 2); // only 2d mesh now
  auto mesh = std::make_shared<netgen::Mesh>();

  netgen::MeshingParameters mp;
  // General parameters
  mp.delaunay2d = true;
  mp.parthread = true;
  mp.minh = autoSelectLinearDeflection(shape);
  mp.maxh = mp.minh * 1000;
  netgen::OCCParameters op;
  op.resthminedgelenenable = true;

  // Set parameters based on fineness
  switch (mFineness) {
  case Fineness::VeryCoarse:
    mp.curvaturesafety = 1;
    mp.segmentsperedge = 0.3;
    mp.grading = 0.7;
    mp.closeedgefac = 0.5;
    mp.optsteps3d = 5;
    op.resthminedgelen = 2;
    break;
  case Fineness::Coarse:
    mp.curvaturesafety = 1.5;
    mp.segmentsperedge = 0.5;
    mp.grading = 0.5;
    mp.closeedgefac = 1;
    mp.optsteps3d = 5;
    op.resthminedgelen = 1;
    break;
  case Fineness::Moderate:
    mp.curvaturesafety = 2;
    mp.segmentsperedge = 1;
    mp.grading = 0.3;
    mp.closeedgefac = 2;
    mp.optsteps3d = 5;
    op.resthminedgelen = 0.2;
    break;
  case Fineness::Fine:
    mp.curvaturesafety = 3;
    mp.segmentsperedge = 2;
    mp.grading = 0.2;
    mp.closeedgefac = 3.5;
    mp.optsteps3d = 5;
    op.resthminedgelen = 0.02;
    break;
  case Fineness::VeryFine:
    mp.curvaturesafety = 5;
    mp.segmentsperedge = 3;
    mp.grading = 0.1;
    mp.closeedgefac = 5;
    mp.optsteps3d = 5;
    op.resthminedgelen = 0.002;
    break;
  }
  std::stringstream ss;
  ss << "Meshing parameters: " << mp;
  spdlog::info(ss.str());

  occ_geometry->SetOCCParameters(op);
  mesh->SetGeometry(occ_geometry);
  occ_geometry->BuildVisualizationMesh(0.01); // optional step

  if (auto result = occ_geometry->GenerateMesh(mesh, mp);
      result != nglib::NG_OK) {
    spdlog::error("Failed to generate NETGEN mesh: {}",
                  to_string(static_cast<nglib::Ng_Result>(result)));
    Ng_Exit();
    return false;
  }

  auto nb_nodes = mesh->GetNP();
  auto nb_triangles = mesh->GetNSE();
  spdlog::info("NETGEN mesh before cleaning: Nodes: {}, Triangles: {}",
               nb_nodes, nb_triangles);

  Eigen::MatrixXd V_Uncleaned(nb_nodes, 3);
  Eigen::MatrixXi F_Uncleaned(nb_triangles, 3);

  for (int i = 1; i <= nb_nodes; i++) {
    const auto &p = mesh->Point(netgen::PointIndex(i));
    V_Uncleaned(i - 1, 0) = p[0];
    V_Uncleaned(i - 1, 1) = p[1];
    V_Uncleaned(i - 1, 2) = p[2];
  }

  for (int i = 0; i < nb_triangles; i++) {
    const auto &t = (*mesh)[netgen::SurfaceElementIndex(i)];
    if (t.GetNP() == 3) {
      F_Uncleaned(i, 0) = t[0] - 1;
      F_Uncleaned(i, 1) = t[1] - 1;
      F_Uncleaned(i, 2) = t[2] - 1;
    } else {
      spdlog::error("Triangle has {} nodes", t.GetNP());
    }
  }

  // Clean the mesh (remove duplicate vertices)
  cleanMesh(V_Uncleaned, F_Uncleaned, V, F);

  spdlog::info("NETGEN mesh after cleaning: Nodes: {}, Triangles: {}", V.rows(),
               F.rows());

  Ng_Exit();
  return true;
}