#include "IMesher.h"
#include "NetgenMesher.h"
#include "OCCMesher.h"

// OpenCASCADE headers
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

// Libigl headers
#include <igl/remove_duplicate_vertices.h>

#include <spdlog/spdlog.h>

std::shared_ptr<IMesher> IMesher::create(const std::string &mesherName) {
  if (mesherName == "OCC") {
    return std::make_shared<OCCMesher>();
  } else if (mesherName == "NETGEN") {
    return std::make_shared<NetgenMesher>();
  }

  // Default to OCC mesher
  return std::make_shared<OCCMesher>();
}

double IMesher::autoSelectLinearDeflection(const TopoDS_Shape &shape,
                                           double linPrec) const {
  Bnd_Box bndBox;
  BRepBndLib::Add(shape, bndBox);

  if (bndBox.IsVoid()) {
    return linPrec;
  }

  bndBox.Enlarge(0.0001);

  // Use a fraction of a bounding diagonal
  const double diag = bndBox.CornerMin().Distance(bndBox.CornerMax());
  double delf = linPrec * diag;
  spdlog::info("Auto linear deflection: {}", delf);
  return delf;
}

void IMesher::cleanMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                        Eigen::MatrixXd &V_Clean,
                        Eigen::MatrixXi &F_Clean) const {
  constexpr double epsilon = 1e-7;
  Eigen::VectorXi I, J; // I is the index of the original vertices, J is the
                        // index of the cleaned vertices
  igl::remove_duplicate_vertices(V, F, epsilon, V_Clean, I, J, F_Clean);
  spdlog::info("Removed {} duplicate vertices", V.rows() - V_Clean.rows());
}