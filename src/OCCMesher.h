#ifndef OCCMESHER_H
#define OCCMESHER_H

#include "IMesher.h"
#include <Bnd_Box.hxx>

class OCCMesher : public IMesher {
public:
  OCCMesher();
  ~OCCMesher() override = default;

  bool generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
                    Eigen::MatrixXi &faces) override;

  std::string getName() const override { return "OCC"; }
};

#endif // OCCMESHER_H