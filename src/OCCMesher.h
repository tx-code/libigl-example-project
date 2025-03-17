#ifndef OCCMESHER_H
#define OCCMESHER_H

#include "IMesher.h"
#include <Bnd_Box.hxx>

class IMeshTools_Parameters;

class OCCMesher : public IMesher {
public:
  OCCMesher();
  ~OCCMesher() override = default;

  bool generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
                    Eigen::MatrixXi &faces) override;

  std::string getName() const override { return "OCC"; }

private:
  void getMeshingParameters(IMeshTools_Parameters &mp,
                            const TopoDS_Shape &shape) const;
};

#endif // OCCMESHER_H