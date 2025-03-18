#ifndef OCCMESHER_H
#define OCCMESHER_H

#include "IMesher.h"
#include <Bnd_Box.hxx>

class IMeshTools_Parameters;

class OCCMesher : public IMesher {
public:
  OCCMesher();
  ~OCCMesher() override = default;

  std::string getName() const override { return "OCC"; }

protected:
  bool generateMesh(const TopoDS_Shape &shape, bool autoClean) override;

private:
  void getMeshingParameters(IMeshTools_Parameters &mp,
                            const TopoDS_Shape &shape) const;
};

#endif // OCCMESHER_H