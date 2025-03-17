#ifndef NETGENMESHER_H
#define NETGENMESHER_H

#include "IMesher.h"
#include <unordered_map>
#include <unordered_set>

class NetgenMesher : public IMesher {
public:
  NetgenMesher();
  ~NetgenMesher() override = default;

  bool generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
                    Eigen::MatrixXi &faces) override;

  std::string getName() const override { return "NETGEN"; }

  // Helper method to generate mesh using Netgen
  bool
  doNetgenMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
               Eigen::MatrixXi &faces,
               std::unordered_map<int, std::unordered_set<int>> &face_elems);
};

#endif // NETGENMESHER_H