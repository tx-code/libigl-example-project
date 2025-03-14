#ifndef NETGENMESHER_H
#define NETGENMESHER_H

#include "IMesher.h"
#include <unordered_map>
#include <unordered_set>

class NetgenMesher : public IMesher {
public:
  enum class Fineness {
    VeryCoarse = 0,
    Coarse = 1,
    Moderate = 2,
    Fine = 3,
    VeryFine = 4
  };

  NetgenMesher(Fineness fineness = Fineness::Fine);
  ~NetgenMesher() override = default;

  bool generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
                    Eigen::MatrixXi &faces) override;

  std::string getName() const override { return "NETGEN"; }

  // Set mesh fineness
  void setFineness(Fineness fineness) { mFineness = fineness; }

  // Get current fineness
  Fineness getFineness() const { return mFineness; }

private:
  Fineness mFineness;

  // Helper method to generate mesh using Netgen
  bool
  doNetgenMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &vertices,
               Eigen::MatrixXi &faces,
               std::unordered_map<int, std::unordered_set<int>> &face_elems);
};

#endif // NETGENMESHER_H