#ifndef IMESHER_H
#define IMESHER_H

#include <Eigen/Dense>
#include <memory>
#include <string>

class TopoDS_Shape;

// Interface for mesh generation from CAD models
class IMesher {
public:
  virtual ~IMesher() = default;

  enum class Fineness {
    VeryCoarse = 0,
    Coarse = 1,
    Moderate = 2,
    Fine = 3,
    VeryFine = 4
  };

  // Set mesh fineness
  void setFineness(Fineness fineness) { mFineness = fineness; }

  // Get current fineness
  Fineness getFineness() const { return mFineness; }

  // Generate mesh from a TopoDS_Shape
  virtual bool generateMesh(const TopoDS_Shape &shape,
                            Eigen::MatrixXd &vertices,
                            Eigen::MatrixXi &faces) = 0;

  // Get the name of the mesher
  virtual std::string getName() const = 0;

  // Create a mesher instance based on the name
  static std::shared_ptr<IMesher> create(const std::string &mesherName);

protected:
  // Helper method to automatically select linear deflection based on model size
  double autoSelectLinearDeflection(const TopoDS_Shape &shape,
                                    double linPrec = 0.001) const;

  // Helper method to clean mesh by removing duplicate vertices
  void cleanMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                 Eigen::MatrixXd &V_Clean, Eigen::MatrixXi &F_Clean) const;

protected:
  Fineness mFineness = Fineness::Fine;
};

#endif // IMESHER_H