#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

// 新增MeshInspector类，专门处理mesh检查

class MeshInspector {
public:
  MeshInspector();

  // 设置要检查的mesh
  void setMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);

  // 重置所有缓存
  void resetCache();

  // 各种检查方法
  bool isEdgeManifold();
  bool isVertexManifold();

  // 获取边界循环
  const std::vector<std::vector<int>> &getBoundaryLoops();

  // 检查自相交
  bool checkSelfIntersections(int &numIntersections);

  // 绘制检查UI
  void drawInspectionUI();

private:
  // 当前mesh数据
  Eigen::MatrixXd m_V;
   Eigen::MatrixXi m_F;

  // 缓存标志和结果
  bool m_meshChanged;
  bool m_edgeManifoldCached;
  bool m_vertexManifoldCached;
  bool m_boundaryLoopsCached;
  bool m_selfIntersectionsCached;

  // general statistics
  int m_irregularVertices{0};

  double m_averageArea{0.0};
  double m_minArea{0.0};
  double m_maxArea{0.0};
  double m_sigmaArea{0.0};

  double m_averageAngle{0.0};
  double m_minAngle{0.0};
  double m_maxAngle{0.0};
  double m_sigmaAngle{0.0};

  bool m_isEdgeManifold;
  bool m_isVertexManifold;
  std::vector<std::vector<int>> m_boundaryLoops;
  bool m_hasSelfIntersections;
  int m_numSelfIntersections;
};