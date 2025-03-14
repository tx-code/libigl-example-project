#include "MeshInspector.h"
#include <igl/PI.h>
#include <igl/boundary_loop.h>
#include <igl/fast_find_self_intersections.h>
#include <igl/internal_angles.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_irregular_vertex.h>
#include <igl/is_vertex_manifold.h>
#include <imgui.h>

MeshInspector::MeshInspector()
    : m_meshChanged(true), m_edgeManifoldCached(false),
      m_vertexManifoldCached(false), m_boundaryLoopsCached(false),
      m_selfIntersectionsCached(false), m_isEdgeManifold(false),
      m_isVertexManifold(false), m_hasSelfIntersections(false),
      m_numSelfIntersections(0) {}

void MeshInspector::setMesh(const Eigen::MatrixXd &V,
                            const Eigen::MatrixXi &F) {
  // 只有当mesh真的改变时才更新
  if (m_V.rows() != V.rows() || m_F.rows() != F.rows() ||
      (m_V.rows() > 0 && (m_V - V).norm() > 1e-10) ||
      (m_F.rows() > 0 && (m_F - F).norm() > 1e-10)) {

    m_V = V;
    m_F = F;
    m_meshChanged = true;
    resetCache();
  }
}

void MeshInspector::resetCache() {
  m_edgeManifoldCached = false;
  m_vertexManifoldCached = false;
  m_boundaryLoopsCached = false;
  m_selfIntersectionsCached = false;
}

bool MeshInspector::isEdgeManifold() {
  if (!m_edgeManifoldCached) {
    m_isEdgeManifold = igl::is_edge_manifold(m_F);
    m_edgeManifoldCached = true;
  }
  return m_isEdgeManifold;
}

bool MeshInspector::isVertexManifold() {
  if (!m_vertexManifoldCached) {
    m_isVertexManifold = igl::is_vertex_manifold(m_F);
    m_vertexManifoldCached = true;
  }
  return m_isVertexManifold;
}

const std::vector<std::vector<int>> &MeshInspector::getBoundaryLoops() {
  if (!m_boundaryLoopsCached) {
    m_boundaryLoops.clear();
    igl::boundary_loop(m_F, m_boundaryLoops);
    m_boundaryLoopsCached = true;
  }
  return m_boundaryLoops;
}

bool MeshInspector::checkSelfIntersections(int &numIntersections) {
  if (!m_selfIntersectionsCached) {
    Eigen::MatrixXi intersect;
    m_hasSelfIntersections =
        igl::fast_find_self_intersections(m_V, m_F, intersect);
    // 0 or 1, and double
    m_numSelfIntersections = intersect.sum() / 2;
    m_selfIntersectionsCached = true;
  }
  numIntersections = m_numSelfIntersections;
  return m_hasSelfIntersections;
}

void MeshInspector::drawInspectionUI() {
  if (m_V.rows() == 0 || m_F.rows() == 0) {
    ImGui::TextDisabled("No mesh loaded");
    return;
  }

  if (m_meshChanged) {
    // Count the number of irregular vertices, the border is ignored
    auto irregular = igl::is_irregular_vertex(m_F);
    m_irregularVertices = std::count(irregular.begin(), irregular.end(), true);

    // Compute areas, min, max, and standard deviation
    Eigen::VectorXd area;
    igl::doublearea(m_V, m_F, area);
    area = area.array() / 2.0;

    m_averageArea = area.mean();
    m_minArea = area.minCoeff() / m_averageArea;
    m_maxArea = area.maxCoeff() / m_averageArea;
    m_sigmaArea =
        sqrt(((area.array() - m_averageArea) / m_averageArea).square().mean());

    // Compute per face angles, min, max and standard deviation
    Eigen::MatrixXd angles;
    igl::internal_angles(m_V, m_F, angles);
    // to degree
    angles = 360.0 * (angles / (2 * igl::PI));

    m_averageAngle = angles.mean();
    m_minAngle = angles.minCoeff();
    m_maxAngle = angles.maxCoeff();
    m_sigmaAngle = sqrt(
        ((angles.array() - m_averageAngle) / m_averageAngle).square().mean());
  }

  // 统计，总是显示
  if (m_irregularVertices > 0) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       "Irregular vertices: %d, ratio: %.2f",
                       m_irregularVertices, m_irregularVertices / m_V.rows());
  }
  ImGui::Text("Average area: %.2f, min: %.2f, max: %.2f, sigma: %.2f",
              m_averageArea, m_minArea, m_maxArea, m_sigmaArea);
  ImGui::Text("Average angle: %.2f, min: %.2f, max: %.2f, sigma: %.2f",
              m_averageAngle, m_minAngle, m_maxAngle, m_sigmaAngle);

  // 基本检查 - 总是显示
  bool isEdgeManifold = this->isEdgeManifold();
  if (!isEdgeManifold) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       "Mesh is not edge manifold");
  } else {
    ImGui::TextDisabled("Mesh is edge manifold");
  }

  bool isVertexManifold = this->isVertexManifold();
  if (!isVertexManifold) {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                       "Mesh is not vertex manifold");
  } else {
    ImGui::TextDisabled("Mesh is vertex manifold");
  }

  // 边界循环 - 折叠节点
  if (ImGui::TreeNode("Boundary Loops")) {
    const auto &boundaryLoops = this->getBoundaryLoops();
    ImGui::Text("Number of boundary loops: %zu", boundaryLoops.size());

    // 显示每个边界循环的顶点数
    for (size_t i = 0; i < boundaryLoops.size(); i++) {
      ImGui::Text("Loop %zu: %zu vertices", i, boundaryLoops[i].size());
    }

    ImGui::TreePop();
  }

  // 自相交检查 - 折叠节点，按需执行
  if (ImGui::TreeNode("Self-Intersections")) {
    int numIntersections = 0;

    if (!m_selfIntersectionsCached) {
      ImGui::TextDisabled("Self-intersection check not performed yet");
    } else {
      if (m_hasSelfIntersections) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                           "Mesh has %d self-intersections",
                           m_numSelfIntersections);
      } else {
        ImGui::TextDisabled("No self-intersections detected");
      }
    }

    // 添加执行检查的按钮
    if (ImGui::Button("Check Self-Intersections", ImVec2(-1, 0))) {
      // 显示进度提示
      ImGui::TextUnformatted("Checking for self-intersections...");
      ImGui::TreePop();

      // 执行检查
      this->checkSelfIntersections(numIntersections);

      // 强制重绘
      return;
    }

    ImGui::TreePop();
  }

  // 添加手动刷新按钮
  if (ImGui::Button("Refresh All Checks", ImVec2(-1, 0))) {
    this->resetCache();
  }
}