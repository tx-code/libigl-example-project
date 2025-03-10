#include "CADWidget.h"
#include <igl/boundary_loop.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/fast_find_self_intersections.h>
#include <imgui.h>

MeshInspector::MeshInspector()
    : m_meshChanged(true),
      m_edgeManifoldCached(false),
      m_vertexManifoldCached(false),
      m_boundaryLoopsCached(false),
      m_selfIntersectionsCached(false),
      m_isEdgeManifold(false),
      m_isVertexManifold(false),
      m_hasSelfIntersections(false),
      m_numSelfIntersections(0) {}

void MeshInspector::setMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
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

const std::vector<std::vector<int>>& MeshInspector::getBoundaryLoops() {
    if (!m_boundaryLoopsCached) {
        igl::boundary_loop(m_F, m_boundaryLoops);
        m_boundaryLoopsCached = true;
    }
    return m_boundaryLoops;
}

bool MeshInspector::checkSelfIntersections(int& numIntersections) {
    if (!m_selfIntersectionsCached) {
        Eigen::MatrixXi intersect;
        m_hasSelfIntersections = igl::fast_find_self_intersections(m_V, m_F, intersect);
        m_numSelfIntersections = intersect.rows();
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
        const auto& boundaryLoops = this->getBoundaryLoops();
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
                               "Mesh has %d self-intersections", m_numSelfIntersections);
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