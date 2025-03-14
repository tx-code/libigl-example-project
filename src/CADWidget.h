#ifndef IGL_OPENGL_GLFW_IMGUI_CADWIDGET_H
#define IGL_OPENGL_GLFW_IMGUI_CADWIDGET_H

#include "../imgui/ImGuiWidget.h"
// 接下来是 Eigen 和标准库
#include <Eigen/Dense>
#include <string>
#include <vector>

// 最后是 OpenCASCADE 头文件
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>

class ImGuiPlugin;

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

  bool m_isEdgeManifold;
  bool m_isVertexManifold;
  std::vector<std::vector<int>> m_boundaryLoops;
  bool m_hasSelfIntersections;
  int m_numSelfIntersections;
};

class CADWidget : public imgui::ImGuiWidget {
public:
  CADWidget();

  void init(igl::opengl::glfw::Viewer *_viewer,
            imgui::ImGuiPlugin *_plugin) override;

  void shutdown() override;

  void draw() override;

  void open_dialog_load_step();

  bool import_step(const std::string &filename);

  void shape_to_mesh_occ(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                         Eigen::MatrixXi &F);

  void shape_to_mesh_netgen(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                            Eigen::MatrixXi &F);

  void display_model();

  void toggle_mesh_algorithm();

  void reset_view();

  // 重置检查结果缓存的辅助函数
  void reset_inspection_cache();

  void draw_tool_cutter();

private:
  TopoDS_Shape shape;

  std::string current_file;

  Eigen::MatrixXd V_occ, V_netgen;
  Eigen::MatrixXi F_occ, F_netgen;

  enum MeshAlgorithm { OCC, NETGEN };
  MeshAlgorithm current_algorithm = OCC;

  int occ_mesh_id = -1;
  int netgen_mesh_id = -1;

  bool show_model_info = true;

  // 添加mesh检查器
  MeshInspector meshInspector;

  void draw_cad_menu();
  void draw_model_info();

  // cache the bounding box of the model
  Eigen::Vector3d bounding_box_min{-1000, -1000, -1000};
  Eigen::Vector3d bounding_box_max{1000, 1000, 1000};

  // Flag to track if NETGEN meshing has been performed
  bool netgen_mesh_generated = false;
};

#endif
