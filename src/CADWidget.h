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

private:
  TopoDS_Shape shape;

  std::string current_file;

  Eigen::MatrixXd V_occ, V_netgen;
  Eigen::MatrixXi F_occ, F_netgen;

  enum MeshAlgorithm { OCC, NETGEN };
  MeshAlgorithm current_algorithm = OCC;

  int occ_mesh_id = -1;
  int netgen_mesh_id = -1;

  int current_mesh_id = -1;

  bool show_model_info = true;

  void draw_cad_menu();
  void draw_model_info();
};

#endif
