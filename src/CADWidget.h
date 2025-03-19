#ifndef IGL_OPENGL_GLFW_IMGUI_CADWIDGET_H
#define IGL_OPENGL_GLFW_IMGUI_CADWIDGET_H

#include "../imgui/ImGuiWidget.h"
#include "IMesher.h"
#include "MeshInspector.h"
// 接下来是 Eigen 和标准库
#include <Eigen/Dense>
#include <memory>
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

  bool mouse_down(int button, int modifier) override;

  void open_dialog_load_step();

  bool import_step(const std::string &filename);

  void display_model();

  void toggle_mesh_algorithm();

  void reset_view();

  // 重置检查结果缓存的辅助函数
  void reset_inspection_cache();

  void draw_tool_cutter();

  // 保存网格到OBJ文件
  void save_mesh_to_obj();

private:
  TopoDS_Shape shape;

  std::string current_file;

  // Meshers
  // the mesher also stores the mesh data
  std::shared_ptr<IMesher> occMesher;
  std::shared_ptr<IMesher> netgenMesher;
  std::shared_ptr<IMesher> currentMesher;

  int occ_mesh_id = -1;
  int netgen_mesh_id = -1;

  bool show_model_info = true;
  bool color_by_topoface = false; // 控制是否按 TopoFace ID 着色的标志

  // 添加mesh检查器
  MeshInspector meshInspector;

  void draw_cad_menu();
  void draw_model_info();
  void update_mesh_colors(); // 更新网格颜色的方法

  // cache the bounding box of the model
  Eigen::Vector3d bounding_box_min{-1000, -1000, -1000};
  Eigen::Vector3d bounding_box_max{1000, 1000, 1000};

  // Flag to track if NETGEN meshing has been performed
  bool netgen_mesh_generated = false;
  int current_fineness = 3; // fine
  int previous_fineness = -1;
};

#endif
