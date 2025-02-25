#include "CADWidget.h"
// OpenCASCADE 头文件
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <imgui.h>

CADWidget::CADWidget() { this->name = "CAD"; }

void CADWidget::init(igl::opengl::glfw::Viewer *_viewer,
                     imgui::ImGuiPlugin *_plugin) {
  ImGuiWidget::init(_viewer, _plugin);
}

void CADWidget::shutdown() { ImGuiWidget::shutdown(); }

void CADWidget::draw() {
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(200.0f, 0.0f), ImGuiCond_FirstUseEver);
  bool _viewer_menu_visible = true;

  ImGui::Begin("CAD Tools", &_viewer_menu_visible);

  // 绘制CAD菜单内容
  draw_cad_menu();

  // 显示当前模型信息
  if (!current_file.empty()) {
    draw_model_info();
  }

  ImGui::End();
}

void CADWidget::draw_cad_menu() {
  if (ImGui::CollapsingHeader("CAD", ImGuiTreeNodeFlags_DefaultOpen)) {
    float w = ImGui::GetContentRegionAvail().x;
    float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::Button("Load##CAD", ImVec2((w - p) / 2.f, 0))) {
      open_dialog_load_step();
    }
    ImGui::SameLine(0, p);
    if (ImGui::Button("Save##CAD", ImVec2((w - p) / 2.f, 0))) {
      // viewer->open_dialog_save_mesh();
    }
  }
}

void CADWidget::draw_model_info() {
  ImGui::Separator();
  ImGui::Text("File: %s", current_file.c_str());
  ImGui::Text("Vertices: %ld", V.rows());
  ImGui::Text("Faces: %ld", F.rows());

  if (ImGui::Button("Reset View", ImVec2(-1, 0))) {
    reset_view();
  }
}

void CADWidget::open_dialog_load_step() {}

bool CADWidget::import_step(const std::string &filename) {
  STEPControl_Reader reader;

  // 读取STEP文件
  if (reader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
    return false;
  }

  // 转换根节点
  reader.TransferRoots();

  shape = reader.OneShape();

  // 创建网格
  shape_to_mesh(shape, V, F);
  current_file = filename;

  // 显示在查看器中
  display_model();

  return true;
}

void CADWidget::shape_to_mesh(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                              Eigen::MatrixXi &F) {
  // 创建形状的网格
  BRepMesh_IncrementalMesh mesh(shape, 0.1); // 0.1是偏转值
  mesh.Perform();

  // 计算顶点和面的总数
  int total_vertices = 0;
  int total_triangles = 0;
  std::vector<int> face_vertex_offsets;

  TopExp_Explorer explorer;
  for (explorer.Init(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location loc;

    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, loc);
    if (!triangulation.IsNull()) {
      face_vertex_offsets.push_back(total_vertices);
      total_vertices += triangulation->NbNodes();
      total_triangles += triangulation->NbTriangles();
    }
  }

  // 调整矩阵大小
  V.resize(total_vertices, 3);
  F.resize(total_triangles, 3);

  // 填充矩阵
  int vertex_index = 0;
  int face_index = 0;
  int face_count = 0;

  for (explorer.Init(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location loc;

    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, loc);
    if (!triangulation.IsNull()) {
      int offset = face_vertex_offsets[face_count++];

      // 获取顶点
      const auto &nodes = triangulation->MapNodeArray();
      for (int i = nodes->Lower(); i <= nodes->Upper(); i++) {
        gp_Pnt p = nodes->Value(i).Transformed(loc);
        V(vertex_index, 0) = p.X();
        V(vertex_index, 1) = p.Y();
        V(vertex_index, 2) = p.Z();
        vertex_index++;
      }

      // 获取三角形
      const auto &triangles = triangulation->MapTriangleArray();
      for (int i = triangles->Lower(); i <= triangles->Upper(); i++) {
        int n1, n2, n3;
        triangles->Value(i).Get(n1, n2, n3);

        // 根据面朝向调整
        if (face.Orientation() == TopAbs_REVERSED) {
          std::swap(n2, n3);
        }

        F(face_index, 0) = offset + n1 - 1;
        F(face_index, 1) = offset + n2 - 1;
        F(face_index, 2) = offset + n3 - 1;
        face_index++;
      }
    }
  }
}

void CADWidget::display_model() {
  viewer->data().clear();
  viewer->data().set_mesh(V, F);
  viewer->data().compute_normals();
  reset_view();
}

void CADWidget::reset_view() { viewer->core().align_camera_center(V); }
