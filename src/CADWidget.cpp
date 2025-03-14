#include "CADWidget.h"
// OpenCASCADE 头文件
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
// Libigl 头文件
#include <igl/boundary_loop.h>
#include <igl/cylinder.h>
#include <igl/dihedral_angles.h>
#include <igl/fast_find_self_intersections.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/unique_simplices.h>

#pragma warning(push, 0)

#ifndef OCCGEOMETRY
#define OCCGEOMETRY
#endif

#include <meshing/meshing.hpp>
#include <occ/occgeom.hpp>

namespace nglib {
#include <nglib.h>
}

#pragma warning(pop)

#include <imgui.h>
#include <nfd.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <unordered_set>

namespace {

std::string to_string(const nglib::Ng_Result &res) {
  switch (res) {
  case nglib::NG_ERROR:
    return "NG_ERROR";
  case nglib::NG_OK:
    return "NG_OK";
  case nglib::NG_SURFACE_INPUT_ERROR:
    return "NG_SURFACE_INPUT_ERROR";
  case nglib::NG_VOLUME_FAILURE:
    return "NG_VOLUME_FAILURE";
  case nglib::NG_STL_INPUT_ERROR:
    return "NG_STL_INPUT_ERROR";
  case nglib::NG_SURFACE_FAILURE:
    return "NG_SURFACE_FAILURE";
  case nglib::NG_FILE_NOT_FOUND:
    return "NG_FILE_NOT_FOUND";
  default:
    return "Unknown";
  }
}

bool auto_select_linear_deflection(const TopoDS_Shape &shape, double &delf,
                                   const double linPrec = 0.001) {
  Bnd_Box bndBox;
  BRepBndLib::Add(shape, bndBox);

  if (bndBox.IsVoid()) {
    return false;
  }

  bndBox.Enlarge(0.0001);

  // use a fraction of a bounding diagonal
  const double diag = bndBox.CornerMin().Distance(bndBox.CornerMax());
  delf = linPrec * diag;
  spdlog::info("Auto linear deflection: {}", delf);
  return true;
}

double auto_select_linear_deflection(const TopoDS_Shape &shape) {
  double linPrec = 1e-6;
  auto_select_linear_deflection(shape, linPrec);
  return linPrec;
}

bool do_netgen(const TopoDS_Shape &shape, const double linDefl,
               const double minH, const double maxH, const double grading,
               Eigen::MatrixXd &V, Eigen::MatrixXi &F,
               std::unordered_map<int, std::unordered_set<int>> &face_elems) {
  // FIXME: 需要优化
  Bnd_Box bbox;
  BRepBndLib::Add(shape, bbox, false);
  const double diag = std::sqrt(bbox.SquareExtent());
  spdlog::info("Diagonal: {}", diag);

  using namespace nglib;
  // Parameters definitions
  netgen::MeshingParameters mp;
  mp.delaunay2d = true;
  mp.minh = 1e-4 * diag;
  mp.maxh = 1e-2 * diag;
  mp.uselocalh = true;
  mp.secondorder = false;
  mp.grading = 0.6;
  std::ostringstream oss;
  mp.Print(oss);
  spdlog::info("Meshing Parameters: \n{}", oss.str());

  Ng_Init();
  auto occ_geometry = std::make_shared<netgen::OCCGeometry>();
  netgen::OCCParameters occ_params;
  netgen::Mesh mesh;
  occ_geometry->shape = shape;
  occ_geometry->changed = 1;
  occ_geometry->BuildFMap();
  occ_geometry->CalcBoundingBox();
  occ_geometry->PrintNrShapes();
  occ_geometry->FixFaceOrientation();
  OCCSetLocalMeshSize(*occ_geometry, mesh, mp, occ_params);

  mesh.SetGeometry(occ_geometry);
  occ_geometry->Analyse(mesh, mp);
  occ_geometry->FindEdges(mesh, mp);
  occ_geometry->MeshSurface(mesh, mp);

  auto nb_nodes = mesh.GetNP();
  auto nb_triangles = mesh.GetNSE();
  spdlog::info("before cleaning: Nodes: {}, Triangles: {}", nb_nodes,
               nb_triangles);

  V.resize(nb_nodes, 3);
  F.resize(nb_triangles, 3);

  for (int i = 1; i <= nb_nodes; i++) {
    const auto &p = mesh[netgen::PointIndex(i)];
    V(i - 1, 0) = p[0];
    V(i - 1, 1) = p[1];
    V(i - 1, 2) = p[2];
  }

  for (int i = 1; i <= nb_triangles; i++) {
    const auto &t = mesh[netgen::SurfaceElementIndex(i)];
    if (t.GetNP() == 3) {
      F(i - 1, 0) = t[0] - 1;
      F(i - 1, 1) = t[1] - 1;
      F(i - 1, 2) = t[2] - 1;
    } else {
      spdlog::error("Triangle has {} nodes", t.GetNP());
    }
  }

  Ng_Exit();
  return true;
}

// 清理网格后，删除重复的顶点(OCC先离散边，再离散面，共享边的顶点会重复#相邻面的数量)
void clean_mesh_after_mesher(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                             Eigen::MatrixXd &V_Clean,
                             Eigen::MatrixXi &F_Clean) {

  constexpr double epsilon = 1e-7;
  Eigen::VectorXi I, J; // I is the index of the original vertices, J is the
                        // index of the cleaned vertices
  igl::remove_duplicate_vertices(V, F, epsilon, V_Clean, I, J, F_Clean);
  spdlog::info("removed {} duplicate vertices", V.rows() - V_Clean.rows());
}

} // namespace

CADWidget::CADWidget() { this->name = "CAD"; }

void CADWidget::init(igl::opengl::glfw::Viewer *_viewer,
                     imgui::ImGuiPlugin *_plugin) {
  ImGuiWidget::init(_viewer, _plugin);
}

void CADWidget::shutdown() { ImGuiWidget::shutdown(); }

void CADWidget::draw() {
  ImGui::SetNextWindowPos(ImVec2(ImGui::GetCursorScreenPos().x + 100, 0),
                          ImGuiCond_FirstUseEver);
  // set the size to (0, 0) to make it auto-resized
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  bool _viewer_menu_visible = true;

  ImGui::Begin("CAD Tools", &_viewer_menu_visible,
               ImGuiWindowFlags_AlwaysAutoResize);

  // 绘制CAD菜单内容
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
  draw_cad_menu();

  // 显示当前模型信息
  if (!current_file.empty()) {
    draw_model_info();
  }

  draw_tool_cutter();

  ImGui::PopItemWidth();
  ImGui::End();
}

// 在随机位置生成指定直径和长度的圆柱体，轴线朝向z轴
void create_random_cylinder(const double radius,           // 圆柱体半径
                            const double height,           // 圆柱体高度
                            const Eigen::Vector3d &center, // 圆柱体中心点
                            const int axis_divisions, // 圆周方向的分段数
                            const int height_divisions, // 高度方向的分段数
                            Eigen::MatrixXd &V,         // 输出顶点
                            Eigen::MatrixXi &F)         // 输出面
{
  // 生成单位圆柱体的侧面（半径为1，高度为1，中心在原点）
  Eigen::MatrixXd V_side;
  Eigen::MatrixXi F_side;
  igl::cylinder(axis_divisions, 2, V_side, F_side); // 高度方向只需要2个分段

  // 计算顶点数和面数
  int num_side_vertices = V_side.rows();
  int num_side_faces = F_side.rows();

  // 创建带有顶部和底部的完整圆柱体
  // 需要添加两个额外的顶点（顶部中心和底部中心）
  V.resize(num_side_vertices + 2, 3);

  // 复制侧面顶点
  V.block(0, 0, num_side_vertices, 3) = V_side;

  // 添加顶部中心点和底部中心点
  V.row(num_side_vertices) = Eigen::Vector3d(0, 0, 1);     // 顶部中心
  V.row(num_side_vertices + 1) = Eigen::Vector3d(0, 0, 0); // 底部中心

  // 计算顶部和底部的面数
  int num_cap_faces =
      axis_divisions * 2; // 顶部和底部各有axis_divisions个三角形

  // 分配面的内存
  F.resize(num_side_faces + num_cap_faces, 3);

  // 复制侧面的面
  F.block(0, 0, num_side_faces, 3) = F_side;

  // 添加顶部的面
  for (int i = 0; i < axis_divisions; i++) {
    int next_i = (i + 1) % axis_divisions;
    F.row(num_side_faces + i) =
        Eigen::Vector3i(i + axis_divisions,      // 顶部外围顶点
                        next_i + axis_divisions, // 下一个顶部外围顶点
                        num_side_vertices        // 顶部中心点
        );
  }

  // 添加底部的面
  for (int i = 0; i < axis_divisions; i++) {
    int next_i = (i + 1) % axis_divisions;
    F.row(num_side_faces + axis_divisions + i) =
        Eigen::Vector3i(next_i,               // 下一个底部外围顶点
                        i,                    // 底部外围顶点
                        num_side_vertices + 1 // 底部中心点
        );
  }

  // 缩放到指定半径和高度
  for (int i = 0; i < V.rows(); i++) {
    // 缩放x和y坐标（半径）
    V(i, 0) *= radius;
    V(i, 1) *= radius;
    // 缩放z坐标（高度）
    V(i, 2) *= height;
  }

  // 平移到指定中心点
  for (int i = 0; i < V.rows(); i++) {
    V.row(i) += (center - Eigen::Vector3d(0, 0, height / 2)).transpose();
  }
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

    // 添加切换网格算法的按钮
    if (!current_file.empty()) {
      if (current_algorithm == OCC) {
        if (ImGui::Button("Switch to NETGEN Mesh", ImVec2(-1, 0))) {
          toggle_mesh_algorithm();
        }
        if (!netgen_mesh_generated) {
          ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f),
                             "NETGEN mesh will be generated when switching");
        }
      } else {
        if (ImGui::Button("Switch to OCC Mesh", ImVec2(-1, 0))) {
          toggle_mesh_algorithm();
        }
      }

      // 显示当前使用的算法
      ImGui::Text("Current Algorithm: %s",
                  current_algorithm == OCC ? "OCC" : "NETGEN");
    }
  }
}

void CADWidget::draw_tool_cutter() {
  // Create and Display a mesh to represent the cylindar cutter
  // Cylinder parameters
  static double radius = 0.4;
  static double height = 10.0;
  constexpr double step_slow = 0.1;
  constexpr double step_fast = 1.0;
  const double pos_min = bounding_box_min.minCoeff();
  const double pos_max = bounding_box_max.maxCoeff();
  const double scale = (pos_max - pos_min) / 2.0;
  const double offset = (pos_max + pos_min) / 2.0;
  static int tool_id = -1;
  static Eigen::Vector3d center(0, 0, 0);
  ImGui::InputScalar("Radius", ImGuiDataType_Double, &radius, &step_slow,
                     &step_fast);
  ImGui::InputScalar("Height", ImGuiDataType_Double, &height, &step_slow,
                     &step_fast);
  ImGui::SliderScalarN("Center", ImGuiDataType_Double, center.data(), 3,
                       &pos_min, &pos_max);
  ImGui::SameLine();
  if (ImGui::SmallButton("Random")) {
    // Eigen::Vector3d::Random() 默认生成的是范围在 [-1, 1] 之间的随机值
    center =
        Eigen::Vector3d::Random() * scale + Eigen::Vector3d::Constant(offset);
  }

  if (ImGui::Button("Create Cylinder", ImVec2(-1, 0))) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    create_random_cylinder(radius, height, center, 20, 10, V, F);

    if (tool_id == -1) {
      tool_id = viewer->append_mesh();
    } else {
      viewer->data(tool_id).clear();
    }
    viewer->data(tool_id).set_mesh(V, F);
    viewer->data(tool_id).compute_normals();
    // white color
    viewer->data(tool_id).set_colors(Eigen::RowVector3d(1, 1, 1));
  }
}

void CADWidget::draw_model_info() {
  ImGui::Separator();
  ImGui::Text("File: %s", current_file.c_str());
  // Topo Shape info
  if (ImGui::CollapsingHeader("Topo Info")) {
    switch (shape.ShapeType()) {
    case TopAbs_COMPOUND:
      ImGui::Text("COMPOUND");
      break;
    case TopAbs_COMPSOLID:
      ImGui::Text("COMPSOLID");
      break;
    case TopAbs_SOLID:
      ImGui::Text("SOLID");
      break;
    case TopAbs_SHELL:
      ImGui::Text("SHELL");
      break;
    case TopAbs_FACE:
      ImGui::Text("FACE");
      break;
    case TopAbs_WIRE:
      ImGui::Text("WIRE");
      break;
    case TopAbs_EDGE:
      ImGui::Text("EDGE");
      break;
    case TopAbs_VERTEX:
      ImGui::Text("VERTEX");
      break;
    case TopAbs_SHAPE:
      ImGui::Text("SHAPE");
      break;
    default:
      ImGui::Text("Unknown Type");
      break;
    }

    // 2. 拓扑元素计数
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

    ImGui::Text("Faces: %d", faceMap.Extent());

    // 3. 包围盒

    ImGui::Text("Bounding Box:");
    ImGui::Text("  X range: %f to %f", bounding_box_min.x(),
                bounding_box_max.x());
    ImGui::Text("  Y range: %f to %f", bounding_box_min.y(),
                bounding_box_max.y());
    ImGui::Text("  Z range: %f to %f", bounding_box_min.z(),
                bounding_box_max.z());
    ImGui::Text("  Size: %f x %f x %f",
                (bounding_box_max.x() - bounding_box_min.x()),
                (bounding_box_max.y() - bounding_box_min.y()),
                (bounding_box_max.z() - bounding_box_min.z()));
  }

  const Eigen::MatrixXd &V_current =
      (current_algorithm == OCC) ? V_occ : V_netgen;
  const Eigen::MatrixXi &F_current =
      (current_algorithm == OCC) ? F_occ : F_netgen;
  // Mesh info
  if (ImGui::CollapsingHeader("Mesh Info")) {
    ImGui::Text("Vertices: %ld", V_current.rows());
    ImGui::Text("Faces: %ld", F_current.rows());
  }

  // Inspection
  if (ImGui::CollapsingHeader("Inspection")) {
    // 使用MeshInspector绘制检查UI
    meshInspector.drawInspectionUI();
  }

  if (ImGui::Button("Reset View", ImVec2(-1, 0))) {
    reset_view();
  }
}

void CADWidget::open_dialog_load_step() {
  // First, pop up a dialog to ask the user to select the file

  NFD_Init();

  nfdu8char_t *outPath = nullptr;
  nfdu8filteritem_t filters[1] = {{"CAD Models", "step,stp"}};
  nfdopendialogu8args_t args = {0};
  args.filterList = filters;
  args.filterCount = 1;
  auto result = NFD_OpenDialogU8_With(&outPath, &args);

  if (result == NFD_OKAY) {
    import_step(outPath);
    NFD_FreePathU8(outPath);
  } else if (result == NFD_CANCEL) {
    // User canceled the dialog
    spdlog::info("User canceled the dialog");
  } else {
    // Handle other error codes
    spdlog::error("Error: {}", NFD_GetError());
  }

  NFD_Quit();
}

bool CADWidget::import_step(const std::string &filename) {
  STEPControl_Reader reader;

  spdlog::stopwatch sw;
  // 读取STEP文件
  if (reader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
    return false;
  }
  spdlog::info("Read step file time from {} in {:.3f}s", filename, sw);

  // 转换根节点
  reader.TransferRoots();

  shape = reader.OneShape();
  spdlog::info("Transfer roots time: {:.3f}s", sw);

  // 只使用OCC算法创建网格
  shape_to_mesh_occ(shape, V_occ, F_occ);
  spdlog::info("OCC mesh time: {:.3f}s", sw);

  // 重置NETGEN网格生成标志
  netgen_mesh_generated = false;

  // 默认使用OCC算法的结果
  current_algorithm = OCC;

  current_file = filename;

  Bnd_Box boundingBox;
  BRepBndLib::Add(shape, boundingBox);
  // Enlarge the bounding box by 10%
  boundingBox.Enlarge(Sqrt(boundingBox.SquareExtent()) * 0.1);

  bounding_box_min =
      Eigen::Vector3d(boundingBox.CornerMin().X(), boundingBox.CornerMin().Y(),
                      boundingBox.CornerMin().Z());
  bounding_box_max =
      Eigen::Vector3d(boundingBox.CornerMax().X(), boundingBox.CornerMax().Y(),
                      boundingBox.CornerMax().Z());

  // 显示在查看器中
  display_model();

  return true;
}

// occ的网格化一般用于可视化
void CADWidget::shape_to_mesh_occ(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                                  Eigen::MatrixXi &F) {
  // 创建形状的网格 (OCC算法)
  IMeshTools_Parameters params;
  params.Deflection = auto_select_linear_deflection(shape);
  params.Angle = 0.5;
  params.Relative = true;
  params.MinSize = params.Deflection * 0.01;
  params.InParallel = true; // enable the parallel mode

  try {
    // Note: will automatically call Perform()
    BRepMesh_IncrementalMesh mesh(shape, params);
  } catch (Standard_Failure &e) {
    spdlog::error("Error: {}", e.GetMessageString());
    return;
  }

  // Summary info
  int total_vertices = 0;
  double max_deflection = 0.0;
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
      max_deflection = std::max(max_deflection, triangulation->Deflection());
    }
  }

  spdlog::info("OCC mesh info before cleaning: {} vertices, {} triangles, max "
               "deflection: {}",
               total_vertices, total_triangles, max_deflection);

  // 调整矩阵大小
  Eigen::MatrixXd V_Uncleaned;
  Eigen::MatrixXi F_Uncleaned;
  V_Uncleaned.resize(total_vertices, 3);
  F_Uncleaned.resize(total_triangles, 3);

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
        V_Uncleaned(vertex_index, 0) = p.X();
        V_Uncleaned(vertex_index, 1) = p.Y();
        V_Uncleaned(vertex_index, 2) = p.Z();
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

        F_Uncleaned(face_index, 0) = offset + n1 - 1;
        F_Uncleaned(face_index, 1) = offset + n2 - 1;
        F_Uncleaned(face_index, 2) = offset + n3 - 1;
        face_index++;
      }
    }
  }

  clean_mesh_after_mesher(V_Uncleaned, F_Uncleaned, V, F);
  spdlog::info("OCC mesh info after cleaning: {} vertices, {} triangles",
               V.rows(), F.rows());
}

void CADWidget::shape_to_mesh_netgen(const TopoDS_Shape &shape,
                                     Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
  const double lineDefl = auto_select_linear_deflection(shape);
  const double minH = lineDefl * 0.05;
  const double maxH = lineDefl * 5.5;
  const double grading = 0.8;

  std::unordered_map<int, std::unordered_set<int>> face_elems;
  do_netgen(shape, lineDefl, minH, maxH, grading, V, F, face_elems);
}

void CADWidget::toggle_mesh_algorithm() {
  if (current_algorithm == OCC) {
    current_algorithm = NETGEN;

    // 如果NETGEN网格尚未生成，则现在生成
    if (!netgen_mesh_generated) {
      spdlog::stopwatch sw;
      shape_to_mesh_netgen(shape, V_netgen, F_netgen);
      netgen_mesh_generated = true;
      spdlog::info("NETGEN mesh time: {:.3f}s", sw);
    }

    spdlog::info("Switched to NETGEN mesh");
  } else {
    current_algorithm = OCC;
    spdlog::info("Switched to OCC mesh");
  }

  // 更新显示
  display_model();
}

void CADWidget::display_model() {
  if (occ_mesh_id == -1) {
    occ_mesh_id = viewer->append_mesh();
  } else {
    viewer->data(occ_mesh_id).clear();
  }

  if (netgen_mesh_id == -1) {
    netgen_mesh_id = viewer->append_mesh();
  } else {
    viewer->data(netgen_mesh_id).clear();
  }

  // 根据当前算法选择要显示的网格数据
  Eigen::MatrixXd V_current;
  Eigen::MatrixXi F_current;

  // we only show one mesh at a time
  if (current_algorithm == OCC) {
    viewer->selected_data_index = viewer->mesh_index(occ_mesh_id);
    viewer->data(occ_mesh_id).set_mesh(V_occ, F_occ);
    viewer->data(occ_mesh_id).compute_normals();
    viewer->data(occ_mesh_id).set_face_based(true);
    V_current = V_occ;
    F_current = F_occ;
  } else {
    // 确保NETGEN网格已经生成
    if (!netgen_mesh_generated) {
      spdlog::warn("NETGEN mesh not generated yet, switching back to OCC");
      current_algorithm = OCC;
      viewer->selected_data_index = viewer->mesh_index(occ_mesh_id);
      viewer->data(occ_mesh_id).set_mesh(V_occ, F_occ);
      viewer->data(occ_mesh_id).compute_normals();
      viewer->data(occ_mesh_id).set_face_based(true);
      V_current = V_occ;
      F_current = F_occ;
    } else {
      viewer->selected_data_index = viewer->mesh_index(netgen_mesh_id);
      viewer->data(netgen_mesh_id).set_mesh(V_netgen, F_netgen);
      viewer->data(netgen_mesh_id).compute_normals();
      viewer->data(netgen_mesh_id).set_face_based(true);
      V_current = V_netgen;
      F_current = F_netgen;
    }
  }

  // 更新mesh检查器
  meshInspector.setMesh(V_current, F_current);

  reset_view();
}

void CADWidget::reset_inspection_cache() {
  // 重置mesh检查器的缓存
  meshInspector.resetCache();
}

void CADWidget::reset_view() {
  // 根据当前算法选择要对齐的网格数据
  const Eigen::MatrixXd &V_current =
      (current_algorithm == OCC) ? V_occ : V_netgen;
  viewer->core().align_camera_center(V_current);
}
