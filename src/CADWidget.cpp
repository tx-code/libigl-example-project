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
#include <igl/boundary_loop.h>
#include <igl/dihedral_angles.h>
#include <igl/fast_find_self_intersections.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>

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
  spdlog::info("Nodes: {}, Triangles: {}", nb_nodes, nb_triangles);

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

// 添加一个全局变量来控制是否需要重置检查缓存
bool g_reset_inspection_cache = false;
} // namespace

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

    // 添加切换网格算法的按钮
    if (!current_file.empty()) {
      if (ImGui::Button("Toggle Mesh Algorithm", ImVec2(-1, 0))) {
        toggle_mesh_algorithm();
      }

      // 显示当前使用的算法
      ImGui::Text("Current Algorithm: %s",
                  current_algorithm == OCC ? "OCC" : "NETGEN");
    }
  }
}

void CADWidget::draw_model_info() {
  ImGui::Separator();
  ImGui::Text("File: %s", current_file.c_str());
  // Topo Shape info
  if (ImGui::CollapsingHeader("Topo Info", ImGuiTreeNodeFlags_DefaultOpen)) {
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
    Bnd_Box boundingBox;
    BRepBndLib::Add(shape, boundingBox);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    boundingBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    ImGui::Text("Bounding Box:");
    ImGui::Text("  X range: %f to %f", xMin, xMax);
    ImGui::Text("  Y range: %f to %f", yMin, yMax);
    ImGui::Text("  Z range: %f to %f", zMin, zMax);
    ImGui::Text("  Size: %f x %f x %f", (xMax - xMin), (yMax - yMin),
                (zMax - zMin));
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
  if (ImGui::CollapsingHeader("Inspection", ImGuiTreeNodeFlags_DefaultOpen)) {
    // 声明所有静态变量
    static bool is_edge_manifold_cached = false;
    static bool is_vertex_manifold_cached = false;
    static bool is_edge_manifold_result = false;
    static bool is_vertex_manifold_result = false;
    static int last_mesh_id = -1;
    static MeshAlgorithm last_algorithm = OCC;
    static bool boundary_loops_cached = false;
    static std::vector<std::vector<int>> boundary_loops;
    static bool self_intersections_cached = false;
    static bool has_self_intersections = false;
    static int num_self_intersections = 0;

    // 检查是否需要重置缓存
    if (g_reset_inspection_cache) {
      is_edge_manifold_cached = false;
      is_vertex_manifold_cached = false;
      boundary_loops_cached = false;
      self_intersections_cached = false;
      g_reset_inspection_cache = false;
    }
    
    // 只有在网格改变时才重新计算
    bool need_recheck = (current_mesh_id != last_mesh_id) || 
                        (current_algorithm != last_algorithm) || 
                        !is_edge_manifold_cached || 
                        !is_vertex_manifold_cached;
    
    if (need_recheck) {
      // 更新缓存
      is_edge_manifold_result = igl::is_edge_manifold(F_current);
      is_vertex_manifold_result = igl::is_vertex_manifold(F_current);
      is_edge_manifold_cached = true;
      is_vertex_manifold_cached = true;
      last_mesh_id = current_mesh_id;
      last_algorithm = current_algorithm;
    }
    
    // 显示缓存的结果
    if (is_edge_manifold_cached) {
      if (!is_edge_manifold_result) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                         "Mesh is not edge manifold");
      } else {
        ImGui::TextDisabled("Mesh is edge manifold");
      }
    }
    
    if (is_vertex_manifold_cached) {
      if (!is_vertex_manifold_result) {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                         "Mesh is not vertex manifold");
      } else {
        ImGui::TextDisabled("Mesh is vertex manifold");
      }
    }
    
    // 边界循环计算
    if (ImGui::TreeNode("Boundary Loops")) {
      if (!boundary_loops_cached) {
        // 只有在需要时才计算边界循环
        boundary_loops.clear();
        igl::boundary_loop(F_current, boundary_loops);
        boundary_loops_cached = true;
      }
      
      ImGui::Text("Number of boundary loops: %zu", boundary_loops.size());
      
      // 显示每个边界循环的顶点数
      for (size_t i = 0; i < boundary_loops.size(); i++) {
        ImGui::Text("Loop %zu: %zu vertices", i, boundary_loops[i].size());
      }
      
      ImGui::TreePop();
    }
    
    // 自相交检查（计算成本高，仅在用户请求时执行）
    if (ImGui::TreeNode("Self-Intersections")) {
      // 检查是否需要重置缓存
      if (g_reset_inspection_cache) {
        self_intersections_cached = false;
      }
      
      // 显示当前状态
      if (!self_intersections_cached) {
        ImGui::TextDisabled("Self-intersection check not performed yet");
      } else {
        if (has_self_intersections) {
          ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                           "Mesh has %d self-intersections", num_self_intersections);
        } else {
          ImGui::TextDisabled("No self-intersections detected");
        }
      }
      
      // 添加执行检查的按钮
      if (ImGui::Button("Check Self-Intersections", ImVec2(-1, 0))) {
        // 执行自相交检查
        Eigen::MatrixXi intersect;
        
        // 显示进度提示
        ImGui::TextUnformatted("Checking for self-intersections...");
        ImGui::TreePop();
        
        // 执行检查（这可能很耗时）
        has_self_intersections = igl::fast_find_self_intersections(V_current, F_current, intersect);
        
        // 更新结果
        num_self_intersections = (intersect.array() == 1).count();
        self_intersections_cached = true;

        // 强制重绘
        return;
      }
      
      ImGui::TreePop();
    }
    
    // 添加手动刷新按钮
    if (ImGui::Button("Refresh All Checks", ImVec2(-1, 0))) {
      is_edge_manifold_cached = false;
      is_vertex_manifold_cached = false;
      boundary_loops_cached = false;
      self_intersections_cached = false;
      g_reset_inspection_cache = true;
    }
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

  // 使用两种算法创建网格
  shape_to_mesh_occ(shape, V_occ, F_occ);
  spdlog::info("OCC mesh time: {:.3f}s", sw);

  shape_to_mesh_netgen(shape, V_netgen, F_netgen);
  spdlog::info("NETGEN mesh time: {:.3f}s", sw);

  // 默认使用OCC算法的结果
  current_algorithm = OCC;

  current_file = filename;

  // 显示在查看器中
  display_model();

  return true;
}

void CADWidget::shape_to_mesh_occ(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                                  Eigen::MatrixXi &F) {
  // 创建形状的网格 (OCC算法)
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
    spdlog::info("Switched to NETGEN mesh");
  } else {
    current_algorithm = OCC;
    spdlog::info("Switched to OCC mesh");
  }

  // 重置检查结果缓存
  reset_inspection_cache();

  // 更新显示
  display_model();
}

void CADWidget::display_model() {
  // 如果使用mesh_id方式管理多个网格，可以这样实现
  if (current_mesh_id >= 0) {
    // 移除当前显示的网格
    viewer->erase_mesh(viewer->mesh_index(current_mesh_id));
  }

  // 根据当前算法选择要显示的网格数据
  const Eigen::MatrixXd &V_current =
      (current_algorithm == OCC) ? V_occ : V_netgen;
  const Eigen::MatrixXi &F_current =
      (current_algorithm == OCC) ? F_occ : F_netgen;

  // 添加新的网格并获取其ID
  viewer->data().clear();
  viewer->data().set_mesh(V_current, F_current);
  viewer->data().compute_normals();
  current_mesh_id = viewer->data().id;

  // 重置检查结果缓存
  reset_inspection_cache();

  reset_view();
}

void CADWidget::reset_inspection_cache() {
  // 设置全局标志，表示需要重置缓存
  g_reset_inspection_cache = true;
}

void CADWidget::reset_view() {
  // 根据当前算法选择要对齐的网格数据
  const Eigen::MatrixXd &V_current =
      (current_algorithm == OCC) ? V_occ : V_netgen;
  viewer->core().align_camera_center(V_current);
}
