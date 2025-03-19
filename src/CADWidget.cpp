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

// Mesher implementations
#include "NetgenMesher.h"
#include "OCCMesher.h"

// Libigl 头文件
#include <igl/cylinder.h>
#include <igl/writeOBJ.h>

#include <filesystem>
#include <imgui.h>
#include <nfd.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <unordered_set>

CADWidget::CADWidget() {
  this->name = "CAD";

  // Initialize meshers
  occMesher = std::make_shared<OCCMesher>();
  netgenMesher = std::make_shared<NetgenMesher>();

  // Set default mesher
  currentMesher = occMesher;
}

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
      save_mesh_to_obj();
    }

    // 添加切换网格算法的按钮
    if (!current_file.empty()) {
      // 添加切换网格精细度的按钮
      ImGui::Combo("Mesh Fineness", &current_fineness,
                   "Very Coarse\0Coarse\0Moderate\0Fine\0Very Fine\0\0");
      ImGui::SameLine();
      if (ImGui::SmallButton("Re-generate Mesh")) {
        if (currentMesher->getName() == "NETGEN") {
          // Update Netgen mesher fineness
          auto netgenMesherPtr =
              std::dynamic_pointer_cast<NetgenMesher>(netgenMesher);
          if (netgenMesherPtr) {
            netgenMesherPtr->setFineness(
                static_cast<NetgenMesher::Fineness>(current_fineness));
          }

          // Re-generate mesh
          if (!netgenMesher->perform(shape)) {
            spdlog::error("Failed to re-generate mesh [Netgen]");
          }
        } else if (currentMesher->getName() == "OCC") {
          auto occMesherPtr = std::dynamic_pointer_cast<OCCMesher>(occMesher);
          if (occMesherPtr) {
            occMesherPtr->setFineness(
                static_cast<OCCMesher::Fineness>(current_fineness));
          }

          // Re-generate mesh
          if (!occMesher->perform(shape)) {
            spdlog::error("Failed to re-generate mesh [OCC]");
          }
        }

        // Update mesh
        display_model();
        previous_fineness = current_fineness;
      }

      if (currentMesher->getName() == "OCC") {
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
      ImGui::Text("Current Algorithm: %s", currentMesher->getName().c_str());
    }

    // 添加 TopoFace ID 着色选项
    if (ImGui::Checkbox("Color by TopoFace ID", &color_by_topoface)) {
      update_mesh_colors();
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

  // Mesh info
  if (ImGui::CollapsingHeader("Mesh Info")) {
    ImGui::Text("Vertices: %ld", currentMesher->V.rows());
    ImGui::Text("Faces: %ld", currentMesher->F.rows());
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

  // 使用OCC算法创建网格
  if (!occMesher->perform(shape)) {
    spdlog::error("Failed to generate mesh [OCC]");
  }
  spdlog::info("OCC mesh time: {:.3f}s", sw);

  // 重置NETGEN网格生成标志
  netgen_mesh_generated = false;

  // 默认使用OCC算法的结果
  currentMesher = occMesher;

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

void CADWidget::toggle_mesh_algorithm() {
  if (currentMesher->getName() == "OCC") {
    currentMesher = netgenMesher;

    // 如果NETGEN网格尚未生成，则现在生成
    if (!netgen_mesh_generated || current_fineness != previous_fineness) {
      spdlog::stopwatch sw;

      // Update Netgen mesher fineness
      auto netgenMesherPtr =
          std::dynamic_pointer_cast<NetgenMesher>(netgenMesher);
      if (netgenMesherPtr) {
        netgenMesherPtr->setFineness(
            static_cast<NetgenMesher::Fineness>(current_fineness));
      }

      // Generate mesh
      if (netgenMesher->perform(shape)) {
        netgen_mesh_generated = true;
        previous_fineness = current_fineness;
        spdlog::info("NETGEN mesh time with fineness {}: {:.3f}s",
                     current_fineness, sw);
      }
    }

    spdlog::info("Switched to NETGEN mesh");
  } else {
    currentMesher = occMesher;
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

  // we only show one mesh at a time
  if (currentMesher->getName() == "OCC") {
    viewer->selected_data_index = viewer->mesh_index(occ_mesh_id);
    viewer->data(occ_mesh_id).set_mesh(currentMesher->V, currentMesher->F);
    viewer->data(occ_mesh_id).set_face_based(true);
  } else {
    // 确保NETGEN网格已经生成
    if (!netgen_mesh_generated) {
      spdlog::warn("NETGEN mesh not generated yet, switching back to OCC");
      currentMesher = occMesher;
      viewer->selected_data_index = viewer->mesh_index(occ_mesh_id);
      viewer->data(occ_mesh_id).set_mesh(currentMesher->V, currentMesher->F);
      viewer->data(occ_mesh_id).set_face_based(true);
    } else {
      viewer->selected_data_index = viewer->mesh_index(netgen_mesh_id);
      viewer->data(netgen_mesh_id).set_mesh(currentMesher->V, currentMesher->F);
      viewer->data(netgen_mesh_id).set_face_based(true);
    }
  }

  // 更新mesh检查器
  meshInspector.setMesh(currentMesher->V, currentMesher->F);

  reset_view();
}

void CADWidget::reset_inspection_cache() {
  // 重置mesh检查器的缓存
  meshInspector.resetCache();
}

void CADWidget::reset_view() {
  // 根据当前算法选择要对齐的网格数据
  const Eigen::MatrixXd &V_current = currentMesher->V;
  viewer->core().align_camera_center(V_current);
}

void CADWidget::update_mesh_colors() {
  if (!viewer || !viewer->data().F.rows())
    return;

  auto &data = viewer->data();
  if (color_by_topoface) {
    // 根据当前使用的网格选择对应的 TopoFace ID 数组
    const Eigen::VectorXi &face_ids = currentMesher->C;

    if (face_ids.size() > 0) {
      data.face_based = true;

      // 找到最大的TopoFace ID用于颜色映射
      double maxId = face_ids.maxCoeff();

      if (maxId > 0) {
        // 为每个面生成颜色
        Eigen::MatrixXd colors(data.F.rows(), 3);
        for (int i = 0; i < data.F.rows(); i++) {
          if (i < face_ids.size()) {
            // 使用HSV颜色空间，将ID映射到色相
            double hue = face_ids(i) / (double)maxId;
            // 转换HSV到RGB (简化版本，假设S=V=1)
            double h = hue * 6.0;
            double x = 1.0 - std::abs(std::fmod(h, 2.0) - 1.0);
            Eigen::RowVector3d rgb;
            if (h < 1.0)
              rgb << 1.0, x, 0.0;
            else if (h < 2.0)
              rgb << x, 1.0, 0.0;
            else if (h < 3.0)
              rgb << 0.0, 1.0, x;
            else if (h < 4.0)
              rgb << 0.0, x, 1.0;
            else if (h < 5.0)
              rgb << x, 0.0, 1.0;
            else
              rgb << 1.0, 0.0, x;
            colors.row(i) = rgb;
          } else {
            colors.row(i) = Eigen::RowVector3d(1.0, 1.0, 1.0); // 默认白色
          }
        }
        data.set_colors(colors);
      }
    }
  } else {
    // 恢复默认颜色
    data.set_colors(Eigen::RowVector3d(1.0, 1.0, 1.0));
  }
}

void CADWidget::save_mesh_to_obj() {
  if (current_file.empty()) {
    spdlog::error("No CAD model loaded, cannot save mesh");
    return;
  }

  // 使用std::filesystem获取基本文件名
  std::filesystem::path file_path(current_file);
  std::string base_filename = file_path.stem().string();

  // 初始化NFD
  NFD_Init();

  // 让用户选择目标文件夹
  nfdu8char_t *outPath = nullptr;
  nfdresult_t result = NFD_PickFolderU8(&outPath, nullptr);

  if (result == NFD_OKAY) {
    std::filesystem::path output_dir(outPath);

    // 使用增量编号命名方式来保存文件
    int counter = 0;

    // OCC网格文件路径
    std::filesystem::path occ_file_path;
    std::string occ_output_path;

    // 尝试找到一个不存在的文件名
    do {
      std::string suffix = counter == 0
                               ? ".occ.obj"
                               : "_" + std::to_string(counter) + ".occ.obj";
      occ_file_path = output_dir / (base_filename + suffix);
      counter++;
    } while (std::filesystem::exists(occ_file_path) &&
             counter < 1000); // 防止无限循环

    occ_output_path = occ_file_path.string();

    // 重置计数器
    counter = 0;

    // Netgen网格文件路径
    std::filesystem::path netgen_file_path;
    std::string netgen_output_path;

    // 尝试找到一个不存在的文件名
    do {
      std::string suffix = counter == 0
                               ? ".netgen.obj"
                               : "_" + std::to_string(counter) + ".netgen.obj";
      netgen_file_path = output_dir / (base_filename + suffix);
      counter++;
    } while (std::filesystem::exists(netgen_file_path) &&
             counter < 1000); // 防止无限循环

    netgen_output_path = netgen_file_path.string();

    // 保存OCC网格
    if (occMesher->V.rows() > 0 && occMesher->F.rows() > 0) {
      if (igl::writeOBJ(occ_output_path, occMesher->V, occMesher->F)) {
        // 使用文件名部分而非完整路径，避免中文路径乱码问题
        spdlog::info("OCC mesh saved to {}", occ_file_path.filename().string());
      } else {
        spdlog::error("Failed to save OCC mesh");
      }
    } else {
      spdlog::warn("OCC mesh is empty, nothing to save");
    }

    // 保存Netgen网格
    if (netgen_mesh_generated && netgenMesher->V.rows() > 0 &&
        netgenMesher->F.rows() > 0) {
      if (igl::writeOBJ(netgen_output_path, netgenMesher->V, netgenMesher->F)) {
        // 使用文件名部分而非完整路径，避免中文路径乱码问题
        spdlog::info("Netgen mesh saved to {}",
                     netgen_file_path.filename().string());
      } else {
        spdlog::error("Failed to save Netgen mesh");
      }
    } else {
      spdlog::warn("Netgen mesh has not been generated yet or is empty, "
                   "nothing to save");
    }

    NFD_FreePathU8(outPath);
  } else if (result == NFD_CANCEL) {
    spdlog::info("User canceled the folder selection dialog");
  } else {
    spdlog::error("Error: {}", NFD_GetError());
  }

  NFD_Quit();
}
