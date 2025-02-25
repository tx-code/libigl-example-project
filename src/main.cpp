#include <filesystem>
#include <igl/AABB.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOFF.h>
#include <igl/screen_space_selection.h>
#include <iostream>

#include "../imgui/ImGuiMenu.h"
#include "../imgui/ImGuiPlugin.h"
#include "../imgui/ImGuizmoWidget.h"
#include "CADWidget.h"

int main(int argc, char* argv[]) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // Load a mesh in OFF format
    // If a path is provided, load the mesh from the path
    if (argc > 1) {
        std::filesystem::path path(argv[1]);
        if (path.extension() == ".off") {
            igl::readOFF(argv[1], V, F);
        } else {
            std::cerr << "Unsupported file extension: " << path.extension()
                    << std::endl;
            return 1;
        }
    } else {
        // load the example mesh from the examples directory
        igl::readOFF("examples/bunny.off", V, F);
    }

    // Init the viewer
    igl::opengl::glfw::Viewer viewer;

    // Attach a menu plugin
    imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);
    CADWidget cad_widget;
    plugin.widgets.push_back(&cad_widget);

    // Add content to the default menu window
    // Plot the mesh
    viewer.data().set_mesh(V, F);
    viewer.data().add_label(viewer.data().V.row(0) +
                            viewer.data().V_normals.row(0).normalized() *
                            0.005,
                            "Hello World!");
    viewer.launch();
}
