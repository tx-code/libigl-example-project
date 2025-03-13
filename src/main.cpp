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

int main(int argc, char *argv[]) {
  igl::opengl::glfw::Viewer viewer;

  // Attach a menu plugin
  imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);
  CADWidget cad_widget;
  plugin.widgets.push_back(&cad_widget);

  viewer.launch();
}
