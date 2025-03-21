#ifndef IGL_OPENGL_GFLW_IMGUI_IMGUIZMOPLUGIN_H
#define IGL_OPENGL_GFLW_IMGUI_IMGUIZMOPLUGIN_H
#include "ImGuiMenu.h"
#include <imgui.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>
#include <Eigen/Dense>

namespace imgui {
    /// Widget for a guizmo  (3D transform manipulator)
    class ImGuizmoWidget : public ImGuiWidget {
    public:
        // callback(T) called when the stored transform T changes
        std::function<void(const Eigen::Matrix4f&)> callback;
        // Whether to display
        bool visible = true;
        // whether rotating, translating or scaling
        ImGuizmo::OPERATION operation;
        // stored transformation
        Eigen::Matrix4f T;
        // Initilize with rotate operation on an identity transform (at origin)
        ImGuizmoWidget(): operation(ImGuizmo::ROTATE), T(Eigen::Matrix4f::Identity()) {
        };

        virtual void init(igl::opengl::glfw::Viewer* _viewer, ImGuiPlugin* _plugin) override;

        virtual void draw() override;
    };
}


#endif
