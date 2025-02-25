// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2022 Alec Jacobson <alecjacobson@gmail.com>
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_IMGUI_IMGUIPLUGIN_H
#define IGL_OPENGL_GLFW_IMGUI_IMGUIPLUGIN_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include "ImGuiWidget.h"

// Forward declarations
struct ImGuiContext;

namespace imgui {
    // Forward declaration of child widget abstract type
    class ImGuiWidget;

    /// Plugin for the viewer to enable imgui widgets
    class ImGuiPlugin : public igl::opengl::glfw::ViewerPlugin {
    protected:
        // Hidpi scaling to be used for text rendering.
        float hidpi_scaling_;
        // Ratio between the framebuffer size and the window size.
        // May be different from the hipdi scaling!
        float pixel_ratio_;
        // ImGui Context
        ImGuiContext* context_ = nullptr;

    public:
        // List of registered widgets
        std::vector<ImGuiWidget *> widgets;

    public:
        virtual void init(igl::opengl::glfw::Viewer* _viewer) override;

        void init_widgets();

        virtual void reload_font(int font_size = 13);

        virtual void shutdown() override;

        virtual bool pre_draw() override;

        virtual bool post_draw() override;

        virtual void post_resize(int width, int height) override;

        virtual bool mouse_down(int button, int modifier) override;

        virtual bool mouse_up(int button, int modifier) override;

        virtual bool mouse_move(int mouse_x, int mouse_y) override;

        virtual bool mouse_scroll(float delta_y) override;

        // Keyboard IO
        virtual bool key_pressed(unsigned int key, int modifiers) override;

        virtual bool key_down(int key, int modifiers) override;

        virtual bool key_up(int key, int modifiers) override;

        void draw_text(
            const Eigen::Vector3d pos,
            const Eigen::Vector3d normal,
            const std::string& text,
            const Eigen::Vector4f color = Eigen::Vector4f(0, 0, 0.04, 1)); // old default color
        float pixel_ratio();

        float hidpi_scaling();
    };
}

#endif
