// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2022 Alec Jacobson <alecjacobson@gmail.com>
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_IMGUI_IMGUIMENU_H
#define IGL_OPENGL_GLFW_IMGUI_IMGUIMENU_H

#include "ImGuiPlugin.h"
#include "ImGuiWidget.h"
#include <memory>


namespace imgui {
    /// Widget for a menu bar and a viewer window.
    class ImGuiMenu : public ImGuiWidget {
    public:
        virtual void init(igl::opengl::glfw::Viewer* _viewer, ImGuiPlugin* _plugin) override;

        virtual void shutdown() override;

        virtual void draw() override;

        // Can be overwritten by `callback_draw_viewer_window`
        virtual void draw_viewer_window();

        // Can be overwritten by `callback_draw_viewer_menu`
        virtual void draw_viewer_menu();

        // Can be overwritten by `callback_draw_custom_window`
        virtual void draw_custom_window() {
        }

        // Customizable callbacks
        std::function<void(void)> callback_draw_viewer_window;
        std::function<void(void)> callback_draw_viewer_menu;
        std::function<void(void)> callback_draw_custom_window;

        float menu_scaling() {
            return plugin->hidpi_scaling() / plugin->pixel_ratio();
        }
    };
} // namespace imgui
#endif
