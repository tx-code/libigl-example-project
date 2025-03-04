// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2022 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_IMGUI_IMGUIWIDGET_H
#define IGL_OPENGL_GLFW_IMGUI_IMGUIWIDGET_H

#include "ImGuiPlugin.h"
#include "ImGuiWidget.h"
#include <memory>

namespace igl {
    namespace opengl {
        namespace glfw {
            class Viewer;
        }
    }
}


namespace imgui {
    // Forward declaration of the parent plugin
    class ImGuiPlugin;

    /// Abstract class for imgui "widgets". A widget is something that uses
    /// imgui, but doesn't own the entire imgui IO stack: the single
    /// ImGuiPlugin owns that and widgets are registered with it.
    class ImGuiWidget {
    public:
        ImGuiWidget() { name = "dummy"; }

        virtual ~ImGuiWidget() {
        }

        virtual void init(igl::opengl::glfw::Viewer* _viewer, ImGuiPlugin* _plugin) {
            viewer = _viewer;
            plugin = _plugin;
        }

        virtual void shutdown() {
        }

        virtual void draw() {
        }

        virtual bool mouse_down(int /*button*/, int /*modifier*/) { return false; }
        virtual bool mouse_up(int /*button*/, int /*modifier*/) { return false; }
        virtual bool mouse_move(int /*mouse_x*/, int /*mouse_y*/) { return false; }
        virtual bool key_pressed(unsigned int /*key*/, int /*modifiers*/) { return false; }
        virtual bool key_down(int /*key*/, int /*modifiers*/) { return false; }
        virtual bool key_up(int /*key*/, int /*modifiers*/) { return false; }
        std::string name;

    protected:
        // Pointer to ImGuiPlugin's parent viewer
        igl::opengl::glfw::Viewer* viewer;
        // Pointer to parent ImGuiPlugin class
        ImGuiPlugin* plugin;
    };
}

#endif
