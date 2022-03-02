#pragma once

#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <Python.h>
#include <stdio.h>

#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>


class MenuHandler
{
public:
    MenuHandler(igl::opengl::glfw::imgui::ImGuiMenu* menu);

    // Menu callback
    void callback(void);

    // Get ports sets flag
    bool are_ports_set(void) { return m_ports_set; }

    // Get left exoskeleton port
    std::string get_left_exoskeleton_port(void) { return m_left_exoskeleton_port; }

    // Get right exoskeleton port
    std::string get_right_exoskeleton_port(void) { return m_right_exoskeleton_port; }

private:
    igl::opengl::glfw::imgui::ImGuiMenu *m_menu;

    // Get available USB ports
    std::vector<std::string> get_available_usb_ports(void);

    // Generate absolute file name 
    std::string generate_absolute_filename(const std::filesystem::path&
        absolute_path, const std::filesystem::path& folder,
        const std::string& relative_filename);

private:
    // Script file relative
    std::string m_script_name_rel = "ports_read.py";
    
    // Ports file relative
    std::string m_ports_file_rel = "available_ports.txt";

    // Left exoskeleton port
    std::string m_left_exoskeleton_port;

    // Right exoskeleton port
    std::string m_right_exoskeleton_port;

    // Ports have been set flag
    bool m_ports_set = 0;
};