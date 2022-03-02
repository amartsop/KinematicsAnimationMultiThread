#include "../include/menu_handler.h"


MenuHandler::MenuHandler(igl::opengl::glfw::imgui::ImGuiMenu* menu)
{
    // Copy menu pointer
    m_menu = menu;
}

void MenuHandler::callback(void)
{
    // Draw viewer menu
    m_menu->draw_viewer_menu();

    // Add new group
    if (ImGui::CollapsingHeader("Set USB Ports", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (!m_ports_set)
        {
            // Get available usb poirts
            std::vector<std::string> available_ports = get_available_usb_ports();

            static std::vector<std::string> choices;

            static int left_exo_idx_choise = 0;
            static int right_exo_idx_choise = 0;
        
            // Get left exoskeleton port
            ImGui::Combo("Left exoskeleton", &left_exo_idx_choise,
                available_ports);

            // Get right exoskeleton port
            ImGui::Combo("Right exoskeleton", &right_exo_idx_choise,
                available_ports);
        
            if (ImGui::Button("OK"))
            {
                m_left_exoskeleton_port = available_ports.at(left_exo_idx_choise);
                m_right_exoskeleton_port = available_ports.at(right_exo_idx_choise);
                m_ports_set = 1;
            }
        }
    }
}

// Get available USB ports
std::vector<std::string> MenuHandler::get_available_usb_ports(void)
{
    // Initialize ports vector    
    std::vector<std::string> ports_vec;

    // Get absolute file path
    std::filesystem::path absolute_path = std::filesystem::current_path();

    // Define include relative folder
    std::filesystem::path include_folder("include");

    // Define script relative folder
    std::filesystem::path script_folder("scripts");

    // Define share relative folder
    std::filesystem::path share_folder("share");

    // Define absolute script folder
    std::filesystem::path script_folder_a = include_folder / script_folder;

    // Get full script path
    std::string script_name = generate_absolute_filename(absolute_path, 
        script_folder_a, m_script_name_rel);

    // Get full available ports path
    std::string ports_path = generate_absolute_filename(absolute_path, 
        share_folder, m_ports_file_rel);
    
    // Run python program
	Py_Initialize();
	FILE* fp = fopen(script_name.c_str(), "r");
    PyRun_SimpleFile(fp, script_name.c_str());
	Py_Finalize();

    // Read available ports
    std::ifstream ports_file(ports_path);
    std::string line;

    while (std::getline(ports_file, line))
    {
        std::istringstream iss(line);
        ports_vec.push_back(line);
    }

    return ports_vec;
}


// Generate absolute file name 
std::string MenuHandler::generate_absolute_filename(
    const std::filesystem::path& absolute_path,
    const std::filesystem::path& folder, const std::string& relative_filename)
{
    // Define absolute script folder
    std::filesystem::path full_folder = absolute_path / folder;

    // Define full script name
    std::filesystem::path full_name = full_folder / relative_filename;

    return full_name.string();
}