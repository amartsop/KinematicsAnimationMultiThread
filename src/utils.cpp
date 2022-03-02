#include "../include/utils.h"

// Convert comma-delimited string to vector of integers
std::vector<int> Utils::analog_str_buf_to_int_vec(const std::string& str)
{
    // Initialize data vector
    std::vector<int> data_vec;

    // Initialize string stream
    std::stringstream ss(str);

    // Get data and convert to analog
    while (ss.good())
    {
        std::string substr;
        std::getline(ss, substr, ',');
        data_vec.push_back(std::stoi(substr));
    }

    return data_vec;
}

// Convert comma-delimited string to vector of floats
std::vector<double> Utils::analog_str_buf_to_double_vec(const std::string& str)
{
    // Initialize data vector
    std::vector<double> data_vec;

    // Initialize string stream
    std::stringstream ss(str);

    // Get data and convert to analog
    while (ss.good())
    {
        std::string substr;
        std::getline(ss, substr, ',');
        data_vec.push_back(std::stod(substr));
    }

    return data_vec;
}