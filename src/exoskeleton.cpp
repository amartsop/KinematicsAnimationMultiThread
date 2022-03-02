#include "../include/exoskeleton.h"

// Initialize 
void Exoskeleton::initialize(const std::string& serial_com,
    unsigned int serial_baudrate)
{
    // Generate serial communication channel
    m_serial = std::make_shared<SerialCOM>(serial_com, serial_baudrate);

    // Initialize stream
    m_serial->initialize_stream();

    // Initialize asychronous function
    m_future_fun = std::async(&Exoskeleton::incoming_data_callback, this);
}

// Read incoming data asychronously
std::vector<double> Exoskeleton::incoming_data_callback(void)
{
    while(!m_return_value)
    {
        m_raw_sensor_data =
            Utils::analog_str_buf_to_double_vec(m_serial->readLine());
    }

    return m_raw_sensor_data;
}

// Get sensor data
std::vector<double> Exoskeleton::get_joint_angles(void)
{
    // Set termination flag for asychronous function
    m_return_value = 1;

    // Get joint angles from future function
    std::vector<double> joint_angles = m_future_fun.get();

    // Convert to rad
    for (size_t i = 0; i < joint_angles.size(); i++)
    {
        joint_angles.at(i) = Utils::deg2rad(joint_angles.at(i));
    }

    // Reinitialize async function
    m_return_value = 0;
    m_future_fun = std::async(&Exoskeleton::incoming_data_callback, this);

    return joint_angles;
}