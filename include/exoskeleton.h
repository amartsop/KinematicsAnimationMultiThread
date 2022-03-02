#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <thread>

#include "utils.h"
#include "serial_com.h"


class Exoskeleton
{

public:

    Exoskeleton() {};

    // Initialize 
    void initialize(const std::string& serial_com, unsigned int serial_baudrate);
        
    // Get thread handle
    void get_thread_handle(std::thread *thread);

    // Read incoming data
    std::vector<double> incoming_data_callback(void);

    // Get sensor data map
    std::vector<std::string> get_sensor_data_keys(void) { return m_sensor_data_keys; }

    // Get joint angles
    std::vector<double> get_joint_angles(void);

private:

    // Measurements num
    const int m_meas_num = 13;

    // Sensors data vector (raw)
    std::vector<double> m_raw_sensor_data;

    // Sensor data keys 
    std::vector<std::string> m_sensor_data_keys = { "theta_i_1", "theta_i_2",
        "theta_i_3", "theta_i_4", "theta_m_1", "theta_m_2", "theta_m_3",
        "theta_m_4", "theta_t_1", "theta_t_2", "theta_t_3", "theta_t_4",
        "theta_t_5" };

private:

    // Serial communication handler
    std::shared_ptr<SerialCOM> m_serial;

    // Future function handle
    std::future<std::vector<double>> m_future_fun;

    // Termination flag for callback function
    bool m_return_value = 0;
};