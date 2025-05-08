#include "path_manager/path_manager.hpp"
PathManager::PathManager(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
    this->declare_parameter("localization_method", "Fixposition");
    this->declare_parameter("read_path_service", "read_path_file");
    this->declare_parameter("return_path_service", "get_robot_Path");
    this->declare_parameter("write_path_service", "write_path_file");
    this->declare_parameter("get_ll_path_service", "get_ll_path");
    this->declare_parameter("get_fix_frame_path_service", "get_fix_frame_path");
    this->declare_parameter("fixed_frame_topic", "fixed_frame_topic");
    this->declare_parameter("ll_path_topic", "ll_path");
    this->declare_parameter("global_parameter_name", "test");}

void PathManager::publish() {

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathManager::on_configure(const rclcpp_lifecycle::State &) {
    timer_ = this->create_wall_timer(1s, [this]() {return this->publish();});
    // Retrieve and use parameters for service and topic names
    std::string read_path_service, return_path_service, write_path_service;
    std::string get_ll_path_service, get_fix_frame_path_service;
    std::string fixed_frame_topic, ll_path_topic;

    this->get_parameter("read_path_service", read_path_service);
    this->get_parameter("return_path_service", return_path_service);
    this->get_parameter("write_path_service", write_path_service);
    this->get_parameter("get_ll_path_service", get_ll_path_service);
    this->get_parameter("get_fix_frame_path_service", get_fix_frame_path_service);
    this->get_parameter("fixed_frame_topic", fixed_frame_topic);
    this->get_parameter("ll_path_topic", ll_path_topic);
    this->get_parameter("localization_method", localization_method);
    this->get_parameter("global_parameter_name", global_param);
    std::cout<< "Global param is "<<global_param<<std::endl;
    robot_path.poses.clear();// IMPORTANT!! robot_path will save the path to be attemptd by the robot when performing autonomous navigation
    // Use the parameters for service and topic names
    readPathServ = this->create_service<path_manager_interfaces::srv::ReadPathFromFile>
    (read_path_service, std::bind(&PathManager::readPath, this, std::placeholders::_1, std::placeholders::_2));
    getRobotPath = this->create_service<path_manager_interfaces::srv::ReturnRobotPath>
    (return_path_service, std::bind(&PathManager::returnPath, this, std::placeholders::_1, std::placeholders::_2));
    writePathServ = this->create_service<path_manager_interfaces::srv::WritePathToFile>
    (write_path_service, std::bind(&PathManager::writePath, this, std::placeholders::_1, std::placeholders::_2));
    getLLPath = this->create_service<path_manager_interfaces::srv::GetLLPath>
    (get_ll_path_service, std::bind(&PathManager::getLatLonPath, this, std::placeholders::_1, std::placeholders::_2));
    getFixFramePath = this->create_service<path_manager_interfaces::srv::GetFixFramePath>
    (get_fix_frame_path_service, std::bind(&PathManager::getFFPath, this, std::placeholders::_1, std::placeholders::_2));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(fixed_frame_topic, 10);
    sub_ll_path = this->create_subscription<nav_msgs::msg::Path>(ll_path_topic, 10, 
    std::bind(&PathManager::ll_path_callback, this, std::placeholders::_1));
    plannerPathServ = this->create_service<path_manager_interfaces::srv::PlanPath>("path_planner", 
    std::bind(&PathManager::planPath, this, std::placeholders::_1, std::placeholders::_2));
    receivePathserv = this->create_service<path_manager_interfaces::srv::RobotPath>("receive_ll_path", 
    std::bind(&PathManager::receivePath, this, std::placeholders::_1, std::placeholders::_2));
    assistEmersrv  = this->create_service<path_manager_interfaces::srv::AssistEmergency>("assist_emergency", 
    std::bind(&PathManager::assistEmergency, this, std::placeholders::_1, std::placeholders::_2));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathManager::on_activate(const rclcpp_lifecycle::State & state) {
    LifecycleNode::on_activate(state);
    std::this_thread::sleep_for(2s); // Emulating work during activation
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathManager::on_deactivate(const rclcpp_lifecycle::State & state) {
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathManager::on_cleanup(const rclcpp_lifecycle::State &) {
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathManager::on_shutdown(const rclcpp_lifecycle::State & state) {
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PathManager::ll_path_callback(const std::shared_ptr<nav_msgs::msg::Path> msg){
    robot_path.poses.clear();
    robot_path = *msg;
    robot_path.header.frame_id = "ll";
}


void PathManager::ConvertUTMToLatLon(double utmNorthing, double utmEasting, int zone, bool north, double& lat, double& lon) {
    GeographicLib::UTMUPS::Reverse(zone, north, utmEasting, utmNorthing, lat, lon);
}

void PathManager::receivePath(const std::shared_ptr<path_manager_interfaces::srv::RobotPath::Request> req,
                           std::shared_ptr<path_manager_interfaces::srv::RobotPath::Response> res) {
    robot_path.poses.clear();
    robot_path = req->path;
    
}

void PathManager::assistEmergency(const std::shared_ptr<path_manager_interfaces::srv::AssistEmergency::Request> req,
                           std::shared_ptr<path_manager_interfaces::srv::AssistEmergency::Response> res) {
        RCLCPP_ERROR(this->get_logger(), "Called assist emergency .");
        std::string package_path = ament_index_cpp::get_package_share_directory("path_manager");
        std::string demaias_planner_path = package_path + "/Demaias_planner_missions";  // Path to your subfolder
        nav_msgs::msg::Path result_path;
        // Check if the directory exists, if not create it
        std::filesystem::path dir(demaias_planner_path);
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);  // Create the directory if it does not exist
        }
        std::cerr << "start x " << req->start.x << " start y." << req->start.y  <<std::endl;
        std::cerr << "goal x " << req->goal.x << " goal y." << req->goal.y  <<std::endl;


        // File paths
        std::string ltl_filename = demaias_planner_path + "/LTL_mission_legion.dat";
        std::string original_mapping_filename = demaias_planner_path + "/mapping_dsm_100cm.dat"; 
        std::string dem_filename = demaias_planner_path + "/LAENTIEC100cm.xyz"; 
        std::string temp_mapping_filename = demaias_planner_path + "/temp_mapping.dat";  
        std::pair<nav_msgs::msg::Path,nav_msgs::msg::Path> planner_output; 
        nav_msgs::msg::Path utm_path;

         // Copy the content of original mapping file to the temporary file
        std::ifstream src(original_mapping_filename, std::ios::binary);
        std::ofstream dst(temp_mapping_filename, std::ios::binary);
        dst << src.rdbuf();
        src.close();
        dst.close();

        // Append new coordinates to the temporary file
        std::ofstream temp_mapping_file(temp_mapping_filename, std::ios_base::app);  // Open in append mode
        if (temp_mapping_file.is_open()) {
            temp_mapping_file << std::to_string(req->goal.x) << " " << std::to_string(req->goal.y) << " 0 legion" << std::endl;
            temp_mapping_file.close();
        } else {
            std::cerr << "Failed to open " << temp_mapping_filename << " for writing." << std::endl;
            res->success = false;
            return;
        }


        // Open, write to, and close LTL_mission.dat
        std::ofstream ltl_file(ltl_filename);
        if (ltl_file.is_open()) {
            ltl_file << std::to_string(req->start.x) << " " << std::to_string(req->start.y) << " " << std::to_string(req->start.z) << std::endl;
            //ltl_file << "<> goal";
            ltl_file << "<>(intermediate1 && <>(intermediate2 && <>legion))";
            ltl_file.close();
        } else {
            std::cerr << "Failed to open " << ltl_filename << " for writing." << std::endl;
        }
          // Simple straight-line path planning logic (1m steps)
        nav_msgs::msg::Path planned_path;
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.orientation.w = 1.0;  // Setting orientation to 0
        double dx = req->goal.x - req->start.x;
        double dy = req->goal.y - req->start.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        std::cerr << "Distance " << distance <<  std::endl;

        int steps = std::ceil(distance);
        int utm_zone = 30;
        bool is_north = true;  // Northern hemisphere
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            double latitude, longitude;
            ConvertUTMToLatLon(req->start.y + t * dy, req->start.x + t * dx, utm_zone, is_north, latitude, longitude);
            pose.pose.position.x = latitude;
            pose.pose.position.y = longitude;
            pose.pose.position.z = 0;  // Assuming constant Z
            planned_path.poses.push_back(pose);
        }
        /*planner_output = DEMAIAS::planner(const_cast<char*>(dem_filename.c_str()), const_cast<char*>(mapping_filename.c_str()),const_cast<char*>(ltl_filename.c_str()));
        utm_path = planner_output.first;

        // Clear the previous path
        robot_path.poses.clear();
        // Assume zone 30 and northern hemisphere for Malaga, Spain
        int utm_zone = 30;
        bool is_north = true;  // Northern hemisphere
        
        for (const auto& utm_pose : utm_path.poses) {
            double latitude, longitude;
            // Convert each UTM pose to latitude and longitude
            ConvertUTMToLatLon(utm_pose.pose.position.y, utm_pose.pose.position.x, utm_zone, is_north, latitude, longitude);
            // Create a new pose with the converted coordinates
            geometry_msgs::msg::PoseStamped geo_pose;
            geo_pose.pose.position.x = latitude;
            geo_pose.pose.position.y = longitude;
            float DEM_GPS_z_offset = 48.122;
            geo_pose.pose.position.z = utm_pose.pose.position.z + DEM_GPS_z_offset;
            geo_pose.header = utm_pose.header;  // Preserving the time and frame information

            // Append to the robot path
            robot_path.poses.push_back(geo_pose);
        }
        */
        //std::remove(temp_mapping_filename.c_str());
        // Set the transformed path in the response
        robot_path.poses.clear();
        robot_path = planned_path;
        res->path = planned_path;
        res->success = true;  // Assume success

        RCLCPP_ERROR(this->get_logger(), "Planned path transformed to geographic coordinates.");
    res->success = true;
}

    void PathManager::planPath(const std::shared_ptr<path_manager_interfaces::srv::PlanPath::Request> req,
                            std::shared_ptr<path_manager_interfaces::srv::PlanPath::Response> res) {
        RCLCPP_ERROR(this->get_logger(), "Called plan path .");

        std::string package_path = ament_index_cpp::get_package_share_directory("path_manager");
        std::string demaias_planner_path = package_path + "/Demaias_planner_missions";  // Path to your subfolder
        nav_msgs::msg::Path result_path;
        // Check if the directory exists, if not create it
        std::filesystem::path dir(demaias_planner_path);
        if (!std::filesystem::exists(dir)) {
            std::filesystem::create_directories(dir);  // Create the directory if it does not exist
        }

        // File paths
        std::string ltl_filename = demaias_planner_path + "/LTL_mission_legion.dat";
        std::string mapping_filename = demaias_planner_path + "/mapping_dsm_100cm.dat"; 
        std::string dem_filename = demaias_planner_path + "/LAENTIEC100cm.xyz";   
        std::pair<nav_msgs::msg::Path,nav_msgs::msg::Path> planner_output; 
        nav_msgs::msg::Path utm_path;

        // Open, write to, and close LTL_mission.dat
        std::ofstream ltl_file(ltl_filename);
        if (ltl_file.is_open()) {
            ltl_file << std::to_string(req->start.x) << " " << std::to_string(req->start.y) << " " << std::to_string(req->start.z) << std::endl;
            //ltl_file << "<> goal";
            //ltl_file << "<>(intermediate1 && <>(intermediate2 && <>legion))";
            ltl_file.close();
        } else {
            std::cerr << "Failed to open " << ltl_filename << " for writing." << std::endl;
        }

        // Open, write to, and close mapping.dat
        std::ofstream mapping_file(mapping_filename, std::ios_base::app);
        if (mapping_file.is_open()) {
            //mapping_file << std::to_string(req->goal.x) << " " << std::to_string(req->goal.y) << " " << std::to_string(req->goal.z) << " legion" << std::endl;
            //mapping_file << std::to_string(req->goal.x) << " " << std::to_string(req->goal.y) << " " << std::to_string(req->goal.z) << "  goal" ;
            mapping_file.close();
        } else {
            std::cerr << "Failed to open " << mapping_filename << " for writing." << std::endl;
        }                   
        if (req->goal.x == 0.0 and req->goal.y == 0.0){
        ltl_filename = demaias_planner_path + "/LTL_mission_test.dat";
        mapping_filename = demaias_planner_path + "/mapping_dsm_100cm.dat"; 
        dem_filename = demaias_planner_path + "/LAENTIEC100cm.xyz";   

        }

        planner_output = DEMAIAS::planner(const_cast<char*>(dem_filename.c_str()), const_cast<char*>(mapping_filename.c_str()),const_cast<char*>(ltl_filename.c_str()));
        utm_path = planner_output.first;

        // Clear the previous path
        robot_path.poses.clear();
        // Assume zone 30 and northern hemisphere for Malaga, Spain
        int utm_zone = 30;
        bool is_north = true;  // Northern hemisphere
        
        for (const auto& utm_pose : utm_path.poses) {
            double latitude, longitude;
            // Convert each UTM pose to latitude and longitude
            ConvertUTMToLatLon(utm_pose.pose.position.y, utm_pose.pose.position.x, utm_zone, is_north, latitude, longitude);
            // Create a new pose with the converted coordinates
            geometry_msgs::msg::PoseStamped geo_pose;
            geo_pose.pose.position.x = latitude;
            geo_pose.pose.position.y = longitude;
            float DEM_GPS_z_offset = 48.122;
            geo_pose.pose.position.z = utm_pose.pose.position.z + DEM_GPS_z_offset;
            geo_pose.header = utm_pose.header;  // Preserving the time and frame information

            // Append to the robot path
            robot_path.poses.push_back(geo_pose);
        }

        // Set the transformed path in the response
        res->path = robot_path;
        res->ack = true;  // Assume success

        RCLCPP_ERROR(this->get_logger(), "Planned path transformed to geographic coordinates.");
    }


    void PathManager::readPath(const std::shared_ptr<path_manager_interfaces::srv::ReadPathFromFile::Request> req,
                            const std::shared_ptr<path_manager_interfaces::srv::ReadPathFromFile::Response> res) {
        std::string full_path = getPathFolder() + "/" + req->filename + ".gpx";
        std::ifstream file(full_path);
        
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", req->filename.c_str());
            res->success = false;
            return;
        }

        robot_path.header.frame_id = "ll";  // path is in lat lon
        robot_path.header.stamp = this->get_clock()->now();
        robot_path.poses.clear();

        std::string line;
        double latitude = 0.0, longitude = 0.0, altitude = 0.0;

        while (std::getline(file, line)) {
            if (line.find("<trkpt") != std::string::npos) {
                // Extract latitude and longitude
                size_t lat_pos = line.find("lat=\"");
                size_t lon_pos = line.find("lon=\"");

                if (lat_pos != std::string::npos && lon_pos != std::string::npos) {
                    lat_pos += 5; // move past 'lat="'
                    lon_pos += 5; // move past 'lon="'

                    size_t lat_end = line.find("\"", lat_pos);
                    size_t lon_end = line.find("\"", lon_pos);

                    if (lat_end != std::string::npos && lon_end != std::string::npos) {
                        latitude = std::stod(line.substr(lat_pos, lat_end - lat_pos));
                        longitude = std::stod(line.substr(lon_pos, lon_end - lon_pos));
                    }
                }
            }

            if (line.find("<ele>") != std::string::npos) {
                // Extract altitude
                size_t ele_pos = line.find("<ele>") + 5;
                size_t ele_end = line.find("</ele>", ele_pos);

                if (ele_end != std::string::npos) {
                    altitude = std::stod(line.substr(ele_pos, ele_end - ele_pos));

                    RCLCPP_INFO(this->get_logger(), "Parsed: %f, %f, %f", latitude, longitude, altitude);

                    geometry_msgs::msg::PoseStamped poseStamped;
                    poseStamped.header = robot_path.header;
                    poseStamped.pose.position.x = latitude;
                    poseStamped.pose.position.y = longitude;
                    poseStamped.pose.position.z = altitude;
                    poseStamped.pose.orientation.w = 1.0;
                    robot_path.poses.push_back(poseStamped);
                }
            }
        }

        file.close();
        res->success = true;
    }

    std::string PathManager::getPathFolder() {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("path_manager");
        std::string path_folder = package_share_directory + "/paths";
        std::cout << "path " << path_folder << std::endl;
        return path_folder;
    }

    void PathManager::writePath(const std::shared_ptr<path_manager_interfaces::srv::WritePathToFile::Request> req,
                                const std::shared_ptr<path_manager_interfaces::srv::WritePathToFile::Response> res) {
        // Construct the full file path
        std::string full_path = getPathFolder() + "/" + req->filename + ".gpx";

        // Open the file for writing
        std::ofstream file(full_path);

        // Check if the file was successfully opened
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", full_path.c_str());
            res->success = false;
            return;
        }

        // Write the GPX file header
        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<gpx version=\"1.1\" creator=\"PathManager - ROS2\">\n";
        file << "  <trk>\n";
        file << "    <name>" << req->filename << "</name>\n";
        file << "    <trkseg>\n";

        // Write each pose to the GPX file
        file << std::fixed << std::setprecision(7);
        for (const auto& pose : robot_path.poses) {
            file << "      <trkpt lat=\"" << pose.pose.position.x << "\" lon=\"" << pose.pose.position.y << "\">\n";
            file << "        <ele>" << pose.pose.position.z << "</ele>\n"; // altitude
            file << "      </trkpt>\n";
        }

        // Write the GPX file footer
        file << "    </trkseg>\n";
        file << "  </trk>\n";
        file << "</gpx>\n";

        // Close the file
        file.close();

        // Indicate success in the response
        res->success = true;
    }

void PathManager::getLatLonPath(const std::shared_ptr<path_manager_interfaces::srv::GetLLPath::Request> req,
                                const std::shared_ptr<path_manager_interfaces::srv::GetLLPath::Response> res) {
    res->path = robot_path; // Assuming robot_path is already in latitude-longitude format
    res->success = true;
}

geometry_msgs::msg::PoseStamped PathManager::ConvertToECEF(double latitude, double longitude, 
                                              double altitude) {
    double lat_rad = latitude * M_PI / 180.0;  // Convert latitude to radians
    double lon_rad = longitude * M_PI / 180.0; // Convert longitude to radians
    //double altitude = 100;
    // Calculate ECEF coordinates
    geometry_msgs::msg::PoseStamped ecef_pose;
    double N = a / sqrt(1 - e_squared * sin(lat_rad) * sin(lat_rad));
    ecef_pose.pose.position.x = (N + altitude) * cos(lat_rad) * cos(lon_rad);
    ecef_pose.pose.position.y = (N + altitude) * cos(lat_rad) * sin(lon_rad);
    ecef_pose.pose.position.z = (N * (1 - e_squared) + altitude) * sin(lat_rad);

    return ecef_pose;
}

void PathManager::getFFPath(const std::shared_ptr<path_manager_interfaces::srv::GetFixFramePath::Request> req,
                            const std::shared_ptr<path_manager_interfaces::srv::GetFixFramePath::Response> res) {
    nav_msgs::msg::Path ff_path;

    if (localization_method == "Fixposition"){
      ff_path.header.frame_id = "ECEF";

      for (const auto& pose : robot_path.poses) {
        std::cout<<"pose"<<std::endl;
          auto ecef_pose = ConvertToECEF(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
          ecef_pose.header.frame_id = "ECEF";
          
          ff_path.poses.push_back(ecef_pose);
      }
    }
    else if(localization_method == "Robot_localization"){
      ff_path.header.frame_id = "utm";

      for (const auto& pose : robot_path.poses) {
          double utmNorthing, utmEasting;
          int zone;
          bool isNorth;

          latLonToUTM(pose.pose.position.x, pose.pose.position.y, utmNorthing, utmEasting, zone, isNorth);

          geometry_msgs::msg::PoseStamped ff_pose;
          ff_pose.pose.position.x = utmEasting;
          ff_pose.pose.position.y = utmNorthing;
          ff_pose.pose.orientation = pose.pose.orientation; // Assuming orientation remains the same
          ff_path.poses.push_back(ff_pose);
      }     
    }


    path_publisher_->publish(ff_path);
    res->path = ff_path;
}


void PathManager::returnPath(const std::shared_ptr<path_manager_interfaces::srv::ReturnRobotPath::Request> req,
                             const std::shared_ptr<path_manager_interfaces::srv::ReturnRobotPath::Response> res) {
    res->path = robot_path;
}

void PathManager::latLonToUTM(double latitude, double longitude, double& utmNorthing, double& utmEasting, int& zone, bool& isNorth) {
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, isNorth, utmEasting, utmNorthing);
}

int main(int argc, char * argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Ensures synchronized stdout when launched from ROS launch files.

    rclcpp::init(argc, argv);

    auto path_manager_node = std::make_shared<PathManager>("path_manager_node");
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(path_manager_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
