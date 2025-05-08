#!/usr/bin/python3
# This Python file uses the following encoding: utf-8
import rclpy
import sys
import math
import time
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import threading
import cv2
import os
from scipy import ndimage
from copy import deepcopy
from GUI_pkg import ros_classes
from GUI_pkg.zoom_image_handler import ZoomImageHandler
from GUI_pkg.path_utils import PathUtils
import random
import utm
from PySide6.QtCore import QTimer, QThread, QObject, Signal, Qt
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QMenu,
    QDialog,
    QVBoxLayout,
    QLabel,
    QWidget,
    QListWidget,
    QPushButton,
)
import GUI_pkg  # Test if the GUI_pkg module can be imported
from GUI_pkg.cythonized_code.cythonize_fncs import draw_UGV_icon
from PySide6.QtGui import (
    QImage,
    QPixmap,
    QAction,
    QGuiApplication,
    QPainter,
    QColor,
    QBrush,
)
import cProfile
import subprocess
from rclpy.duration import Duration

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, and
#     pyside6-rcc resources.qrc -o resources_rc.py
from .ui_tabs import Ui_MainWindow


class ConnectivityWorker(QObject):
    connectivity_signal = Signal(int)

    def __init__(self, ip_address):
        super().__init__()
        self.ip_address = ip_address
        self.is_running = True

    def ping_ip(self):
        while self.is_running:
            command = ["ping", "-c", "4", self.ip_address]
            try:
                output = subprocess.run(command, stdout=subprocess.PIPE, text=True)
                if output.returncode == 0:
                    packet_loss = self.parse_packet_loss(output.stdout)
                    avg_rtt = self.parse_avg_rtt(output.stdout)
                    level = self.determine_connectivity_level(packet_loss, avg_rtt)
                else:
                    level = 0  # Disconnected
            except Exception as e:
                level = 0  # Disconnected
            self.connectivity_signal.emit(level)
            time.sleep(5)
            
    def parse_packet_loss(self, ping_output):
        """Parse the packet loss percentage from ping output."""
        for line in ping_output.splitlines():
            if "packet loss" in line:
                return float(line.split("%")[0].split()[-1])
        return 100.0  # Assume 100% packet loss if parsing fails

    def parse_avg_rtt(self, ping_output):
        """Parse the average round-trip time (RTT) from ping output."""
        for line in ping_output.splitlines():
            if "rtt min/avg/max/mdev" in line:
                return float(line.split("/")[4])
        return float("inf")  # Assume infinite RTT if parsing fails

    def determine_connectivity_level(self, packet_loss, avg_rtt):
        """Determine connectivity level based on packet loss and RTT."""
        if packet_loss == 100.0:
            return 0  # Disconnected
        elif packet_loss > 50.0 or avg_rtt > 200:
            return 1  # Bad connection
        elif packet_loss > 20.0 or avg_rtt > 100:
            return 2  # Poor connection
        elif packet_loss > 5.0 or avg_rtt > 50:
            return 3  # Fair connection
        else:
            return 4  # Good connection

    def stop(self):
        self.is_running = False


class WorkerTimer(QObject):
    mission_state_signal = Signal(str)
    platform_state_signal = Signal(str)
    # fsm_mission_state_signal = Signal(int)
    possible_states_signal = Signal(object)
    security_status_signal = Signal(bool)

    def __init__(self, ros_class):
        super(WorkerTimer, self).__init__()
        self.ros = ros_class
        self.timer_refresh_servs1 = QTimer()
        self.timer_refresh_servs1.timeout.connect(self.timer_slow)
        self.timer_refresh_servs1.setInterval(2000)
        self.timer_refresh_servs1.start()

    def timer_slow(self):
        res_m = self.ros.send_lc_state_request()
        self.mission_state_signal.emit(res_m)
        res_p = self.ros.send_pltform_lc_state_request()
        self.platform_state_signal.emit(res_p)
        #res_fsm = self.ros.get_J8_FSM_mode()
        # self.fsm_mission_state_signal.emit(res_fsm)
        # res_pos_states = self.ros.get_pos_transitions()
        # self.possible_states_signal.emit(res_pos_states)
        res_security_status = self.ros.get_security_status_srv()
        self.security_status_signal.emit(res_security_status)


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.profiler = cProfile.Profile()
        #self.profiler.enable()  # Start profiling
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        rclpy.init(args=None)
        self.ros_class_topics = ros_classes.ROSclass_topics()
        topics_executor = rclpy.executors.SingleThreadedExecutor()
        topics_executor.add_node(self.ros_class_topics)
        topics_executor_thread = threading.Thread(
            target=topics_executor.spin, daemon=True
        )
        topics_executor_thread.start()
        self.ros_class_srv = ros_classes.ROSclass_srv()
        self.gui_path = []
        self.nav_path = Path()
        # thread = threading.Thread(target=rclpy.spin(ros_class), daemon=True)
        # thread.start()
        self.map_gui_scale = 9.9
        self.UTM_x_up_north = 4064630.45
        self.UTM_y_up_north = 366809.917
        self.m_to_pix_x = 0.1
        self.m_to_pix_y = 0.1
        self.path_utils = PathUtils(
            self.UTM_x_up_north, self.UTM_y_up_north, self.m_to_pix_x, self.m_to_pix_y
        )
        self.current_mouse_x = 0
        self.current_mouse_y = 0
        self.draw_path_pixels = []
        self.clicked_pix_x = []
        self.clicked_pix_y = []
        self.gui_path = []
        self.goal_pix = []
        self.nav_utm_path = Path()
        self.robot_ip = "127.0.0.1"
        self.timer_refresh_vars = QTimer()
        self.timer_refresh_vars.timeout.connect(self.timer_fast)
        self.timer_refresh_vars.setInterval(25)
        self.timer_refresh_fsm = QTimer()
        self.timer_refresh_vars.start()
        self.timer_refresh_fsm.timeout.connect(self.timer_fsm)
        self.timer_refresh_fsm.setInterval(3000)
        self.timer_refresh_fsm.start()
        self.thread = QThread()
        self.worker = WorkerTimer(self.ros_class_srv)
        self.worker.moveToThread(self.thread)
        self.worker.mission_state_signal.connect(self.mission_state_slot)
        self.worker.platform_state_signal.connect(self.platform_state_slot)
        self.worker.security_status_signal.connect(self.security_check_slot)
        self.thread.start()
        script_dir = os.path.dirname(__file__)
        j8_icon_path = os.path.join(script_dir, 'resources', 'j8_icon.PNG')
        map_real_path = os.path.join(script_dir, 'resources', 'map.png')
        dem_map_path = os.path.join(script_dir, 'resources', 'dem.png')

        self.j8_icon = cv2.imread(j8_icon_path)
        self.map_real = cv2.imread(map_real_path)
        self.map_dem = cv2.imread(dem_map_path)
        self.image_handler = ZoomImageHandler(
            self.map_real, self.j8_icon
        )  # Initialize the handler
        self.zoom_corners = [0, self.map_real.shape[0], 0, self.map_real.shape[1]]
        self.pix_x_UGV = self.map_real.shape[0]
        self.pix_y_UGV = self.map_real.shape[1]
        self.map_mode = True
        self.map = self.map_real
        self.path_width = 6
        self.live_pos = []
        self.drawing_rectangle = True
        self.save_live_pos = False
        self.zoom_active = False
        self.select_poi_active = False
        self.draw_path = False
        self.drawing_path = False
        self.handled_emergency = False
        self.zoom_scale = 1.0
        self.utm_zone_number = 30
        self.utm_zone_letter = "S"
        self.icon_width = self.j8_icon.shape[0]
        self.icon_length = self.j8_icon.shape[1]
        self.security_status = True
        self.icon_scale = 10
        self.state_names = [
            "Ready",
            "PathFollowing",
            "Teleop",
            "BackHome",
            "E-Stop",
            "RecordPath",
        ]
        self.possible_transitions = []
        self.local_path_gui = Path()
        self.local_path_utm = Path()
        self.live_path_color = [
            int(random.randint(0, 255)),
            int(random.randint(0, 255)),
            int(random.randint(0, 255)),
        ]
        self.node_status_window = (
            None  # This will hold the instance of NodeStatusWindow
        )
        self.populate_fsm_dropdown()
        # Set up the worker and thread for connectivity checking
        self.connectivity_worker = ConnectivityWorker(self.robot_ip)
        self.connectivity_thread = QThread()
        self.connectivity_worker.moveToThread(self.connectivity_thread)
        self.connectivity_worker.connectivity_signal.connect(
            self.update_connectivity_label
        )
        self.connectivity_thread.started.connect(self.connectivity_worker.ping_ip)
        self.connectivity_thread.start()
        # Create a submenu in the menu bar
        self.status_colors = {
            "Not running": "black",
            "active": "green",
            "finalized": "red",
            "unconfigured": "yellow",
            "inactive": "blue",
        }

        self.mode_names = {
            0: "Ready",
            1: "PathFollowing",
            2: "Teleoperation",
            3: "GoingHome",
            4: "EmergencyStop",
            5: "RecordPath",
            # Add other modes as necessary
        }

    def update_localization_label(self):
        color = "black"  # Default to black for "Not started"
        if self.ros_class_topics.localization_status == 1:
            color = "red"  # Vision only
        elif self.ros_class_topics.localization_status == 2:
            color = "yellow"  # Visual inertial fusion
        elif self.ros_class_topics.localization_status == 3:
            color = "blue"  # Inertial-GNSS fusion
        elif self.ros_class_topics.localization_status == 4:
            color = "green"  # Visual-inertial-GNSS fusion
        self.ui.fixposition_status_val.setPixmap(self.create_colored_icon(color))

    def update_connectivity_label(self, level):
        if level == 0:
            self.ui.label_4.setPixmap(self.create_colored_icon("black"))
        elif level == 1:
            self.ui.label_4.setPixmap(self.create_colored_icon("red"))
        elif level == 2:
            self.ui.label_4.setPixmap(self.create_colored_icon("yellow"))
        elif level == 3:
            self.ui.label_4.setPixmap(self.create_colored_icon("blue"))
        elif level == 4:
            self.ui.label_4.setPixmap(self.create_colored_icon("green"))

    def get_key_from_value(self, d, value):
        for key, val in d.items():
            if val == value:
                return key
        return None  # Return None or an appropriate value if not found

    def update_lc_state(self, response):
        if response is not None:
            # Update the GUI based on the response
            # For example, if response contains a state label:
            state_label = response.current_state.label
            # Now update a label or a status bar in your GUI:
            self.ui.someStatusLabel.setText(state_label)
        else:
            # Handle the case where response is None
            print("No response received or an error occurred")

    def qImageToNumpy(self, incomingImage):
        """Converts a QImage into an opencv MAT format"""

        incomingImage = incomingImage.convertToFormat(QImage.Format.Format_RGBA8888)
        width = incomingImage.width()
        height = incomingImage.height()
        ptr = incomingImage.constBits()
        arr = np.array(ptr).reshape(height, width, 4)  #  Copies the data
        arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        return arr

    def send_config_ctl(self):
        self.draw_path_pixels = []
        self.draw_path = not (self.draw_path)

    def select_POI_cb(self):
        self.select_poi_active = not (self.select_poi_active)

    def toggle_security_check(self):
        print("inc")
        current_text = self.ui.change_security.currentText()
        if current_text == "Enable":
            self.security_status = True
        elif current_text == "Disable":
            self.security_status = False
        print(self.security_status)
        self.ros_class_srv.toggle_security_srv(self.security_status)

    def del_live_pos(self):
        self.save_live_pos = False
        self.live_pos = []

    def on_security_changed(self):
        current_text = self.ui.change_security.currentText()
        if current_text == "Enable":
            self.security_status = True
        elif current_text == "Disable":
            self.security_status = False

    def save_live_pos_callback(self):
        self.save_live_pos = not (self.save_live_pos)

    def emergency_stop(self):
        self.ros_class_srv.send_FSM_change_request(int(9))

    def send_drawn_path_cb(self):
        print("chec")
        print("no resampled path " + str(self.draw_path_pixels))

        resampled_pixel_path = self.path_utils.resample_path(self.draw_path_pixels)
        lat_lon_drawn_path = self.path_utils.convert_pixel_path_to_latlon(
            resampled_pixel_path
        )
        print("resampled path " + str(resampled_pixel_path))
        self.ros_class_srv.send_draw_path_srv(lat_lon_drawn_path)

    def path_file_read(self):
        filename = self.ui.filename_to_read.toPlainText()
        res = self.ros_class_srv.read_path_from_file(filename)
        res = self.ros_class_srv.get_J8_path_request()
        self.nav_path = res.path
        self.nav_utm_path = self.path_utils.convert_path_to_utm(res.path)
        print(self.nav_utm_path)

    def path_file_write(self):
        filename = self.ui.filename_to_write.toPlainText()
        res = self.ros_class_srv.write_path_to_file(filename)

    def set_state_icon(self, label, state):
        color = self.status_colors.get(
            state, "black"
        )  # Default to black if state is unknown
        icon = self.create_colored_icon(color)
        label.setPixmap(icon)

    def create_colored_icon(self, color, width=20, length=80):
        pixmap = QPixmap(length, width)
        pixmap.fill(Qt.transparent)  # Transparent background
        painter = QPainter(pixmap)
        brush = QBrush(QColor(color))
        painter.setBrush(brush)
        painter.setPen(Qt.NoPen)
        painter.drawRect(0, 0, length, width)
        painter.end()
        return pixmap

    def mission_state_slot(self, res):
        self.set_state_icon(self.ui.lc_mission_state, res)

    def platform_state_slot(self, res):
        self.set_state_icon(self.ui.lc_pltform_state, res)

    def security_check_slot(self, res):
        self.security_status = res
        if res == True:
            self.ui.security_status.setText("Status: Enabled")
        else:
            self.ui.security_status.setText("Status: Disabled")

    def update_fsm_dropdown(self, possible_transitions):
        current_selection = self.ui.fsm_transition.currentText()
        self.ui.fsm_transition.clear()
        state = -1
        valid_selections = []
        for transition in possible_transitions:
            state += 1
            if transition != -1:  # Assuming -1 indicates an unreachable state
                state_name = self.mode_names[state]
                self.ui.fsm_transition.addItem(state_name)
                valid_selections.append(state_name)
        # Restore the selection if it's still valid
        if current_selection in valid_selections:
            index = self.ui.fsm_transition.findText(current_selection)
            if index >= 0:
                self.ui.fsm_transition.setCurrentIndex(index)

    def possible_state_slot(self, res):
        self.possible_transitions = res
        self.update_fsm_dropdown(res)

    def fsm_mission_state_slot(self, res):
        current_time = self.ros_class_topics.get_ros_time()
        if (current_time - self.ros_class_topics.last_fsm_mode_time) > Duration(
            seconds=5
        ):
            self.ui.fsm_mission_state.setText("Not running")
        else:
            self.ui.fsm_mission_state.setText(self.state_names[res])

    def get_path(self):
        res = self.ros_class_srv.get_J8_path_request()
        self.nav_path = res.path
        self.nav_utm_path = Path()
        self.nav_utm_path = self.path_utils.convert_path_to_utm(res.path)
        print(self.nav_utm_path)

    def clear_path(self):
        self.nav_path = Path()
        self.nav_utm_path = Path()

    def toggle_view_(self):
        if self.map_mode == True:
            self.map_mode = False
            self.map = self.map_dem
        else:
            self.map_mode = True
            self.map = self.map_real

    def get_UGV_pix(self):
        pix_x = int(
            (self.UTM_x_up_north - self.ros_class_topics.UGV_UTM_x) / self.m_to_pix_x
        )
        pix_y = int(
            (self.ros_class_topics.UGV_UTM_y - self.UTM_y_up_north) / self.m_to_pix_y
        )
        return [pix_x, pix_y]

    def timer_fsm(self):
        self.update_fsm_dropdown(self.ros_class_topics.possible_transitions)

    def timer_fast(self):
        self.fsm_mission_state_slot(self.ros_class_topics.fsm_mode)
        if self.ros_class_topics.is_emergency and not (self.handled_emergency):
            self.handled_emergency = True
            self.get_path()

        if self.draw_path:
            self.ui.start_draw_path.setStyleSheet("background-color: rgb(0, 255, 0)")
            self.select_poi_active = False
        else:
            self.ui.start_draw_path.setStyleSheet("background-color: red")
        if self.select_poi_active:
            self.ui.select_POI_btn.setStyleSheet("background-color: rgb(0, 255, 0)")
            self.draw_path = False
        else:
            self.ui.select_POI_btn.setStyleSheet("background-color: red")
        if self.save_live_pos:
            self.ui.record_live_pos.setStyleSheet("background-color: rgb(0, 255, 0)")
            self.live_path_color = [
                int(random.randint(0, 255)),
                int(random.randint(0, 255)),
                int(random.randint(0, 255)),
            ]
        else:
            self.ui.record_live_pos.setStyleSheet("background-color: red")
        if self.save_live_pos:
            point = [self.ros_class_topics.UGV_UTM_x, self.ros_class_topics.UGV_UTM_y]
            self.live_pos.append(point)
        self.update_localization_label()
        self.ui.robot_latitude.setText(str(self.ros_class_topics.UTM_lat))
        self.ui.robot_longitude.setText(str(self.ros_class_topics.UTM_lon))
        self.ui.robot_heading.setText(str(self.ros_class_topics.UGV_yaw))
        if len(self.ros_class_topics.local_path.poses) > 0:
            self.local_path_utm = self.path_utils.convert_path_to_utm(
                self.ros_class_topics.local_path
            )
        self.refresh_img()

    def populate_fsm_dropdown(self):
        MODE_NAMES = {
            0: "Ready",
            1: "PathFollowing",
            2: "Teleoperation",
            3: "GoingHome",
            4: "EmergencyStop",
            5: "RecordPath",
            # ... other modes
        }
        for mode_id, mode_name in MODE_NAMES.items():
            self.ui.fsm_transition.addItem(mode_name, mode_id)

    def on_fsm_dropdown_changed(self, index):
        mode_name = self.ui.fsm_transition.currentText()
        mode_id = self.ui.fsm_transition.itemData(index)
        print(f"Selected Mode: {mode_name}, ID: {mode_id}")

    def send_transition(self):
        mode = self.ui.fsm_transition.currentText()
        print("Asked " + mode + "transition")
        transition_index = self.get_key_from_value(self.mode_names, mode)
        print(self.get_key_from_value(self.mode_names, mode))
        transition = self.ros_class_topics.possible_transitions[transition_index]
        if transition == 0:  # If the transition is ato auto mode, record the path
            self.save_live_pos = True
        self.ros_class_srv.send_FSM_change_request(int(transition))

    def is_within_image_area(self, x, y):
        # Assuming self.ui.map_ is the widget displaying the image
        image_rect = self.ui.map_.geometry()
        return image_rect.contains(x, y)

    def mousePressEvent(self, event):
        if not self.is_within_image_area(event.position().x(), event.position().y()):
            return  # Ignore clicks outside the image area
        print(
            f"unzoomed coordinates: ({int(event.position().x() * self.map_gui_scale)}, {int(event.position().y() * self.map_gui_scale)})"
        )
        if self.zoom_active:
            # Adjust the click position for zoom origin and then scale
            print("zoom scale " + str(self.zoom_scale))
            gui_x = (
                event.position().x() / self.zoom_scale
                + self.zoom_origin[0] / self.map_gui_scale
            ) * self.map_gui_scale
            gui_y = (
                event.position().y() / self.zoom_scale
                + self.zoom_origin[1] / self.map_gui_scale
            ) * self.map_gui_scale
            # Now scale up to original image dimensions
            original_x = int(gui_x * self.map_gui_scale)
            original_y = int(gui_y * self.map_gui_scale)

            print(
                f"Zoomed Click Coordinates (in display): ({int(event.position().x())}, {int(event.position().y())})"
            )
            print(f"Original high-resolution image coordinates: ({gui_x}, {gui_y})")
            # return
        if self.select_poi_active:
            if self.zoom_active:
                unzoomed_x = (
                    event.position().x() / self.zoom_scale
                    + self.zoom_origin[0] / self.map_gui_scale
                ) * self.map_gui_scale
                unzoomed_y = (
                    event.position().y() / self.zoom_scale
                    + self.zoom_origin[1] / self.map_gui_scale
                ) * self.map_gui_scale
                self.goal_pix = [int(unzoomed_x), int(unzoomed_y)]
            else:
                self.goal_pix = [
                    int(event.position().x() * self.map_gui_scale),
                    int(event.position().y() * self.map_gui_scale),
                ]
            return

        if self.draw_path:
            self.drawing_path = True  # Start capturing points for the path
        else:
            self.clicked_pix_x = [
                int(event.position().x() * self.map_gui_scale),
                int(event.position().x() * self.map_gui_scale),
            ]
            self.clicked_pix_y = [
                int(event.position().y() * self.map_gui_scale),
                int(event.position().y() * self.map_gui_scale),
            ]
            self.drawing_rectangle = True

    def mouseMoveEvent(self, event):
        if not self.is_within_image_area(event.position().x(), event.position().y()):
            return  # Ignore clicks outside the image area
        if self.zoom_active:
            pass  # Skip if zooming or drawing a path
        if not self.zoom_active and not self.draw_path:
            self.clicked_pix_x[1] = int(event.position().x() * self.map_gui_scale)
            self.adjust_zoom_rectangle(event)
        if self.draw_path:
            # Append the current position to the path list
            if self.zoom_active:
                unzoomed_x = (
                    event.position().x() / self.zoom_scale
                    + self.zoom_origin[0] / self.map_gui_scale
                ) * self.map_gui_scale
                unzoomed_y = (
                    event.position().y() / self.zoom_scale
                    + self.zoom_origin[1] / self.map_gui_scale
                ) * self.map_gui_scale
                self.draw_path_pixels.append((int(unzoomed_x), int(unzoomed_y)))
            else:
                self.draw_path_pixels.append(
                    (
                        int(event.pos().x() * self.map_gui_scale),
                        int(event.pos().y() * self.map_gui_scale),
                    )
                )

    def mouseReleaseEvent(self, event):
        if not self.is_within_image_area(event.position().x(), event.position().y()):
            return  # Ignore clicks outside the image area
        if self.select_poi_active or self.zoom_active or self.draw_path:
            return  # Skip if any modal operation is active
        self.clicked_pix_x[1] = int(event.position().x() * self.map_gui_scale)
        self.adjust_zoom_rectangle(event)

    def zoom_out(self):
        self.zoom_active = False

    def zoom_in_callback(self):
        self.zoom_active = True
        # Calculate zoom origin in terms of GUI coordinates
        self.zoom_origin = (self.clicked_pix_x[0], self.clicked_pix_y[0])
        self.zoom_width = abs(self.clicked_pix_x[1] - self.clicked_pix_x[0])
        self.zoom_height = abs(self.clicked_pix_y[1] - self.clicked_pix_y[0])
        # Calculate how much a pixel in the zoomed section is magnified compared to the original image
        self.zoom_scale = (self.map.shape[1]) / self.zoom_width

    def adjust_zoom_rectangle(self, event):
        width = abs(
            int(event.position().x() * self.map_gui_scale) - self.clicked_pix_x[0]
        )
        height = int(width * (self.map.shape[0] / self.map.shape[1]))

        if event.position().y() * self.map_gui_scale > self.clicked_pix_y[0]:
            self.clicked_pix_y[1] = min(
                self.clicked_pix_y[0] + height, self.map.shape[0]
            )
        else:
            self.clicked_pix_y[1] = max(0, self.clicked_pix_y[0] - height)

    def change_controller_cb(self):
        print("change controller")
        if self.ui.control_selected.currentText() == "Pure Pursuit":
            new_controller_type = "pure_pursuit"
        elif self.ui.control_selected.currentText() == "Regulated Pure Pursuit":
            new_controller_type = "regulated_pure_pursuit"
        elif self.ui.control_selected.currentText() == "Dynamic Pure Pursuit":
            new_controller_type = "dynamic_pure_pursuit"
        elif self.ui.control_selected.currentText() == "Dynamic LA Pure Pursuit":
            new_controller_type = "dynamic_la_pure_pursuit"
        print(new_controller_type)
        self.ros_class_srv.change_controller_type_srv(new_controller_type)

    def plan_path_cb(self):
        start = Point()
        start.x = self.ros_class_topics.UGV_UTM_y
        start.y = self.ros_class_topics.UGV_UTM_x
        print("pixel goal is " + str(self.goal_pix))
        utm_goal = self.path_utils.pix_to_UTM(self.goal_pix[1], self.goal_pix[0])
        goal = Point()
        goal.x = utm_goal[1]
        goal.y = utm_goal[0]
        print("utm goal is " + str(utm_goal))
        result = self.ros_class_srv.plan_path_srv(start, goal)
        print(result.ack)
        self.nav_utm_path = result.path
        self.get_path()
        print(self.nav_utm_path)

    def config_pure_pursuit_cb(self):
        print("config pure pursuit controller")
        look_ahead = float(self.ui.look_ahead_pure_pur_val.toPlainText())
        linear_speed = float(self.ui.linear_speed_pure_pur_val.toPlainText())
        self.ros_class_srv.config_pure_pursuit_srv(look_ahead, linear_speed)

    def config_regulated_cb(self):
        linear_speed = float(self.ui.regulated_linear_speed_val.toPlainText())
        look_ahead_distance = float(self.ui.regulated_look_ahead_dis_val.toPlainText())
        r_min = float(self.ui.regulated_r_min_val.toPlainText())
        self.ros_class_srv.config_regulated_pure_srv(
            linear_speed, look_ahead_distance, r_min
        )

    def config_stanley_cb(self):
        print("Drawn")
        pass

    def config_dynamic_la_pp_cb(self):
        print("config dynamic la controller")
        max_linear_speed = float(self.ui.dynamic_la_max_linear_speed_val.toPlainText())
        max_angular_acc = float(self.ui.dynamic_la_max_angular_acc_val.toPlainText())
        max_angular_dec = float(self.ui.dynamic_la_max_angular_dec_val.toPlainText())
        max_linear_acc = float(self.ui.dynamic_la_max_linear_acc_val.toPlainText())
        max_linear_dec = float(self.ui.dynamic_la_max_linear_dec_val.toPlainText())
        look_ahead_v_gain = float(self.ui.dynamic_la_look_ahead_gain_val.toPlainText())
        speed_pow = float(self.ui.dynamic_la_speed_power_val.toPlainText())
        min_lookahead_d  = float(self.ui.dynamic_la_min_look_ahead_d_val.toPlainText())
        self.ros_class_srv.config_dynamic_la_pure_srv(
            look_ahead_v_gain,
            max_linear_speed,
            max_angular_acc,
            max_angular_dec,
            max_linear_acc,
            max_linear_dec,
            speed_pow,
            min_lookahead_d
        )

    def config_dynamic_pp_cb(self):
        print("config dynamic controller")
        max_linear_speed = float(self.ui.dynamic_max_linear_speed_val.toPlainText())
        max_angular_acc = float(self.ui.dynamic_max_angular_acc_val.toPlainText())
        max_angular_dec = float(self.ui.dynamic_max_angular_dec_val.toPlainText())
        max_linear_acc = float(self.ui.dynamic_max_linear_acc_val.toPlainText())
        max_linear_dec = float(self.ui.dynamic_max_angular_dec_val.toPlainText())
        look_ahead_dist = float(self.ui.dynamic_look_ahead_dis_val.toPlainText())
        self.ros_class_srv.config_dynamic_pure_srv(
            max_linear_speed,
            max_angular_acc,
            max_angular_dec,
            max_linear_acc,
            max_linear_dec,
            look_ahead_dist,
        )

    def draw_paths(self, image, poses, path_width, color, local=False):
        """Draws paths on the image."""
        for i in range(len(poses) - 1):
            pose1 = poses[i]
            pose2 = poses[i + 1]
            [pix_x1, pix_y1] = self.path_utils.UTM_to_pix(
                pose1.pose.position.y, pose1.pose.position.x
            )
            [pix_x2, pix_y2] = self.path_utils.UTM_to_pix(
                pose2.pose.position.y, pose2.pose.position.x
            )
            cv2.line(image, (pix_y1, pix_x1), (pix_y2, pix_x2), color, path_width)
            if int(i / 10) == self.ros_class_topics.look_ahead_idx and local:
                circle_radius = 10  # Radius of the circle
                circle_color = (0, 0, 255)  # Green color for the circle, in BGR format
                circle_thickness = -1  # Solid fill
                cv2.circle(
                    image,
                    (pix_y1, pix_x1),
                    circle_radius,
                    circle_color,
                    circle_thickness,
                )

    def draw_live_pos(self, image, live_pos, path_width, color):
        """Draws the live positions on the image."""
        for point in live_pos:
            [pix_x, pix_y] = self.path_utils.UTM_to_pix(point[0], point[1])
            image[
                pix_x - path_width : pix_x + path_width,
                pix_y - path_width : pix_y + path_width,
                :,
            ] = color

    def refresh_img(self):
        image = np.copy(self.map)
        path_width = int(self.path_width / self.zoom_scale)
        if path_width < 1:
            path_width = 5
        path_width = 5
        if self.ros_class_topics.emergency_call and self.handled_emergency:
            color = (0, 0, 255)  # Red color for nav path on emergency
        else:
            color = (0, 255, 0)  # Green color for nav path
        self.draw_paths(
            image, self.nav_utm_path.poses, path_width, color
        )  # Green color for nav path
        self.draw_paths(
            image, self.local_path_utm.poses, path_width, (255, 0, 0), local=True
        )  # Red color for local path
        self.draw_live_pos(
            image, self.live_pos, 2, (255, 0, 0)
        )  # Red color for live positions

        UGV_pix = self.get_UGV_pix()
        rover_pix = self.path_utils.UTM_to_pix(
            self.ros_class_topics.rover_x, self.ros_class_topics.rover_y
        )
        base_pix = self.path_utils.UTM_to_pix(
            self.ros_class_topics.base_x, self.ros_class_topics.base_y
        )
        if not (self.ros_class_topics.human_y == 0.0):
            human_pix = self.path_utils.UTM_to_pix(
                self.ros_class_topics.human_x, self.ros_class_topics.human_y
            )
            if self.ros_class_topics.is_emergency == True:
                cv2.circle(image, (human_pix[1], human_pix[0]), 20, (0, 0, 255), -1)
            else:
                cv2.circle(image, (human_pix[1], human_pix[0]), 20, (0, 255, 255), -1)

        if len(self.draw_path_pixels) > 0:
            # Draw the path
            for i in range(len(self.draw_path_pixels) - 1):
                cv2.line(
                    image,
                    self.draw_path_pixels[i],
                    self.draw_path_pixels[i + 1],
                    (0, 255, 255),
                    10,
                )  # Yellow path
        # Check if we're dynamically drawing or if the rectangle should be static
        if (
            self.drawing_rectangle
            and len(self.clicked_pix_x) == 1
            and not (self.select_poi_active)
        ):
            # Dynamic drawingwith current mouse position
            cv2.rectangle(
                image,
                (self.clicked_pix_x[0], self.clicked_pix_y[0]),
                (self.current_mouse_x, self.current_mouse_y),
                (0, 255, 0),
                8,
            )  # Green rectangle
        elif len(self.clicked_pix_x) == 2 and not (self.select_poi_active):
            # Static rectangle after mouse release
            cv2.rectangle(
                image,
                (self.clicked_pix_x[0], self.clicked_pix_y[0]),
                (self.clicked_pix_x[1], self.clicked_pix_y[1]),
                (0, 255, 0),
                8,
            )
        map_n_UGV = self.image_handler.print_UGV_pos(
            image,
            UGV_pix,
            rover_pix,
            base_pix,
            self.zoom_active,
            self.ros_class_topics.UGV_yaw,
        )
        if len(self.goal_pix) > 0:
            map_n_UGV[
                self.goal_pix[1] - 20 : self.goal_pix[1] + 20,
                self.goal_pix[0] - 20 : self.goal_pix[0] + 20,
                :,
            ] = [255, 123, 100]
        if (
            self.zoom_active
            and len(self.clicked_pix_x) >= 2
            and len(self.clicked_pix_y) >= 2
        ):
            start_x, end_x = sorted(self.clicked_pix_x)
            start_y, end_y = sorted(self.clicked_pix_y)
            map_n_UGV = self.image_handler.apply_zoom_with_aspect(
                map_n_UGV, start_x, start_y, end_x, end_y
            )

        height, width, channel = map_n_UGV.shape
        bytesPerLine = 3 * width
        qImg = QImage(map_n_UGV.data, width, height, bytesPerLine, QImage.Format_RGB888)
        pixmap01 = QPixmap.fromImage(qImg)
        self.ui.map_.setPixmap(pixmap01)
        self.ui.map_.update()
        # self.ui.map_.mousePressEvent = self.getPixel

    def closeEvent(self, event):
        self.quit_app()
        event.accept()  # Propagates the closing event after handling

    def quit_app(self):
        self.profiler.dump_stats(
            "init_profile_results.prof"
        )  # Save initialization profiling
        self.thread.quit()  # Instruct the thread to exit
        self.thread.wait()  # Wait for the thread to finish
        QApplication.quit()  # Close the application



def main(args=None):
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec())
