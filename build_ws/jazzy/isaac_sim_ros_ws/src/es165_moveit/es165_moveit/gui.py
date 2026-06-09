import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Float32MultiArray, Int8, String
from sensor_msgs.msg import JointState

import sys
import numpy as np
from pathlib import Path
import os

from PyQt6.QtWidgets import QApplication, QComboBox, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QLineEdit, QHBoxLayout, QTabWidget
from PyQt6.QtCore import QThread, pyqtSignal, QObject

class GuiPublisher(Node):
    """
    Creates the GUI for the arm controller.
    This code is NOT efficient, but it is easy to understand so don't judge it
    """
    def __init__(self):
        super().__init__('gui_publisher_node')
        self.reset = self.create_publisher(Bool, '/reset', 10)
        self.set_arm_position = self.create_publisher(Float32MultiArray, '/update_position', 10)
        self.send_torque = self.create_publisher(Float32MultiArray, '/torque_input', 10)
        self.set_rw_speed = self.create_publisher(Float32MultiArray, '/rw_speed', 10)
        self.reset_speed = self.create_publisher(Bool, '/reset_speed', 10)
        self.torque_profile_pub = self.create_publisher(String, "/profile_path", 10)
        self.torque_start_profile = self.create_publisher(Bool, "/start_profile",10)
        self.torque_stop_profile_pub = self.create_publisher(Bool, "/stop_profile",10)

        self.create_subscription(JointState, '/joint_states', self.update_joint_telem, 10)
        self.create_subscription(Float32MultiArray, '/rw_speed', self.update_rw_telem, 10)
        self.create_subscription(Float32MultiArray, '/torque_input', self.update_torque_telem, 10)
        self.create_subscription(Float32MultiArray, '/ee_pose', self.update_ee_pose, 10)
        self.create_subscription(Int8, '/servo_node/status', self.update_servo_status, 10)
        self.create_subscription(Bool, "/profile_loaded", self.profile_loaded, 10)

        self.status_codes = \
        {
            -1: "INVALID",
            0: "NO WARNING",
            1: "DECELERATE FOR APPROACHING SINGULARITY",
            2: "HALT FOR SINGULARITY",
            3: "DECELERATE FOR COLLISION",
            4: "HALT FOR COLLISION",
            5: "JOINT BOUND",
            6: "DECELERATE FOR LEAVING SINGULARITY"
        }


        self.qt = None
        self.joint_state_qt_objects = None
        self.rw_state_qt_objects = None
        self.torque_state_qt_objects = None
        self.ee_pose_state_qt_objects = None
        self.servo_status_box = None
        self.load_box = None

    def initialize_qt(self,qt):
        self.qt = qt
    
    def profile_loaded(self,msg):
        self.qt.loaded=msg.data
        if self.load_box is not None:
            self.load_box.setStyleSheet("color: green;")

    def set_joint_boxes(self,boxes):
        self.joint_state_qt_objects = boxes
    
    def set_rw_boxes(self,boxes):
        self.rw_state_qt_objects = boxes

    def set_torque_boxes(self,boxes):
        self.torque_state_qt_objects = boxes
    
    def set_ee_pose_boxes(self,boxes):
        self.ee_pose_state_qt_objects = boxes
    
    def set_servo_status_box(self,box,lookup_box):
        self.servo_status_box = (box,lookup_box)

    def stop_torque_profile(self):
        self.torque_stop_profile_pub.publish(Bool(data=True))

    def update_ee_pose(self,msg):
        if self.ee_pose_state_qt_objects is not None:
            for box, state in zip(self.ee_pose_state_qt_objects, msg.data):
                box.setText(f"{round(state,2)}")

    def update_joint_telem(self,msg):
        if self.joint_state_qt_objects is not None:
            for box, state in zip(self.joint_state_qt_objects, msg.position):
                box.setText(f"{round(np.degrees(state),2)}")


    def update_rw_telem(self,msg):
        if self.rw_state_qt_objects is not None:
            for box, state in zip(self.rw_state_qt_objects, msg.data):
                box.setText(f"{round(state,2)}")
        
    def update_torque_telem(self,msg):
        if self.torque_state_qt_objects is not None:
            for box, state in zip(self.torque_state_qt_objects, msg.data):
                box.setText(f"{round(state,2)}")

    def update_servo_status(self,msg):
        if self.servo_status_box is not None:
            box,lookup = self.servo_status_box
            box.setText(f"{msg.data}")
            lookup.setText(self.status_codes[msg.data])

# Threads the ros node within Qt to not block GUI function
class RosWorker(QObject):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        # This keeps the ROS 2 node alive in the background
        rclpy.spin(self.node)


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        self.setWindowTitle("Arm Controller GUI")
        self.setMinimumSize(600, 600)

        # Tracking
        self.loaded = False

        # Layout
        self.layout = QVBoxLayout()
        self.container = QWidget()

        self.ros_node.initialize_qt(self)

        self.make_gui()

    def reset_arm(self):
        self.ros_node.reset.publish(Bool(data=True))
    
    def reset_speed(self):
        self.ros_node.reset_speed.publish(Bool(data=True))
    
    def update_text_color(self,box):
        box.setStyleSheet("color: white;")

    def load_torque_profile(self,box):
        self.loaded = False
        box.setStyleSheet("color: red;")
        self.ros_node.load_box = box
        path = box.currentText()
        self.ros_node.torque_profile_pub.publish(String(data=path))
    
    def send_torque_profile(self):
        self.ros_node.torque_start_profile.publish(Bool(data=True))

    def publish_float_array(self,text_boxes,publisher,convert=False):
        if not any([len(i.text())==0 for i in text_boxes]):
            joint_states = np.array([i.text() for i in text_boxes], dtype=np.float64)
            joint_states = np.deg2rad(joint_states) if convert else joint_states
            msg = Float32MultiArray()
            msg.data = joint_states.tolist()
            publisher.publish(msg)

    def reset_boxes(self,text_boxes):
        for box in text_boxes:
            box.setText("0")

    def make_gui(self):
        tabs = QTabWidget()
        control_tab = QWidget()
        control_tab_layout = QVBoxLayout()

        telemetry_tab = QWidget()
        telemetry_tab_layout = QVBoxLayout()

        ###### Reset Arm #######################
        reset_button = QPushButton("Reset Arm")
        reset_button.clicked.connect(self.reset_arm)
        control_tab_layout.addWidget(reset_button)
        ##########################################

        ###### Reset Speed #######################
        reset_speed_button = QPushButton("Reset RW Speed")
        reset_speed_button.clicked.connect(self.reset_speed)
        control_tab_layout.addWidget(reset_speed_button)
        ##########################################


        ########### Reset joint state #############
        box = QVBoxLayout()
        arm_label = QLabel("Arm Position Inputs (deg):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        control_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        arm_state_boxes = []
        for link in ["S", "L", "U", "R", "B", "T"]:
            at1 = QLineEdit("0")
            arm_state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)
        reset_button = QPushButton("Reset")
        reset_button.setFixedWidth(50)
        reset_button.clicked.connect(lambda x: self.reset_boxes(arm_state_boxes))
        arm_box.addWidget(reset_button)
        box.addLayout(arm_box)
        joint_button = QPushButton("Set Joint State")
        joint_button.clicked.connect(lambda x, boxes=arm_state_boxes: self.publish_float_array(boxes,self.ros_node.set_arm_position,True))
        box.addWidget(joint_button)
        control_tab_layout.addLayout(box)
        #####################################

        ######### Torque Input ###############
        box = QVBoxLayout()
        arm_label = QLabel("Torque Input (Nm):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        control_tab_layout.addWidget(arm_label)
        for i in range(3):
            arm_box = QHBoxLayout()
            torque_state_boxes = []
            for link in ["Tx","Ty","Tz"]:
                at1 = QLineEdit("0")
                torque_state_boxes.append(at1)
                at1.setPlaceholderText(link)
                at1.setFixedWidth(50)
                arm_box.addWidget(at1)
            torque_button = QPushButton("Send")
            torque_button.setFixedWidth(50)
            # Create a copy of the list to avoid closure issues
            boxes_copy = list(torque_state_boxes)
            torque_button.clicked.connect(lambda y, boxes=boxes_copy: self.publish_float_array(boxes,self.ros_node.send_torque))
            arm_box.addWidget(torque_button)
            reset_button = QPushButton("Reset")
            reset_button.setFixedWidth(50)
            # Create a copy for the reset button as well
            reset_boxes_copy = list(torque_state_boxes)
            reset_button.clicked.connect(lambda x, boxes=reset_boxes_copy: self.reset_boxes(boxes))
            arm_box.addWidget(reset_button)
            box.addLayout(arm_box)
        control_tab_layout.addLayout(box)
        ################################################

        ######### Reaction Wheel Speed Input ###############
        box = QVBoxLayout()
        arm_label = QLabel("Reaction Wheel Speed (rad/s):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        control_tab_layout.addWidget(arm_label)
        for i in range(3):
            arm_box = QHBoxLayout()
            rw_state_boxes = []
            for link in ["RW1","RW2","RW3","RW4"]:
                at1 = QLineEdit("0")
                rw_state_boxes.append(at1)
                at1.setPlaceholderText(link)
                at1.setFixedWidth(50)
                arm_box.addWidget(at1)
            rw_button = QPushButton("Send")
            rw_button.setFixedWidth(50)
            # Create a copy of the list to avoid closure issues
            boxes_copy = list(rw_state_boxes)
            rw_button.clicked.connect(lambda y, boxes=boxes_copy: self.publish_float_array(boxes,self.ros_node.set_rw_speed))
            arm_box.addWidget(rw_button)
            reset_button = QPushButton("Reset")
            reset_button.setFixedWidth(50)
            # Create a copy for the reset button as well
            reset_boxes_copy = list(rw_state_boxes)
            reset_button.clicked.connect(lambda x, boxes=reset_boxes_copy: self.reset_boxes(boxes))
            arm_box.addWidget(reset_button)
            box.addLayout(arm_box)
        control_tab_layout.addLayout(box)
        #################################################

        ######### Torque Profile Starter ################
        container = QHBoxLayout()
        arm_label = QLabel("Load Torque Profile:")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        container.addWidget(arm_label)

        # input_ = QLineEdit()
        # input_.setPlaceholderText("Input Profile Path")
        # input_.setFixedWidth(300)
        # container.addWidget(input_)
        dropdown = QComboBox()
        share_dir = get_package_share_directory("es165_moveit")
        config_path = Path(os.path.join(share_dir,"config/"))
        for profile in config_path.glob("*.profile"):
            dropdown.addItem(str(profile).split("/")[-1])
        dropdown.currentTextChanged.connect(lambda goobermcgee: self.update_text_color(dropdown))
        container.addWidget(dropdown)

        load_button = QPushButton("Load")
        load_button.setFixedWidth(50)
        load_button.clicked.connect(lambda goober: self.load_torque_profile(dropdown))
        container.addWidget(load_button)

        send_button = QPushButton("Start")
        send_button.setFixedWidth(50)
        send_button.clicked.connect(self.send_torque_profile)
        container.addWidget(send_button)
        control_tab_layout.addLayout(container)

        stop_button = QPushButton("Stop")
        stop_button.setFixedWidth(50)
        stop_button.clicked.connect(self.ros_node.stop_torque_profile)
        container.addWidget(stop_button)
        control_tab_layout.addLayout(container)

        ########## Joint Telemetry ################
        box = QVBoxLayout()
        
        arm_label = QLabel("Joint State (deg):")
        # arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        for link in ["S", "L", "U", "R", "B", "T"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)
        box.addStretch()
        self.ros_node.set_joint_boxes(state_boxes)
        telemetry_tab_layout.addLayout(box)
        ###################################################

        ########## RW Telemetry ################
        arm_label = QLabel("RW Speed (rad/s):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        for link in ["RW1", "RW2", "RW3", "RW4"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)

        # box.setSpacing(0)
        # box.setContentsMargins(0, 0, 0, 0)
        box.addStretch()
        self.ros_node.set_rw_boxes(state_boxes)
        telemetry_tab_layout.addLayout(box)
        ###################################################

        ########## Torque Telemetry ################
        arm_label = QLabel("Torque (Nm):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        for link in ["Tx", "Ty", "Tz"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)

        # box.setSpacing(0)
        # box.setContentsMargins(0, 0, 0, 0)
        box.addStretch()
        self.ros_node.set_torque_boxes(state_boxes)
        telemetry_tab_layout.addLayout(box)
        ###################################################

        ########## EE Pose Telemetry ################
        arm_label = QLabel("EE Pose (m, rad, m/s, rad/s):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        label = QLabel("XYZ, RPY:")
        arm_box.addWidget(label)
        for link in ["x", "y", "z", "rx", "ry", "rz"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box2 = QHBoxLayout()
        label = QLabel("VX,VY,VZ,WX,WY,WZ:")
        box2.addWidget(label)
        for link in ["vx", "vy", "vz","wx", "wy", "wz"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            box2.addWidget(at1)

        box.addLayout(arm_box)
        box.addLayout(box2)
        box.addStretch()
        self.ros_node.set_ee_pose_boxes(state_boxes)
        telemetry_tab_layout.addLayout(box)
        ###################################################

        ########## Servo Status ################
        nb = QHBoxLayout()
        arm_label = QLabel("Servo Status:")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        nb.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        for link in ["Status"]:
            at1 = QLineEdit("0")
            at1.setReadOnly(True)
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        nb.addLayout(arm_box)
        status_lookup = QLabel("")
        # status_lookup.setReadOnly(True)
        status_lookup.setFixedWidth(200)
        nb.addWidget(status_lookup)
        # box.setSpacing(0)
        # box.setContentsMargins(0, 0, 0, 0)
        nb.addStretch()
        self.ros_node.set_servo_status_box(state_boxes[0],status_lookup)
        telemetry_tab_layout.addLayout(nb)
        ###################################################

        control_tab.setLayout(control_tab_layout)
        control_tab_layout.addStretch()
        telemetry_tab.setLayout(telemetry_tab_layout)
        telemetry_tab_layout.addStretch()
        tabs.addTab(control_tab, "Control")
        tabs.addTab(telemetry_tab, "Telemetry")
        self.layout.addWidget(tabs)
        self.container.setLayout(self.layout)
        self.setCentralWidget(self.container)

def main():
    rclpy.init()
    
    # Create the Node
    node = GuiPublisher()

    # Start PyQt6
    app = QApplication(sys.argv)
    
    # Run the node in a background thread as to not interfere with the GUI
    try:
        ros_thread = QThread()
        worker = RosWorker(node)
        worker.moveToThread(ros_thread)
        ros_thread.started.connect(worker.run)
        ros_thread.start()

        window = MainWindow(node)
        window.show()
    except KeyboardInterrupt:
        pass

    # Close app
    exit_code = app.exec()
    # Shutdown ROS 2 when GUI closes
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.quit()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()