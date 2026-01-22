import sys
from numpy import atleast_1d
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import JointState

import numpy as np

from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QLineEdit, QHBoxLayout, QTabWidget
from PyQt6.QtCore import QThread, pyqtSignal, QObject

class GuiPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher_node')
        self.reset = self.create_publisher(Bool, '/reset', 10)
        self.set_arm_position = self.create_publisher(Float32MultiArray, '/update_position', 10)
        self.send_torque = self.create_publisher(Float32MultiArray, '/torque_input', 10)
        self.set_rw_speed = self.create_publisher(Float32MultiArray, '/rw_speed', 10)

        self.create_subscription(JointState, '/joint_states', self.update_joint_telem, 10)

        self.qt = None
        self.joint_state_qt_objects = None
    
    def initiliaze_qt(self,qt):
        self.qt = qt
    
    def set_joint_boxes(self,boxes):
        self.joint_state_qt_objects = boxes

    def update_joint_telem(self,msg):
        if self.joint_state_qt_objects is not None:
            for box, state in zip(self.joint_state_qt_objects, msg.position):
                box.setText(f"{round(np.degrees(state),2)}")



        

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

        # Layout
        self.layout = QVBoxLayout()
        self.container = QWidget()

        self.make_gui()

    def reset_arm(self):
        self.ros_node.reset.publish(Bool(data=True))
    
    def publish_float_array(self,text_boxes,publisher,convert=False):
        print(text_boxes)
        if not any([len(i.text())==0 for i in text_boxes]):
            joint_states = np.array([i.text() for i in text_boxes], dtype=np.float64)
            joint_states = np.deg2rad(joint_states) if convert else joint_states
            msg = Float32MultiArray()
            msg.data = joint_states.tolist()
            publisher.publish(msg)


    def make_gui(self):
        buttons = [
            ("Reset Arm", self.reset_arm),
            ("Set Arm Position", lambda x: None),
            ("Send Torque", lambda x: None),
            ("Set RW Speed", lambda x: None),
        ]

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


        ########### Reset joint state #############
        box = QVBoxLayout()
        arm_label = QLabel("Arm Position Inputs (Degrees):")
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
        arm_box = QHBoxLayout()
        torque_state_boxes = []
        for link in ["Tx","Ty","Tz"]:
            at1 = QLineEdit("0")
            torque_state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)
        torque_button = QPushButton("Publish Torque")
        torque_button.clicked.connect(lambda y, boxes=torque_state_boxes: self.publish_float_array(boxes,self.ros_node.send_torque))
        box.addWidget(torque_button)
        control_tab_layout.addLayout(box)
        ################################################

        ######### Reaction Wheel Speed Input ###############
        box = QVBoxLayout()
        arm_label = QLabel("Reaction Wheel Speed (rad/s):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        control_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        rw_state_boxes = []
        for link in ["RW1","RW2","RW3","RW4"]:
            at1 = QLineEdit("0")
            rw_state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)
        rw_button = QPushButton("Publish Reaction Wheel Speed")
        print(f'rw tabs: {rw_state_boxes}')
        rw_button.clicked.connect(lambda _, boxes=rw_state_boxes: self.publish_float_array(boxes,self.ros_node.set_rw_speed))
        box.addWidget(rw_button)
        control_tab_layout.addLayout(box)
        #################################################


        ########## Joint Telemetry ################
        box = QVBoxLayout()
        arm_label = QLabel("Joint State (deg):")
        arm_label.setStyleSheet("padding: 0px; margin: 0px;")
        box.addWidget(arm_label)

        telemetry_tab_layout.addWidget(arm_label)
        arm_box = QHBoxLayout()
        state_boxes = []
        for link in ["S", "L", "U", "R", "B", "T"]:
            at1 = QLineEdit("0")
            state_boxes.append(at1)
            at1.setPlaceholderText(link)
            at1.setFixedWidth(50)
            arm_box.addWidget(at1)

        box.addLayout(arm_box)
        self.ros_node.set_joint_boxes(state_boxes)
        telemetry_tab_layout.addLayout(box)
        ###################################################


        control_tab.setLayout(control_tab_layout)
        telemetry_tab.setLayout(telemetry_tab_layout)
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