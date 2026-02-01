# gui_gyro_visualizer.py
"""
STM32F429 Discovery Board - Gyroscope 3D Visualizer
Reads gyroscope data from UART (COM13) and displays real-time orientation
"""

import sys
import serial
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtWidgets import QOpenGLWidget
import re
import math

class STM32DiscoveryBoard(QOpenGLWidget):
    """3D OpenGL widget that renders STM32F429 Discovery board"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0  # Rotation around X-axis
        self.roll = 0.0   # Rotation around Y-axis
        self.yaw = 0.0    # Rotation around Z-axis
        
        # Camera control
        self.camera_distance = 10.0
        self.camera_angle = 20.0
        # ✅ Dead zone threshold (filtreleme için)
        self.dead_zone = 1.0  # DPS
        
    def initializeGL(self):
        """Initialize OpenGL settings"""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Light position
        glLightfv(GL_LIGHT0, GL_POSITION, [5.0, 10.0, 5.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.4, 0.4, 0.4, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])
        
        glClearColor(0.15, 0.15, 0.2, 1.0)
        
    def resizeGL(self, w, h):
        """Handle window resize"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        """Render the 3D scene"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Camera position - isometric view
        eye_x = self.camera_distance * math.sin(math.radians(self.camera_angle))
        eye_y = 6.0
        eye_z = self.camera_distance * math.cos(math.radians(self.camera_angle))
        
        gluLookAt(eye_x, eye_y, eye_z,   # Eye position
                  0, 0, 0,                # Look at point
                  0, 1, 0)                # Up vector
        
        # Draw ground grid first (before rotations)
        self.draw_ground_grid()
        
        # Apply rotations (gyroscope data)
        glRotatef(self.pitch, 1, 0, 0)  # Pitch (X-axis)
        glRotatef(self.roll, 0, 1, 0)   # Roll (Y-axis)
        glRotatef(self.yaw, 0, 0, 1)    # Yaw (Z-axis)
        
        # Draw STM32F429 Discovery board shape
        self.draw_stm32_board()
        
        # Draw local axes on board
        self.draw_board_axes()
        
    def draw_ground_grid(self):
        """Draw a ground grid to represent the table/floor"""
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        
        grid_size = 10
        grid_step = 1.0
        
        glBegin(GL_LINES)
        glColor3f(0.3, 0.3, 0.35)
        
        for i in range(-grid_size, grid_size + 1):
            # Lines parallel to X-axis
            glVertex3f(-grid_size * grid_step, -1.5, i * grid_step)
            glVertex3f(grid_size * grid_step, -1.5, i * grid_step)
            
            # Lines parallel to Z-axis
            glVertex3f(i * grid_step, -1.5, -grid_size * grid_step)
            glVertex3f(i * grid_step, -1.5, grid_size * grid_step)
        
        glEnd()
        
        # Draw world axes (fixed, not rotating with board)
        glLineWidth(3.0)
        glBegin(GL_LINES)
        
        # X-axis (Red)
        glColor3f(1.0, 0.2, 0.2)
        glVertex3f(0, -1.4, 0)
        glVertex3f(3, -1.4, 0)
        
        # Y-axis (Green)
        glColor3f(0.2, 1.0, 0.2)
        glVertex3f(0, -1.4, 0)
        glVertex3f(0, 1.6, 0)
        
        # Z-axis (Blue)
        glColor3f(0.2, 0.2, 1.0)
        glVertex3f(0, -1.4, 0)
        glVertex3f(0, -1.4, 3)
        
        glEnd()
        
        # Draw axis labels
        self.draw_text_3d(3.2, -1.4, 0, "X", (1.0, 0.2, 0.2))
        self.draw_text_3d(0, 1.8, 0, "Y", (0.2, 1.0, 0.2))
        self.draw_text_3d(0, -1.4, 3.2, "Z", (0.2, 0.2, 1.0))
        
        # Draw compass/orientation indicator
        self.draw_compass()
        
        glEnable(GL_LIGHTING)
        
    def draw_compass(self):
        """Draw a simple compass showing North direction"""
        glDisable(GL_LIGHTING)
        
        # Compass circle at corner
        compass_x = -7.0
        compass_z = -7.0
        compass_y = -1.3
        radius = 0.5
        
        # Draw circle
        glLineWidth(2.0)
        glBegin(GL_LINE_LOOP)
        glColor3f(0.7, 0.7, 0.8)
        for i in range(32):
            angle = 2 * math.pi * i / 32
            x = compass_x + radius * math.cos(angle)
            z = compass_z + radius * math.sin(angle)
            glVertex3f(x, compass_y, z)
        glEnd()
        
        # Draw North pointer
        glBegin(GL_TRIANGLES)
        glColor3f(1.0, 0.3, 0.3)
        glVertex3f(compass_x, compass_y, compass_z + radius * 0.8)
        glVertex3f(compass_x - 0.15, compass_y, compass_z)
        glVertex3f(compass_x + 0.15, compass_y, compass_z)
        glEnd()
        
        # Draw "N" label
        self.draw_text_3d(compass_x, compass_y, compass_z + radius * 1.2, "N", (1.0, 0.3, 0.3))
        
        glEnable(GL_LIGHTING)
        
    def draw_stm32_board(self):
      """Draw a 3D representation of STM32F429 Discovery board - Similar to real hardware"""
      # Real board dimensions (scaled for visibility)
      # Actual board: ~70mm x 118mm
      board_width = 2.8   # X direction (narrower)
      board_length = 4.7  # Z direction (longer)
      board_thickness = 0.12
      
      # Main PCB (blue like STM32 Discovery boards)
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.2, 0.2, 0.2, 1.0])
      glMaterialf(GL_FRONT, GL_SHININESS, 20)
      glColor3f(0.0, 0.3, 0.6)  # STM32 Discovery blue PCB
      self.draw_box(0, 0, 0, board_width, board_thickness, board_length)
      
      # LCD Screen (positioned at top/center, white when off)
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.8, 0.8, 0.8, 1.0])
      glMaterialf(GL_FRONT, GL_SHININESS, 80)
      glColor3f(0.85, 0.9, 0.95)  # Light blue/white screen
      glPushMatrix()
      glTranslatef(0, board_thickness + 0.02, 0.5)  # Positioned higher on board
      self.draw_box(0, 0, 0, 1.9, 0.04, 2.4)
      glPopMatrix()
      
      # LCD Frame/Connector (black frame around screen)
      glColor3f(0.05, 0.05, 0.05)
      glPushMatrix()
      glTranslatef(0, board_thickness + 0.005, 0.5)
      # Top frame
      self.draw_box(0, 0, 1.25, 2.0, 0.02, 0.1)
      # Bottom frame  
      self.draw_box(0, 0, -0.25, 2.0, 0.02, 0.1)
      # Left frame
      self.draw_box(-1.0, 0, 0.5, 0.1, 0.02, 2.5)
      # Right frame
      self.draw_box(1.0, 0, 0.5, 0.1, 0.02, 2.5)
      glPopMatrix()
      
      # Main MCU chip (STM32F429ZIT6 - large LQFP176 package)
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.3, 0.3, 0.3, 1.0])
      glColor3f(0.15, 0.15, 0.15)
      glPushMatrix()
      glTranslatef(0.3, board_thickness + 0.12, -0.8)
      self.draw_box(0, 0, 0, 1.0, 0.25, 1.0)
      glPopMatrix()
      
      # ST logo area on MCU (lighter square)
      glColor3f(0.25, 0.25, 0.25)
      glPushMatrix()
      glTranslatef(0.3, board_thickness + 0.25, -0.8)
      self.draw_box(0, 0, 0, 0.6, 0.01, 0.6)
      glPopMatrix()
      
      # Gyroscope sensor (I3G4250D - small package, positioned near top-left)
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.4, 0.4, 0.4, 1.0])
      glMaterialf(GL_FRONT, GL_SHININESS, 40)
      glColor3f(0.2, 0.2, 0.2)
      glPushMatrix()
      glTranslatef(-0.9, board_thickness + 0.06, 1.5)  # Top-left position
      self.draw_box(0, 0, 0, 0.25, 0.12, 0.25)
      glPopMatrix()
      
      # Gyro marker (red dot)
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.5, 0.0, 0.0, 1.0])
      glColor3f(1.0, 0.2, 0.2)
      glPushMatrix()
      glTranslatef(-0.9, board_thickness + 0.13, 1.5)
      self.draw_box(0, 0, 0, 0.1, 0.02, 0.1)
      glPopMatrix()
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.0, 0.0, 0.0, 1.0])
      
      # SDRAM chip (bottom right)
      glColor3f(0.1, 0.1, 0.1)
      glPushMatrix()
      glTranslatef(0.7, board_thickness + 0.08, -1.6)
      self.draw_box(0, 0, 0, 0.7, 0.16, 0.5)
      glPopMatrix()
      
      # USB Mini connector (top edge, silver)
      glColor3f(0.75, 0.75, 0.75)
      glPushMatrix()
      glTranslatef(-0.7, board_thickness + 0.08, board_length/2 + 0.15)
      self.draw_box(0, 0, 0, 0.4, 0.16, 0.35)
      glPopMatrix()
      
      # USB Micro OTG connector (side edge)
      glColor3f(0.7, 0.7, 0.7)
      glPushMatrix()
      glTranslatef(board_width/2 + 0.1, board_thickness + 0.06, 0.8)
      self.draw_box(0, 0, 0, 0.2, 0.12, 0.35)
      glPopMatrix()
      
      # Reset button (black, near top)
      glColor3f(0.1, 0.1, 0.15)
      glPushMatrix()
      glTranslatef(0.9, board_thickness + 0.08, 1.8)
      self.draw_box(0, 0, 0, 0.25, 0.16, 0.25)
      glPopMatrix()
      
      # User button (blue)
      glColor3f(0.1, 0.3, 0.7)
      glPushMatrix()
      glTranslatef(-0.5, board_thickness + 0.08, -2.0)
      self.draw_box(0, 0, 0, 0.25, 0.16, 0.25)
      glPopMatrix()
      
      # Pin headers - morpho connectors (gold/yellow on both sides)
      glColor3f(0.85, 0.7, 0.2)
      pin_count = 10
      pin_spacing = 0.35
      pin_start_z = -1.5
      
      for side in [-1, 1]:  # Both sides
          for i in range(pin_count):
              glPushMatrix()
              z_pos = pin_start_z + (i * pin_spacing)
              glTranslatef(side * (board_width/2 + 0.08), board_thickness + 0.08, z_pos)
              self.draw_box(0, 0, 0, 0.15, 0.16, 0.12)
              glPopMatrix()
      
      # LEDs (positioned near top)
      # LD3 - Red Power LED
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.4, 0.0, 0.0, 1.0])
      glColor3f(0.9, 0.1, 0.1)
      glPushMatrix()
      glTranslatef(-1.0, board_thickness + 0.06, -1.8)
      self.draw_box(0, 0, 0, 0.12, 0.12, 0.08)
      glPopMatrix()
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.0, 0.0, 0.0, 1.0])
      
      # LD4 - Green User LED
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.0, 0.4, 0.0, 1.0])
      glColor3f(0.2, 1.0, 0.2)
      glPushMatrix()
      glTranslatef(-0.7, board_thickness + 0.06, -1.8)
      self.draw_box(0, 0, 0, 0.12, 0.12, 0.08)
      glPopMatrix()
      glMaterialfv(GL_FRONT, GL_EMISSION, [0.0, 0.0, 0.0, 1.0])
      
      # Oscillator (silver metal can)
      glColor3f(0.7, 0.7, 0.75)
      glPushMatrix()
      glTranslatef(0.5, board_thickness + 0.05, -0.3)
      self.draw_box(0, 0, 0, 0.2, 0.1, 0.15)
      glPopMatrix()
      
      # Various small capacitors/resistors (black SMD components)
      glColor3f(0.15, 0.12, 0.1)
      component_positions = [
          (-0.3, -1.2), (0.6, -1.3), (-0.8, -0.5),
          (0.9, 0.2), (-0.4, 1.0), (0.8, 1.3)
      ]
      for x, z in component_positions:
          glPushMatrix()
          glTranslatef(x, board_thickness + 0.03, z)
          self.draw_box(0, 0, 0, 0.08, 0.06, 0.12)
          glPopMatrix()
      
      # Board mounting holes (at corners)
      glDisable(GL_LIGHTING)
      glColor3f(0.3, 0.3, 0.3)
      hole_offset_x = board_width/2 - 0.15
      hole_offset_z = board_length/2 - 0.15
      for x_sign in [-1, 1]:
          for z_sign in [-1, 1]:
              glPushMatrix()
              glTranslatef(x_sign * hole_offset_x, 0, z_sign * hole_offset_z)
              # Draw hole as dark circle (simplified as small box)
              self.draw_box(0, 0, 0, 0.1, board_thickness * 1.2, 0.1)
              glPopMatrix()
      glEnable(GL_LIGHTING)
      
      # Reset material properties
      glMaterialfv(GL_FRONT, GL_SPECULAR, [0.0, 0.0, 0.0, 1.0])
      glMaterialf(GL_FRONT, GL_SHININESS, 0)
    def draw_board_axes(self):
        """Draw local coordinate axes on the board"""
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        
        axis_length = 1.5
        
        glBegin(GL_LINES)
        
        # X-axis (Red) - Pitch
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(axis_length, 0, 0)
        
        # Y-axis (Green) - Roll
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, axis_length, 0)
        
        # Z-axis (Blue) - Yaw
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, axis_length)
        
        glEnd()
        
        # Arrow heads for better visibility
        self.draw_arrow_head(axis_length, 0, 0, (1,0,0), (1.0, 0.0, 0.0))
        self.draw_arrow_head(0, axis_length, 0, (0,1,0), (0.0, 1.0, 0.0))
        self.draw_arrow_head(0, 0, axis_length, (0,0,1), (0.0, 0.0, 1.0))
        
        glEnable(GL_LIGHTING)
        
    def draw_arrow_head(self, x, y, z, direction, color):
        """Draw an arrow head at the end of an axis"""
        glColor3f(*color)
        size = 0.15
        
        glPushMatrix()
        glTranslatef(x, y, z)
        
        # Simple cone as arrow head
        if direction[0] != 0:  # X-axis
            glRotatef(90, 0, 1, 0)
        elif direction[2] != 0:  # Z-axis
            glRotatef(-90, 1, 0, 0)
        
        quadric = gluNewQuadric()
        gluCylinder(quadric, size, 0, size*2, 8, 1)
        gluDeleteQuadric(quadric)
        
        glPopMatrix()
        
    def draw_text_3d(self, x, y, z, text, color):
        """Draw simple 3D text (placeholder - draws small cubes)"""
        # This is a simple placeholder - proper text rendering needs GLUT or texture mapping
        glDisable(GL_LIGHTING)
        glColor3f(*color)
        glPointSize(5.0)
        glBegin(GL_POINTS)
        glVertex3f(x, y, z)
        glEnd()
        glEnable(GL_LIGHTING)
        
    def draw_box(self, x, y, z, width, height, depth):
        """Draw a 3D box centered at (x,y,z)"""
        w, h, d = width/2, height/2, depth/2
        
        glBegin(GL_QUADS)
        
        # Front face
        glNormal3f(0, 0, 1)
        glVertex3f(x-w, y-h, z+d)
        glVertex3f(x+w, y-h, z+d)
        glVertex3f(x+w, y+h, z+d)
        glVertex3f(x-w, y+h, z+d)
        
        # Back face
        glNormal3f(0, 0, -1)
        glVertex3f(x-w, y-h, z-d)
        glVertex3f(x-w, y+h, z-d)
        glVertex3f(x+w, y+h, z-d)
        glVertex3f(x+w, y-h, z-d)
        
        # Top face
        glNormal3f(0, 1, 0)
        glVertex3f(x-w, y+h, z-d)
        glVertex3f(x-w, y+h, z+d)
        glVertex3f(x+w, y+h, z+d)
        glVertex3f(x+w, y+h, z-d)
        
        # Bottom face
        glNormal3f(0, -1, 0)
        glVertex3f(x-w, y-h, z-d)
        glVertex3f(x+w, y-h, z-d)
        glVertex3f(x+w, y-h, z+d)
        glVertex3f(x-w, y-h, z+d)
        
        # Right face
        glNormal3f(1, 0, 0)
        glVertex3f(x+w, y-h, z-d)
        glVertex3f(x+w, y+h, z-d)
        glVertex3f(x+w, y+h, z+d)
        glVertex3f(x+w, y-h, z+d)
        
        # Left face
        glNormal3f(-1, 0, 0)
        glVertex3f(x-w, y-h, z-d)
        glVertex3f(x-w, y-h, z+d)
        glVertex3f(x-w, y+h, z+d)
        glVertex3f(x-w, y+h, z-d)
        
        glEnd()
        
    def update_orientation(self, gyro_x, gyro_y, gyro_z, dt):
      """Update orientation based on gyroscope angular velocity with dead zone filtering"""
      # Apply dead zone filter - ignore small movements (drift compensation)
      if abs(gyro_x) < self.dead_zone:
          gyro_x = 0.0
      if abs(gyro_y) < self.dead_zone:
          gyro_y = 0.0
      if abs(gyro_z) < self.dead_zone:
          gyro_z = 0.0
      
      # ✅ DÜZELTME: Eksen yönlerini ve haritalamalarını düzelt
      # X ekseni: yönü ters çevir
      self.pitch += (-gyro_x) * dt
      

      self.yaw += gyro_y * dt      # Y sensör -> Yaw (Z rotasyon)
      self.roll += gyro_z * dt     # Z sensör -> Roll (Y rotasyon)
      
      # Keep angles in reasonable range
      self.pitch = self.pitch % 360
      self.roll = self.roll % 360
      self.yaw = self.yaw % 360
      
      self.update()
        
    def reset_orientation(self):
        """Reset orientation to initial state"""
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.update()
        
    def set_camera_angle(self, angle):
        """Set camera rotation angle"""
        self.camera_angle = angle
        self.update()


class GyroscopeVisualizer(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_data)
        
        # Gyro data
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        # Timing for integration
        import time
        self.last_update_time = time.time()
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("STM32F429 Discovery - Gyroscope 3D Visualizer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Title
        title = QLabel("🔄 STM32F429 Discovery Board - Real-time 3D Orientation")
        title.setFont(QFont("Arial", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #4da6ff; padding: 10px;")
        main_layout.addWidget(title)
        
        # 3D View
        self.gl_widget = STM32DiscoveryBoard()
        main_layout.addWidget(self.gl_widget, stretch=5)
        
        # Camera control
        camera_layout = QHBoxLayout()
        camera_label = QLabel("📷 Camera Angle:")
        camera_label.setFont(QFont("Arial", 10))
        camera_layout.addWidget(camera_label)
        
        self.camera_slider = QSlider(Qt.Horizontal)
        self.camera_slider.setMinimum(0)
        self.camera_slider.setMaximum(360)
        self.camera_slider.setValue(20)
        self.camera_slider.valueChanged.connect(self.on_camera_change)
        camera_layout.addWidget(self.camera_slider)
        
        main_layout.addLayout(camera_layout)
        
        # Data display panel
        data_layout = QHBoxLayout()
        
        self.label_x = QLabel("⬌ X: 0.00 DPS")
        self.label_y = QLabel("⬍ Y: 0.00 DPS")
        self.label_z = QLabel("↻ Z: 0.00 DPS")
        self.label_status = QLabel("⚫ Disconnected")
        
        for label in [self.label_x, self.label_y, self.label_z]:
            label.setFont(QFont("Courier New", 12, QFont.Bold))
            label.setStyleSheet("padding: 10px; background-color: #1a1a2e; color: #0cf574; border-radius: 5px; border: 2px solid #16213e;")
            data_layout.addWidget(label)
        
        self.label_status.setFont(QFont("Courier New", 12, QFont.Bold))
        self.label_status.setStyleSheet("padding: 10px; background-color: #1a1a2e; color: #ff6b6b; border-radius: 5px; border: 2px solid #16213e;")
        data_layout.addWidget(self.label_status)
        
        main_layout.addLayout(data_layout)
        
        # Orientation display
        orient_layout = QHBoxLayout()
        self.label_pitch = QLabel("Pitch: 0.0°")
        self.label_roll = QLabel("Roll: 0.0°")
        self.label_yaw = QLabel("Yaw: 0.0°")
        
        for label in [self.label_pitch, self.label_roll, self.label_yaw]:
            label.setFont(QFont("Arial", 11))
            label.setStyleSheet("padding: 5px; background-color: #2b2b3c; color: #ffd93d; border-radius: 3px;")
            orient_layout.addWidget(label)
        
        main_layout.addLayout(orient_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.btn_connect = QPushButton("🔌 Connect COM13")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.btn_connect.setStyleSheet("""
            font-size: 14px; 
            padding: 12px; 
            background-color: #0e639c; 
            color: white; 
            border-radius: 6px;
            font-weight: bold;
        """)
        
        self.btn_reset = QPushButton("🔄 Reset Orientation")
        self.btn_reset.clicked.connect(self.reset_orientation)
        self.btn_reset.setStyleSheet("""
            font-size: 14px; 
            padding: 12px; 
            background-color: #6c5ce7; 
            color: white; 
            border-radius: 6px;
            font-weight: bold;
        """)
        
        button_layout.addWidget(self.btn_connect)
        button_layout.addWidget(self.btn_reset)
        
        main_layout.addLayout(button_layout)
        
        # Status bar with info
        self.statusBar().showMessage("ℹ️  Ready to connect to COM13 (115200 baud) | Move your board to see real-time orientation")
        self.statusBar().setStyleSheet("background-color: #2b2b3c; color: #aaaaaa; font-size: 11px; padding: 5px;")
        
    def on_camera_change(self, value):
        """Handle camera angle slider change"""
        self.gl_widget.set_camera_angle(value)
        
    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
            
    def connect_serial(self):
        """Connect to COM13"""
        try:
            print("Attempting to connect to COM13...")
            self.serial_port = serial.Serial(
                port='COM13',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            print(f"Serial port opened: {self.serial_port.is_open}")
            
            # ✅ AUTO RESET ON CONNECT
            self.reset_orientation()
            
            self.timer.start(50)  # Read every 50ms
            self.btn_connect.setText("🔌 Disconnect")
            self.btn_connect.setStyleSheet("""
                font-size: 14px; 
                padding: 12px; 
                background-color: #d63031; 
                color: white; 
                border-radius: 6px;
                font-weight: bold;
            """)
            self.label_status.setText("🟢 Connected")
            self.label_status.setStyleSheet("padding: 10px; background-color: #1a1a2e; color: #0cf574; border-radius: 5px; border: 2px solid #16213e;")
            self.statusBar().showMessage("✅ Connected to COM13 - Receiving gyroscope data")
            
            import time
            self.last_update_time = time.time()
            
            print("Connection successful!")
            
        except Exception as e:
            print(f"Connection error: {e}")
            self.statusBar().showMessage(f"❌ Connection failed: {str(e)}")
            self.label_status.setText("🔴 Error")
            self.label_status.setStyleSheet("padding: 10px; background-color: #1a1a2e; color: #ff6b6b; border-radius: 5px; border: 2px solid #16213e;")
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.timer.stop()
        if self.serial_port:
            self.serial_port.close()
        self.btn_connect.setText("🔌 Connect COM13")
        self.btn_connect.setStyleSheet("""
            font-size: 14px; 
            padding: 12px; 
            background-color: #0e639c; 
            color: white; 
            border-radius: 6px;
            font-weight: bold;
        """)
        self.label_status.setText("⚫ Disconnected")
        self.label_status.setStyleSheet("padding: 10px; background-color: #1a1a2e; color: #ff9900; border-radius: 5px; border: 2px solid #16213e;")
        self.statusBar().showMessage("⚠️  Disconnected from COM13")
        
    def read_serial_data(self):
        """Read and parse gyroscope data from serial port"""
        if not self.serial_port or not self.serial_port.is_open:
            return
            
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse format: "X:1234  Y:-567  Z:890 MDPS"
                match = re.search(r'X:\s*([-\d]+)\s+Y:\s*([-\d]+)\s+Z:\s*([-\d]+)\s+MDPS', line)
                
                if match:
                    # mdps'yi dps'ye çevir (1000'e böl)
                    self.gyro_x = float(match.group(1)) / 1000.0
                    self.gyro_y = float(match.group(2)) / 1000.0
                    self.gyro_z = float(match.group(3)) / 1000.0
                    
                    # Update labels
                    self.label_x.setText(f"⬌ X: {self.gyro_x:+7.2f} DPS")
                    self.label_y.setText(f"⬍ Y: {self.gyro_y:+7.2f} DPS")
                    self.label_z.setText(f"↻ Z: {self.gyro_z:+7.2f} DPS")
                    
                    # Update orientation labels
                    self.label_pitch.setText(f"Pitch: {self.gl_widget.pitch:.1f}°")
                    self.label_roll.setText(f"Roll: {self.gl_widget.roll:.1f}°")
                    self.label_yaw.setText(f"Yaw: {self.gl_widget.yaw:.1f}°")
                    
                    # Calculate time delta for integration
                    import time
                    current_time = time.time()
                    dt = current_time - self.last_update_time
                    self.last_update_time = current_time
                    
                    # Update 3D visualization
                    self.gl_widget.update_orientation(self.gyro_x, self.gyro_y, self.gyro_z, dt)
                    
        except Exception as e:
            print(f"Exception in read_serial_data: {e}")
            self.statusBar().showMessage(f"❌ Read error: {str(e)}")

    def reset_orientation(self):
        """Reset the 3D orientation"""
        self.gl_widget.reset_orientation()
        self.label_pitch.setText("Pitch: 0.0°")
        self.label_roll.setText("Roll: 0.0°")
        self.label_yaw.setText("Yaw: 0.0°")
        self.statusBar().showMessage("🔄 Orientation reset to default position")
        
    def closeEvent(self, event):
        """Handle window close"""
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Modern dark theme
    app.setStyleSheet("""
        QMainWindow {
            background-color: #0f0f1e;
        }
        QWidget {
            background-color: #0f0f1e;
        }
        QLabel {
            color: #e0e0e0;
        }
        QPushButton:hover {
            opacity: 0.8;
        }
        QSlider::groove:horizontal {
            background: #2b2b3c;
            height: 8px;
            border-radius: 4px;
        }
        QSlider::handle:horizontal {
            background: #4da6ff;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }
    """)
    
    window = GyroscopeVisualizer()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()