import sys
import math
import json
import re, math, vtk
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QColorDialog, QLabel, QComboBox, QGroupBox,
    QFormLayout, QDoubleSpinBox, QFrame, QMenu, QFileDialog,
    QInputDialog
)

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QCursor
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from optiland.fileio import zemax_handler
from optiland.fileio.zemax_handler import load_zemax_file as optiland_load_zemax_file

###############################################################################
# Custom Interactor Style
#
# This interactor style supports:
# - Left-click on an interactive object: select/unselect and (if dragging) move it.
# - Left-click on empty space: pan the camera.
# - Left-click near the center of the orientation marker (the intersection of the X, Y, Z arrows):
#   reset the camera to the original view.
# - Right-click on an interactive object: rotate that object.
# - Right-click on empty space: rotate the camera.
###############################################################################
class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, mainWindow, renderer):
        super(CustomInteractorStyle, self).__init__()
        self.mainWindow = mainWindow
        self.renderer = renderer

        # Register our custom event observers.
        self.AddObserver("LeftButtonPressEvent", self.leftButtonPressEvent)
        self.AddObserver("LeftButtonReleaseEvent", self.leftButtonReleaseEvent)
        self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
        self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
        self.AddObserver("MouseWheelForwardEvent", self.onMouseWheelForwardEvent)
        self.AddObserver("MouseWheelBackwardEvent", self.onMouseWheelBackwardEvent)

        # --- For left-button handling (object translation vs. camera pan) ---
        self.leftButtonDown = False
        self.PickedActor = None          # The interactive actor picked at left-button press.
        self.PickPosition = None         # World coordinate at pick.
        self.ActorInitialPosition = None # Actor’s position at pick time.
        self.IsDragging = False          # True if motion exceeds threshold.
        self.StartPosition = None        # Screen coordinates at left-button press.
        self.cameraPanActive = False     # True if left-click was on empty space → camera pan.

        # --- For right-button handling (object rotation vs. camera rotation) ---
        self.rightButtonDown = False
        self.rightLastPos = None
        self.cameraRotateActive = False  # True if right-click was on empty space → camera rotate.

        self.rightHeld = False
        self.rightHoldTimer = QTimer()
        self.rightHoldTimer.setSingleShot(True)
        self.rightHoldTimer.timeout.connect(self.onRightHoldTimeout)

    def leftButtonPressEvent(self, obj, event):
            # If we are already in connection mode, ignore normal left-click behavior.
        if self.mainWindow.connecting:
            # In connection mode, a left-click should finalize the connection.
            clickPos = self.GetInteractor().GetEventPosition()
            picker = vtk.vtkPropPicker()
            picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
            picked = picker.GetActor()
            if picked is not None and picked in self.mainWindow.actor_types:
                self.mainWindow.finalize_connection(picked)
            else:
                self.cameraPanActive = True
                super(CustomInteractorStyle, self).OnMiddleButtonDown()
            return
        clickPos = self.GetInteractor().GetEventPosition()
        # --- NEW: Check if the click is near the center of the orientation marker widget ---
        # Orientation marker viewport is set to (0.0,0.0,0.2,0.2) so its center is roughly at (0.1,0.1) in normalized coordinates.
        size = self.GetInteractor().GetRenderWindow().GetSize()  # (width, height) in pixels
        centerX = 0.1 * size[0]
        centerY = 0.1 * size[1]
        dx_center = clickPos[0] - centerX
        dy_center = clickPos[1] - centerY
        if math.sqrt(dx_center*dx_center + dy_center*dy_center) < 20:
            # Click is close to the center of the orientation marker → reset camera view.
            self.mainWindow.reset_camera_view()
            return
        # --- End of new code ---

        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
        picked = picker.GetActor()

        if picked is not None and self.mainWindow.actor_types.get(picked) == "connection":
            if self.mainWindow.selected_actor == picked:
                self.mainWindow.set_selected_actor(None)
            else:
                self.mainWindow.set_selected_actor(picked)
            # Do not initiate dragging for connections.
            return

        if picked is None or picked not in self.mainWindow.actor_types:
            # Click in empty space: simulate a middle-button press to pan the camera.
            self.cameraPanActive = True
            super(CustomInteractorStyle, self).OnMiddleButtonDown()
        else:
            # Clicked on an interactive object.
            self.cameraPanActive = False
            self.leftButtonDown = True
            self.StartPosition = clickPos
            self.PickedActor = picked
            self.PickPosition = picker.GetPickPosition()
            self.ActorInitialPosition = picked.GetPosition()
            self.IsDragging = False

    def leftButtonReleaseEvent(self, obj, event):
        if self.cameraPanActive:
            self.cameraPanActive = False
            super(CustomInteractorStyle, self).OnMiddleButtonUp()
        elif self.leftButtonDown:
            if self.PickedActor:
                if not self.IsDragging:
                    # Toggle selection: if already selected, unselect; otherwise, select.
                    if self.mainWindow.selected_actor == self.PickedActor:
                        self.mainWindow.set_selected_actor(None)
                    else:
                        self.mainWindow.set_selected_actor(self.PickedActor)
            self.leftButtonDown = False
            self.PickedActor = None
            self.PickPosition = None
            self.ActorInitialPosition = None
            self.IsDragging = False
            self.StartPosition = None
        else:
            super(CustomInteractorStyle, self).OnLeftButtonUp()

    def rightButtonPressEvent(self, obj, event):
        clickPos = self.GetInteractor().GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
        picked = picker.GetActor()
        # If nothing interactive was picked, use camera rotation.
        if picked is None or picked not in self.mainWindow.actor_types:
            self.cameraRotateActive = True
            self.rightButtonDown = True
            self.rightLastPos = clickPos
            # Call the left-button behavior to rotate the camera.
            super(CustomInteractorStyle, self).OnLeftButtonDown()
            return
        # Otherwise (an interactive object was right-clicked), start the timer
        # (to distinguish a quick click from a hold).
        self.rightHeld = False
        self.rightHoldTimer.start(300)  # 300ms threshold; adjust as needed.
        self.rightButtonDown = True
        self.rightLastPos = clickPos


    def onRightHoldTimeout(self):
        self.rightHeld = True

    def rightButtonReleaseEvent(self, obj, event):
        self.rightHoldTimer.stop()
        if self.cameraRotateActive:
            self.cameraRotateActive = False
            self.rightButtonDown = False
            self.rightLastPos = None
            super(CustomInteractorStyle, self).OnLeftButtonUp()
            return
        elif not self.rightHeld:
            # It was a quick click on an interactive object:
            clickPos = self.GetInteractor().GetEventPosition()
            picker = vtk.vtkPropPicker()
            picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
            picked = picker.GetActor()
            if picked is not None and picked in self.mainWindow.actor_types:
                from PyQt5.QtWidgets import QMenu
                menu = QMenu()
                connectAction = menu.addAction("Connect to...")
                addLabelAction = menu.addAction("Add Label")
                editNotesAction = menu.addAction("Edit Notes")
                action = menu.exec_(QCursor.pos())
                if action == connectAction:
                    self.mainWindow.start_connection(picked)
                elif action == addLabelAction:
                    self.mainWindow.add_label_to_object(picked, clickPos)
                elif action == editNotesAction:
                    self.mainWindow.edit_notes_for_object(picked)
        # Clear state variables.
        self.rightButtonDown = False
        self.rightHeld = False
        self.rightLastPos = None
        super(CustomInteractorStyle, self).OnLeftButtonUp()

    def mouseMoveEvent(self, obj, event):
        if self.mainWindow.connecting and not (self.cameraPanActive or self.cameraRotateActive):
            self.mainWindow.update_connection_line()
            return
        if self.cameraPanActive:
            # In camera pan mode (left-click in empty space), let the parent handle panning.
            super(CustomInteractorStyle, self).OnMouseMove()
        elif self.leftButtonDown and not self.cameraPanActive:
            # In object-translation mode.
            currentPos = self.GetInteractor().GetEventPosition()
            dx = currentPos[0] - self.StartPosition[0]
            dy = currentPos[1] - self.StartPosition[1]
            if abs(dx) > 5 or abs(dy) > 5:
                self.IsDragging = True
            if self.IsDragging and self.PickedActor and (self.PickedActor == self.mainWindow.selected_actor):
                picker = vtk.vtkPropPicker()
                picker.Pick(currentPos[0], currentPos[1], 0, self.renderer)
                newPickPos = picker.GetPickPosition()
                delta = (newPickPos[0] - self.PickPosition[0],
                         newPickPos[1] - self.PickPosition[1],
                         newPickPos[2] - self.PickPosition[2])
                newPos = (self.ActorInitialPosition[0] + delta[0],
                          self.ActorInitialPosition[1] + delta[1],
                          self.ActorInitialPosition[2] + delta[2])
                self.PickedActor.SetPosition(newPos)
                self.GetInteractor().GetRenderWindow().Render()
            else:
                super(CustomInteractorStyle, self).OnMouseMove()
        elif self.rightButtonDown:
            # If not already flagged as held and movement is detected, cancel timer.
            if not self.rightHeld:
                self.rightHoldTimer.stop()
                self.rightHeld = True
            if self.cameraRotateActive:
                super(CustomInteractorStyle, self).OnMouseMove()
            elif self.mainWindow.selected_actor is not None:
                currentPos = self.GetInteractor().GetEventPosition()
                dx = currentPos[0] - self.rightLastPos[0]
                dy = currentPos[1] - self.rightLastPos[1]
                self.rightLastPos = currentPos
                sensitivity = 0.5  # Adjust sensitivity as needed.
                actor = self.mainWindow.selected_actor
                actor.RotateY(dx * sensitivity)
                actor.RotateX(dy * sensitivity)
                self.GetInteractor().GetRenderWindow().Render()
            else:
                super(CustomInteractorStyle, self).OnMouseMove()
        else:
            super(CustomInteractorStyle, self).OnMouseMove()

    def onMouseWheelForwardEvent(self, obj, event):
        interactor = self.GetInteractor()
        # Check if we are in connection mode and Control is held.
        if self.mainWindow.connecting and interactor.GetControlKey():
            # Increase thickness (adjust the step as needed)
            self.mainWindow.connectionCylinderRadius += 0.5
            if hasattr(self.mainWindow, 'connectionTubeFilter') and self.mainWindow.connectionTubeFilter:
                self.mainWindow.connectionTubeFilter.SetRadius(self.mainWindow.connectionCylinderRadius)
                self.mainWindow.connectionTubeFilter.Modified()
                self.mainWindow.vtkWidget.GetRenderWindow().Render()
            return  # Do not pass event further.
        # Otherwise, use default behavior.
        self.OnMouseWheelForward()

    def onMouseWheelBackwardEvent(self, obj, event):
        interactor = self.GetInteractor()
        if self.mainWindow.connecting and interactor.GetControlKey():
            # Decrease thickness but prevent it from going too low.
            new_thickness = self.mainWindow.connectionCylinderRadius - 0.5
            if new_thickness < 0.1:
                new_thickness = 0.1
            self.mainWindow.connectionCylinderRadius = new_thickness
            if hasattr(self.mainWindow, 'connectionTubeFilter') and self.mainWindow.connectionTubeFilter:
                self.mainWindow.connectionTubeFilter.SetRadius(self.mainWindow.connectionCylinderRadius)
                self.mainWindow.connectionTubeFilter.Modified()
                self.mainWindow.vtkWidget.GetRenderWindow().Render()
            return
        self.OnMouseWheelBackward()

###############################################################################
# Main Window and Application
###############################################################################
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Optical System Designer")
        self.resize(1200, 800)

        # Connection mode variables.
        self.connecting = False          # True when a connection is active.
        self.connection_base = None      # The actor from which the connection originates.
        self.connectionLineSource = None
        self.connectionLineActor = None
        self.connect_mode = False
        self.connect_source = None
        self.connect_target = None

        # Default connection line parameters.
        self.connectionCylinderRadius = 3.0
        self.connection_line_type = "Straight"  # "Straight" or "Curved"
        self.connectionLineThickness = 5.0
        self.connection_arrow = "None"            # "None", "Start", "End", "Both"
        self.connection_linestyle = "Solid"         # "Solid", "Dashed", "Dotted"
        self.connection_color = (0, 0, 1)           # Black

        # Track objects along with their type and source (for editing dimensions).
        self.actors = []
        self.actor_types = {}    # Maps actor -> type (e.g., "cube", "sphere", "cylinder")
        self.actor_sources = {}  # Maps actor -> VTK source (to update dimensions)
        self.selected_actor = None  # The actor currently selected for editing
        self.connections = []  # Will store connection info dictionaries.
        self.connection_actors = []  # Will hold the vtkActor objects representing connections.
        self.connection_dict = {}     # Mapping from connection actor to its parameters dictionary.
        self.actor_labels = {}   # Will store label actors for each object
        self.actor_notes = {}    # Will store text notes for each object

        # Set up the main widget and layout.
        self.frame = QWidget()
        self.setCentralWidget(self.frame)
        self.mainLayout = QHBoxLayout()
        self.frame.setLayout(self.mainLayout)

        # Create a control panel on the left.
        self.controlPanel = QWidget()
        self.controlLayout = QVBoxLayout()
        self.controlPanel.setLayout(self.controlLayout)
        self.mainLayout.addWidget(self.controlPanel, 0)

        # Create a menu bar with File > Save Scene and Load Scene.
        menuBar = self.menuBar()
        fileMenu = menuBar.addMenu("File")

        saveAction = fileMenu.addAction("Save Scene")
        saveAction.triggered.connect(self.save_scene)

        loadAction = fileMenu.addAction("Load Scene")
        loadAction.triggered.connect(self.load_scene)

        zemaxAction = fileMenu.addAction("Load Zemax ZMX")
        zemaxAction.triggered.connect(self.load_zemax_file)

        # --- Object Creation and Connection Controls ---
        self.addCubeButton = QPushButton("Add Cube")
        self.addCubeButton.clicked.connect(self.add_cube)
        self.controlLayout.addWidget(self.addCubeButton)

        self.addSphereButton = QPushButton("Add Sphere")
        self.addSphereButton.clicked.connect(self.add_sphere)
        self.controlLayout.addWidget(self.addSphereButton)

        self.addCylinderButton = QPushButton("Add Cylinder")
        self.addCylinderButton.clicked.connect(self.add_cylinder)
        self.controlLayout.addWidget(self.addCylinderButton)

        # --- Separator ---
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        self.controlLayout.addWidget(sep)

        # --- Shape Dimension Editor ---
        self.shapeEditorGroup = QGroupBox("Modify Selected Shape")
        self.shapeEditorLayout = QFormLayout()
        self.shapeEditorGroup.setLayout(self.shapeEditorLayout)
        self.controlLayout.addWidget(self.shapeEditorGroup)
        self.shapeEditorGroup.setVisible(False)
        self.dimension_controls = {}  # Will hold QDoubleSpinBoxes keyed by parameter name

        self.controlLayout.addStretch(1)

        # Create the VTK widget (for the 3D view) on the right.
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self.mainLayout.addWidget(self.vtkWidget, 1)

        # Set up the VTK renderer and interactor.
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.9, 0.9, 0.9)  # Light gray background.
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtkWidget.GetRenderWindow().GetInteractor()

        # Use our custom interactor style.
        self.style = CustomInteractorStyle(mainWindow=self, renderer=self.renderer)
        self.interactor.SetInteractorStyle(self.style)

        # --- Orientation Marker Widget ---
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.0, 1.0, 1.0)
        
        # self.orientationWidget.SetOrientationMarker(axes)
        
        # Place the orientation marker in the bottom-left corner.

        # --- NEW: Create a Reset Camera "Button" at the Intersection of the Orientation Arrows ---
        # Here we create a small sphere and add it to the orientation marker assembly.
        # We do this by creating an assembly that contains the axes and the sphere.
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(0.1)  # Small sphere
        sphereSource.SetThetaResolution(32)
        sphereSource.SetPhiResolution(32)
        sphereSource.Update()
        sphereMapper = vtk.vtkPolyDataMapper()
        sphereMapper.SetInputConnection(sphereSource.GetOutputPort())
        sphereActor = vtk.vtkActor()
        sphereActor.SetMapper(sphereMapper)
        # Color the sphere distinctively (e.g., black).
        sphereActor.GetProperty().SetColor(0, 0, 0)
        sphereActor.SetPosition(0, 0, 0)  # At the intersection of the axes
        # sphereActor.SetScale(0.00000001, 0.00000001, 0.00000001)

        # Create an assembly to combine the axes and the reset sphere.
        assembly = vtk.vtkAssembly()
        assembly.AddPart(axes)
        assembly.AddPart(sphereActor)
        # Set the assembly as the orientation marker.
        self.orientationWidget = vtk.vtkOrientationMarkerWidget()
        self.orientationWidget.SetOrientationMarker(assembly)
        self.orientationWidget.SetInteractor(self.interactor)
        self.orientationWidget.SetViewport(0.0, 0.0, 0.2, 0.2)
        self.orientationWidget.SetEnabled(1)
        self.orientationWidget.InteractiveOff()
        
        # Store a reference to the sphere actor so we can detect clicks.
        # self.resetMarkerSphere = sphereActor

        #For continuous updating of selected object parameters shown in the left panel
        self.dim_update_timer = QTimer(self)
        self.dim_update_timer.timeout.connect(self.refresh_dimension_controls)
        self.dim_update_timer.start(10)  # Refresh every 500 ms


        self.interactor.Initialize()
        self.interactor.Start()

    # --- New method to reset the camera view to the original orientation ---
    def reset_camera_view(self):
        camera = self.renderer.GetActiveCamera()
        # Set the camera position so that the camera is along +Z, looking at the origin.
        camera.SetPosition(0, 0, 50)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 1, 0)
        self.renderer.ResetCameraClippingRange()
        self.vtkWidget.GetRenderWindow().Render()
        print("Camera view reset.")

    # --- Connection parameter setters ---
    def set_line_type(self, text):
        self.connection_line_type = text

    def set_arrow_type(self, text):
        self.connection_arrow = text

    def set_line_style(self, text):
        self.connection_linestyle = text

    def set_line_thickness(self, value):
        self.connectionLineThickness = value

    def add_label_to_object(self, actor, clickPos):
        """
        Adds a label to the specified actor.
        Uses a cell picker to find a face center if possible.
        """
        cellPicker = vtk.vtkCellPicker()
        cellPicker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
        if cellPicker.GetCellId() >= 0:
            labelPos = self.get_cell_center(actor, cellPicker)
        else:
            labelPos = actor.GetPosition()
        text, ok = QInputDialog.getText(self, "Add Label", "Enter label text:")
        if ok and text:
            labelActor = vtk.vtkBillboardTextActor3D()
            labelActor.SetInput(text)
            labelActor.SetPosition(labelPos)
            labelActor.GetTextProperty().SetFontSize(12)
            labelActor.GetTextProperty().SetColor(0, 0, 0)  # Black text
            self.renderer.AddActor(labelActor)
            self.vtkWidget.GetRenderWindow().Render()
            if actor not in self.actor_labels:
                self.actor_labels[actor] = []
            self.actor_labels[actor].append(labelActor)
            print(f"Label '{text}' added to object.")

    def edit_notes_for_object(self, actor):
        """
        Opens a multi-line dialog to view or edit notes attached to the actor.
        """
        existing_text = self.actor_notes.get(actor, "")
        text, ok = QInputDialog.getMultiLineText(self, "Edit Notes", "Enter notes for this object:", existing_text)
        if ok:
            self.actor_notes[actor] = text
            print("Notes updated for object.")

    def start_connection(self, base_actor):
        print("Connection started from", base_actor)
        self.connecting = True
        self.connection_base = base_actor
        basePos = base_actor.GetPosition()

        self.connectionLineSource = vtk.vtkLineSource()
        self.connectionLineSource.SetPoint1(basePos)
        self.connectionLineSource.SetPoint2(basePos)
        self.connectionLineSource.Update()
        
        # Create a tube filter to thicken the line.
        tubeFilter = vtk.vtkTubeFilter()
        tubeFilter.SetInputConnection(self.connectionLineSource.GetOutputPort())
        tubeFilter.SetRadius(self.connectionCylinderRadius)  # use your defined radius
        tubeFilter.SetNumberOfSides(32)
        tubeFilter.Update()

        # Save a reference to the tube filter so we can update its radius later.
        self.connectionTubeFilter = tubeFilter

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tubeFilter.GetOutputPort())
        
        self.connectionLineActor = vtk.vtkActor()
        self.connectionLineActor.SetMapper(mapper)
        self.connectionLineActor.GetProperty().SetColor(self.connection_color)
        self.renderer.AddActor(self.connectionLineActor)

        self.connectionLineActor.PickableOff()

        self.vtkWidget.GetRenderWindow().Render()

    def get_cell_center(self, actor, picker):
        """
        Given an actor and a vtkCellPicker that has just picked one of its cells,
        compute and return the center of that cell in world coordinates.
        """
        cell_id = picker.GetCellId()
        if cell_id < 0:
            return None
        # Get the polydata from the actor’s mapper.
        polydata = actor.GetMapper().GetInput()
        # Get the picked cell.
        cell = polydata.GetCell(cell_id)
        pts = cell.GetPoints()
        num_points = pts.GetNumberOfPoints()
        center = [0.0, 0.0, 0.0]
        for i in range(num_points):
            pt = pts.GetPoint(i)
            center[0] += pt[0]
            center[1] += pt[1]
            center[2] += pt[2]
        # Average to compute the cell (face) center (in model coordinates).
        center = [c / num_points for c in center]

        # Convert the center point from model to world coordinates.
        transform = vtk.vtkTransform()
        transform.SetMatrix(actor.GetMatrix())
        world_center = transform.TransformPoint(center)
        return world_center

    def update_connection_line(self):
        if not self.connecting:
            return

        pos = self.interactor.GetEventPosition()

        # If Shift is held, try to snap to a face on a second object.
        if self.interactor.GetShiftKey():
            cellPicker = vtk.vtkCellPicker()
            cellPicker.Pick(pos[0], pos[1], 0, self.renderer)
            target = cellPicker.GetActor()
            # Check that we have a valid target and that a cell (face) was actually picked.
            if target is not None and target in self.actor_types and cellPicker.GetCellId() >= 0:
                center = self.get_cell_center(target, cellPicker)
                if center:
                    self.connectionLineSource.SetPoint2(center)
                    self.snappedEndpoint = center  # Store the snapped point for finalize.
                else:
                    # No valid cell center found; fall back to normal picking.
                    self.snappedEndpoint = None
                    propPicker = vtk.vtkPropPicker()
                    propPicker.Pick(pos[0], pos[1], 0, self.renderer)
                    newPos = propPicker.GetPickPosition()
                    self.connectionLineSource.SetPoint2(newPos)
            else:
                # Either no target was picked or no valid face exists under the mouse.
                self.snappedEndpoint = None
                propPicker = vtk.vtkPropPicker()
                propPicker.Pick(pos[0], pos[1], 0, self.renderer)
                newPos = propPicker.GetPickPosition()
                self.connectionLineSource.SetPoint2(newPos)
        else:
            # Shift is not held: use the normal behavior.
            self.snappedEndpoint = None
            propPicker = vtk.vtkPropPicker()
            propPicker.Pick(pos[0], pos[1], 0, self.renderer)
            newPos = propPicker.GetPickPosition()
            self.connectionLineSource.SetPoint2(newPos)

        self.connectionLineSource.Modified()
        self.vtkWidget.GetRenderWindow().Render()

    def finalize_connection(self, target_actor):
        if not self.connecting:
            return
        clickPos = self.interactor.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
        # If you’re using snapping, use self.snappedEndpoint if available:
        if self.snappedEndpoint:
            targetPos = self.snappedEndpoint
        else:
            clickPos = self.interactor.GetEventPosition()
            picker = vtk.vtkPropPicker()
            picker.Pick(clickPos[0], clickPos[1], 0, self.renderer)
            targetPos = picker.GetPickPosition()
        self.connectionLineSource.SetPoint2(targetPos)
        self.connectionLineSource.Modified()
        self.vtkWidget.GetRenderWindow().Render()

        # Retrieve the two endpoints from the line source.
        point1 = self.connectionLineSource.GetPoint1()
        point2 = self.connectionLineSource.GetPoint2()

        print("Connection finalized between", self.connection_base, "and", target_actor)
        
        # Record the connection information including extra parameters.
        if self.connection_base in self.actors and target_actor in self.actors:
            index1 = self.actors.index(self.connection_base)
            index2 = self.actors.index(target_actor)
            connection_info = {
                "from": index1,
                "to": index2,
                "point1": point1,
                "point2": point2,
                "connection_color": list(self.connection_color),
                "connection_cylinder_radius": self.connectionCylinderRadius,
                "connection_line_thickness": self.connectionLineThickness,
                "connection_line_type": self.connection_line_type,
                "connection_arrow": self.connection_arrow,
                "connection_linestyle": self.connection_linestyle
            }
            self.connections.append(connection_info)
            # Create a permanent connection actor.
            conn_actor = self.create_connection_line_from_points(point1, point2, connection_info)
        
            # Remove the temporary connection actor if it exists.
        if self.connectionLineActor:
            self.renderer.RemoveActor(self.connectionLineActor)
            self.connectionLineActor = None
        
        # Reset connection variables.
        self.connecting = False
        self.connection_base = None
        self.snappedEndpoint = None
        # Optionally, you might store the connection line actor for future editing.

    def select_connection_color(self):
        color = QColorDialog.getColor()
        if color.isValid():
            self.connection_color = (
                color.red() / 255.0,
                color.green() / 255.0,
                color.blue() / 255.0,
            )

    def activate_connect_mode(self):
        self.connect_mode = True
        self.connect_source = None
        self.connect_target = None
        print("Connect mode activated. Left-click on two objects to connect them.")

    # --- Methods to add objects ---
    def add_cube(self):
        cubeSource = vtk.vtkCubeSource()
        cubeSource.SetXLength(10)
        cubeSource.SetYLength(10)
        cubeSource.SetZLength(10)
        cubeSource.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cubeSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1, 0, 0)
        actor.SetPosition(0, 0, 0)
        self.renderer.AddActor(actor)
        self.actors.append(actor)
        self.actor_types[actor] = "cube"
        self.actor_sources[actor] = cubeSource

        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()

    def add_sphere(self):
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(5)
        sphereSource.SetThetaResolution(64)
        sphereSource.SetPhiResolution(64)
        sphereSource.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0, 1, 0)
        actor.SetPosition(20, 0, 0)
        self.renderer.AddActor(actor)
        self.actors.append(actor)
        self.actor_types[actor] = "sphere"
        self.actor_sources[actor] = sphereSource

        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()

    def add_cylinder(self):
        cylinderSource = vtk.vtkCylinderSource()
        cylinderSource.SetRadius(3)
        cylinderSource.SetHeight(15)
        cylinderSource.SetResolution(64)
        cylinderSource.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinderSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0, 0, 1)
        actor.SetPosition(-20, 0, 0)
        self.renderer.AddActor(actor)
        self.actors.append(actor)
        self.actor_types[actor] = "cylinder"
        self.actor_sources[actor] = cylinderSource

        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()

    def create_curved_disk_polydata(curv, diam, conic=0.0, radial_res=32, circum_res=32):
        """
        Create a disk (as vtkPolyData) whose surface sag is computed from the curvature (curv)
        and the conic constant (conic). For a conic surface the sag is computed as
        z(r) = (r^2/R) / (1 + sqrt(1 - (1+conic)*(r/R)^2))
        where R = 1/|curv|. For curv near 0 a flat disk is produced.
        """
        outer_radius = diam / 2.0
        pts = vtk.vtkPoints()
        polys = vtk.vtkCellArray()
        numTheta = circum_res
        if abs(curv) < 1e-6:
            R = float('inf')
        else:
            R = 1.0 / abs(curv)
        for i in range(radial_res + 1):
            r = (i / radial_res) * outer_radius
            for j in range(numTheta):
                theta = (j / numTheta) * 2 * math.pi
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                if abs(curv) < 1e-6:
                    z = 0.0
                else:
                    # If a conic constant is provided and nonzero, use the conic sag formula.
                    if abs(conic) > 1e-6:
                        try:
                            denom = 1 + math.sqrt(1 - (1+conic)*(r/R)**2)
                            sag = (r*r / R) / denom if denom != 0 else 0.0
                        except ValueError:
                            sag = 0.0
                    else:
                        try:
                            sag = R - math.sqrt(R*R - r*r)
                        except ValueError:
                            sag = 0.0
                    z = math.copysign(sag, curv)
                pts.InsertNextPoint(x, y, z)
        # Build connectivity (each annular quad becomes two triangles)
        for i in range(radial_res):
            for j in range(numTheta):
                p0 = i * numTheta + j
                p1 = (i + 1) * numTheta + j
                p2 = (i + 1) * numTheta + ((j + 1) % numTheta)
                p3 = i * numTheta + ((j + 1) % numTheta)
                tri1 = vtk.vtkTriangle()
                tri1.GetPointIds().SetId(0, p0)
                tri1.GetPointIds().SetId(1, p1)
                tri1.GetPointIds().SetId(2, p2)
                polys.InsertNextCell(tri1)
                tri2 = vtk.vtkTriangle()
                tri2.GetPointIds().SetId(0, p0)
                tri2.GetPointIds().SetId(1, p2)
                tri2.GetPointIds().SetId(2, p3)
                polys.InsertNextCell(tri2)
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(pts)
        polydata.SetPolys(polys)
        return polydata

    def load_zemax_file(self):
        """
        Uses Optiland’s ZemaxToOpticConverter to load a Zemax ZMX file and then iterates over the
        surfaces (from optic.surface_group.surfaces) to obtain a VTK polydata from each surface’s geometry.
        All the resulting polydata pieces are appended and rendered as a single actor.
        
        If your surfaces’ geometry objects have a method called to_polydata(), it will be used.
        Otherwise, an error message is printed.
        """
        filename, _ = QFileDialog.getOpenFileName(self, "Load Zemax ZMX File", "", "Zemax Files (*.zmx)")
        if not filename:
            return

        try:
            from optiland.fileio.zemax_handler import ZemaxToOpticConverter
            converter = ZemaxToOpticConverter(filename)
            optic = converter.convert()
        except Exception as e:
            print("Error converting Zemax file via optiland:", e)
            return

        # For debugging, print the optic object's attributes.
        print("Optic object attributes:", optic.__dict__)
        # Try to access the surfaces from optic.surface_group
        surfaces = getattr(optic.surface_group, "surfaces", [])
        if not surfaces:
            print("No surfaces found in optic.surface_group.surfaces")
            return

        polydata_list = []
        # Iterate over each surface.
        for i, surf in enumerate(surfaces):
            geom = getattr(surf, "geometry", None)
            if geom is None:
                print(f"Surface {i} has no geometry attribute.")
                continue
            # Check if the geometry object has a to_polydata() method.
            if hasattr(geom, "to_polydata"):
                try:
                    pd = geom.to_polydata()
                    polydata_list.append(pd)
                    print(f"Surface {i} converted to polydata.")
                except Exception as e:
                    print(f"Error converting surface {i} geometry to polydata: {e}")
            else:
                print(f"Surface {i} geometry does not support to_polydata().")
        
        if not polydata_list:
            print("No polydata available from any surfaces.")
            return

        # Append all surface polydata into one.
        appendFilter = vtk.vtkAppendPolyData()
        for pd in polydata_list:
            appendFilter.AddInputData(pd)
        appendFilter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(appendFilter.GetOutputPort())
        deviceActor = vtk.vtkActor()
        deviceActor.SetMapper(mapper)
        deviceActor.GetProperty().SetColor(1, 1, 1)
        self.renderer.AddActor(deviceActor)
        self.vtkWidget.GetRenderWindow().Render()
        print("Loaded Zemax device from surface geometry.")
        self.actors.append(deviceActor)
        self.actor_types[deviceActor] = "device"
        self.actor_sources[deviceActor] = None

    def create_curved_disk_polydata(self, curv, diam, conic=0.0, radial_res=32, circum_res=32):
        """
        Creates a vtkPolyData representing a disk whose surface sag is computed from curvature and conic constant.
        For a conic surface the sag is:
            z(r) = (r^2 / R) / (1 + sqrt(1 - (1+conic)*(r/R)^2))
        where R = 1/|curv|. For near-zero curvature, a flat disk is returned.
        """
        outer_radius = diam / 2.0
        pts = vtk.vtkPoints()
        polys = vtk.vtkCellArray()
        numTheta = circum_res
        if abs(curv) < 1e-6:
            R = float('inf')
        else:
            R = 1.0 / abs(curv)
        for i in range(radial_res + 1):
            r = (i / radial_res) * outer_radius
            for j in range(numTheta):
                theta = (j / numTheta) * 2 * math.pi
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                if abs(curv) < 1e-6:
                    z = 0.0
                else:
                    if abs(conic) > 1e-6:
                        try:
                            denom = 1 + math.sqrt(1 - (1+conic) * (r / R)**2)
                            sag = (r * r / R) / denom if denom != 0 else 0.0
                        except ValueError:
                            sag = 0.0
                    else:
                        try:
                            sag = R - math.sqrt(R*R - r*r)
                        except ValueError:
                            sag = 0.0
                    z = math.copysign(sag, curv)
                pts.InsertNextPoint(x, y, z)
        for i in range(radial_res):
            for j in range(numTheta):
                p0 = i * numTheta + j
                p1 = (i + 1) * numTheta + j
                p2 = (i + 1) * numTheta + ((j + 1) % numTheta)
                p3 = i * numTheta + ((j + 1) % numTheta)
                tri1 = vtk.vtkTriangle()
                tri1.GetPointIds().SetId(0, p0)
                tri1.GetPointIds().SetId(1, p1)
                tri1.GetPointIds().SetId(2, p2)
                polys.InsertNextCell(tri1)
                tri2 = vtk.vtkTriangle()
                tri2.GetPointIds().SetId(0, p0)
                tri2.GetPointIds().SetId(1, p2)
                tri2.GetPointIds().SetId(2, p3)
                polys.InsertNextCell(tri2)
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(pts)
        polydata.SetPolys(polys)
        return polydata
    
    # --- Method to create a connection line between two objects ---
    def create_connection_line(self, actor1, actor2):
        # Get the positions of the two objects.
        pos1 = actor1.GetPosition()  # e.g. (x1, y1, z1)
        pos2 = actor2.GetPosition()  # e.g. (x2, y2, z2)

        # Compute the difference vector and distance.
        diff = [pos2[i] - pos1[i] for i in range(3)]
        distance = math.sqrt(sum([d * d for d in diff]))
        if distance == 0:
            print("Warning: zero-length connection")
            distance = 1.0

        # Compute the midpoint.
        mid = [(pos1[i] + pos2[i]) / 2.0 for i in range(3)]

        # Create a cylinder source.
        cylSource = vtk.vtkCylinderSource()
        # Use the connectionCylinderRadius that you define in __init__
        cylSource.SetRadius(self.connectionCylinderRadius)
        cylSource.SetHeight(distance)
        cylSource.SetResolution(64)
        cylSource.Update()

        # The default vtkCylinderSource produces a cylinder centered at (0,0,0) along the z-axis.
        # We need to rotate it so that it aligns with the vector from pos1 to pos2,
        # then translate it to the midpoint.

        # Compute a normalized direction vector for the connection.
        norm = distance
        direction = [d / norm for d in diff]

        # The default direction of the cylinder is (0, 0, 1). Compute the rotation.
        defaultDir = [0, 0, 1]
        # Compute the cross product to get the rotation axis.
        rotAxis = [0, 0, 0]
        vtk.vtkMath.Cross(defaultDir, direction, rotAxis)
        # Compute the angle in degrees between defaultDir and direction.
        dot = vtk.vtkMath.Dot(defaultDir, direction)
        # Clamp dot to avoid numerical issues.
        dot = max(min(dot, 1.0), -1.0)
        angle = math.degrees(math.acos(dot))

        # Create a transform.
        transform = vtk.vtkTransform()
        if angle != 0:
            transform.RotateWXYZ(angle, rotAxis)
        # Translate to the midpoint.
        transform.Translate(mid)

        # Apply the transformation to the cylinder.
        transFilter = vtk.vtkTransformPolyDataFilter()
        transFilter.SetTransform(transform)
        transFilter.SetInputConnection(cylSource.GetOutputPort())
        transFilter.Update()

        # Create a mapper and actor for the transformed cylinder.
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transFilter.GetOutputPort())
        cylActor = vtk.vtkActor()
        cylActor.SetMapper(mapper)
        cylActor.GetProperty().SetColor(self.connection_color)

        # Add the connection cylinder to the renderer.
        self.renderer.AddActor(cylActor)
        self.vtkWidget.GetRenderWindow().Render()

    def add_arrow(self, pos1, pos2, at_start=True):
        arrowSource = vtk.vtkArrowSource()
        arrowSource.SetTipLength(3)
        arrowSource.SetTipRadius(1)
        arrowSource.SetShaftRadius(0.5)
        arrowSource.Update()

        if at_start:
            start = pos1
            end = pos2
        else:
            start = pos2
            end = pos1

        dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length == 0:
            length = 1
        direction = (dx/length, dy/length, dz/length)

        transform = vtk.vtkTransform()
        transform.Translate(start)
        def vector_angle(u, v):
            dot = np.dot(u, v)
            norm_u = np.linalg.norm(u)
            norm_v = np.linalg.norm(v)
            cos_angle = dot/(norm_u*norm_v)
            return np.arccos(np.clip(cos_angle, -1, 1)) * 180/math.pi
        axis = np.cross((1,0,0), direction)
        angle = vector_angle((1,0,0), direction)
        transform.RotateWXYZ(angle, axis)

        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        transformFilter.SetInputConnection(arrowSource.GetOutputPort())
        transformFilter.Update()

        tubeFilter = vtk.vtkTubeFilter()
        tubeFilter.SetInputConnection(transformFilter.GetOutputPort())
        # Set the tube radius using the connectionLineThickness.
        tubeFilter.SetRadius(self.connectionLineThickness)
        tubeFilter.SetNumberOfSides(12)
        tubeFilter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tubeFilter.GetOutputPort())
        arrowActor = vtk.vtkActor()
        arrowActor.SetMapper(mapper)
        arrowActor.GetProperty().SetColor(self.connection_color)
        self.renderer.AddActor(arrowActor)

    # --- Dimension Editing Methods ---
    def set_selected_actor(self, actor):
        self.selected_actor = actor
        self.update_dimension_editor()

    def update_dimension_editor(self):
        # Clear the layout completely.
        while self.shapeEditorLayout.count():
            item = self.shapeEditorLayout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
        self.dimension_controls.clear()

        if self.selected_actor is None or self.selected_actor not in self.actor_types:
            label = QLabel("No shape selected")
            self.shapeEditorLayout.addRow(label)
            self.shapeEditorGroup.setVisible(False)
            return

        shape_type = self.actor_types[self.selected_actor]
        label = QLabel(f"Selected: {shape_type.capitalize()}")
        self.shapeEditorLayout.addRow(label)

        # If the selected actor is a connection, use connection-specific controls.
        if shape_type == "connection":
            connection_params = self.connection_dict.get(self.selected_actor, {})
            # Change Color button:
            colorButton = QPushButton("Change Connection Color")
            colorButton.clicked.connect(self.change_selected_connection_color)
            self.shapeEditorLayout.addRow(colorButton)
            # Thickness control:
            thicknessSpin = QDoubleSpinBox()
            thicknessSpin.setRange(0.1, 100.0)
            thicknessSpin.setSingleStep(0.5)
            current_thickness = connection_params.get("connection_cylinder_radius", self.connectionCylinderRadius)
            thicknessSpin.setValue(current_thickness)
            thicknessSpin.valueChanged.connect(self.update_selected_connection_thickness)
            self.dimension_controls["connection_thickness"] = thicknessSpin
            self.shapeEditorLayout.addRow("Thickness:", thicknessSpin)
            # Delete button:
            deleteButton = QPushButton("Delete Connection")
            deleteButton.clicked.connect(self.delete_selected_actor)
            self.shapeEditorLayout.addRow(deleteButton)
        else:
            # For shapes, retrieve the VTK source to update dimensions.
            source = self.actor_sources[self.selected_actor]
            if shape_type == "cube":
                for param in ["XLength", "YLength", "ZLength"]:
                    spin = QDoubleSpinBox()
                    spin.setRange(0.1, 1000)
                    spin.setDecimals(2)
                    if param == "XLength":
                        spin.setValue(source.GetXLength())
                    elif param == "YLength":
                        spin.setValue(source.GetYLength())
                    elif param == "ZLength":
                        spin.setValue(source.GetZLength())
                    self.dimension_controls[param] = spin
                    self.shapeEditorLayout.addRow(param + ":", spin)
            elif shape_type == "sphere":
                spin = QDoubleSpinBox()
                spin.setRange(0.1, 1000)
                spin.setDecimals(2)
                spin.setValue(source.GetRadius())
                self.dimension_controls["Radius"] = spin
                self.shapeEditorLayout.addRow("Radius:", spin)
            elif shape_type == "cylinder":
                for param in ["Radius", "Height"]:
                    spin = QDoubleSpinBox()
                    spin.setRange(0.1, 1000)
                    spin.setDecimals(2)
                    if param == "Radius":
                        spin.setValue(source.GetRadius())
                    else:
                        spin.setValue(source.GetHeight())
                    self.dimension_controls[param] = spin
                    self.shapeEditorLayout.addRow(param + ":", spin)
            elif shape_type == "device":
                # Devices have fixed shape; only allow editing of position and orientation.
                pos = self.selected_actor.GetPosition()
                posX = QDoubleSpinBox()
                posX.setRange(-1000, 1000)
                posX.setValue(pos[0])
                self.dimension_controls["Position X"] = posX
                self.shapeEditorLayout.addRow("Position X:", posX)
                posY = QDoubleSpinBox()
                posY.setRange(-1000, 1000)
                posY.setValue(pos[1])
                self.dimension_controls["Position Y"] = posY
                self.shapeEditorLayout.addRow("Position Y:", posY)
                posZ = QDoubleSpinBox()
                posZ.setRange(-1000, 1000)
                posZ.setValue(pos[2])
                self.dimension_controls["Position Z"] = posZ
                self.shapeEditorLayout.addRow("Position Z:", posZ)
                
                ori = self.selected_actor.GetOrientation()
                oriX = QDoubleSpinBox()
                oriX.setRange(-360, 360)
                oriX.setValue(ori[0])
                self.dimension_controls["Orientation X"] = oriX
                self.shapeEditorLayout.addRow("Orientation X:", oriX)
                oriY = QDoubleSpinBox()
                oriY.setRange(-360, 360)
                oriY.setValue(ori[1])
                self.dimension_controls["Orientation Y"] = oriY
                self.shapeEditorLayout.addRow("Orientation Y:", oriY)
                oriZ = QDoubleSpinBox()
                oriZ.setRange(-360, 360)
                oriZ.setValue(ori[2])
                self.dimension_controls["Orientation Z"] = oriZ
                self.shapeEditorLayout.addRow("Orientation Z:", oriZ)
                
                deleteButton = QPushButton("Delete Device")
                deleteButton.clicked.connect(self.delete_selected_actor)
                self.shapeEditorLayout.addRow(deleteButton)
            else:
                info = QLabel("No editable parameters for this shape.")
                self.shapeEditorLayout.addRow(info)

            # Now add controls for position.
            pos = self.selected_actor.GetPosition()
            posX = QDoubleSpinBox()
            posX.setRange(-1000, 1000)
            posX.setValue(pos[0])
            self.dimension_controls["Position X"] = posX
            self.shapeEditorLayout.addRow("Position X:", posX)
            posY = QDoubleSpinBox()
            posY.setRange(-1000, 1000)
            posY.setValue(pos[1])
            self.dimension_controls["Position Y"] = posY
            self.shapeEditorLayout.addRow("Position Y:", posY)
            posZ = QDoubleSpinBox()
            posZ.setRange(-1000, 1000)
            posZ.setValue(pos[2])
            self.dimension_controls["Position Z"] = posZ
            self.shapeEditorLayout.addRow("Position Z:", posZ)

            # And add controls for orientation.
            ori = self.selected_actor.GetOrientation()  # Euler angles in degrees.
            oriX = QDoubleSpinBox()
            oriX.setRange(-360, 360)
            oriX.setValue(ori[0])
            self.dimension_controls["Orientation X"] = oriX
            self.shapeEditorLayout.addRow("Orientation X:", oriX)
            oriY = QDoubleSpinBox()
            oriY.setRange(-360, 360)
            oriY.setValue(ori[1])
            self.dimension_controls["Orientation Y"] = oriY
            self.shapeEditorLayout.addRow("Orientation Y:", oriY)
            oriZ = QDoubleSpinBox()
            oriZ.setRange(-360, 360)
            oriZ.setValue(ori[2])
            self.dimension_controls["Orientation Z"] = oriZ
            self.shapeEditorLayout.addRow("Orientation Z:", oriZ)

            # Add the Apply Changes button.
            applyButton = QPushButton("Apply Changes")
            applyButton.clicked.connect(self.update_selected_actor_dimensions)
            self.shapeEditorLayout.addRow(applyButton)
            # Add a Delete button.
            deleteButton = QPushButton("Delete Shape")
            deleteButton.clicked.connect(self.delete_selected_actor)
            self.shapeEditorLayout.addRow(deleteButton)
            # Add a Change Color button.
            colorButton = QPushButton("Change Color")
            colorButton.clicked.connect(self.change_selected_actor_color)
            self.shapeEditorLayout.addRow(colorButton)
            # Edit object notes
            notesButton = QPushButton("Edit Notes")
            notesButton.clicked.connect(lambda: self.edit_notes_for_object(self.selected_actor))
            self.shapeEditorLayout.addRow(notesButton)

        self.shapeEditorGroup.setVisible(True)

    def refresh_dimension_controls(self):
        # Only update if an actor is selected and it is not a connection.
        if self.selected_actor is None or self.actor_types.get(self.selected_actor) == "connection":
            return

        # Check if any spin box in our dimension_controls currently has focus.
        for key, spin in self.dimension_controls.items():
            if spin.hasFocus():
                return  # User is actively editing—don’t override.

        # Retrieve the current position and orientation from the selected actor.
        pos = self.selected_actor.GetPosition()
        ori = self.selected_actor.GetOrientation()
        
        # If the controls exist, update them with the current values.
        if "Position X" in self.dimension_controls:
            self.dimension_controls["Position X"].setValue(pos[0])
        if "Position Y" in self.dimension_controls:
            self.dimension_controls["Position Y"].setValue(pos[1])
        if "Position Z" in self.dimension_controls:
            self.dimension_controls["Position Z"].setValue(pos[2])
        if "Orientation X" in self.dimension_controls:
            self.dimension_controls["Orientation X"].setValue(ori[0])
        if "Orientation Y" in self.dimension_controls:
            self.dimension_controls["Orientation Y"].setValue(ori[1])
        if "Orientation Z" in self.dimension_controls:
            self.dimension_controls["Orientation Z"].setValue(ori[2])

    def change_selected_connection_color(self):
        if self.selected_actor is None or self.actor_types.get(self.selected_actor) != "connection":
            return
        color = QColorDialog.getColor()
        if color.isValid():
            # Compute the new color as a tuple of floats in [0, 1].
            new_color = (color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0)
            # Immediately update the actor's property.
            self.selected_actor.GetProperty().SetColor(new_color)
            # Also update the connection info in the dictionary.
            if self.selected_actor in self.connection_dict:
                self.connection_dict[self.selected_actor]["connection_color"] = list(new_color)
            self.vtkWidget.GetRenderWindow().Render()

    def update_selected_connection_thickness(self, value):
        if self.selected_actor is None or self.actor_types.get(self.selected_actor) != "connection":
            return
        # Update the stored thickness.
        if self.selected_actor in self.connection_dict:
            self.connection_dict[self.selected_actor]["connection_cylinder_radius"] = value
            conn_params = self.connection_dict[self.selected_actor]
            # Remove the old actor.
            self.renderer.RemoveActor(self.selected_actor)
            if self.selected_actor in self.connection_actors:
                self.connection_actors.remove(self.selected_actor)
            # (Also remove it from actor_types and connection_dict.)
            if self.selected_actor in self.actor_types:
                del self.actor_types[self.selected_actor]
            if self.selected_actor in self.connection_dict:
                del self.connection_dict[self.selected_actor]
            # Create a new connection actor with the updated thickness.
            new_actor = self.create_connection_line_from_points(conn_params["point1"], conn_params["point2"], conn_params)
            # Update selection.
            self.selected_actor = new_actor
        self.vtkWidget.GetRenderWindow().Render()


    def update_selected_actor_dimensions(self):
        if self.selected_actor is None or self.selected_actor not in self.actor_types:
            return
        shape_type = self.actor_types[self.selected_actor]
        # For shapes, update both the VTK source (for dimensions) and the actor (for position/orientation).
        if shape_type in ["cube", "sphere", "cylinder"]:
            source = self.actor_sources[self.selected_actor]
            if shape_type == "cube":
                source.SetXLength(self.dimension_controls["XLength"].value())
                source.SetYLength(self.dimension_controls["YLength"].value())
                source.SetZLength(self.dimension_controls["ZLength"].value())
            elif shape_type == "sphere":
                source.SetRadius(self.dimension_controls["Radius"].value())
            elif shape_type == "cylinder":
                source.SetRadius(self.dimension_controls["Radius"].value())
                source.SetHeight(self.dimension_controls["Height"].value())
            source.Modified()
            self.selected_actor.GetMapper().Update()
            # Now update position.
            pos = (
                self.dimension_controls["Position X"].value(),
                self.dimension_controls["Position Y"].value(),
                self.dimension_controls["Position Z"].value()
            )
            self.selected_actor.SetPosition(pos)
            # Update orientation.
            ori = (
                self.dimension_controls["Orientation X"].value(),
                self.dimension_controls["Orientation Y"].value(),
                self.dimension_controls["Orientation Z"].value()
            )
            self.selected_actor.SetOrientation(ori)
            self.selected_actor.Modified()
            self.vtkWidget.GetRenderWindow().Render()
            print(f"{shape_type.capitalize()} updated.")
        elif shape_type == "device":
            pos = (
                self.dimension_controls["Position X"].value(),
                self.dimension_controls["Position Y"].value(),
                self.dimension_controls["Position Z"].value()
            )
            self.selected_actor.SetPosition(pos)
            ori = (
                self.dimension_controls["Orientation X"].value(),
                self.dimension_controls["Orientation Y"].value(),
                self.dimension_controls["Orientation Z"].value()
            )
            self.selected_actor.SetOrientation(ori)
            self.selected_actor.Modified()
            self.vtkWidget.GetRenderWindow().Render()
            print("Device updated.")
        else:
            # If it is a connection, handle updates separately.
            pass  # (Your connection update code goes here, if needed.)

    def delete_selected_actor(self):
        if self.selected_actor is None:
            return
        # Check the type.
        actor_type = self.actor_types.get(self.selected_actor, None)
        # Remove the actor from the renderer.
        self.renderer.RemoveActor(self.selected_actor)
        # Remove from our internal lists/dictionaries.
        if actor_type == "connection":
            if self.selected_actor in self.connection_actors:
                self.connection_actors.remove(self.selected_actor)
            if self.selected_actor in self.connection_dict:
                del self.connection_dict[self.selected_actor]
        else:
            if self.selected_actor in self.actors:
                self.actors.remove(self.selected_actor)
            if self.selected_actor in self.actor_types:
                del self.actor_types[self.selected_actor]
        if self.selected_actor in self.actor_sources:
            del self.actor_sources[self.selected_actor]
        self.selected_actor = None
        self.vtkWidget.GetRenderWindow().Render()
        self.update_dimension_editor()
        print("Selected shape deleted.")

    def change_selected_actor_color(self):
        if self.selected_actor is None:
            return
        color = QColorDialog.getColor()
        if color.isValid():
            r = color.red() / 255.0
            g = color.green() / 255.0
            b = color.blue() / 255.0
            self.selected_actor.GetProperty().SetColor(r, g, b)
            self.vtkWidget.GetRenderWindow().Render()
            print("Selected shape color updated.")

    def save_scene(self):
        scene = {"objects": [], "connections": []}
        for actor in self.actors:
            obj = {}
            shape_type = self.actor_types[actor]
            obj["type"] = shape_type
            obj["position"] = list(actor.GetPosition())
            obj["color"] = list(actor.GetProperty().GetColor())
            obj["orientation"] = list(actor.GetOrientation())
            source = self.actor_sources[actor]
            if shape_type == "cube":
                obj["dimensions"] = {
                    "XLength": source.GetXLength(),
                    "YLength": source.GetYLength(),
                    "ZLength": source.GetZLength()
                }
            elif shape_type == "sphere":
                obj["dimensions"] = {"Radius": source.GetRadius()}
            elif shape_type == "cylinder":
                obj["dimensions"] = {
                    "Radius": source.GetRadius(),
                    "Height": source.GetHeight()
                }
            if actor in self.actor_notes:
                obj["notes"] = self.actor_notes[actor]
            if actor in self.actor_labels:
                label_list = []
                for labelActor in self.actor_labels[actor]:
                    label_list.append({
                        "text": labelActor.GetInput(),
                        "position": list(labelActor.GetPosition())
                    })
                obj["labels"] = label_list
            scene["objects"].append(obj)

        # Instead of using self.connections, build the connection list from connection_dict.
        for conn_info in self.connection_dict.values():
            scene["connections"].append(conn_info)
        
        filename, _ = QFileDialog.getSaveFileName(self, "Save Scene", "", "JSON Files (*.json)")
        if filename:
            with open(filename, "w") as f:
                json.dump(scene, f, indent=4)
            print("Scene saved to", filename)

    def create_connection_line_from_points(self, point1, point2, connection_params):
        # Create a line source using the saved endpoints.
        lineSource = vtk.vtkLineSource()
        lineSource.SetPoint1(point1)
        lineSource.SetPoint2(point2)
        lineSource.Update()
        
        # Create a tube filter to give the line thickness.
        tubeFilter = vtk.vtkTubeFilter()
        tubeFilter.SetInputConnection(lineSource.GetOutputPort())
        # Use your saved cylinder radius (or line thickness) parameter.
        tubeFilter.SetRadius(connection_params.get("connection_cylinder_radius", 3.0))
        tubeFilter.SetNumberOfSides(32)
        tubeFilter.Update()
        
        # Create a mapper and actor.
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tubeFilter.GetOutputPort())
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        # Set the actor's color from the saved parameters.
        actor.GetProperty().SetColor(connection_params.get("connection_color", (0, 1, 0)))
        actor.SetPickable(True)

        self.renderer.AddActor(actor)
        self.vtkWidget.GetRenderWindow().Render()

        # Keep track of this connection actor.
        self.connection_actors.append(actor)
        # Also add it to actor_types with a special type.
        self.actor_types[actor] = "connection"
        # Save its connection parameters.
        self.connection_dict[actor] = connection_params.copy()

        return actor

    def load_scene(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Scene", "", "JSON Files (*.json)")
        if not filename:
            return
        with open(filename, "r") as f:
            scene = json.load(f)
        
        # Clear current objects and connections.
        for actor in self.actors:
            self.renderer.RemoveActor(actor)
        self.actors = []
        self.actor_types = {}
        self.actor_sources = {}
        
        # Also clear any connection actors.
        for conn_actor in self.connection_actors:
            self.renderer.RemoveActor(conn_actor)
        self.connection_actors = []
        self.connection_dict = {}
        self.connections = []

        # Recreate objects.
        for obj in scene["objects"]:
            shape_type = obj["type"]
            pos = obj["position"]
            color = obj["color"]
            dims = obj["dimensions"]
            orientation = obj.get("orientation", [0, 0, 0])
            if shape_type == "cube":
                cubeSource = vtk.vtkCubeSource()
                cubeSource.SetXLength(dims["XLength"])
                cubeSource.SetYLength(dims["YLength"])
                cubeSource.SetZLength(dims["ZLength"])
                cubeSource.Update()
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(cubeSource.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.SetPosition(pos)
                actor.SetOrientation(orientation)
                actor.GetProperty().SetColor(color)
                self.renderer.AddActor(actor)
                self.actors.append(actor)
                self.actor_types[actor] = "cube"
                self.actor_sources[actor] = cubeSource
            elif shape_type == "sphere":
                sphereSource = vtk.vtkSphereSource()
                sphereSource.SetRadius(dims["Radius"])
                sphereSource.SetThetaResolution(64)
                sphereSource.SetPhiResolution(64)
                sphereSource.Update()
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(sphereSource.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.SetPosition(pos)
                actor.SetOrientation(orientation)
                actor.GetProperty().SetColor(color)
                self.renderer.AddActor(actor)
                self.actors.append(actor)
                self.actor_types[actor] = "sphere"
                self.actor_sources[actor] = sphereSource
            elif shape_type == "cylinder":
                cylinderSource = vtk.vtkCylinderSource()
                cylinderSource.SetRadius(dims["Radius"])
                cylinderSource.SetHeight(dims["Height"])
                cylinderSource.SetResolution(64)
                cylinderSource.Update()
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(cylinderSource.GetOutputPort())
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.SetPosition(pos)
                actor.SetOrientation(orientation)
                actor.GetProperty().SetColor(color)
                self.renderer.AddActor(actor)
                self.actors.append(actor)
                self.actor_types[actor] = "cylinder"
                self.actor_sources[actor] = cylinderSource
            if "notes" in obj:
                self.actor_notes[actor] = obj["notes"]
            if "labels" in obj:
                for lab in obj["labels"]:
                    labelActor = vtk.vtkBillboardTextActor3D()
                    labelActor.SetInput(lab["text"])
                    labelActor.SetPosition(lab["position"])
                    labelActor.GetTextProperty().SetFontSize(12)
                    labelActor.GetTextProperty().SetColor(0, 0, 0)  # Black text.
                    self.renderer.AddActor(labelActor)
                    if actor not in self.actor_labels:
                        self.actor_labels[actor] = []
                    self.actor_labels[actor].append(labelActor)
        # Recreate connections.
        for connection in scene.get("connections", []):
            index1 = connection["from"]
            index2 = connection["to"]
            if index1 < len(self.actors) and index2 < len(self.actors):
                # Retrieve the saved endpoints.
                point1 = connection["point1"]
                point2 = connection["point2"]
                # Create the connection using the helper function.
                self.create_connection_line_from_points(point1, point2, connection)
                self.connections.append(connection)
                
        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()
        print("Scene loaded from", filename)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
