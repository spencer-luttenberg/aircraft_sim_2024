"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        1/13/2021 - TWM
        7/13/2023 - RWB
        1/16/2024 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation
from tools.drawing import rotate_points, translate_points, points_to_mesh


class DrawMAV:
    def __init__(self, state, window, scale=10):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = scale
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        # get points that define the non-rotated, non-translated MAV and the mesh colors
        self.sc_points, self.sc_index, self.sc_meshColors = self.get_sc_points()
        self.sc_body = self.add_object(
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)
        window.addItem(self.sc_body)  # add MAV to plot     

    def update(self, state):
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        self.sc_body = self.update_object(
            self.sc_body,
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)

    def add_object(self, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def get_sc_points(self):
        """"
            Points that define the MAV, and the colors of the triangular mesh
            Define the points on the MAV following information in Appendix C.3
        """
        # points are in XYZ coordinates
        #   define the points on the MAV according to Appendix C.3
        points = self.unit_length * np.array([
            [2.4,0,0.4], #0: p1
            [1.432,0.648,-0.516], #1: p2
            [1.432,-0.648,-0.516], #2: p3
            [1.432,-0.648,0.516], #3: p4
            [1.432,0.648,0.516], #4: p5
            [-4.416,0,0], #5: p6
            [0,2.904,0], #6: p7
            [-1.776,2.904,0], #7: p8
            [-1.776,-2.904,0], #8: p9
            [0,-2.904,0], #9 p10
            [-3.312,1.488,0], #10 p11
            [-4.416,1.488,0], #11 p12
            [-4.416,-1.488,0], #12 p13
            [-3.312,-1.488,0], #13 p14
            [-3.312,0,0], #14 p15
            [-4.416,0,-1.344] #15 p16
            ]).T
        # point index that defines the mesh
        index = np.array([
            [0,1,2], #0: Top Front Face
            [0,2,3], #1: Left Front Face
            [0,1,4], #2: Right Front Face
            [0,4,3], #3: Bottom Front Face
            [1,2,5], #4: Top Side Body
            [3,4,5], #5: Bottom Side Body
            [2,3,5], #6: Left Side Body
            [1,4,5], #7: Right Side Body
            [5,14,15], #8: Stabilizer
            [6,7,9], #9: Wing 1
            [7, 9, 8], #10: Wing 2
            [10, 13, 12], #11: Rudder 1
            [11, 10, 12] #12: Rudder 2
            ])
        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((13, 3, 4), dtype=np.float32)
        meshColors[0] = yellow  # front 1
        meshColors[1] = yellow  # front 2
        meshColors[2] = yellow  # back 1
        meshColors[3] = yellow  # back 2
        meshColors[4] = blue  # right 1
        meshColors[5] = blue  # right 2
        meshColors[6] = blue  # left 1
        meshColors[7] = blue  # left 2
        meshColors[8] = red  # top 1
        meshColors[9] = red  # top 2
        meshColors[10] = red  # bottom 1
        meshColors[11] = green  # bottom 2
        meshColors[12] = green  # bottom 2
        return points, index, meshColors

