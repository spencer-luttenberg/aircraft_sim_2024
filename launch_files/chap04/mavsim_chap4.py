"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/27/2018 - RWB
        1/17/2019 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from python_tools.quit_listener import QuitListener
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
from models.mav_dynamics_control import MavDynamics
from models.wind_simulation import WindSimulation
from viewers.mav_viewer import MavViewer
from viewers.data_viewer import DataViewer
from message_types.msg_delta import MsgDelta
import keyboard
import pygame


#quitter = QuitListener()

#pygame stuff


pygame.init()

X = 1000
Y = 1000

font = pygame.font.SysFont("comicsansms", 22)
display_surface = pygame.display.set_mode((X, Y))

white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)

pygame.display.set_caption('Display my controls!')





VIDEO = False
PLOTS = True
ANIMATION = True
SAVE_PLOT_IMAGE = False

if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap4_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#initialize the visualization
if ANIMATION or PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if PLOTS:
    # initialize view of data plots
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time
plot_time = sim_time
end_time = 60

# main simulation loop
print("Press 'Esc' to exit...")

default_delta_elevator = -0.1248
default_delta_aileron = 0.001836
default_delta_rudder = -0.0003026
default_delta_throrttle = 0.6768






while sim_time < end_time:





    # ------- set control surfaces -------------
    if keyboard.is_pressed('u'):
        default_delta_elevator += 0.005
    elif keyboard.is_pressed('j'):
        default_delta_elevator -= 0.005        
    if keyboard.is_pressed('h'):
        default_delta_aileron -= 0.005
    elif keyboard.is_pressed('k'):
        default_delta_aileron += 0.005
    if keyboard.is_pressed('p'):
        default_delta_rudder -= 0.005
    elif keyboard.is_pressed('o'):
        default_delta_rudder += 0.005
    # if keyboard.is_pressed('r'):
    #         delta.throttle += 0.01
    # elif keyboard.is_pressed('f'):
    #         delta.throttle -= 0.01
    # if keyboard.is_pressed('esc'):
    #     break

    if keyboard.is_pressed('w'):
        delta.elevator = 0.2
    elif keyboard.is_pressed('s'):
        delta.elevator = -0.2
    else:
        delta.elevator = default_delta_elevator#keep it level    


    if keyboard.is_pressed('a'):
        delta.aileron = -0.2
    elif keyboard.is_pressed('d'):
        delta.aileron = 0.2
    else:
        delta.aileron = default_delta_aileron

    if keyboard.is_pressed('q'):
            delta.rudder = -0.2
    elif keyboard.is_pressed('e'):
            delta.rudder = 0.2
    else:
        delta.rudder = default_delta_rudder

    if keyboard.is_pressed('r'):
            delta.throttle += 0.01
    elif keyboard.is_pressed('f'):
            delta.throttle -= 0.01
    if keyboard.is_pressed('esc'):
        break

    #pygame


    my_string = "Elevator: " + str(delta.elevator) + " Aileron: " + str(delta.aileron) + " Rudder: " + str(delta.rudder) + " Throttle: " + str(delta.throttle)
    text = font.render(my_string, True, blue)
    textRect = text.get_rect()
    textRect.center = (X // 2, Y // 2)

    display_surface.fill(white)
    display_surface.blit(text, textRect)
    for event in pygame.event.get():
 
        # if event object type is QUIT
        # then quitting the pygame
        # and program both.
        if event.type == pygame.QUIT:
 
            # deactivates the pygame library
            pygame.quit()
 
            # quit the program.
            quit()
 
        # Draws the surface object to the screen.
    pygame.display.update()




    # ------- physical system -------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    if ANIMATION:
        mav_view.update(mav.true_state)  # plot body of MAV
    if PLOTS:
        plot_time = sim_time
        data_view.update(mav.true_state,  # true states
                            None,  # estimated states
                            None,  # commanded states
                            delta)  # inputs to aircraft
    if ANIMATION or PLOTS:
        app.processEvents()
    if VIDEO is True:
        video.update(sim_time)
        
    # # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation


if SAVE_PLOT_IMAGE:
    data_view.save_plot_image("ch4_plot")

if VIDEO is True:
    video.close()