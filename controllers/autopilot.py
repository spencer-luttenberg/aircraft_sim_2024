"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
# from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pid_control import PIDControl
from controllers.pd_control_with_rate import PDControlWithRate
from controllers.tf_control import TFControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from controllers.pd_control import PDControl

AP.airspeed_throttle_kp = 1.0/50.0 #50 m/s being max value
AP.airspeed_throttle_ki = 0.0001

yaw_damper_kp = 10.0
yaw_damper_kd = 1.0


alpha_elevator_kp = -(1/np.deg2rad(13)) #set 13 degrees max
alpha_elevator_ki = 0.00001
alpha_elevator_kd = 0.1*alpha_elevator_kp

class Autopilot:
    def __init__(self, delta, mav, ts_control):


        self.throttle_from_airspeed = PIControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0,
                        init_integrator=delta.throttle/AP.airspeed_throttle_ki)
        self.elevator_from_alpha = PIDControl(
            kp=alpha_elevator_kp,
            ki=alpha_elevator_ki,
            kd=alpha_elevator_kd,
            limit=1.0,
            Ts=ts_control,
            init_integrator=delta.elevator/alpha_elevator_ki
        )
        self.yaw_damper = PDControl(kp=yaw_damper_kp, kd=yaw_damper_kd, Ts=ts_control, limit=1.0)

        # # instantiate lateral-directional controllers
        # self.roll_from_aileron = PDControlWithRate(
        #                 kp=AP.roll_kp,
        #                 kd=AP.roll_kd,
        #                 limit=np.radians(45))
        # self.course_from_roll = PIControl(
        #                 kp=AP.course_kp,
        #                 ki=AP.course_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        # # self.yaw_damper = TransferFunction(
        # #                 num=np.array([[AP.yaw_damper_kr, 0]]),
        # #                 den=np.array([[1, AP.yaw_damper_p_wo]]),
        # #                 Ts=ts_control)
        # self.yaw_damper = TFControl(
        #                 k=AP.yaw_damper_kr,
        #                 n0=0.0,
        #                 n1=1.0,
        #                 d0=AP.yaw_damper_p_wo,
        #                 d1=1,
        #                 Ts=ts_control)

        # # instantiate longitudinal controllers

        # self.altitude_from_pitch = PIControl(
        #                 kp=AP.altitude_kp,
        #                 ki=AP.altitude_ki,
        #                 Ts=ts_control,
        #                 limit=np.radians(30))
        self.commanded_state = MsgState()

    def update(self, cmd, state):
	
	#### TODO #####
        # lateral autopilot


        # longitudinal autopilot


        # construct control outputs and commanded states
        delta = MsgDelta(elevator=0,
                         aileron=0,
                         rudder=0,
                         throttle=0)
        
        
        delta.throttle = self.throttle_from_airspeed.update(23, state.Va)
        delta.elevator = self.elevator_from_alpha.update(np.deg2rad(3), state.alpha)
        #delta.elevator = self.elevator_from_alpha.update(state.alpha, state.alpha)
        delta.rudder = self.yaw_damper.update(0, state.beta)


        self.commanded_state.altitude = 0
        self.commanded_state.Va = 0
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = 0
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
