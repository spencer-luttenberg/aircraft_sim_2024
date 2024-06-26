"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import numpy as np
from models.mav_dynamics import MavDynamics as MavDynamicsForces

# load message types
from message_types.msg_state import MsgState
import parameters.sensor_parameters as SENSOR

from message_types.msg_sensors import MsgSensors
from message_types.msg_delta import MsgDelta
import parameters.aerosonde_parameters as MAV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation, euler_to_quaternion


class MavDynamics(MavDynamicsForces):
    def __init__(self, Ts):
        super().__init__(Ts)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self.initialize_velocity(MAV.u0, 0., 0.)
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.
        self.current_forces = [0,0,0,0,0,0]
        



    def initialize_velocity(self, Va, alpha, beta):
        self._Va = Va
        self._alpha = alpha
        self.beta = beta
        self._state[3] = Va*np.cos(alpha)*np.cos(beta)
        self._state[4] = Va*np.sin(beta)
        self._state[5] = Va*np.sin(alpha)*np.cos(beta)
        self._update_velocity_data()

        self._update_true_state()


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        Vg_b = self._state[3:6]
        Va_b = Vg_b - steady_state

        ur, vr, wr = Va_b[:,0]
        self._Va = np.linalg.norm(Va_b, axis=0)[0]

        self._alpha = np.arctan2(wr, ur)
        self._beta = np.arcsin(vr/self._Va)




        ##### TODO #####
        # convert wind vector from world to body frame (self._wind = ?)

        




        # velocity vector relative to the airmass ([ur , vr, wr]= ?)

        # compute airspeed (self._Va = ?)

        # compute angle of attack (self._alpha = ?)
        
        # compute sideslip angle (self._beta = ?)
    def calculate_trim_output(self, x):
        alpha, elevator, throttle = x
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        self._state[6:10] = euler_to_quaternion(phi, alpha, psi)

        self.initialize_velocity(self._Va, alpha, self._beta)
        delta = MsgDelta()
        delta.elevator = elevator
        delta.throttle = throttle
        forces = self._forces_moments(delta=delta)
        



        return(forces[0]**2 + forces[2]**2 + forces[4]**2)

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
       




        # simulate rate gyros(units are rad / sec)
        sensor_noise = np.radians(0.5)
        self._sensors.gyro_x = self.true_state.p + np.random.normal(0, sensor_noise)
        self._sensors.gyro_y = self.true_state.q + np.random.normal(0, sensor_noise)
        self._sensors.gyro_z = self.true_state.r + np.random.normal(0, sensor_noise)

        # simulate accelerometers(units of g)
        self._sensors.accel_x =self.current_forces[0][0] / MAV.mass + MAV.gravity * np.sin(self.true_state.theta) + np.random.normal(0, SENSOR.accel_sigma)
        self._sensors.accel_y = self.current_forces[1][0] / MAV.mass - MAV.gravity * np.cos(self.true_state.theta) * np.sin(self.true_state.phi) + np.random.normal(0, SENSOR.accel_sigma)
        self._sensors.accel_z = self.current_forces[2][0] / MAV.mass - MAV.gravity * np.cos(self.true_state.theta) * np.cos(self.true_state.phi) + np.random.normal(0, SENSOR.accel_sigma)
        
        inc = np.deg2rad(64)
        dec = np.deg2rad(2)

        m_strength = 50046.3 * (10**(-9)) #nT

        M_h = np.sin(inc)* m_strength

        M_n = np.cos(self.true_state.psi-dec)*M_h                # North

        M_e = np.sin(self.true_state.psi- dec)*M_h                # East

        M_d = np.cos(inc)*m_strength                # Down

        M0_v1 = (np.matrix([M_n, M_e, M_d])).T

        mv1 = (np.linalg.inv(euler_to_rotation(phi=self.true_state.phi, theta=self.true_state.theta, psi=0)))*M0_v1




        # R_matrix = (euler_to_rotation(0, -inc, dec)).T
        # tmp_mat = np.matrix([[1], [0], [0]])
        # m_i = R_matrix*tmp_mat
        # mb = (euler_to_rotation(phi=self.true_state.phi, theta=self.true_state.theta, psi=self.true_state.psi)).T * m_i

        mv1[0] = mv1[0] #+ np.random.normal(0, SENSOR.mag_sigma)
        mv1[1] = mv1[1] #+ np.random.normal(0, SENSOR.mag_sigma)
        mv1[2] = mv1[2] #+ np.random.normal(0, SENSOR.mag_sigma)
        self._sensors.mag_x = mv1[0]
        self._sensors.mag_y = mv1[1]
        self._sensors.mag_z = mv1[2]

        
        mv1_o = (euler_to_rotation(phi=self.true_state.phi, theta=self.true_state.theta, psi=0))*mv1
        psi_m = -np.arctan2(mv1_o[1], mv1_o[0])
        psi_out = psi_m + dec


        #print(np.rad2deg(psi_out)-np.rad2deg(self.true_state.psi))

        P0 = 101325
        L0 = -0.0065
        T0 = 288.15
        M_M = 0.0289644 #kg/mol
        R = 8.31432
        
        self._sensors.abs_pressure = P0 * (T0/((L0 * self.true_state.altitude)+T0)) ** ((MAV.gravity * M_M) / (R * L0)) + np.random.normal(0, SENSOR.abs_pres_sigma)
        self._sensors.diff_pressure = ((MAV.rho * self.true_state.Va ** 2) / 2) + np.random.normal(0, SENSOR.diff_pres_sigma)
        
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_n + np.random.normal(0, SENSOR.gps_n_sigma) #use a sample time of 0.2
            self._gps_eta_e = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_e + np.random.normal(0, SENSOR.gps_e_sigma)
            self._gps_eta_h = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_h + np.random.normal(0, SENSOR.gps_h_sigma)
            self._gps_eta_Vg = np.random.normal(0, SENSOR.gps_Vg_sigma)
            self._gps_eta_course = np.random.normal(0, SENSOR.gps_course_sigma)
            
            self._sensors.gps_n = self.true_state.north + self._gps_eta_n
            self._sensors.gps_e = self.true_state.east + self._gps_eta_e
            self._sensors.gps_h = self.true_state.altitude + self._gps_eta_h
            
            self._sensors.gps_Vg = np.sqrt((self.true_state.Va*np.cos(self.true_state.psi)+self.true_state.wn)**2+(self.true_state.Va*np.sin(self.true_state.psi)+self.true_state.we)**2) + self._gps_eta_Vg
            self._sensors.gps_course =  (np.arctan2(self.true_state.Va*np.sin(self.true_state.psi)+self.true_state.we, self.true_state.Va*np.cos(self.true_state.psi)+self.true_state.wn) + self._gps_eta_course)
            self._t_gps = 0
        else:
            self._t_gps += self._ts_simulation
        return self._sensors
        
  
    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        ##### TODO ######
        # extract states (phi, theta, psi, p, q, r)
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        phi, theta, psi = quaternion_to_euler(self._state[6:10])

        fg_b = np.matmul(euler_to_rotation(phi, theta, psi).T, [0, 0, MAV.mass*MAV.gravity])


        M_minus = np.exp(-MAV.M*(self._alpha-MAV.alpha0))
        M_plus = np.exp(MAV.M*(self._alpha+MAV.alpha0))

        sigmoid = (1+ M_minus + M_plus)/((1+M_minus)*(1+M_plus))
        CL = (1-sigmoid)*(MAV.C_L_0 + MAV.C_L_alpha*self._alpha) + sigmoid*(2*np.sign(self._alpha)*np.sin(self._alpha)**2*np.cos(self._alpha))
        CD = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * self._alpha)**2/(np.pi * MAV.e * MAV.AR)
        
    
        q_bar = .5 * MAV.rho * self._Va**2 

        F_lift = q_bar * MAV.S_wing * (CL + MAV.C_L_q*(MAV.c/(2*self._Va))*self.true_state.q +MAV.C_L_delta_e * delta.elevator)
        F_drag = q_bar * MAV.S_wing * (CD + (MAV.C_D_q * (MAV.c/(2*self._Va)) * self.true_state.q) + MAV.C_D_delta_e * delta.elevator)

        # compute gravitational forces ([fg_x, fg_y, fg_z])

        # compute Lift and Drag coefficients (CL, CD)

        # compute Lift and Drag Forces (F_lift, F_drag)

        # propeller thrust and torque
        

        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)

        c_x = -CD*np.cos(self._alpha) + CL*np.sin(self._alpha)
        c_x_q_term = -MAV.C_D_q*np.cos(self._alpha)+MAV.C_L_q*np.sin(self._alpha)
        c_x_delta_e = -MAV.C_D_delta_e*np.cos(self._alpha)+MAV.C_L_delta_e*np.sin(self._alpha)


        z_c = -CD*np.sin(self._alpha) - CL*np.cos(self._alpha)
        c_z_q = -MAV.C_D_q*np.sin(self._alpha)-MAV.C_L_q*np.cos(self._alpha)
        c_z_delta_e = -MAV.C_D_delta_e*np.sin(self._alpha)-MAV.C_L_delta_e*np.cos(self._alpha)



        fx = fg_b[0] + thrust_prop + q_bar*MAV.S_wing*(c_x + c_x_q_term*(MAV.c/(2*self._Va))*q) + q_bar*MAV.S_wing*c_x_delta_e*delta.elevator
        fy = fg_b[1] + q_bar*MAV.S_wing*(MAV.C_Y_0 + MAV.C_Y_beta*self._beta + MAV.C_Y_p*(self._beta) + p*MAV.C_Y_p*(MAV.b/(2*self._Va)) + MAV.C_Y_r*(MAV.b/(2*self._Va))*r) + q_bar*MAV.S_wing*(MAV.C_Y_delta_a*delta.aileron + MAV.C_Y_r*delta.rudder)
        fz = fg_b[2] + q_bar * MAV.S_wing * (z_c + (c_z_q*MAV.c*self.true_state.q)/(2*self._Va) + c_z_delta_e*delta.elevator)
       

        Mx = torque_prop+q_bar*MAV.S_wing*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta * self._beta + (MAV.C_ell_p*self.true_state.p)*(MAV.b/(2*self._Va))+(MAV.C_ell_r*MAV.b*self.true_state.r)/(2*self._Va) + MAV.C_ell_delta_a*delta.aileron + MAV.C_ell_delta_r*delta.rudder)
        My = q_bar * MAV.S_wing*MAV.c*(MAV.C_m_0+MAV.C_m_alpha*self._alpha+(MAV.C_m_q*self.true_state.q)*(MAV.c/(2*self._Va))) + q_bar*MAV.S_wing*MAV.c*MAV.C_m_delta_e*delta.elevator
        Mz = q_bar * MAV.S_wing*MAV.b*(MAV.C_n_0 + MAV.C_n_beta*self._beta+(MAV.C_n_p*self.true_state.p)*(MAV.b/(2*self._Va))+(MAV.C_n_r*self.true_state.r)*(MAV.b/(2*self._Va))+MAV.C_n_delta_a*delta.aileron + MAV.C_n_delta_r*delta.rudder)
        

        forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T
        self.current_forces = forces_moments
        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        ##### TODO #####
        # map delta_t throttle command(0 to 1) into motor input voltage
        # v_in =
        # Angular speed of propeller (omega_p = ?)
        # compute thrust and torque due to propeller
        ##### TODO #####
        # map delta_t throttle command(0 to 1) into motor input voltage
        # v_in = MAV.V_max * delta_t
        
        # a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) / ((2*np.pi)**2)
        # b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4) / (2*np.pi)) * self._Va + MAV.KQ**2/MAV.R_motor
        # c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) * self._Va**2 - (MAV.KQ/MAV.R_motor) * v_in + MAV.KQ*MAV.i0

        # omega_p = (-b + np.sqrt(b**2-4*a*c))/(2*a)
        # J_op = 2*np.pi * self._Va / (omega_p * MAV.D_prop)
        # C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0
        # C_Q = MAV.C_Q2 * J_op**2 + MAV.C_Q1 * J_op + MAV.C_Q0
        
        # n = omega_p / (2 *np.pi)
        # # Angular speed of propeller (omega_p = ?)

        # # thrust and torque due to propeller
        # thrust_prop = MAV.rho * n**2 * np.power(MAV.D_prop, 4) * C_T
        # torque_prop = -MAV.rho * n**2 * np.power(MAV.D_prop, 5) * C_Q
        thrust_prop = 0.5*MAV.rho*MAV.S_prop*((MAV.K_motor*delta_t)**2 - Va**2)
        torque_prop = 0

        return thrust_prop, torque_prop


    def _update_true_state(self):
        # rewrite this function because we now have more information
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(-pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0
