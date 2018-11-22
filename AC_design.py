# Copyright (C) 2017 Murat Bronz, Gautier Hattenberger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

"""

Drone Design Practical Work

Mineure Drone ENAC

"""

from numpy import sin, arcsin, cos, pi
import pdb

#--- COEFFS ---
gee        = 9.81        # m/s2
nu         = 0.000015    # m2/s  kinematic viscosity
k_batt     = 170.0       # Specific energy [Wh/kg]

#
# Exception when flight condition is not valid
#
class InvalidCondition(Exception):
    pass

#
# Aircraft object
#
class Aircraft(object):
    """ Aircraft performance model"""

    def __init__(self, b=1.0, S=0.2,
                 thick_prof=0.095, rho_mat=45.0,
                 k_f_weight=80.0, thick_skin=0.01,
                 cap_batt=25.0,
                 W_system=1.0, W_payload=0.5,
                 P_system=5, P_payload=2.2,
                 eff_prop=0.45,
                 CL_max=None, P_max=None):
        """
        Initialize parameters with user values or default
        :param b: wing span [m]
        :param S: wing surface area [m^2]
        :param thick_prof: airfoil thickness [%]
        :param rho_mat: material density [kg/m^3]
        :param k_f_weight: weight ratio
        :param thick_skin: fuselage skin thickness [?]
        :param cap_batt: battery capacity [Wh]
        :param W_system: system weight [N]
        :param W_payload: payload weight [N]
        :param P_system: system power consumption [W]
        :param P_payload: payload power consumption [W]
        :param eff_prop: propulsion efficiency [%]
        :param CL_max: maximum lift coefficient
        :param P_max: maximum propulsive power [N]
        """
        self.b = b                      # Wing span [m]
        self.S = S                      # Wing surface area [m2]
        self.c = self.S / self.b        # Wing Chord [m]
        self.AR = self.b / self.c       # Aspect ratio
        self.thick_prof = thick_prof    # Airfoil thickness [%]
        self.rho_mat = rho_mat          # Material density [kg/m3]
        self.k_f_weight = k_f_weight    # Weight ratio
        self.thick_skin = thick_skin    # Fuselage skin thickness [m]
        self.cap_batt = cap_batt        # Battery capacity [Wh]
        self.W_system = W_system        # System weigth [N]
        self.W_payload = W_payload      # Payload weight [N]
        self.P_system = P_system        # System power consumption [W]
        self.P_payload = P_payload      # Payload power consumption [W]
        self.eff_prop = eff_prop        # Propulstion efficiency
        self.CL_max = CL_max            # Maximum lift coeff
        self.P_max = P_max              # Maximum propulsive power [N]

        # Compute some weights
        W_wing = 0.5*self.c*self.b*self.thick_prof*self.rho_mat*gee
        d_fuse = 0.05*self.b
        W_fuse = 0.6*self.b*2.0*pi*(d_fuse/2.0)*self.k_f_weight*self.thick_skin*gee
        W_batt = self.cap_batt/k_batt * gee
        self.W_total= W_wing + W_fuse + W_batt + self.W_system + self.W_payload

        # Init some variables
        self.V = 0.
        self.ROC = 0.
        self.CL = 0.
        self.Lift = 0.
        self.CD = 0.
        self.Drag = 0.
        self.Thrust = 0.
        self.P_total = 0.

    def calc_perfo_basic(self, V, ROC, rho=1.225):
        """
        Compute basic performance parameters
        may raise InvalidCondition exception
        :param V: airspeed [m/s]
        :param ROC: rate of climb [m/s]
        :rho: air density [kg/m^3]

        :return: (total power consumption, lift coefficient)
        """
        self.V      = V
        self.ROC    = ROC
        self.e      = 1.78*(1-0.045*self.AR**0.68) - 0.64 # Raymer's emp formula for straight wing A/C
        self.theta  = arcsin(ROC / V)
        self.Lift   = self.W_total * cos(self.theta)
        self.q      = 0.5 * rho * V**2
        self.CL     = self.Lift / (self.q * self.S)
        if self.CL_max is not None and self.CL > self.CL_max:
            # raise execption for stalling condition
            #print("Stalled !")
            raise InvalidCondition
        self.RE     = (V * self.c) / nu
        self.CD0    = -0.03*self.RE**0.1 + 0.13
        self.CD     = self.CD0 + (self.CL**2.0)/(pi*self.AR*self.e)
        self.Drag   = self.q * self.S * self.CD
        self.Thrust = self.Drag + self.W_total * sin(self.theta)
        if self.Thrust < 0.:
            # raise execption for negative thrust
            #print("Negative Thrust!")
            raise InvalidCondition
        if self.Thrust < 0. :
            self.P_prop = 0.
        else:
            self.P_prop = V * self.Thrust / self.eff_prop
        if self.P_max is not None and self.P_prop > self.P_max:
            # raise execption for stalling condition
            #print("Too much power !")
            raise InvalidCondition
        # total power consumption
        self.P_total = self.P_prop + self.P_system + self.P_payload
        return (self.P_total, self.CL)

    def __str__(self):
        """
        Display aircraft info and state
        """
        return "\
Aircraft information\n\
    wing span: %f m; wing area: %f m/s^2; chord: %f m; aspect ratio: %f\n\
    speed: %f m/s; rate of climb: %f m/s\n\
    total weight: %f N; Lift: %f N (CL: %f)\n\
    thrust: %f N; Drag: %f N (CD: %f)\n\
    total power consumption: %f Wh\n\
\n\
" % (self.b, self.S, self.c, self.AR, self.V, self.ROC, self.W_total, self.Lift, self.CL, self.Thrust, self.Drag, self.CD, self.P_total)

#
# Mission object
#
class Mission(object):
    """ Mission description

    corresponds to a list of climb, cruise and descent phases
    """

    def __init__(self, ac=None, rho=1.225):
        """ initialize with an aircraft model """
        self.ac = ac
        self.rho = rho
        self.mission = []

    def set_ac(self, ac):
        """ set a new aircraft model """
        self.ac = ac

    def add_cruise(self, dist, rho=1.225):
        """ add a cruise phase to the mission
        :param dist: horizontal distance in cruise flight
        """
        el = { 'type': "cruise", 'dist': dist, 'density': rho }
        self.mission.append(el)

    # def add_cruise_density(self, rho):
    #     """ add a specific desity at cruise phase to the mission
    #     :param density: density
    #     """
    #     el = { 'type': "cruise", 'density': rho }
    #     self.mission.append(el)

    def add_climb(self, height):
        """ add a climb phase to the mission
        :param height: vertical distance in climb phase
        """
        el = { 'type': "climb", 'height': height }
        self.mission.append(el)

    def add_descent(self, height):
        """ add a descent phase to the mission
        :param height: vertical distance in descent phase
        """
        el = { 'type': "descent", 'height': height }
        self.mission.append(el)

    def clear_mission(self):
        """ clear all mission elements """
        self.mission = []

    def calc_mission(self, V, ROC):
        """ compute a mission

        :param V: airspeed [m/s]
        :param ROC: rate of climb [m/s]
        :return: (total energy consumption, total mission time)
        """
        E_total = 0.
        T_total = 0.
        for el in self.mission:
            if el['type'] == "cruise":
                # pdb.set_trace()
                P, CL = self.ac.calc_perfo_basic(V, 0., el['density'])
                duration = el['dist'] / V
            elif el['type'] == "climb":
                P, CL = self.ac.calc_perfo_basic(V, ROC)
                duration = el['height'] / ROC
            elif el['type'] == "descent":
                P, CL = self.ac.calc_perfo_basic(V, -ROC)
                duration = el['height'] / abs(ROC)
            E_total = E_total + duration * P
            T_total = T_total + duration
        if E_total/3600 >= self.ac.cap_batt:
            #print("Insufficient Battery Capacity")
            raise InvalidCondition
        return (E_total/3600, T_total/3600)
