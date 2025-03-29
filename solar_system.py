import json
import math
import numpy as np
import matplotlib.pyplot as plt

with open('parameters_solar.json') as f:
    parameters_solar = json.load(f)

class Simulation():

    def __init__(self):
        self.timestep = parameters_solar['timestep']
        self.num_iterations = parameters_solar['num_iterations']

        # holds the body objects
        self.body_list = []
        # patch list is for graphics
        self.patch_list = []

    def read_input_data(self):
        body_object_list = []
        for body in parameters_solar['bodies']:
            body_object = Body(
                body['name'], 
                body['mass'], 
                body['orbital_radius'],
                body['colour']
            )
            body_object_list.append(body_object)
        #return body_object_list

    def run_simulation(self):
        self.read_input_data()

        """
        
        """

    def step_forward():
        pass 

    def calc_acceleration():
        pass

    def update_calculation():
        pass

    def calc_PE():
        pass

    def calc_total_energy():
        pass


class Body():

    def __init__(self, name, mass, orbital_radius, colour):
        self.timestep = parameters_solar['timestep']
        # load each individual planet in
        self.name = name
        self.mass = mass
        self.orbital_radius = orbital_radius
        self.colour = colour

        # each planet has initial position velocity and current/previous 
        # acceleration
        self.position = np.array([orbital_radius, 0], dtype=float)
        # print(f"initial position: {self.position}")

        if orbital_radius != 0:
            initial_velocity = math.sqrt(
                (parameters_solar['grav_const'] * 
                parameters_solar['bodies'][0]['mass']) /
                orbital_radius
            )
        else:
            initial_velocity = 0
        
        self.velocity = np.array([0, initial_velocity], dtype=float)
        # print(f"initial velocity: {self.velocity}")

        self.acceleration = np.zeros(2)
        self.prev_acceleration = np.zeros(2)


    def update_position(self):
        next_position = (
            self.position + (self.velocity * self.timestep)
            + (4 * self.acceleration - self.prev_acceleration) 
            * (self.timestep ** 2) / 6
        )

    def update_velocity(self, new_a):
        next_velocity = (
            self.velocity + 
            ((2 * new_a) + (5 * self.acceleration) - prev_a)

        )

    def calc_KE(self):
        pass

    def check_orbital_period():
        pass

def main():

    """

    Upon running program, user is asked whether to use default parameters or 
    enter custom

    default:

    in order to run simulation there must be a list of body objects to pass to
    simulation object

    """

    # default

    simulation = Simulation()
    simulation.run_simulation()
    # print(f"The timestep is {parameters_solar['timestep']}")
    # for body in parameters_solar['bodies']: print(f"{body['name']} has mass {body['mass']}")

main()




