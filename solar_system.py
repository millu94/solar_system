import json
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

    def read_input_data():
        pass

    def run_simulation():
        pass

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
        # load each individual planet in
        self.name = name
        self.mass = mass
        self.orbital_radius = orbital_radius
        self.colour = colour

    def create_body_list():
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

    body_object_list = []

    for body in parameters_solar['bodies']:
        body_object = Body(
            body['name'], 
            body['mass'], 
            body['orbital_radius'],
            body['colour']
        )
        # parameters_solar['bodies'][body]
        body_object_list.append(body_object)
    
    print(body_object_list)
    print(body_object_list[0].mass)

main()





# print(f"The timestep is {parameters_solar['timestep']}")
# for body in parameters_solar['bodies']: print(f"{body['name']} has mass {body['mass']}")