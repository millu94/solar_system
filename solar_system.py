import json
import math
import numpy as np
import matplotlib.pyplot as plt

with open('parameters_solar.json') as f:
    parameters_solar = json.load(f)

class Simulation():

    def __init__(self):
        self.num_iterations = parameters_solar['num_iterations']
        self.timestep = parameters_solar['timestep']
        self.grav_constant = parameters_solar['grav_const']
        self.sun_mass = parameters_solar['bodies'][0]['mass']

        # holds the body objects
        self.body_list = []
        # patch list is for graphics
        self.patch_list = []

    def read_input_data(self):
        # body_object_list = []
        for body in parameters_solar['bodies']:
            body_object = Body(
                body['name'], 
                body['mass'], 
                body['orbital_radius'],
                body['colour']
            )
            self.body_list.append(body_object)
        #return self.body_list

    def calc_initial_conditions(self):

        for planet in range(len(self.body_list)):
            # use position-based version for initialisation
            self.body_list[planet].next_acceleration = (
                self.calc_acceleration_by_position(
                    self.body_list[planet].position
                )
            )
            # print(f"{self.body_list[3].name}'s next acceleration: {self.body_list[3].next_acceleration}")

            (self.body_list[planet].previous_acceleration, 
            self.body_list[planet].acceleration) = self.update_accelerations(
                planet
            )

            # set previous acceleration to current acceleration for 
            # inital condition 
            self.body_list[planet].previous_acceleration = (
                self.body_list[planet].acceleration
            ) 

    def run_simulation(self):

        self.read_input_data()
        self.calc_initial_conditions()
        
        print(f"earth initial position: {self.body_list[3].position}")
        # print(f"earth initial velocity: {self.body_list[3].velocity}")
        # print(f"earth initial timestep: {self.body_list[3].timestep}")
        # print(f"earth initial acceleration: {self.body_list[3].acceleration}")
        # print(f"earth initial previous acceleration: {self.body_list[3].previous_acceleration}")

        # for timestep in range(self.num_iterations):
        for timestep in range(1000):
            print(f"timestep: {timestep}")
            self.step_forward()
            # print(f"earth updated position: {self.body_list[3].position}")

    
    def step_forward(self):

        for planet in range(len(self.body_list)):
            # update the position
            self.body_list[planet].position = (
                self.body_list[planet].update_position()
            )
            # append the new position to the list of positions
            self.body_list[planet].positions.append(
                self.body_list[planet].position
            )
            # calculate the next acceleration
            self.body_list[planet].next_acceleration = (
                self.calc_acceleration_by_index(planet)
            )
            # update the velocity
            self.body_list[planet].velocity = (
                self.body_list[planet].update_velocity()
            )
            # update accelerations
            (self.body_list[planet].previous_acceleration, 
            self.body_list[planet].acceleration) = (
                self.update_accelerations(planet)
            )
         

    def calc_acceleration_by_position(self, next_position):
        """
        calculates net acceleration from body i on all other bodies j

        intially, next_position is the orbital radius of the planet whose
        acceleration is being calculated 
        """
        total_acceleration = np.zeros(2)
        
        for body in self.body_list:
            if np.array_equal(body.position, next_position):  # Skip self
                continue
                
            r_ji = next_position - body.position
            r_mag = np.linalg.norm(r_ji)
            
            if r_mag > 0:  # Avoid division by zero
                total_acceleration += (
                    -self.grav_constant 
                    * body.mass 
                    * r_ji 
                    / r_mag**3
                )
        
        return total_acceleration

    def calc_acceleration_by_index(self, body_index):
        """
        Calculates acceleration for a specific body in body_list (used during 
        simulation).
        More efficient as it can access body properties directly.
        """
        return self.calc_acceleration_by_position(
            self.body_list[body_index].position
        )

    def update_accelerations(self, planet):
        prev_a = self.body_list[planet].acceleration.copy()
        new_a = self.body_list[planet].next_acceleration
        return prev_a, new_a
    
    def visualise_orbits(self):
        plt.figure(figsize=(10, 10))
        ax = plt.gca()
        
        # Set equal aspect and grid
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.7)
        
        # Plot each body's orbit
        for body in self.body_list:
            positions = np.array(body.positions)
            x = positions[:, 0]
            y = positions[:, 1]
            
            # Plot orbit path
            plt.plot(x, y, color=body.colour, label=body.name, alpha=0.7)
            
            # Plot current position
            plt.scatter(x[-1], y[-1], color=body.colour, s=100 if body.name.lower() == 'sun' else 30)
        
        # Set labels and legend
        plt.title('Solar System Simulation')
        plt.xlabel('X Position (AU)')
        plt.ylabel('Y Position (AU)')
        plt.legend(loc='upper right')
        
        # Set limits based on maximum orbital radius
        max_radius = max(body.orbital_radius for body in self.body_list)
        plt.xlim(-max_radius*1.1, max_radius*1.1)
        plt.ylim(-max_radius*1.1, max_radius*1.1)
        
        plt.show()

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

        #list of positions that will be graphically represented
        self.positions = [self.position.copy()]

        if orbital_radius != 0:
            initial_velocity = math.sqrt(
                (parameters_solar['grav_const'] * 
                parameters_solar['bodies'][0]['mass']) /
                orbital_radius
            )
        else:
            initial_velocity = 0
        
        self.velocity = np.array([0, initial_velocity], dtype=float)
        #print(f"initial velocity: {self.velocity}")

        self.acceleration = np.zeros(2)
        self.next_acceleration = np.zeros(2)
        self.previous_acceleration = np.zeros(2)


    def update_position(self):
        next_position = (
            self.position + (self.velocity * self.timestep) +
            (4 * self.acceleration - self.previous_acceleration) *
            self.timestep ** 2 / 6
        )
        return next_position

    def update_velocity(self):
        next_velocity = (
            self.velocity + (
                2 * self.next_acceleration
                + 5 * self.acceleration
                - self.previous_acceleration
            ) * self.timestep / 6
        )
        return next_velocity

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
    simulation.visualise_orbits()
    # print(f"The timestep is {parameters_solar['timestep']}")
    # for body in parameters_solar['bodies']: print(f"{body['name']} has mass {body['mass']}")

main()




