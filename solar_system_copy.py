import json
import math
import numpy as np
import matplotlib.pyplot as plt

with open('parameters_solar.json') as f:
    parameters_solar = json.load(f)

class Simulation():

    def __init__(self, integration_method):
        self.num_iterations = parameters_solar['num_iterations']
        self.timestep = parameters_solar['timestep']
        self.grav_constant = parameters_solar['grav_const']
        self.sun_mass = parameters_solar['bodies'][0]['mass']
        self.integration_method = integration_method

        # holds the body objects
        self.body_list = []
        # patch list is for graphics
        self.patch_list = []

        # New attributes for energy tracking
        self.time_points = []
        self.kinetic_energies = []
        self.potential_energies = []
        self.total_energies = []

    def read_input_data(self):
        """
        Reads input data from parameters_solar.json
        """
        for body in parameters_solar['bodies']:
            body_object = Body(
                body['name'], 
                body['mass'], 
                body['orbital_radius'],
                body['colour']
            )
            self.body_list.append(body_object)

    def calc_initial_conditions(self):
        """
        For first iteration the previous acceleration is set to the current
        acceleration, which is calculated using initial positions.
        """
        for planet in range(len(self.body_list)):
            # calculates current acceleration
            self.body_list[planet].acceleration = (
                self.calc_acceleration_by_position(
                    self.body_list[planet].position
                )
            )
            # sets previous acceleration to current acceleration
            self.body_list[planet].previous_acceleration = (
                self.body_list[planet].acceleration
            )

    def run_simulation(self):

        self.read_input_data()
        self.calc_initial_conditions()

        print(f"initial total energy: {self.calc_total_energy()} AU")
        print(f"initial total energy: {self.calc_total_energy() * 4.47e37} J")

        for num_timestep in range(self.num_iterations):
            #print(f"timestep: {num_timestep}")
            self.step_forward(num_timestep)

        print(f"final total energy: {self.calc_total_energy()} AU")
        print(f"final total energy: {self.calc_total_energy() * 4.47e37} J")

        print(self.integration_method)

    def step_forward(self, timestep):
        """
        Calls several functions to move each of the planets forward by one 
        timestep based on their positions, velocities and past current and 
        future accelerations, all determined on their relation to each other
        """

            # Record energies at each timestep
        self.record_energies(timestep)

        for planet in range(len(self.body_list)):

            # save old position for orbital period calc
            old_position = self.body_list[planet].position.copy()

            if self.integration_method == "Beeman Method":
                # update the position
                self.body_list[planet].position = (
                    self.body_list[planet].update_position()
                )
                # check whether a planet has completed an orbital period
                self.determine_orbital_period(timestep, old_position, planet)

                # append the new position to the list of positions
                self.body_list[planet].positions.append(
                    self.body_list[planet].position
                )
                # calculate the next acceleration
                self.body_list[planet].next_acceleration = (
                    self.calc_acceleration_by_position(
                        self.body_list[planet].position
                    )
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

            if self.integration_method == "Euler Cromer":
                # Euler-Cromer method
                (self.body_list[planet].velocity, 
                self.body_list[planet].position)  = (
                    self.body_list[planet].euler_cromer()
                )
                # check whether a planet has completed an orbital period
                self.determine_orbital_period(timestep, old_position, planet)
                
                # append the new position to the list of positions
                self.body_list[planet].positions.append(
                    self.body_list[planet].position
                )
                # For Euler-Cromer, we just need current acceleration
                self.body_list[planet].acceleration = (
                    self.calc_acceleration_by_position(
                        self.body_list[planet].position
                    )
                )

            if self.integration_method == "Direct Euler":
                # Direct Euler Method
                (self.body_list[planet].position, 
                 self.body_list[planet].velocity)  = (
                    self.body_list[planet].direct_euler()
                )
                # check whether a planet has completed an orbital period
                self.determine_orbital_period(timestep, old_position, planet)
                
                # append the new position to the list of positions
                self.body_list[planet].positions.append(
                    self.body_list[planet].position
                )
                # For Direct Euler, we just need current acceleration
                self.body_list[planet].acceleration = (
                    self.calc_acceleration_by_position(
                        self.body_list[planet].position
                    )
                )


    def determine_orbital_period(self, timestep, old_position, planet):
        """
        Compare old position to new position and if y coordinate has 
        changed from negative to positive then save orbital period.
        """
        if old_position[1] < 0 and self.body_list[planet].position[1] > 0:
            if self.body_list[planet].orbital_period == 0:
                self.body_list[planet].orbital_period =  (
                    round(timestep * self.timestep, 3)
                )
                print(
                    f"{self.body_list[planet].name} orbital period: "
                    f"{self.body_list[planet].orbital_period} years"
                    f" at timestep number {timestep} of size {self.timestep}"
                )
         

    def calc_acceleration_by_position(self, next_position):
        """
        Calculates net acceleration from body i on all other bodies j,
        intially next_position is the orbital radius of the planet whose
        acceleration is being calculated. 
        """
        total_acceleration = np.zeros(2)
        sun_position = self.body_list[0].position
        
        for body in self.body_list:
            # skip a planet calculating acceleration with itself 
            if np.array_equal(body.position, next_position):
                continue

            # calculates magnitude of position vector relative to sun, using
            # just the y coordinate of the sun doesn't make a difference   
            r_ji = next_position - body.position - sun_position
            r_mag = np.linalg.norm(r_ji)

            # adds to total acceleration vector
            total_acceleration += (
                -self.grav_constant * body.mass * r_ji / r_mag**3
            )
        
        return total_acceleration

    def update_accelerations(self, planet):
        """
        Sets previous acceleration to equal the current acceleration and 
        current acceleration to next acceleration, both values in turn used by
        update_position().
        """
        prev_a = self.body_list[planet].acceleration.copy()
        new_a = self.body_list[planet].next_acceleration
        return prev_a, new_a
    
    def visualise_orbits(self):
        """
        Displays the orbits statically using matplotlib.pyplot
        """
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
            plt.scatter(
                x[-1], y[-1], color=body.colour,
                s=100 if body.name.lower() == 'sun' else 30
            )
        
        # Set labels and legend
        plt.title('Solar System Simulation')
        plt.xlabel('X Position (AU)')
        plt.ylabel('Y Position (AU)')
        plt.legend(loc='upper right')
        
        # Ensures display remains centred based on max radius
        max_radius = max(body.orbital_radius for body in self.body_list)
        plt.xlim(-max_radius*1.1, max_radius*1.1)
        plt.ylim(-max_radius*1.1, max_radius*1.1)
        
        plt.show()

    def plot_energy_conservation(self):
        """
        Plots kinetic, potential and total energy over time
        """
        plt.figure(figsize=(12, 6))
        
        # Convert time points to years
        time_years = np.array(self.time_points)
        
        # Plot energies
        plt.plot(time_years, self.kinetic_energies, label='Kinetic Energy')
        plt.plot(
            time_years, self.potential_energies, label='Potential Energy'
        )
        plt.plot(
            time_years, self.total_energies,
            label='Total Energy', linestyle='--'
        )
        
        # Formatting
        plt.title('Energy Conservation in Solar System Simulation')
        plt.xlabel('Time (years)')
        plt.ylabel('Energy (AU)')
        plt.legend()
        plt.grid(True)
        
        # Calculate and display energy variation
        initial_energy = self.total_energies[0]
        final_energy = self.total_energies[-1]
        energy_variation = abs(
            (final_energy - initial_energy) / initial_energy
        ) * 100
        
        plt.figtext(0.5, 0.01, 
                f"Total energy variation: {energy_variation:.2e}%", 
                ha="center", fontsize=10,
                bbox={"facecolor":"white", "alpha":0.5}
        )
        
        plt.show()

    def calc_PE(self):
        """
        Calculates Potential Energy, which is the sum of potential energies
        between each pair of bodies
        """
        total_potential = 0
        sun_position = self.body_list[0].position

        for body in self.body_list:
            for other_body in self.body_list:
                if np.array_equal(body.position, other_body.position):
                    continue

                r_ij = body.position - other_body.position - sun_position
                r_mag = np.linalg.norm(r_ij)

                if r_mag > 0:
                    total_potential += (
                        self.grav_constant * body.mass * other_body.mass / r_mag
                    ) / -2
        return total_potential

    def calc_total_energy(self):
        """
        Sums the total potential and kinetic energy
        """
        total_potential = self.calc_PE()
        total_kintetic = 0

        for body in self.body_list:
            total_kintetic += body.calc_KE()

        total_energy = total_kintetic + total_potential

        return total_energy
    
    def record_energies(self, timestep):
        """
        Records kinetic, potential and total energy at each timestep
        """
        ke = 0
        for body in self.body_list:
            ke += body.calc_KE()
        pe = self.calc_PE()
        total = ke + pe
        
        self.time_points.append(timestep * self.timestep)
        self.kinetic_energies.append(ke)
        self.potential_energies.append(pe)
        self.total_energies.append(total)


class Body():

    def __init__(self, name, mass, orbital_radius, colour):
        self.timestep = parameters_solar['timestep']
        self.name = name
        self.mass = mass
        self.orbital_radius = orbital_radius
        self.colour = colour

        # each planet has initial position velocity and current/previous 
        # acceleration
        self.position = np.array([orbital_radius, 0], dtype=float)

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

        self.acceleration = np.zeros(2)
        self.next_acceleration = np.zeros(2)
        self.previous_acceleration = np.zeros(2)

        self.orbital_period = 0

    def update_position(self):
        """
        Updates position using current position and velocity, current and
        previous acceleration, and timestep
        """
        next_position = (
            self.position + (self.velocity * self.timestep) +
            (4 * self.acceleration - self.previous_acceleration) *
            self.timestep ** 2 / 6
        )
        return next_position
    
    def euler_cromer(self):
        """
        Updates velocity then uses new velocity to determine position, 
        Euler-Cromer method
        """
        # First update velocity using current acceleration
        next_velocity = self.velocity + self.acceleration * self.timestep
        # Then update position using new velocity
        next_position = self.position + next_velocity * self.timestep
        
        return next_velocity, next_position
    
    def direct_euler(self):
        """
        Updates position then velocity, Direct Euler method
        """
        next_position = self.position + self.velocity * self.timestep
        next_velocity = self.velocity + self.acceleration * self.timestep

        return next_position, next_velocity

    def update_velocity(self):
        """
        Updates the velocity of a particular body using previous, current, and
        next acceleration values, as well as current velocity and timestep
        """
        next_velocity = (
            self.velocity + (
                2 * self.next_acceleration
                + 5 * self.acceleration
                - self.previous_acceleration
            ) * self.timestep / 6
        )
        return next_velocity

    def calc_KE(self):
        """
        Calculates Kinetic Energy for a particular body
        """
        kinetic_energy = self.mass * (np.linalg.norm(self.velocity) ** 2)
        return kinetic_energy
    

def main():

    print(
        "Simulation of the Solar System using various numerical integration"
        " techniques."
        )

    print("Choose an integration method:")
    print("1. Beeman")
    print("2. Euler-Cromer")
    print("3. Direct Euler")
    
    # Get user input
    while True:
        choice = input("Enter your choice (1/2/3): ").strip()
        if choice == '1':
            method = "Beeman Method"
            break
        elif choice == '2':
            method = "Euler Cromer"
            break
        elif choice == '3':
            method = "Direct Euler"
            break
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")

    simulation = Simulation(method)
    simulation.run_simulation()
    simulation.visualise_orbits()
    simulation.plot_energy_conservation() 

main()

if __name__ == "__main__":
    main()



