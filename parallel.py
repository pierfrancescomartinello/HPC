from mpi4py import MPI
import numpy as np
from typing import Any, List, Callable, Tuple, Dict, Set
import math
import random
from copy import deepcopy
import time
import argparse
import csv
import glob
import os

Particle = List[Tuple[float, float, float]]

#Each task can access those functions
def scalar_product(v:List | Tuple,
                   scalar:int | float
)-> List | Tuple:
    t =list()
    for i in v:
        t.append(i*scalar)
    return tuple(t)

def dot_product(v:List | Tuple,
                scalar:int | float
)-> int | float:
    t =list()
    for i in v:
        t.append(i*scalar)
    return sum(t)

def axis_components(v:Tuple,
                    w:Tuple
)-> Tuple[float, float, float]:
    a = math.sqrt((v[0] -w[0])**2 + (v[1] -w[1]))   # ((x_i - x_j)^2 + (y_i - y_j))^(1/2)
    b = math.sqrt((v[0] -w[0])**2 + (v[2] -w[2]))   # ((x_i - x_j)^2 + (z_i - z_j))^(1/2)
    c = math.sqrt((v[1] -w[1])**2 + (v[2] -w[2]))   # ((y_i - y_j)^2 + (z_i - z_j))^(1/2)
    return a,b,c

def diff(v:Tuple,
         w:Tuple
)-> Tuple:
    t = list()
    for i in range(len(v)):
        t.append(v[i] - w[i])
    return tuple(t)

def rounding(v:Tuple)->Tuple:
    f = list()
    for i in v:
        f.append(round(i, 3))
    return tuple(f)

def add(v:Tuple,
        w:Tuple
)-> Tuple:
    t = list()
    for i in range(len(v)):
        t.append(v[i] + w[i])
    return tuple(t)

def collision_angle(deltas:list,v:Particle):
    try:
        f=sum(deltas[i] * abs(v[1][i]) for i in range(len(deltas)))
        g= sum(i for i in deltas)
        h=sum(abs(j) for j in v[1])
        if g*h == 0:
            if g == 0:
                g = 1
            elif h == 0:
                h = 1
            else:
                g = 1
                h = 1
        return math.acos(math.sqrt(f/(g*h if g != 0 else h)))
    except ZeroDivisionError:
        print("You want to divide by zero\n", deltas, v, f,g,h)
        return False

def velocity_calculation(one:Particle,
                         two:Particle
)-> Tuple[Particle, Particle]:
    



    (delta_x, delta_y, delta_z) = (abs(one[0][i] - two[0][i]) for i in range(3))
    TD_Angle_1 = collision_angle((delta_x, delta_y, delta_z),one)
    TD_Angle_2 = collision_angle((delta_x, delta_y, delta_z),two)
    v_center_one = scalar_product(one[1], math.cos(TD_Angle_1))
    v_center_two = scalar_product(two[1], math.cos(TD_Angle_2))
    v_normal_one = diff(one[1], v_center_one)
    v_normal_two = diff(two[1], v_center_two)
    v_center_one, v_center_two = v_center_two, v_center_one

    return rounding(add(v_normal_one, v_center_one)), rounding(add(v_normal_two, v_center_two))

def distance(one:Particle,
            two:Particle)->float:
    sum = 0
    for i in range(3):
        sum = (one[0][i] - two[0][i])**2
    return math.sqrt(sum)

def init():
    # Parsing time
    parser = argparse.ArgumentParser(
                            prog="Serial",
                            description="The serial version of the collision code",
                            epilog="Pacman Effect is real, at least in this code")


    parser.add_argument("-pn" , "--particle_number", action="store", required=True)
    parser.add_argument("-dt" , "--delta_t", action="store", required=True)
    parser.add_argument("-itn" , "--iteration_number", action="store", required=True)
    parser.add_argument("-r", "--range", action="store", required=True)
    parser.add_argument("-s", "--seed", required=False, default=False, action=argparse.BooleanOptionalAction)

    args = parser.parse_args()
    n = int(args.iteration_number)
    temporal_delta = float(args.delta_t )
    p = int(args.particle_number)
    max_range = int(args.range)

    if args.seed:
        random.seed(n*temporal_delta*max_range*p)
    
    return n, p, max_range, temporal_delta

# Only task zero can access this function wrapper
def rank_0_functions():
    def particles_within_delta(particles_dict: Dict[int, Particle], delta: float) -> List[Set[int]]:
        result: List[Set[int]] = []
        added_sets: Set[frozenset] = set()  # Use a set of frozensets to track added sets

        # Iterate over each particle in the dictionary
        for idx1, particle1 in particles_dict.items():
            # Create a set to store particles within delta of particle1
            nearby_particles: Set[int] = set()

            # Compare particle1 with every other particle in the dictionary
            for idx2, particle2 in particles_dict.items():
                # Calculate the distance between particle1 and particle2
                dist = distance(particle1, particle2)

                # If the distance is smaller than delta, add the index of particle2 to the set
                if dist < delta:
                    nearby_particles.add(idx2)

            # Convert the set to a frozenset for comparison
            nearby_particles_frozen = frozenset(nearby_particles)

            # Add the set of nearby particles to the result list if the set size is greater than 1
            if len(nearby_particles) > 1 and nearby_particles_frozen not in added_sets:
                result.append(list(nearby_particles))
                added_sets.add(nearby_particles_frozen)

        return result

    # Representation: dictionary of particles
    def random_particles(number_of_particles:int,
                         range_x:Tuple,
                         range_y:Tuple,
                         range_z:Tuple,
                         speed_x:float,
                         speed_y:float,
                         speed_z:float
    ) -> Dict[int, Particle]:
        part_dict:Dict[int, Particle] = dict()
        for i in range(1, number_of_particles +1):
            x,y,z = round(random.random() * (range_x[1] -range_x[0]) + range_x[0], 3), round(random.random() * (range_y[1] -range_y[0]) + range_y[0], 3), round(random.random() * (range_z[1] -range_z[0]) + range_z[0], 3)
            v_x, v_y, v_z = round(random.random()*speed_x - speed_x/2, 3), round(random.random()*speed_y - speed_y/2 , 3), round(random.random()*speed_z - speed_z/2,3)
            temp:Particle = [(x,y,z),(v_x,v_y,v_z)]
            part_dict[i] = temp
        return part_dict

    def coordinates_update(v:Particle,temporal_delta:float, max_range:int | float):
        new_coord = list()
        for i in range(len(v[0])):
            new_coord.append(((v[0])[i] + temporal_delta *(v[1])[i]) % max_range)
        return [tuple(new_coord), v[1]]

    def divide_list(lst, n):
        """
            Divide a list into n approximately equal portions.

            Args:
                lst (list): The list to be divided.
                n (int): The number of portions to divide the list into.

            Returns:
                list: A list of lists, each containing approximately equal portions of the original list.
        """
        # Calculate the approximate size of each portion
        size = len(lst) // n
        remainder = len(lst) % n

        # Initialize the start index for slicing
        start = 0

        # Initialize the list to store portions
        portions = []

        # Divide the list into portions
        for i in range(n):
            # Calculate the end index for slicing
            end = start + size + (1 if i < remainder else 0)

            # Append the portion to the list of portions
            portions.append(lst[start:end])

            # Update the start index for the next portion
            start = end

        return portions

    return particles_within_delta, random_particles, coordinates_update, divide_list



if __name__ == "__main__":
    # MPI startup
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    Particle = List[Tuple[float, float, float]]
    # Initialization of the command line arguments
    parsing_list = init()

    # This is done only one time at the beginning of the execution of task 0 
    if rank == 0: # Rank zero initiate the population of particles and sets up the logging
    
        #Gets the function name of those function only he can access
        particles_within_delta, random_particles, coordinates_update, divide_list = rank_0_functions()
        dict_reconstruction_time, g_time, vel_calc_time, tot_time , oth_task_time = 0,0,0,0,0
        #Calulates the random population of particle
        f = random_particles(parsing_list[1], (0,parsing_list[2]),(0,parsing_list[2]),(0,parsing_list[2]), 0.57,0.86,0.76)

        #Logging Stuff
        # filename = f'{os.getcwd()}/parallel/parallel--{size}-{parsing_list}.csv'
        # counter = len([file for file in glob.glob(filename[:-4] +"*" +".csv")])
        # if counter > 0:
        #     filename = filename[:-4] + f"({counter}).csv"
        # 
        # file = open(filename, "x")
        # file.close()
        # 
        # file = open(filename, "w+", newline="")
        # writer = csv.writer(file)
        # writer.writerow(["Iteration", "Total Time", "Clustering time", "Vel Calculation Time", "Dictionary Reconstruction Time", "Communication time", "Number of clusters", "Number of collisions"])
    
    
        start_time = time.time() # Starting time for each interaction
    for iteration in range(parsing_list[0]):
        
        if rank == 0: # Rank zero calculates the clusters and sends the correspective part to the other tasks
            g_start_time = time.time()
            g = particles_within_delta(f, 0.3) # Cluster calculation
            g_end_time = time.time()

            numb_of_collisions = sum([len(f) for f in g]) # Logging data 
            numb_of_clusters = len(g)
            g_end_time_for_sure = time.time()

            portions = divide_list(g, size) # The list of clusters is portioned and sent
            for index, i in enumerate(portions[1:]):
                comm.send((f, i), dest = index+1, tag = index+1+rank)

            g = deepcopy(portions[0])
        else: # Receives data from task 0
            (f,g) = comm.recv(source=0, tag=rank + 0)


        # Each task does this with its own portion of data
        vel_calc_start_time = time.time()
        for v in g:
            if len(v) == 2: # Simple collision between two particles
                temp1 , temp2 = v[0],v[1]
                v_1, v_2 = velocity_calculation(f[v[0]],f[v[1]])
                f[v[0]] = [f[temp1][0],v_1]
                f[v[1]] = [f[temp2][0],v_2]
                pass
            else: # In case multiple particles collide...
                sorted_dict = sorted(v, key=lambda x: f[x][1])
                for i in range(len(v) - 1):
                    temp1, temp2 = sorted_dict[i],sorted_dict[i+1]
                    v_1, v_2 = velocity_calculation(f[sorted_dict[i]],f[sorted_dict[i+1]])
                    f[sorted_dict[i]] = [f[temp1][0],v_1]
                    f[sorted_dict[i+1]] = [f[temp2][0],v_2]
        vel_calc_end_time = time.time()

        if rank != 0: # other tasks send their results to the task 0
            comm.send((f, vel_calc_end_time -vel_calc_start_time), dest=0, tag = rank + 0)
        else:
            communications = list()
            
            for i in range(1, size):
                (f,t) = comm.recv(source=i, tag= i+0)
                communications.append(f) # and the total amount of time they took
                oth_task_time += t
            communications.insert(0, deepcopy(f))

            f = dict()
            dict_reconstruction_start_time = time.time()
            for k in range(len(communications[0])):
                k_particles = [i[k+1] for i in communications] 
                # The velocity of each particle is the highest from what the tasks yielded
                get_fastest_velocity = sorted(k_particles, key= lambda x: (x[1][0]**2 + x[1][1]**2 + x[1][2]**2)**0.5)[0]
                # The coordinates are upgraded
                f[k+1] = coordinates_update(get_fastest_velocity, parsing_list[3], parsing_list[2])
            dict_reconstruction_end_time = time.time()

        # Miscellaneous logging stuff
    if rank == 0:
        end_time = time.time()
        dict_reconstruction_time += dict_reconstruction_end_time-dict_reconstruction_start_time
        g_time += g_end_time - g_start_time
        vel_calc_time += vel_calc_end_time -vel_calc_start_time
        tot_time +=  end_time- start_time
        print(oth_task_time)
            # writer.writerow([iteration+1, tot_time, g_time, vel_calc_time + oth_task_time, dict_reconstruction_time, tot_time - (dict_reconstruction_time +g_time+vel_calc_time+misc_time), numb_of_clusters, numb_of_collisions])

    if rank == 0: 
        # file.close()
        
        print("Total time, Clustering time, Velocity calculation time, Dict Reconstruction time, Other Time (communication mainly) ")
        print(tot_time, g_time, vel_calc_time + oth_task_time, dict_reconstruction_time, tot_time - (dict_reconstruction_time +g_time+vel_calc_time+oth_task_time))
