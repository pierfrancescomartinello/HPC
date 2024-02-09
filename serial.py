from typing import Any, List, Callable, Tuple, Dict, Set
import math
import random
from copy import deepcopy
import time
import argparse
import csv
import glob
import os

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
n = int(args.particle_number)
temporal_delta = float(args.delta_t )
p = int(args.iteration_number)
max_range = int(args.range)

if args.seed:
    random.seed(n*temporal_delta*max_range*p)

# Particle = ((x,y,z), (v_x, v_y, v_z))
Particle = List[Tuple[float, float, float]]

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


filename = f'{os.getcwd()}/serial/serial--{n}-{p}-{str(temporal_delta).replace(".","+")}-{max_range}.csv'
counter = len([file for file in glob.glob(filename)])
if counter > 0:
    filename = filename[:-4] + f"({counter}).csv"

f = open(filename, "x")
f.close()

f = random_particles(p,(0,max_range),(0,max_range),(0,max_range),0.57,0.86,0.76)
with open(filename, "w+", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(['Iteration', 'Exec time for clustering', 'Exec time for vectors calculation', 'Exec time for collisions', "Number of clusters", "Numbers of collisions"])
    for iteration in range(n):
        g_start_time = time.time()
        g = particles_within_delta(f, 0.3)
        g_end_time = time.time()

        v_start_time = time.time()
        for v in g:
            if len(v) == 2:
                temp1 , temp2 = v[0],v[1]
                v_1, v_2 = velocity_calculation(f[v[0]],f[v[1]])
                f[v[0]] = [f[temp1][0],v_1]
                f[v[1]] = [f[temp2][0],v_2]
                pass
            else:
                sorted_dict = sorted(v, key=lambda x: f[x][1])
                for i in range(len(v) -1):
                    temp1, temp2 = sorted_dict[i],sorted_dict[i+1]
                    v_1, v_2 = velocity_calculation(f[sorted_dict[i]],f[sorted_dict[i+1]])
                    f[sorted_dict[i]] = [f[temp1][0],v_1]
                    f[sorted_dict[i+1]] = [f[temp2][0],v_2]

        v_end_time = time.time()

        c_start_time = time.time()

        for k,v in f.items():
            f[k] = coordinates_update(v, temporal_delta, max_range)

        c_end_time = time.time()
        colliding_particles = sum(len(v) for v in g)
        to_write = [iteration+1, round(g_end_time-g_start_time, 4), round(v_end_time -v_start_time,4), round(c_end_time -c_start_time), len(g), colliding_particles]

        writer.writerow(to_write)
