# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1647904'
__group__ = 'DM.10'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2021 - 2022
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________
from types import NoneType

from SubwayMap import *
from utils import *
import os
import math
import copy


''''
PREGUNTAS:
en el documento se usa como head de la ruta el último de cada path, pero en la teoria el primero
coord2station dice index, no es id?
'''

def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """

    expanded_path = []
    head_of_path = path.last

    connections = map.connections.get(head_of_path)

    for stationId in connections.keys():

        new_path = Path(list(path.route))
        new_path.add_route(stationId)
        expanded_path.append(new_path)

    return expanded_path


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    cycles_removed_path_list = []

    for path in path_list:
        if len(path.route) == len(set(path.route)):
            cycles_removed_path_list.append(path)

    return cycles_removed_path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted

    """
    return expand_paths + list_of_path[1:]


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """

    paths = [Path(origin_id)]

    while paths != [] and paths[0].last != destination_id:

        head = paths[0]
        expanded_head = expand(head, map)
        cycle_free_expanded_head = remove_cycles(expanded_head)
        paths = insert_depth_first_search(cycle_free_expanded_head, paths)

    if len(paths) > 0:
        return paths[0]
    else:
        return []


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """

    return list_of_path[1:] + expand_paths


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    paths = [Path(origin_id)]

    while paths != [] and paths[0].last != destination_id:
        head = paths[0]
        expanded_head = expand(head, map)
        cycle_free_expanded_head = remove_cycles(expanded_head)
        paths = insert_breadth_first_search(cycle_free_expanded_head, paths)

    if len(paths) > 0:
        return paths[0]
    else:
        return []


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """

    if type_preference == 0:
        return calculate_adjacency_cost(expand_paths, map)

    elif type_preference == 1:
        return calculate_time_cost(expand_paths, map)

    elif type_preference == 2:
        return calculate_distance_cost(expand_paths, map)

    else:
        return calculate_transfers_cost(expand_paths, map)


def calculate_adjacency_cost(expand_paths, map):
    for path in expand_paths:
        path.g = (len(path.route) - 1)

    return expand_paths


def calculate_time_cost(expand_paths, map):

    for path in expand_paths:
        cost = 0
        for stop_index in range(len(path.route) - 1):
            current_stop = path.route[stop_index]
            next_stop = path.route[stop_index + 1]
            cost += map.connections.get(current_stop).get(next_stop)
        path.update_g(cost)

    return expand_paths


def calculate_distance_cost(expand_paths, map):
    ''' for path in expand_paths:

    current_station = path.last
    last_station = path.penultimate

    current_station_coord = [map.stations.get(current_station).get("x"), map.stations.get(current_station).get("y")]
    last_station_coord = [map.stations.get(last_station).get("x"), map.stations.get(last_station).get("y")]

    distance_cost = euclidean_dist(current_station_coord, last_station_coord)

    path.update_g(distance_cost)
    '''

    for path in expand_paths:
        last_station = path.last
        penultimate_station = path.penultimate
        line_last = map.stations[path.last].get('line')
        line_penultimate = map.stations[path.penultimate].get('line')

        if line_last == line_penultimate:
            velocity = map.stations[path.penultimate].get("velocity")
            time = map.connections[penultimate_station][last_station]
            distance = velocity * time
            path.update_g(distance)

        elif map.stations[path.last].get('x') == map.stations[path.penultimate].get('x') and map.stations[
            path.last].get('y') == map.stations[path.penultimate].get('y'):
            path.update_g(0)

        else:
            g = map.connections[path.penultimate][path.last]
            path.update_g(g)

    return expand_paths


def calculate_transfers_cost(expand_paths, map):
    for path in expand_paths:
        if map.stations[path.last].get("x") == map.stations[path.penultimate].get("x") and map.stations[path.last].get(
                "y") == map.stations[path.penultimate].get("y"):
            path.update_g(1)
    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """

    head = get_lower_cost(list_of_path, False)
    list_of_path.remove(head)
    return expand_paths + list_of_path


def get_lower_cost(list_of_path, get_f_value):
    lower_cost_path = list_of_path[0]

    for path in list_of_path:
        cost_value = path.g if not get_f_value else path.f
        lower_cost_path_value = lower_cost_path.g if not get_f_value else path.f
        if cost_value < lower_cost_path_value:
            lower_cost_path = path

    return lower_cost_path


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    paths = [Path(origin_id)]

    while paths != [] and get_lower_cost(paths, False).last != destination_id:
        head = get_lower_cost(paths, False)
        expanded_head = expand(head, map)
        cycle_free_expanded_head = remove_cycles(expanded_head)
        cost_updated_expanded_head = calculate_cost(cycle_free_expanded_head, map, type_preference)
        paths = insert_cost(cost_updated_expanded_head, paths)

    if len(paths) > 0:
        return get_lower_cost(paths, False)
    else:
        return []


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    if type_preference == 0:
        return calculate_adjacency_heuristics(expand_paths, destination_id, map)

    elif type_preference == 1:
        return calculate_time_heuristics(expand_paths, destination_id, map)

    elif type_preference == 2:
        return calculate_distance_heuristics(expand_paths, destination_id, map)

    else:
        return calculate_transfers_heuristics(expand_paths, destination_id, map)


def calculate_adjacency_heuristics(expand_paths, destination_id, map):
    for path in expand_paths:
        if path.last != destination_id:
            path.update_h(1)
        else:
            path.update_h(0)

    return expand_paths


def calculate_time_heuristics(expand_paths, destination_id, map):

    # TODO: Heuristic to use: time to the goal station in a straight line from the current head (best possible time)

    destination_coordinates = [map.stations.get(destination_id).get("x"), map.stations.get(destination_id).get("y")]
    velocity = max(list(map.velocity.values()))
    for path in expand_paths:
        current_coordinates = [map.stations.get(path.last).get("x"), map.stations.get(path.last).get("y")]
        distance_to_destination = euclidean_dist(destination_coordinates, current_coordinates)
        heuristic_value = distance_to_destination / velocity
        path.update_h(heuristic_value)

    return expand_paths


def calculate_distance_heuristics(expand_paths, destination_id, map):
    for path in expand_paths:
        coord_destination = [map.stations[destination_id].get("x"), map.stations[destination_id].get("y")]
        coord_current = [map.stations[path.last].get("x"), map.stations[path.last].get("y")]
        path.update_h(euclidean_dist(coord_destination, coord_current))
    return expand_paths


def calculate_transfers_heuristics(expand_paths, destination_id, map):
    for path in expand_paths:
        if map.stations[destination_id].get("line") == map.stations[path.last].get("line"):
            path.update_h(0)
        else:
            path.update_h(1)
    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """

    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g in this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
    """

    for expanded_path in expand_paths:
        head = expanded_path.last
        pc = visited_stations_cost.get(head) if not isinstance(visited_stations_cost.get(head), NoneType) else float("inf")
        if expanded_path.g <= pc:

            # remove previously added path
            for path in list_of_path:
                if path.last == head:
                    list_of_path.remove(path)

            # update visited_stations_cost
            visited_stations_cost.update({head: expanded_path.g})
        else:
            expand_paths.remove(expanded_path)

    return expand_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    head = get_lower_cost(list_of_path, True)
    list_of_path.remove(head)
    return expand_paths + list_of_path


def coord2station(coord, map):
    """
        From coordinates, it searches the closest station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes ***?*** of stations, which corresponds to the closest station
    """

    stations = map.stations.items()
    current_minimum_distance = INF
    minimum_distance_stations = []

    for stationId, stationInfo in stations:

        distance_to_station = euclidean_dist(coord, [stationInfo.get("x"), stationInfo.get("y")])

        if distance_to_station < current_minimum_distance:
            current_minimum_distance = distance_to_station
            minimum_distance_stations = [stationId]

        elif distance_to_station == current_minimum_distance:
            minimum_distance_stations += [stationId]

    return minimum_distance_stations


def Astar(origin_coor, dest_coor, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (list): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    origin_id = coord2station(origin_coor, map)[0]
    destination_id = coord2station(dest_coor, map)[0]

    paths = [Path(origin_id)]
    pct = dict()

    while paths != [] and get_lower_cost(paths, True).last != destination_id:
        head = get_lower_cost(paths, True)
        expanded_head = expand(head, map)
        expanded_head = remove_cycles(expanded_head)
        expanded_head = calculate_cost(expanded_head, map, type_preference)
        expanded_head = calculate_heuristics(expanded_head, map, destination_id, type_preference)
        print(head.route, head.g, head.h)
        update_f(expanded_head)
        expanded_head, paths, pct = remove_redundant_paths(expanded_head, paths, pct)
        paths = insert_cost_f(expanded_head, paths)

    head = get_lower_cost(paths, True)
    print(head.route, head.g, head.h)

    if len(paths) > 0:
        return get_lower_cost(paths, True)
    else:
        return []

    pass