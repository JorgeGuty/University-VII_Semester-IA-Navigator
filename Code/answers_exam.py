from SearchAlgorithm import *
from SubwayMap import *
from utils import *

if __name__=="__main__":
    ROOT_FOLDER = '../CityInformation/Barcelona_City/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)



    ### BELOW HERE YOU CAN CALL ANY FUNCTION THAT yoU HAVE PROGRAMED TO ANSWER THE QUESTIONS OF THE EXAM ###
    ### this code is just for you, you won't have to upload it after the exam ###


    #this is an example of how to call some of the functions that you have programed
    example_path=uniform_cost_search(9, 3, map, 1)
    print_list_of_path_with_cost([example_path])

    print("2")
    c = coord2station([206,7],map)
    print(c)

    print("3")
    b = remove_cycles([Path([7,8,6,5,3]), Path([7,8,6,5,1]), Path([7,8,6,5,7]), Path([7,8,6,5,8]), Path([7,8,6,5,4])])
    print([path.h for path in b])

    print("4")
    a = breadth_first_search(10, 5, map)
    print(a.route)

   # print("6")
    #d = calculate_heuristics([Path([13, 11])], 12, map, 1)
    #print(d[0].g)

    print("7")
    d = calculate_cost([Path([9,10,23,22])], map, 1)
    print(d[0].g)


    print("9")
    da = Astar([111,194], [59,239], map, 3)
    print(da.route)

    print("10")
    df = Astar([143,195], [88,69], map, 3)
    print(df.g)


