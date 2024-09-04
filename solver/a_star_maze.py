import math
import heapq
import matplotlib.pyplot as plt
import shapely
from shapely.geometry import Point, Polygon
import numpy as np


class Maze:
    def __init__(self, walls, tPoint = None, maze_width = None, maze_height = None, concave_record = None, graph = None, theory_map = None):
        if len(walls[0][0]) == 3:
            self.walls = [[twoD[:2] for twoD in threeD] for threeD in walls]
        elif len(walls[0][0]) == 2:
            self.walls = walls
        else:
            print("Error: Obstacles size wired")
            return
        
        if tPoint is None:
            self.tPoint = []
        else:
            self.tPoint = tPoint[:2]
            
        if maze_width is None:
            self.l_maze_width = -4
            self.r_maze_width = 4
        else: 
            self.l_maze_width = maze_width[0]
            self.r_maze_width = maze_width[1]
            
        if maze_height is None:
            self.d_maze_height = -4
            self.u_maze_height = 4
        else: 
            self.d_maze_height = maze_height[0]
            self.u_maze_height = maze_height[1]
    

        if concave_record is None:
            self.concave_record = self.record_index()
        else:
            self.concave_record = concave_record

        if graph is None:
            self.basic_graph , self.basic_theory_map = self.generate_graph()
            self.graph = self.basic_graph
            self.theory_map = self.basic_theory_map
        else:
            self.basic_graph = graph
            self.basic_theory_map = theory_map

        self.curGraph = None
        self.curMap = None
        self.path = []
        self.sPath = self.path 


        if self.tPoint:
            self.set_target(tPoint[:2])

    def euclidean_distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)


    def visualize_walls(self):
        coor_x = []
        coor_y = []
        for i in range(len(self.walls)):
            coor_x.append([point[0] for point in self.walls[i]])
            coor_y.append([point[1] for point in self.walls[i]])
            if (self.walls[i][0][0] > self.l_maze_width and
                self.walls[i][0][0] < self.r_maze_width - 1 and
                self.walls[i][0][1] > self.d_maze_height and
                self.walls[i][0][1] < self.u_maze_height - 1 and
                self.walls[i][-1][0] > self.l_maze_width and
                self.walls[i][-1][0] < self.r_maze_width - 1 and
                self.walls[i][-1][1] > self.d_maze_height and
                self.walls[i][-1][1] < self.u_maze_height - 1):
                coor_x[i].append(self.walls[i][0][0])
                coor_y[i].append(self.walls[i][0][1])

        for j in range(len(self.walls)):
            plt.plot(coor_x[j], coor_y[j], color = "grey", linewidth=2.5)
            # plt.fill(coor_x[j], coor_y[j], color="m", alpha=0.8)


    def visualize_map(self, var = 1):    #var == 1(with start) / 0(without start)
        coor_x = []
        coor_y = []
        if var == 1:
            map = self.curMap if self.curMap else self.theory_map
            for i in range(len(map)):
                coor_x.append([point[0] for point in map[i]])
                coor_y.append([point[1] for point in map[i]])
            for j in range(len(coor_x)):
                plt.plot(coor_x[j], coor_y[j], color = "lightblue", linewidth=2)
        if var == 0:
            map = self.theory_map
            for i in range(len(map)):
                coor_x.append([point[0] for point in map[i]])
                coor_y.append([point[1] for point in map[i]])
            for j in range(len(coor_x)):
                plt.plot(coor_x[j], coor_y[j], color = "lightblue", linewidth=2)


    def visualize_path(self,switch = 0):
        if switch == 1:
            self.sPath = self.path
        coor_x = [point[0] for point in self.sPath]
        coor_y = [point[1] for point in self.sPath]
        plt.plot(coor_x, coor_y, color="r", linewidth = 2)
        for i in range(0, len(coor_x)-1):
            plt.arrow(coor_x[i],coor_y[i], coor_x[i+1] - coor_x[i], coor_y[i+1] - coor_y[i], head_width=(self.r_maze_width-self.l_maze_width)/150, head_length=(self.u_maze_height-self.d_maze_height)/150, linewidth=2, length_includes_head = True, linestyle='solid', color = 'r')


    def visualize_vector_field(self):
        coor_x = []
        coor_y = []
        if self.vector_field:
            keys = self.vector_field.keys()
            for key in keys:
                plt.arrow(key[0], key[1], self.vector_field[key][0][0], self.vector_field[key][0][1], head_width=(self.r_maze_width-self.l_maze_width)/200, head_length=(self.u_maze_height-self.d_maze_height)/200, length_includes_head = True, linestyle='solid', color = 'r')



    def visualize(self, var = 3):   
                                #  default:3   
                                ##  0: only walls;  
                                ##  1: walls and basic graph;
                                ##  2: walls and graph with start_point;
                                ##  3: walls and graph and path
                                ##  4: only walls and path
        plt.clf()
        plt.figure()
        plt.locator_params(axis='x', integer=True)
        plt.locator_params(axis='y', integer=True)
        xWall = [self.l_maze_width, self.r_maze_width, self.r_maze_width, self.l_maze_width, self.l_maze_width]
        yWall = [self.d_maze_height, self.d_maze_height, self.u_maze_height, self.u_maze_height, self.d_maze_height]
        plt.axis([self.l_maze_width, self.r_maze_width, self.d_maze_height, self.u_maze_height])
        plt.axis('equal')
        
        # elif var == 6:
        #     self.visualize_walls()
        #     self.visualize_path()
        #     self.visualize_vector_field()
        
        # elif var == 5:
        #     self.visualize_walls()
        #     self.visualize_vector_field()
        if var == 4:
            self.visualize_walls()
            self.visualize_map()
            self.visualize_path(1)
        elif var == 3:
            self.visualize_walls()
            self.visualize_map()
            self.visualize_path()
        elif var == 2:
            self.visualize_walls()
            self.visualize_path()
        elif var == 1:
            self.visualize_walls()
            self.visualize_map(0)
        elif var == 0:
            self.visualize_walls()
        plt.plot(xWall,yWall,color = 'k', linewidth = 2.8)
            
        plt.show()


            

    def generate_graph(self):
        theory_map = []
        graph = {}
        for i in range(0,len(self.walls)):
            
            if (self.walls[i][0][0] > self.l_maze_width and
                self.walls[i][0][0] < self.r_maze_width and
                self.walls[i][0][1] > self.d_maze_height and
                self.walls[i][0][1] < self.u_maze_height and
                self.walls[i][-1][0] > self.l_maze_width and
                self.walls[i][-1][0] < self.r_maze_width and
                self.walls[i][-1][1] > self.d_maze_height and
                self.walls[i][-1][1] < self.u_maze_height and 
                0 not in self.concave_record[i] and 
                len(self.walls[i])-1 not in self.concave_record[i]):       #if the walls has no connection with the boundary, connect the first wall point and the last wall point
                
                theory_map.append([self.walls[i][-1], self.walls[i][0]])
                distance = self.euclidean_distance(self.walls[i][-1],self.walls[i][0])

                if tuple(self.walls[i][0]) in graph:
                    if (tuple(self.walls[i][-1]), distance) not in graph[tuple(self.walls[i][0])]:
                        graph[tuple(self.walls[i][0])].append((tuple(self.walls[i][-1]), distance))
                else:
                    graph[tuple(self.walls[i][0])] = [(tuple(self.walls[i][-1]), distance)]
                if tuple(self.walls[i][-1]) in graph:
                    if (tuple(self.walls[i][0]), distance) not in graph[tuple(self.walls[i][-1])]:
                        graph[tuple(self.walls[i][-1])].append((tuple(self.walls[i][0]), distance))
                else:
                    graph[tuple(self.walls[i][-1])] = [(tuple(self.walls[i][0]), distance)]
                
            for j in range(0,len(self.walls[i])):

                if i == len(self.walls)-1 and j == len(self.walls[i])-1:
                    return graph, theory_map
                if j in self.concave_record[i]:
                    continue

                if (self.walls[i][j][0] <= self.l_maze_width or 
                    self.walls[i][j][0] >= self.r_maze_width or 
                    self.walls[i][j][1] <= self.d_maze_height or 
                    self.walls[i][j][1] >= self.u_maze_height):
                    continue   
                else:
                    if j == len(self.walls[i])-1:
                        k = i+1
                    if j < len(self.walls[i])-1:
                        if (self.walls[i][j][0] > self.l_maze_width and 
                            self.walls[i][j][0] < self.r_maze_width and 
                            self.walls[i][j][1] > self.d_maze_height and 
                            self.walls[i][j][1] < self.u_maze_height and 
                            self.walls[i][j + 1][0] > self.l_maze_width and 
                            self.walls[i][j + 1][0] < self.r_maze_width and 
                            self.walls[i][j + 1][1] > self.d_maze_height and 
                            self.walls[i][j + 1][1] < self.u_maze_height and 
                            int(j + 1) not in self.concave_record[i]):
                            theory_map.append([self.walls[i][j], self.walls[i][j+1]])
                            distance = self.euclidean_distance(self.walls[i][j],self.walls[i][j+1])
                            if tuple(self.walls[i][j]) in graph:
                                if (tuple(self.walls[i][j+1]), distance) not in graph[tuple(self.walls[i][j])]:
                                    graph[tuple(self.walls[i][j])].append((tuple(self.walls[i][j+1]), distance))
                            else:
                                graph[tuple(self.walls[i][j])] = [(tuple(self.walls[i][j+1]), distance)]

                            if tuple(self.walls[i][j+1]) in graph:
                                if (tuple(self.walls[i][j]), distance) not in graph[tuple(self.walls[i][j+1])]:
                                    graph[tuple(self.walls[i][j+1])].append((tuple(self.walls[i][j]), distance))
                            else:
                                graph[tuple(self.walls[i][j+1])] = [(tuple(self.walls[i][j]), distance)]
                        k = i
                    
                for m in range(k, len(self.walls)):  #only detect forward, no need to detect backward
                    if m == i:
                        l = j+1
                    else:
                        l = 0
                    for n in range(l,len(self.walls[m])):
                        if n in self.concave_record[m]:
                            continue
                        if (self.walls[m][n][0] == self.l_maze_width or 
                            self.walls[m][n][0] == self.r_maze_width or 
                            self.walls[m][n][1] == self.d_maze_height or 
                            self.walls[m][n][1] == self.u_maze_height):
                            continue
                        point1 = self.walls[i][j]
                        point2 = self.walls[m][n]
                        if self.check_mapLine(point1, point2):
                            theory_map.append([point1, point2])
                            distance = self.euclidean_distance(point1,point2)

                            if tuple(point1) in graph:
                                if (tuple(point2), distance) not in graph[tuple(point1)]:
                                    graph[tuple(point1)].append((tuple(point2), distance))
                            else:
                                graph[tuple(point1)] = [(tuple(point2), distance)]
                            if tuple(point2) in graph:
                                if (tuple(point1), distance) not in graph[tuple(point2)]:
                                    graph[tuple(point2)].append((tuple(point1), distance))
                            else:
                                graph[tuple(point2)] = [(tuple(point1), distance)]
        return graph, theory_map

    def cross_product(self, vec0, vec1):
        return vec0[0] * vec1[1] - vec0[1] * vec1[0]
            
    def find_concave_points(self, polygon):
        concave_index = []
        n = len(polygon)
        for i in range(n-2):
            p0 = polygon[i]
            p1 = polygon[i+1]
            p2 = polygon[i+2]
            
            vec0 = [p1[0]-p0[0],p1[1]-p0[1]]
            vec1 = [p2[0]-p1[0],p2[1]-p1[1]]
            if self.cross_product(vec0, vec1) > 0:
                concave_index.append(i+1)
        return concave_index
    


    def record_index(self):
        index_record = [[] for i in range(len(self.walls))]
        for i in range(0,len(self.walls)):
            index_record[i] = self.find_concave_points(self.walls[i])
        return index_record

    def check_repeat(self):
        count = 0
        repeat_loc = []
        for i in range(len(self.theory_map)-1):
            for j in range(i+1, len(self.theory_map)):
                if self.theory_map[i][0] == self.theory_map[j][0] and self.theory_map[i][1] == self.theory_map[j][1]:
                    count += 1
                    repeat_loc.append([i,j])
                elif self.theory_map[i][0] == self.theory_map[j][1] and self.theory_map[i][1] == self.theory_map[j][0]:
                    count += 1
                    repeat_loc.append([i,j])
        print('count =',count,'repeat_loc =',repeat_loc)
        return count, repeat_loc



    def set_start(self, point):
        point = point[:2]
        sGraph = {}
        sMap = []
        for i in range(len(self.walls)):
            for j in range(len(self.walls[i])):
                if (self.walls[i][j][0] <= self.l_maze_width or 
                    self.walls[i][j][0] >= self.r_maze_width or 
                    self.walls[i][j][1] <= self.d_maze_height or 
                    self.walls[i][j][1] >= self.u_maze_height):
                    continue
                if j in self.concave_record[i]:
                    continue
                elif self.check_mapLine(point, self.walls[i][j]):
                    # sMap.append([point, self.walls[i][j]])
                    distance = self.euclidean_distance(point,self.walls[i][j])

                    if tuple(point) in sGraph:
                        if (tuple(self.walls[i][j]), distance) not in sGraph[tuple(point)]:
                            sGraph[tuple(point)].append((tuple(self.walls[i][j]), distance))
                    else:
                        sGraph[tuple(point)] = [(tuple(self.walls[i][j]), distance)] 

                    # if tuple(self.walls[i][j]) in self.graph:
                    #     if (tuple(point), distance) not in self.graph[tuple(self.walls[i][j])]:
                    #         self.graph[tuple(self.walls[i][j])].append((tuple(point), distance))
                    # else:
                    #     self.graph[tuple(self.walls[i][j])] = [(tuple(point), distance)]
        if self.check_mapLine(point,self.tPoint):
            distance_t = self.euclidean_distance(point, self.tPoint)
            if tuple(point) in sGraph:
                if (tuple(self.tPoint), distance_t) not in sGraph[tuple(point)]:
                    sGraph[tuple(point)].append((tuple(self.tPoint), distance_t))
            else:
                sGraph[tuple(point)] = [(tuple(self.tPoint), distance_t)] 

        self.curGraph =  {**self.graph, **sGraph}
        # self.curMap = self.theory_map + sMap

    def set_target(self, point):
        point = point[:2]
        self.graph = self.basic_graph
        self.theory_map = self.basic_theory_map
        self.tPoint = point
        for i in range(len(self.walls)):
            for j in range(len(self.walls[i])):
                if (self.walls[i][j][0] == self.l_maze_width or 
                    self.walls[i][j][0] == self.r_maze_width or 
                    self.walls[i][j][1] == self.d_maze_height or 
                    self.walls[i][j][1] == self.u_maze_height):
                    continue
                if j in self.concave_record[i]:
                    continue
                if self.check_mapLine(point, self.walls[i][j]):
                    distance = self.euclidean_distance(point, self.walls[i][j])

                    if tuple(point) in self.graph:
                        if (tuple(self.walls[i][j]), distance) not in self.graph[tuple(point)]:
                            self.graph[tuple(point)].append((tuple(self.walls[i][j]), distance))
                    else:
                        self.graph[tuple(point)] = [(tuple(self.walls[i][j]), distance)] 
                    
                    if tuple(self.walls[i][j]) in self.graph:
                        if (tuple(point), distance) not in self.graph[tuple(self.walls[i][j])]:
                            self.graph[tuple(self.walls[i][j])].append((tuple(point), distance))
                    else:
                        self.graph[tuple(self.walls[i][j])] = [(tuple(point), distance)]


    def check_mapLine(self, point1, point2):    #if between point1 and point2 there is no barria, return True
        check_line = shapely.geometry.LineString([point1, point2])
        polygons = []
        for i in range(len(self.walls)):
            polygons.append(shapely.geometry.Polygon(self.walls[i]))
        intersection_points = []  # Initialize a list to store intersection points

        for polygon in polygons:
            intersection = check_line.intersection(polygon)
            
            if intersection.is_empty:
                continue
            elif intersection.geom_type == 'Point':
                point = [intersection.x, intersection.y]
                if point[0] == point1[0] and point[1] == point1[1]:
                    intersection_points.insert(0,point)
                elif point[0] == point2[0] and point[1] == point2[1]:
                    intersection_points.append(point)
                else: 
                    return False
            elif intersection.geom_type == 'MultiPoint':
                points_list = [list(list(point.coords)[0]) for point in intersection.geoms]
                if len(points_list) == 2:
                    if points_list[0] == point1 and points_list[1] == point2:
                        intersection_points.append(points_list[0])
                        intersection_points.append(points_list[1])
                    elif points_list[0] == point2 and points_list[1] == point1:
                        intersection_points.append(points_list[1])
                        intersection_points.append(points_list[0])
                    else:
                        return False
                else:
                    return False

            else:
                return False
                
        if intersection_points == [point1, point2] or intersection_points == [point2] or intersection_points == [point1] or intersection_points == []:
            # print('intersection_points = ', intersection_points)
            return True

    def A_star(self, start_state):  #(start_state) is tuple
        start_state = start_state[:2]
        if not isinstance(start_state, tuple):
            start_state = tuple(start_state)
        self.set_start(list(start_state))   #(start_state) is tuple
        start_node = Node(state=start_state, cost=0, heuristic=0)
        frontier = [start_node]
        heapq.heapify(frontier)
        explored = set()

        while frontier:
            current_node = heapq.heappop(frontier)
            if current_node.state == tuple(self.tPoint):
                return current_node
            explored.add(current_node.state)
            for neighbor, cost in self.curGraph[current_node.state]:
                if neighbor not in explored:
                    new_cost = current_node.cost + cost
                    new_node = Node(state=neighbor, parent=current_node, action=None,
                                    cost=new_cost, heuristic=self.euclidean_distance(neighbor, tuple(self.tPoint)))
                    in_frontier = any(node.state == neighbor for node in frontier)           
                    if not in_frontier:
                        heapq.heappush(frontier, new_node)
                    else:
                        for node in frontier:
                            if node.state == neighbor:
                                if new_cost < node.cost:
                                    node.cost = new_cost
                                    node.parent = current_node
                                    node.total_cost = new_cost + node.heuristic
                                    heapq.heapify(frontier)
                                    break
        return None


    def A_star_path(self, start_state, target_state = None):   #input: tuple
        if target_state is not None:
            self.set_target(target_state)
        result_node = self.A_star(start_state)
        if result_node:
            path = []
            while result_node:
                path.append(result_node.state)
                result_node = result_node.parent
            path.reverse()
            # print("Path found:", path)
            self.path = path
        else:
            print("No path found.")

#-----------------------------------------------------------------------#


    def locate_point(self, i, point):
        for j in range(len(self.walls[i])):
            if self.walls[i][j] == point:
                return j
        return False



    def spread_operation(self, r = 0.2, origin_cplist = 0):
        if origin_cplist == 0:
            self.sPath = self.path
        for j in range(1,len(self.sPath)-1):
            path_point = list(self.sPath[j])
            new_point = Point(path_point)
            for i in range(len(self.walls)):
                new_polygon = Polygon(self.walls[i])
                if new_polygon.touches( new_point ):
                    id = None
                    id = self.locate_point(i , path_point)
                    if id is not None:
                        if id == 0:
                            p1 = self.walls[i][-1]
                        else:
                            p1 = self.walls[i][id-1]
                        p2 = self.walls[i][id]
                        if id == len(self.walls[i])-1:
                            p3 = self.walls[i][0]  
                        else:
                            p3 = self.walls[i][id+1]
                        v1 = np.array(p2) - np.array(p1)
                        v1 = v1/np.linalg.norm(v1)
                        v2 = np.array(p3) - np.array(p2)
                        v2 = v2/np.linalg.norm(v2)
                        v_sum = v1+v2
                        v_sum_unit = v_sum / np.linalg.norm(v_sum)
                        v_perp = np.array([-v_sum_unit[1], v_sum_unit[0]])
                        path_point = p2 + r * v_perp
                    self.sPath[j] = tuple(path_point)

    def update_cpList(self,r):
        loop = 1
        while loop == 1:
            loop = 0
            i = 0
            record_sPath = []
            for j in range(len(self.sPath)):
                record_sPath.append(self.sPath[j])
            while i < len(record_sPath)-1:
                # print('i:',i,'len(a)',len(self.sPath))
                # print(self.sPath[i])
                if self.check_mapLine(record_sPath[i],record_sPath[i+1]) == False:
                    self.set_target(record_sPath[i+1])
                    self.A_star_path(record_sPath[i])
                    self.spread_operation(r)
                    assert len(self.sPath) > 2
                    k = i
                    for new_pathpoint in  self.sPath[1:-1]:
                        record_sPath.insert(k+1,new_pathpoint)
                        k += 1
                    loop == 1
                # if self.sPath[i] == 2:
                #     self.sPath.insert(i+1,2.5)
                # if self.sPath[i] == 2.5:
                #     self.sPath.insert(i+1,2.7)
                i += 1
        self.sPath = record_sPath
                
    def generate_cpList(self,r = 0.2):
        self.spread_operation(r)
        self.update_cpList(r)
        # print(self.sPath)
#------------------------------------------------------------------------------------------------------------------------

class Node:
    def __init__(self, state, parent=None, action=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    def __lt__(self, other):
        return self.total_cost < other.total_cost





# walls0 = [
#     [
#         [0.508064516129032, 0.560483870967742],
#         [0.48387096774193505, 3.512096774193548],
#         [1.008064516129032, 3.463709677419354],
#         [0.9919354838709671, 1.56854838709677],
#         [1.943548387096774, 1.576612903225806],
#         [2.0, 1.020161290322581],
#         [1.008064516129032, 1.012096774193549],
#         [1.008064516129032, 0.600806451612903]
#     ],
#     [
#         [1.370967741935483, 0.657258064516129],
#         [2.61290322580645, 0.955645161290323],
#         [2.411290322580644, 3.915322580645162],
#         [2.975806451612902, 3.875],
#         [3.201612903225806, 0.6088709677419359],
#         [1.39516129032258, 0.30241935483870996]
#     ],
#     [
#         [1.42741935483871, 2.681451612903226],
#         [2.153225806451612, 2.69758064516129],
#         [2.17741935483871, 2.30241935483871],
#         [1.4112903225806441, 2.310483870967742],
#     ]
# ]
# mazeCur = Maze(walls0,[1.7822580645161281,0.125],[0,4],[0,4])
# # mazeCur.visualize(1)
# mazeCur.A_star_path((1.903225806451612,3.65725806451613))
# mazeCur.spread_operation()
# # mazeCur.visualize(3)
# graph0 = mazeCur.basic_graph
# theory_map0 = mazeCur.theory_map
# concave_record0 = mazeCur.concave_record
# mazeTest = Maze(walls0,[-2,2], concave_record=concave_record0, graph=graph0, theory_map=theory_map0)
# mazeTest.A_star_path((3.8, 3))
# # mazeTest.visualize(3)
# print(mazeTest.sPath)
# mazeTest.spread_operation()
# print(mazeTest.sPath)
# plt.show()

# walls = [
#     [[0, 5], [7, 5], [6, 2], [5, 2], [6, 4], [0, 4]],
#     [[0, 14], [10, 23], [11, 23],[0, 13]],
#     [[9, 0], [9, 17], [15, 17], [15, 7], [14, 7], [14, 16], [10, 16], [10, 0]],
#     [[14, 25], [14, 21], [21, 21], [21, 16], [20, 16], [20, 20], [13, 20], [13, 25]],
#     [[22, 0], [16, 12], [23, 12], [23, 23], [15, 23], [15, 24], [24, 24], [24, 11], [17, 11], [23, 0]],
#     [[25, 0], [25, 13], [32, 13], [32, 7], [28, 7],[28, 8],[31, 8],[31, 12],[26, 12],[26, 0]],
#     [[35, 3], [29, 3], [29, 4], [35, 4]],
#     [[35, 16], [27, 16], [26, 19], [27, 19], [28, 17],[35, 17]],
#     [[31, 25], [28, 22], [33, 22], [33, 19], [32, 19], [32, 21], [27, 21], [25, 22], [30, 25]]
# ]
# mazeCur = Maze(walls, 36,26,[12,14])
# mazeCur.A_star_path((20,8))


# walls = [
#     [[0, 5], [4, 5], [4, 8], [3, 8], [3, 6], [2, 6], [2, 9], [5, 9], [5, 4], [0, 4]],
#     [[3, 25], [3, 19], [5, 19], [5, 23], [8, 23], [8, 16], [7, 16], [7, 22], [6, 22], [6, 18], [2, 18], [2, 25]],
#     [[25, 21], [13, 21], [13, 23], [10, 23], [10, 17], [12, 17], [12, 11], [4, 11], [4, 16], [5, 16], [5, 12], [11, 12], [11, 16], [9, 16], [9, 24], [14, 24], [14, 22], [25, 22]],
#     [[13, 0], [13, 9], [8, 9], [8, 4], [11, 4], [11, 3], [7, 3], [7, 10], [14, 10], [14, 0]],
#     [[15, 0], [18, 4], [16, 7], [16, 8], [15,9],[16,10], [18,9 ],[17, 7],[19, 4],[16, 0]],
#     [[25, 12], [19, 12], [19, 18], [15, 18], [15, 14], [20, 10], [20, 6], [19, 6], [19, 9], [14, 13], [14, 19], [20, 19], [20, 13], [25, 13]],
#     [[25, 16], [22, 16], [22, 19], [23, 19], [23, 17],[25, 17]],
#     [[25, 3], [22, 4], [22, 11], [23, 11], [23, 5],[25, 4]]
# ]
# mazeCur = Maze(walls, 26,26,[3.5,7])
# mazeCur.A_star_path((21,11))

# obstacles = [[[-2.419354838709678, -3.008064516129032], [-1.5000000000000018, -2.943548387096774], [-1.53225806451613, -0.5403225806451601], [-1.0806451612903238, -0.5403225806451601], [-1.048387096774194, -3.39516129032258], [-2.483870967741936, -3.42741935483871]], [[-1.5000000000000018, 2.217741935483872], [-0.9677419354838719, 2.266129032258064], [-1.03225806451613, 0.9758064516129039], [1.4838709677419342, 1.0887096774193559], [1.5322580645161281, -2.92741935483871], [1.016129032258064, -3.008064516129032], [1.016129032258064, 0.508064516129032], [-1.5161290322580658, 0.49193548387096797]], [[0.032258064516128115, 4.10483870967742], [0.4999999999999982, 4.10483870967742], [0.45161290322580605, 2.024193548387098], [0.06451612903225623, 2.088709677419356]], [[-3.048387096774194, 3.169354838709676], [-2.419354838709678, 3.5564516129032278], [-2.5000000000000018, -1.959677419354838], [-2.967741935483872, -2.056451612903226]]]
# mazeCur = Maze(obstacles,tPoint=[-2.004677419354838719,2.7258064516129])
# # mazeCur.visualize(3)
# mazeCur.A_star_path((2.0041935483870968,0.005967741935483986))



# # mazeCur.visualize(3)
# # mazeCur.spread_operation(0.5)
# mazeCur.generate_cpList(0.5)
# mazeCur.visualize(3)
# plt.show()


        # if var == 6:
        #     self.visualize_vector_field()
        #     self.visualize_walls()
        #     self.visualize_path()
        
        # elif var == 5:
        #     self.visualize_vector_field()
        #     self.visualize_walls()
        # elif var == 4:
        #     self.visualize_walls()
        #     self.visualize_path()
        # elif var == 3:
        #     self.visualize_walls()
        #     self.visualize_map()
        #     self.visualize_path()
        # elif var == 2:
        #     self.visualize_walls()
        #     self.visualize_path()
        # elif var == 1:
        #     self.visualize_walls()
        #     self.visualize_map(0)
        # elif var == 0:
        #     self.visualize_walls()

