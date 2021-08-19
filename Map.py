from collections import namedtuple
from copy import deepcopy

from Graph import WeightedGraph
from Floor import Floor
from NavigationMesh import NavMesh
from Vector2D import Vec2D
from Constants import GlobalConstants


class Map:
    # Link ids can never be integers, so this should ensure no links are connected to them
    START = 1
    END = 0

    LinkTuple = namedtuple("LinkTuple", "vert_id link_num")
    LinkTuple2 = namedtuple("LinkTuple2", "link_num floor_num link_id")

    def __init__(self, data=None):
        # These dictionaries are used to keep track of which links are on which floors and share which IDs
        self.__id_indexed_nodes = dict()
        self.__floor_indexed_nodes = dict()
        self.__link_graph_id_indexed_nodes = dict()
        self.__reset_data = None

        self.floors = dict()
        self.nav_meshes = dict()
        self.link_graph = WeightedGraph()
        if data is None:
            self.floors[0] = Floor()
        else:
            floor_num = None
            for line_num in range(1, len(data)):
                line = data[line_num].split(" ")
                if len(line) == 0:
                    raise Exception("Invalid file")
                if line[0] == "F":
                    if len(line) > 1 and Map.__is_int(line[1]):
                        floor_num = int(line[1])
                        self.floors[floor_num] = Floor(Empty=True)
                    else:
                        raise Exception("Invalid file")
                elif line[0] == "L":
                    if floor_num is not None and len(line) > 3 and Map.__is_float(line[-2]) and Map.__is_float(
                            line[-1]):
                        self.floors[floor_num].add_link(" ".join(line[1:len(line)-2]), Vec2D(float(line[-2]), float(
                            line[-1])))
                    else:
                        raise Exception("Invalid file")
                elif line[0] == "V":
                    if floor_num is not None and len(line) > 2 and Map.__is_float(line[1]) and Map.__is_float(line[2]):
                        self.floors[floor_num].walls.add_vertex(Vec2D(float(line[1]), float(line[2])))
                    else:
                        raise Exception("Invalid file")
                elif line[0] == "W":
                    if (floor_num is not None and len(line) > 3 and Map.__is_float(line[1]) and
                            Map.__is_int(line[2]) and Map.__is_float(line[3]) and int(line[2]) <
                            self.floors[floor_num].walls.vertices and int(line[3]) <
                            self.floors[floor_num].walls.vertices):
                        self.floors[floor_num].walls.set_edge(int(line[2]), int(line[3]), float(line[1]))
                else:
                    raise Exception("Invalid file")

    def load_nav_graph_edits(self, data):
        floor_num = None
        for line_num in range(1, len(data)):
            line = data[line_num].split(" ")
            if len(line) == 0:
                raise Exception("Invalid file")
            else:
                if line[0] == "M":
                    if len(line) > 1 and Map.__is_int(line[1]):
                        floor_num = int(line[1])
                    else:
                        raise Exception("Invalid file")
                elif line[0] == "E":
                    if (floor_num is not None and len(line) > 3 and Map.__is_float(line[1]) and
                            Map.__is_int(line[2]) and Map.__is_float(line[3]) and int(line[2]) <
                            self.nav_meshes[floor_num].nav_graph.vertices and int(line[3]) <
                            self.nav_meshes[floor_num].nav_graph.vertices):
                        self.nav_meshes[floor_num].nav_graph.set_edge(int(line[2]), int(line[3]), float(line[1]),
                                                                      bi_directional=False,
                                                                      update_heuristic_scale=True)

    @staticmethod
    def __is_int(number):
        try:
            int(number)
            return True
        except ValueError:
            return False

    @staticmethod
    def __is_float(number):
        try:
            float(number)
            return True
        except ValueError:
            return False

    def get_save_data(self, store_nav_graphs=False):
        save_data = "MAP FILE"
        for floor_key in self.floors.keys():
            save_data += "\nF "+str(floor_key)
            for link in self.floors[floor_key].links:
                save_data += "\nL "+str(link.link_id)+" "+str(link.position.x)+" "+str(link.position.y)
            for vertex_pos in self.floors[floor_key].walls.vertex_positions:
                save_data += "\nV "+str(vertex_pos.x)+" "+str(vertex_pos.y)
            for vert1_id in range(self.floors[floor_key].walls.vertices):
                for vert2_id in range(vert1_id):
                    if self.floors[floor_key].walls.get_edge(vert1_id, vert2_id):
                        save_data += "\nW "+str(self.floors[floor_key].walls.get_edge_val(
                            vert1_id, vert2_id))+" "+str(vert1_id)+" "+str(vert2_id)
        if not store_nav_graphs:
            return save_data, None
        else:
            save_data2 = "NAV GRAPH FILE"
            for floor_key in self.nav_meshes.keys():
                save_data2 += "\nM " + str(floor_key)
                for vert1_id in range(self.nav_meshes[floor_key].nav_graph.vertices):
                    for vert2_id in range(self.nav_meshes[floor_key].nav_graph.vertices):
                        # Edge weights of -1 must be saved as well
                        # No point of storing weights that are just the default heuristic
                        if (self.nav_meshes[floor_key].nav_graph.get_edge_val(vert1_id, vert2_id) != 0 and
                                self.nav_meshes[floor_key].nav_graph.get_edge_val(vert1_id, vert2_id) !=
                                self.nav_meshes[floor_key].nav_graph.heuristic(vert1_id, vert2_id)):
                            save_data2 += "\nE "+str(self.nav_meshes[floor_key].nav_graph.get_edge_val(
                                vert1_id, vert2_id))+" "+str(vert1_id)+" "+str(vert2_id)
            return save_data, save_data2

    def generate_nav_graphs(self, master, verts_per_edge):
        # Generates the nav_graphs and every so often updates loading bar of main UI to show progress is being made
        # Floor nums are not necessarily in order and can be negative, so a second variable is used to keep track of
        # progress
        floors_done = 0
        for floor_num in self.floors.keys():
            self.nav_meshes[floor_num] = NavMesh(self.floors[floor_num].walls, self.floors[floor_num].links,
                                                 verts_per_edge)
            floors_done += 1
            if not master.nav_mesh_generation_update_prog(floor_num/len(self.floors.keys())):
                # Calls prev_stage and returns if cancelled
                master.prev_stage()
                return
        # Final check for if map splitting process has been cancelled
        if not master.nav_mesh_generation_update_prog(1):
            master.prev_stage()
            return
        master.next_stage()

    def join_links(self, precompute_link_dist, primary_algorithm, link_weight):
        self.__id_indexed_nodes = dict()
        self.__floor_indexed_nodes = dict()
        self.__link_graph_id_indexed_nodes = dict()
        self.link_graph = WeightedGraph()
        for floor_num in self.floors.keys():
            for link_num in range(len(self.floors[floor_num].links)):
                self.__add_link(floor_num, link_num, precompute_link_dist, primary_algorithm, link_weight)

    def __add_link(self, floor_num, link_num, precompute_link_dist, primary_algorithm, link_weight):
        # Each link must be joined in the link graph to all other links sharing the id (case 1) and sharing floors
        # (case 2)
        link = self.floors[floor_num].links[link_num]
        # Position on link graph is arbitrary - this is the only graph in program where positions don't make much sense
        # By using a y value of the floor number, the heuristic will aim to take floors towards the intended
        # floor, which seems logical for most real-life buildings, though I have not decided, as I write this
        # comment, whether I will allow the selection of heuristic-dependant algorithms for finding paths
        # through the link graph
        self.link_graph.add_vertex(Vec2D(0, floor_num))
        if link.link_id in self.__id_indexed_nodes.keys():
            # Case 1
            for prev_link_id in self.__id_indexed_nodes[link.link_id]:
                # Links with the same id are connected
                self.link_graph.set_edge(self.link_graph.vertices - 1, prev_link_id, link_weight)
        else:
            self.__id_indexed_nodes[link.link_id] = []
        if floor_num in self.__floor_indexed_nodes.keys():
            # Case 2
            for prev_link in self.__floor_indexed_nodes[floor_num]:
                # Links on the same floor so attempt to pathfind
                link_id = self.nav_meshes[floor_num].get_nav_graph_link_id(link_num)
                prev_link_id = self.nav_meshes[floor_num].get_nav_graph_link_id(prev_link.link_num)
                if precompute_link_dist:
                    # If distances are to be cached, then the path between the links must be found
                    if primary_algorithm == GlobalConstants.A_STAR:
                        returned = self.nav_meshes[floor_num].nav_graph.a_star(link_id, prev_link_id)
                        returned2 = self.nav_meshes[floor_num].nav_graph.a_star(prev_link_id, link_id)
                    elif primary_algorithm == GlobalConstants.DIJKSTRA:
                        returned = self.nav_meshes[floor_num].nav_graph.dijkstra(link_id, prev_link_id)
                        returned2 = self.nav_meshes[floor_num].nav_graph.dijkstra(prev_link_id, link_id)
                    elif primary_algorithm == GlobalConstants.GREEDY:
                        returned = self.nav_meshes[floor_num].nav_graph.greedy(link_id, prev_link_id)
                        returned2 = self.nav_meshes[floor_num].nav_graph.greedy(prev_link_id, link_id)
                    else:
                        raise Exception("Invalid primary algorithm.")
                    if returned is not None:
                        _, distances = returned
                        new_weight = distances[prev_link_id]
                        if (not self.link_graph.get_edge(self.link_graph.vertices - 1, prev_link.vert_id) or
                                new_weight < self.link_graph.get_edge_val(self.link_graph.vertices - 1,
                                                                          prev_link.vert_id)):
                            self.link_graph.set_edge(self.link_graph.vertices - 1, prev_link.vert_id, new_weight,
                                                     bi_directional=False)
                    if returned2 is not None:
                        _, distances = returned2
                        new_weight = distances[link_id]
                        if (not self.link_graph.get_edge(prev_link.vert_id, self.link_graph.vertices - 1) or new_weight
                                < self.link_graph.get_edge_val(prev_link.vert_id, self.link_graph.vertices - 1)):
                            self.link_graph.set_edge(prev_link.vert_id, self.link_graph.vertices - 1, new_weight,
                                                     bi_directional=False)
                else:
                    # Otherwise, heuristic is used as long as it is accessible and the heuristic weight is lower
                    # than the previous weight
                    if self.nav_meshes[floor_num].nav_graph.dfs(link_id)[prev_link_id]:
                        new_weight = self.nav_meshes[floor_num].nav_graph.heuristic(link_id, prev_link_id)
                        if (not self.link_graph.get_edge(self.link_graph.vertices - 1, prev_link.vert_id) or new_weight
                                < self.link_graph.get_edge_val(self.link_graph.vertices - 1, prev_link.vert_id)):
                            self.link_graph.set_edge(self.link_graph.vertices - 1, prev_link.vert_id, new_weight,
                                                     bi_directional=False)
                    if self.nav_meshes[floor_num].nav_graph.dfs(prev_link_id)[link_id]:
                        new_weight = self.nav_meshes[floor_num].nav_graph.heuristic(link_id, prev_link_id)
                        if (not self.link_graph.get_edge(prev_link.vert_id, self.link_graph.vertices - 1) or new_weight
                                < self.link_graph.get_edge_val(prev_link.vert_id, self.link_graph.vertices - 1)):
                            self.link_graph.set_edge(prev_link.vert_id, self.link_graph.vertices - 1, new_weight,
                                                     bi_directional=False)
        else:
            self.__floor_indexed_nodes[floor_num] = []
        self.__id_indexed_nodes[link.link_id].append(self.link_graph.vertices - 1)
        self.__floor_indexed_nodes[floor_num].append(self.LinkTuple(self.link_graph.vertices - 1, link_num))
        self.__link_graph_id_indexed_nodes[self.link_graph.vertices - 1] = self.LinkTuple2(link_num, floor_num,
                                                                                           link.link_id)

    def pathfind(self, start_pos, start_floor_num, end_pos, end_floor_num, primary_algorithm, link_algorithm,
                 precompute_link_dist, link_weight):
        # Finds a path between start_pos and end_pos by adding them as vertices to the link graph, finding the shortest
        # path on the link graph and then finding shortest paths on the nav_graph for everything in-between
        # Returns a dictionary of all paths on a floor so the path can be displayed
        final_paths = dict()
        self.floors[start_floor_num].add_link(self.START, start_pos)
        start_link_num = len(self.floors[start_floor_num].links)-1
        self.floors[end_floor_num].add_link(self.END, end_pos)
        end_link_num = len(self.floors[end_floor_num].links) - 1
        # Reverting the nav meshes to their previous state is more trouble than it's worth tbh, so they are copied
        # Probably later I should add actual code for reverting them properly to cut down on memory usage, but for now
        # this works fine
        old_nav_mesh_copy1 = deepcopy(self.nav_meshes[start_floor_num])
        if end_floor_num == start_floor_num:
            old_nav_mesh_copy2 = old_nav_mesh_copy1
        else:
            old_nav_mesh_copy2 = deepcopy(self.nav_meshes[end_floor_num])
        old_floor_indexed_nodes = deepcopy(self.__floor_indexed_nodes)
        old_link_graph_id_indexed_nodes = deepcopy(self.__link_graph_id_indexed_nodes)
        self.nav_meshes[start_floor_num].add_link(start_pos)
        self.nav_meshes[end_floor_num].add_link(end_pos)
        # Link weight can equal 0 as the start and end points are not actual links to be connected to anything, they are
        # simply placed inside the link graph for convenience with pathfinding
        self.__add_link(start_floor_num, start_link_num, precompute_link_dist, primary_algorithm, 0)
        self.__add_link(end_floor_num, end_link_num, precompute_link_dist, primary_algorithm, 0)
        if link_algorithm == GlobalConstants.BFS:
            returned = self.link_graph.bfs(self.link_graph.vertices - 1, self.link_graph.vertices - 2)
        elif link_algorithm == GlobalConstants.DIJKSTRA:
            returned = self.link_graph.dijkstra(self.link_graph.vertices - 1, self.link_graph.vertices - 2)
        elif link_algorithm == GlobalConstants.A_STAR:
            # A* or greedy are unlikely to work well as the heuristic will become meaningless, but I will
            # still include them as options here just in case
            returned = self.link_graph.a_star(self.link_graph.vertices - 1, self.link_graph.vertices - 2)
        elif link_algorithm == GlobalConstants.GREEDY:
            returned = self.link_graph.greedy(self.link_graph.vertices - 1, self.link_graph.vertices - 2)
        else:
            raise Exception("Invalid link algorithm set.")
        if returned is None:
            # Revert changes
            final_paths = None
        else:
            prev, _ = returned
            current_vert = self.link_graph.vertices - 2
            link_path = [current_vert]
            while link_path[-1] != self.link_graph.vertices - 1:
                current_vert = prev[current_vert]
                link_path.append(current_vert)
            for edge_num in range(len(link_path)-1):
                vert1_id = link_path[edge_num]
                vert2_id = link_path[edge_num + 1]
                if (self.__link_graph_id_indexed_nodes[vert1_id].floor_num == self.__link_graph_id_indexed_nodes[
                        vert2_id].floor_num and not (self.__link_graph_id_indexed_nodes[vert1_id].link_id ==
                                                     self.__link_graph_id_indexed_nodes[vert2_id].link_id and
                                                     self.link_graph.get_edge_val(vert1_id, vert2_id) == link_weight)):
                    # Second check ensure an actual floor path is needed and the links are not just being moved between
                    floor_num = self.__link_graph_id_indexed_nodes[vert1_id].floor_num
                    # Get vertex nav graph ids
                    vert1_nav_graph_id = self.nav_meshes[floor_num].get_nav_graph_link_id(
                        self.__link_graph_id_indexed_nodes[vert1_id].link_num)
                    vert2_nav_graph_id = self.nav_meshes[floor_num].get_nav_graph_link_id(
                        self.__link_graph_id_indexed_nodes[vert2_id].link_num)
                    # These paths must succeed as have already been precomputed as valid with the link graph
                    if primary_algorithm == GlobalConstants.A_STAR:
                        prev, _ = self.nav_meshes[self.__link_graph_id_indexed_nodes[
                            vert1_id].floor_num].nav_graph.a_star(vert2_nav_graph_id, vert1_nav_graph_id)
                    elif primary_algorithm == GlobalConstants.DIJKSTRA:
                        prev, _ = self.nav_meshes[self.__link_graph_id_indexed_nodes[
                            vert1_id].floor_num].nav_graph.dijkstra(vert2_nav_graph_id, vert1_nav_graph_id)
                    elif primary_algorithm == GlobalConstants.GREEDY:
                        prev, _ = self.nav_meshes[self.__link_graph_id_indexed_nodes[
                            vert1_id].floor_num].nav_graph.greedy(vert2_nav_graph_id, vert1_nav_graph_id)
                    else:
                        raise Exception("Invalid primary algorithm.")
                    current_vert = vert1_nav_graph_id
                    floor_path = [current_vert]
                    while floor_path[-1] != vert2_nav_graph_id:
                        current_vert = prev[current_vert]
                        floor_path.append(current_vert)
                    # In some cases, multiple paths can be on a floor, so to ensure they stay unique, edge_num is used
                    # as part of the key
                    final_paths[(edge_num, floor_num)] = floor_path
        # Finally, return everything back to how it was
        self.floors[start_floor_num].links.pop()
        self.floors[end_floor_num].links.pop()
        for _ in range(2):
            self.link_graph.delete_vertex(self.link_graph.vertices - 1)
        self.__id_indexed_nodes.pop(self.START)
        self.__id_indexed_nodes.pop(self.END)
        self.__floor_indexed_nodes = old_floor_indexed_nodes
        self.__link_graph_id_indexed_nodes = old_link_graph_id_indexed_nodes
        self.__reset_data = (start_floor_num, end_floor_num, old_nav_mesh_copy1, old_nav_mesh_copy2)
        return final_paths

    def reset_nav_meshes(self):
        # Nav mesh resetting must be delayed so that the path can be drawn on screen using the new start and goal
        # position links
        # (The canvas updating code could be modified to know to use the start and end position locations if the
        # index is out of range, but this method is easier).
        if self.__reset_data is not None:
            self.nav_meshes[self.__reset_data[0]] = self.__reset_data[2]
            self.nav_meshes[self.__reset_data[1]] = self.__reset_data[3]
            self.__reset_data = None
