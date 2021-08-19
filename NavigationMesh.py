from Vector2D import Vec2D
from Graph import WeightedGraph
from copy import deepcopy
from SquareMatrix import SquareMat
from Floor import Floor


class NavMesh:

    THRESHOLD_DIST = 0.01 ** 2

    def __init__(self, walls, links, verts_per_edge):
        self.__num_of_links = 0
        self.__nav_mesh = self.__split(walls)
        self.__regions = self.__find_regions()
        self.__region_positions = self.__find_avg_region_positions()
        self.nav_graph, self.__edge_vert_ids = self.__create_nav_graph(verts_per_edge)
        self.__region_indexed_links = dict()
        self.__add_links_to_nav_graph(links)
        self.old_nav_graph = None

    def __add_links_to_nav_graph(self, links):
        # Adds the links between floors to the nav graph
        for link in links:
            self.add_link(link.position)

    def get_nav_graph_link_id(self, link_num):
        # Links are added last to the nav graph, so their id can be found by going backwards
        return self.nav_graph.vertices - self.__num_of_links + link_num

    def add_link(self, point):
        self.__num_of_links += 1
        self.nav_graph.add_vertex(point)
        sorted_regions_nums = self.__sort_region_nums_by_dist_from(point)
        for region_num in sorted_regions_nums:
            region = self.__regions[region_num]
            if Vec2D.point_in_polygon(point, [self.__nav_mesh.vertex_positions[vert_id] for vert_id in region]):
                # Join to links sharing region
                if region_num in self.__region_indexed_links.keys():
                    for vert_id in self.__region_indexed_links[region_num]:
                        self.nav_graph.set_edge(vert_id, self.nav_graph.vertices - 1, Vec2D.distance_between(
                            self.nav_graph.vertex_positions[vert_id], self.nav_graph.vertex_positions[
                                self.nav_graph.vertices - 1]))
                    self.__region_indexed_links[region_num].append(self.nav_graph.vertices - 1)
                else:
                    self.__region_indexed_links[region_num] = [self.nav_graph.vertices - 1]
                # Also join to links sharing edges
                for region_edge_num in range(len(region)):
                    vert1_id = region[region_edge_num]
                    # vert2_id must wrap back to 0 for the edge between the first and last vertices in the region
                    vert2_id = region[(region_edge_num + 1) % len(region)]
                    if self.__nav_mesh.get_edge_val(vert1_id, vert2_id) == 3:
                        # nav_mesh is symmetrical so swapping vert ids should give same key
                        if vert1_id > vert2_id:
                            dict_key = (vert1_id, vert2_id)
                        else:
                            dict_key = (vert2_id, vert1_id)
                        for vert_id in self.__edge_vert_ids[dict_key]:
                            self.nav_graph.set_edge(vert_id, self.nav_graph.vertices - 1, Vec2D.distance_between(
                                self.nav_graph.vertex_positions[vert_id], self.nav_graph.vertex_positions[
                                    self.nav_graph.vertices - 1]))
                # Link can only be in one region
                return

    def __sort_region_nums_by_dist_from(self, position):
        # Sorts the list of regions by their respective distances from position
        sorted_region_nums = list(range(len(self.__regions)))
        sorted_region_nums.sort(key=lambda region_num: Vec2D.squared_distance_between(position,
                                self.__region_positions[region_num]))
        sorted_region_nums = [region_num for region_num in sorted_region_nums]
        return sorted_region_nums

    def edit_weight(self, master, edit_pos):
        # Edits weight of edge on nav graph at edit_pos
        for vert1_id in range(self.nav_graph.vertices):
            for vert2_id in range(vert1_id):
                # Need to include edges of -1 weight
                if self.nav_graph.get_edge_val(vert1_id, vert2_id) != 0:
                    # Gets closest point on the line of the edge
                    clamped_vert_pos = Vec2D.get_closest_point(self.nav_graph.vertex_positions[vert1_id],
                                                               self.nav_graph.vertex_positions[vert2_id], edit_pos)
                    if (clamped_vert_pos is not None and Vec2D.squared_distance_between(clamped_vert_pos, edit_pos) <
                            self.THRESHOLD_DIST):
                        # If this is not the case, the user may have still clicked near an edge, but it will be close
                        # to the end of it and therefore is ambiguous between multiple edges, so should be ignored
                        prev_weight1 = self.nav_graph.get_edge_val(vert1_id, vert2_id)
                        prev_weight2 = self.nav_graph.get_edge_val(vert2_id, vert1_id)
                        if prev_weight1 != -1 and prev_weight2 != -1:
                            # Edge is bi-directional
                            new_weight = master.get_float("Previous weight was " + str(round(prev_weight1*10, 2)) +
                                                          "\nEnter new weight:")
                            if new_weight is not None:
                                if new_weight > 0:
                                    self.nav_graph.set_edge(vert1_id, vert2_id, new_weight/10,
                                                            update_heuristic_scale=True)
                                else:
                                    master.info("Weights less than or equal to 0 are not valid.")
                            pass
                        elif prev_weight1 != -1:
                            # Edge is uni-directional from vert1 to vert2
                            new_weight = master.get_float("Previous weight was " + str(round(prev_weight1 * 10, 2)) +
                                                          "\nEnter new weight:")
                            if new_weight is not None:
                                if new_weight > 0:
                                    self.nav_graph.set_edge(vert1_id, vert2_id, new_weight/10, bi_directional=False)
                                else:
                                    master.info("Weights less than or equal to 0 are not valid.")
                            pass
                        elif prev_weight2 != -1:
                            # Edge is uni-directional from vert2 to vert1
                            new_weight = master.get_float("Previous weight was " + str(round(prev_weight2 * 10, 2)) +
                                                          "\nEnter new weight:")
                            if new_weight is not None:
                                if new_weight > 0:
                                    self.nav_graph.set_edge(vert2_id, vert1_id, new_weight/10, bi_directional=False)
                                else:
                                    master.info("Weights less than or equal to 0 are not valid.")
                        else:
                            # Edge was blocked
                            new_weight = master.get_float("Edge was previously blocked\nEnter new weight:")
                            if new_weight is not None:
                                if new_weight > 0:
                                    self.nav_graph.set_edge(vert1_id, vert2_id, new_weight/10)
                                else:
                                    master.info("Weights less than or equal to 0 are not valid.")
                        return

    def change_direction(self, master, change_pos):
        # Changes direction of edge on nav graph at change_pos
        # Cycles between bi-directional, uni-directional from vert1 to vert2 and uni-directional from vert2 to vert1
        for vert1_id in range(self.nav_graph.vertices):
            for vert2_id in range(vert1_id):
                # Need to include edges of -1 weight
                if self.nav_graph.get_edge_val(vert1_id, vert2_id) != 0:
                    # Gets closest point on the line of the edge
                    clamped_vert_pos = Vec2D.get_closest_point(self.nav_graph.vertex_positions[vert1_id],
                                                               self.nav_graph.vertex_positions[vert2_id], change_pos)
                    if (clamped_vert_pos is not None and Vec2D.squared_distance_between(clamped_vert_pos, change_pos) <
                            self.THRESHOLD_DIST):
                        # If this is not the case, the user may have still clicked near an edge, but it will be close
                        # to the end of it and therefore is ambiguous between multiple edges, so should be ignored
                        prev_weight1 = self.nav_graph.get_edge_val(vert1_id, vert2_id)
                        prev_weight2 = self.nav_graph.get_edge_val(vert2_id, vert1_id)
                        if prev_weight1 != -1 and prev_weight2 != -1:
                            # Edge was bi-directional
                            self.nav_graph.set_edge(vert2_id, vert1_id, -1, bi_directional=False)
                        elif prev_weight1 != -1:
                            # Edge was uni-directional from vert1 to vert2
                            self.nav_graph.set_edge(vert1_id, vert2_id, -1, bi_directional=False)
                            self.nav_graph.set_edge(vert2_id, vert1_id, prev_weight1, bi_directional=False)
                        elif prev_weight2 != -1:
                            # Edge was uni-directional from vert2 to vert1
                            self.nav_graph.set_edge(vert1_id, vert2_id, prev_weight2, bi_directional=False)
                        else:
                            # Edge is blocked
                            master.info("Cannot change direction of blocked edge.")
                        return

    def block_path(self, master, block_pos):
        # Blocks edge on nav graph
        for vert1_id in range(self.nav_graph.vertices):
            for vert2_id in range(vert1_id):
                # Weights of -1 must also be checked for
                if self.nav_graph.get_edge_val(vert1_id, vert2_id) != 0:
                    # Gets closest point on the line of the edge
                    clamped_vert_pos = Vec2D.get_closest_point(self.nav_graph.vertex_positions[vert1_id],
                                                               self.nav_graph.vertex_positions[vert2_id], block_pos)
                    if (clamped_vert_pos is not None and Vec2D.squared_distance_between(clamped_vert_pos, block_pos) <
                            self.THRESHOLD_DIST):
                        # If this is not the case, the user may have still clicked near an edge, but it will be close
                        # to the end of it and therefore is ambiguous between multiple edges, so should be ignored
                        prev_weight1 = self.nav_graph.get_edge_val(vert1_id, vert2_id)
                        prev_weight2 = self.nav_graph.get_edge_val(vert2_id, vert1_id)
                        if prev_weight1 != -1 or prev_weight2 != -1:
                            self.nav_graph.set_edge(vert1_id, vert2_id, -1)
                        else:
                            # Edge is blocked
                            master.info("Edge is already blocked.")
                        return

    def __create_nav_graph(self, verts_per_edge):
        # Creates the navigation graph using the nav mesh and the region list
        nav_graph = WeightedGraph()
        edge_vert_ids = dict()
        for region in self.__regions:
            to_connect = []
            for region_edge_num in range(len(region)):
                vert1_id = region[region_edge_num]
                # vert2_id must wrap back to 0 for the edge between the first and last vertices in the region
                vert2_id = region[(region_edge_num + 1) % len(region)]
                if self.__nav_mesh.get_edge_val(vert1_id, vert2_id) == 3:
                    # nav_mesh is symmetrical so swapping vert ids should give same key
                    if vert1_id > vert2_id:
                        dict_key = (vert1_id, vert2_id)
                    else:
                        dict_key = (vert2_id, vert1_id)
                    if dict_key in edge_vert_ids.keys():
                        for vert_id in edge_vert_ids[dict_key]:
                            to_connect.append(vert_id)
                    else:
                        vert1_pos = self.__nav_mesh.vertex_positions[vert1_id]
                        vert2_pos = self.__nav_mesh.vertex_positions[vert2_id]
                        edge_vert_ids[dict_key] = []
                        for i in range(1, verts_per_edge + 1):
                            # Interpolates between vert1 and vert2 using i - this gets a series of equally spaced
                            # positions between vert1 and vert2 which can be used as vertices on the navigation graph
                            lerp_pos = Vec2D.lerp(vert1_pos, vert2_pos, i/(verts_per_edge + 1))
                            nav_graph.add_vertex(lerp_pos)
                            to_connect.append(nav_graph.vertices - 1)
                            edge_vert_ids[dict_key].append(nav_graph.vertices - 1)
            # All connected vertices must be joined in the navigation graph
            for vert1_id in to_connect:
                for vert2_id in to_connect:
                    if not vert1_id == vert2_id:
                        vert1_pos = nav_graph.vertex_positions[vert1_id]
                        vert2_pos = nav_graph.vertex_positions[vert2_id]
                        # Euclidean distance is used to calculate the weight of the new edge
                        nav_graph.set_edge(vert1_id, vert2_id, Vec2D.distance_between(vert1_pos, vert2_pos))
        return nav_graph, edge_vert_ids

    def __find_avg_region_positions(self):
        # Gets the average position of each region which can be used as a heuristic to accelerate finding which region a
        # point is within
        region_positions = []
        for region in self.__regions:
            avg_pos = Vec2D(0, 0)
            for vert_id in region:
                avg_pos = Vec2D.add(avg_pos, self.__nav_mesh.vertex_positions[vert_id])
            region_positions.append(avg_pos.scalar_multiply(1 / len(region)))
        return region_positions

    def __find_regions(self):
        # Finds all the regions and adds them to the region list
        # Region matrix stores how many regions each vertex is in
        region_matrix = SquareMat(self.__nav_mesh.vertices)
        regions = []
        # Initialises all outer edges with value 1 as they cannot be in more than one region (in essence, the outside of
        # the graph is a region itself but is not useful to find and is obviously not convex.)
        # Every other edge will be in exactly 2 regions
        for vert1_id in range(self.__nav_mesh.vertices):
            for vert2_id in range(vert1_id):
                if self.__nav_mesh.get_edge_val(vert1_id, vert2_id) == 2:
                    region_matrix.set_item(vert1_id, vert2_id, 1)
        weighted_nav_mesh = self.__weight_euclidean(self.__nav_mesh)
        for pass_num in range(0, 2):
            # The region checking must be run in two passes, first for all edges not in any region and second for all
            # edges in one region so that the same region is not counted multiple times.
            # For example, consider the case where the edge AB is checked and a region ABC is found a,s excluding edge
            # AB, the fastest route from A to B is A->C->B.
            # Then, the edge AC is checked, and edges AB, AC and BC are still only in one region ABC and the fastest
            # route from A to C, excluding edge AC is A->B->C.
            # Therefore, the region ABC has been found twice. This is prevented by delaying the searches for regions on
            # edges with values of 1 in the region matrix until after the first pass as other regions will always be
            # found that contain at least one of edges AB, AC and BC in that first pass.)
            # (This final fact seems relatively intuitive as long as the regions are convex, but there is a chance I am
            # wrong and a check that a region is not already inside the region list is required; however, in my pretty
            # extensive testing, I have yet to find a graph that causes this behaviour.)
            for vert1_id in range(weighted_nav_mesh.vertices):
                for vert2_id in range(vert1_id):
                    if (weighted_nav_mesh.get_edge(vert1_id, vert2_id) and region_matrix.get_item(vert1_id, vert2_id)
                            == pass_num):
                        graph_copy = deepcopy(weighted_nav_mesh)
                        # Removes the edge between vert1 and vert2, requiring the shortest cycle to be found
                        graph_copy.set_edge(vert1_id, vert2_id, 0)
                        prev, distances = graph_copy.a_star(vert1_id, vert2_id)
                        #  Work backwards through prev to get the shortest path and therefore, the region
                        path = [vert2_id]
                        region_matrix.set_item(vert1_id, vert2_id, region_matrix.get_item(vert1_id, vert2_id) + 1)
                        if region_matrix.get_item(vert1_id, vert2_id) == 2:
                            weighted_nav_mesh.set_edge(vert1_id, vert2_id, 0)
                        next_vert_id = vert2_id
                        while not next_vert_id == vert1_id:
                            current_vert_id = path[-1]
                            next_vert_id = prev[current_vert_id]
                            region_matrix.set_item(current_vert_id, next_vert_id,
                                                   region_matrix.get_item(current_vert_id, next_vert_id) + 1)
                            if region_matrix.get_item(current_vert_id, next_vert_id) == 2:
                                weighted_nav_mesh.set_edge(current_vert_id, next_vert_id, 0)
                            path.append(next_vert_id)
                        regions.append(path)
        # Additional check for debugging
        regions_sets = [set(region) for region in regions]
        if len(regions_sets) < len(regions):
            raise Exception("Duplicate regions counted!")
        return regions

    @staticmethod
    def __weight_euclidean(nav_mesh):
        weighted_graph = WeightedGraph(nav_mesh.vertex_positions)
        for vert1_id in range(nav_mesh.vertices):
            # Assuming graph no one-way edges
            for vert2_id in range(vert1_id):
                if nav_mesh.get_edge(vert1_id, vert2_id):
                    weighted_graph.set_edge(vert1_id, vert2_id,
                                            Vec2D.distance_between(nav_mesh.vertex_positions[vert1_id],
                                                                   nav_mesh.vertex_positions[vert2_id]))
        return weighted_graph

    @staticmethod
    def __split(graph):
        # Splits a weighted graph of walls by adding new edges such that there are no angles >= 180 degrees
        # Checked is the set of vertex pairs that have been checked so performance is not wasted re-checking edges
        # The input graph will be edited and so must be deep copied
        split_graph = deepcopy(graph)
        checked = SquareMat(split_graph.vertices)
        edge_added = True
        while edge_added:
            edge_added = False
            for vert1_id in range(split_graph.vertices):
                vert1_pos = split_graph.vertex_positions[vert1_id]
                # First make sure vertex is not on the outer edge
                for vert2_id in range(split_graph.vertices):
                    if split_graph.get_edge_val(vert1_id, vert2_id) == 2:
                        # Vertex is on outer edge, so ignore
                        break
                else:
                    # Vertex is not on outer edge
                    for vert2_id in range(split_graph.vertices):
                        if checked.get_item(vert1_id, vert2_id) == 0 and split_graph.get_edge(vert1_id, vert2_id):
                            checked.set_item(vert1_id, vert2_id, 1, mirrored=False)
                            right_found = False
                            left_found = False
                            vert2_pos = split_graph.vertex_positions[vert2_id]
                            edge1_vec = Vec2D.sub(vert2_pos, vert1_pos)
                            for vert3_id in range(split_graph.vertices):
                                if split_graph.get_edge(vert1_id, vert3_id):
                                    vert3_pos = split_graph.vertex_positions[vert3_id]
                                    edge2_vec = Vec2D.sub(vert3_pos, vert1_pos)
                                    # A positive perp dot product means the vector is to the left, a negative perp dot
                                    # product means it is to the right, a perp dot product of 0 means it has the same or
                                    # opposite direction
                                    if Vec2D.perp_dot_prod(edge1_vec, edge2_vec) > 0:
                                        right_found = True
                                    elif Vec2D.perp_dot_prod(edge1_vec, edge2_vec) < 0:
                                        left_found = True
                                    if right_found and left_found:
                                        # If edges to the left and right are found, then it does not need to be checked
                                        break
                            else:
                                edge_added = True
                                for vert3_id in NavMesh.__find_pot_vert(split_graph, vert1_id, vert2_id, right_found,
                                                                        left_found):
                                    split_graph.set_edge(vert1_id, vert3_id, 3)
                                continue
        return split_graph

    @staticmethod
    def __find_pot_vert(graph, vert1_id, vert2_id, right_found, left_found):
        # Returns a list of 1 or 2 vertices to join vert1 to
        vert1_pos = graph.vertex_positions[vert1_id]
        vert2_pos = graph.vertex_positions[vert2_id]
        edge1_vec = Vec2D.sub(vert2_pos, vert1_pos)
        join_verts = []
        # Sorts vertex ids, excluding vert1 and 2, by squared distance from vert 1
        sorted_verts = list(range(graph.vertices))
        sorted_verts.pop(vert1_id)
        # Offset by 1 if vert1_id < vert2_id as the index moves
        sorted_verts.pop(vert2_id - int(vert1_id < vert2_id))
        sorted_verts.sort(key=lambda sort_vert_id: Vec2D.squared_distance_between(vert1_pos, graph.vertex_positions[
            sort_vert_id]))
        for vert3_id in sorted_verts:
            vert3_pos = graph.vertex_positions[vert3_id]
            edge2_vec = Vec2D.sub(vert3_pos, vert1_pos)
            if (not right_found and not graph.get_edge(vert1_id, vert3_id) and Vec2D.perp_dot_prod(edge1_vec, edge2_vec)
                    > 0):
                # Check or edges to see if there is an intersection
                if not Floor.check_for_intersections(graph, vert1_id, vert3_id, skip_case_1=True):
                    join_verts.append(vert3_id)
                    right_found = True
            elif (not left_found and not graph.get_edge(vert1_id, vert3_id) and Vec2D.perp_dot_prod(edge1_vec,
                                                                                                    edge2_vec) < 0):
                if not Floor.check_for_intersections(graph, vert1_id, vert3_id, skip_case_1=True):
                    join_verts.append(vert3_id)
                    left_found = True
            if right_found and left_found:
                break
        return join_verts
