from Vector2D import Vec2D
from Graph import WeightedGraph
from Constants import GlobalConstants


class Link:
    def __init__(self, link_id, position):
        self.link_id = link_id
        self.position = position


class Floor:
    # Square distance here to avoid a square root
    THRESHOLD_DIST = (2 * GlobalConstants.NODE_RADIUS) ** 2
    HALF_THRESHOLD_DIST = GlobalConstants.NODE_RADIUS ** 2

    def __init__(self, Empty=False):
        self.walls = WeightedGraph()
        self.links = []
        if not Empty:
            self.walls.add_vertex(Vec2D(0, 0))
            self.walls.add_vertex(Vec2D(GlobalConstants.FLOOR_RATIO.x, 0))
            self.walls.add_vertex(Vec2D(GlobalConstants.FLOOR_RATIO.x, GlobalConstants.FLOOR_RATIO.y))
            self.walls.add_vertex(Vec2D(0, GlobalConstants.FLOOR_RATIO.y))
            self.walls.set_edge(0, 1, 2)
            self.walls.set_edge(1, 2, 2)
            self.walls.set_edge(2, 3, 2)
            self.walls.set_edge(3, 0, 2)

    @property
    def edited(self):
        # Returns if the floor is any different to the default
        if self.walls.vertices > 4 or len(self.links) > 0:
            return True
        return False

    def add_wall(self, wall_start_pos, wall_end_pos):
        # Attempts to add a wall at the specified start and end positions and returns if it was successful
        if Vec2D.squared_distance_between(wall_start_pos, wall_end_pos) < self.THRESHOLD_DIST:
            # Wall cannot have both vertices be too close
            return False
        wall_start_id = self.__place_vertex(wall_start_pos)
        wall_end_id = self.__place_vertex(wall_end_pos)
        # Checks for overlaps with other walls
        if self.check_for_intersections(self.walls, wall_start_id, wall_end_id):
            # Remove vertices of wall that were added
            offset = int(self.__check_if_remove(wall_start_id))
            self.__check_if_remove(wall_end_id - offset * int(wall_start_id < wall_end_id))
            return False
        else:
            if (self.walls.vertex_positions[wall_start_id].x == self.walls.vertex_positions[wall_end_id].x == 0 or
                    self.walls.vertex_positions[wall_start_id].x == self.walls.vertex_positions[wall_end_id].x ==
                    GlobalConstants.FLOOR_RATIO.x or
                    self.walls.vertex_positions[wall_start_id].y == self.walls.vertex_positions[wall_end_id].y == 0 or
                    self.walls.vertex_positions[wall_start_id].y == self.walls.vertex_positions[wall_end_id].y ==
                    GlobalConstants.FLOOR_RATIO.y):
                # If new wall is created on the edge of the floor
                self.walls.set_edge(wall_start_id, wall_end_id, 2)
            else:
                self.walls.set_edge(wall_start_id, wall_end_id, 1)
            return True

    @staticmethod
    def check_for_intersections(walls, wall_start_id, wall_end_id, skip_case_1=False):
        # Checks for intersections of a new wall with all previously placed ones
        # Two cases must be considered:
        # Case 1: The new wall intersects with the radius of a vertex
        # Case 2: The two walls themselves intersect
        # Similarly to when adding a new vertex, the vertex intersection test must be considered first if it is being
        # used
        if not skip_case_1:
            for vert_id in range(walls.vertices):
                if vert_id not in (wall_start_id, wall_end_id):
                    # Case 1
                    # Wall intersects with circle if the distance to closest point to the center of said circle on the
                    # wall from the center of the circle is less than or equal the radius
                    vertex_pos = walls.vertex_positions[vert_id]
                    wall_start_pos = walls.vertex_positions[wall_start_id]
                    wall_end_pos = walls.vertex_positions[wall_end_id]
                    clamped_closest = Vec2D.get_closest_point(wall_start_pos, wall_end_pos, vertex_pos)
                    if (clamped_closest is not None and Vec2D.squared_distance_between(vertex_pos, clamped_closest) <=
                            Floor.HALF_THRESHOLD_DIST):
                        return True
        for vert1_id in range(walls.vertices):
            # Case 2
            for vert2_id in range(vert1_id):
                if walls.get_edge(vert1_id, vert2_id) and not (vert1_id in (wall_start_id, wall_end_id) or
                                                               vert2_id in (wall_start_id, wall_end_id)):
                    intersection = Vec2D.intersect(walls.vertex_positions[vert1_id],
                                                   walls.vertex_positions[vert2_id],
                                                   walls.vertex_positions[wall_start_id],
                                                   walls.vertex_positions[wall_end_id])
                    if intersection[0]:
                        return True
        # Otherwise, no intersections
        return False

    def __place_vertex(self, new_vert_pos):
        # 3 potential cases placing a vertex
        # 1) Use existing vertex,
        # 2) Split existing wall,
        # 3) Create entirely new vertex
        # Returns vert id of the placed vertex
        for vert_id in range(self.walls.vertices):
            # Case 1
            if (Vec2D.squared_distance_between(self.walls.vertex_positions[vert_id], new_vert_pos) <
                    self.THRESHOLD_DIST):
                return vert_id
        for vert1_id in range(self.walls.vertices):
            # Case 2
            for vert2_id in range(vert1_id):
                if self.walls.get_edge(vert1_id, vert2_id):
                    # Gets closest point on the line of the edge
                    clamped_vert_pos = Vec2D.get_closest_point(self.walls.vertex_positions[vert1_id],
                                                               self.walls.vertex_positions[vert2_id], new_vert_pos)
                    if (clamped_vert_pos is not None and
                            Vec2D.squared_distance_between(clamped_vert_pos, new_vert_pos) < self.HALF_THRESHOLD_DIST):
                        # If this is not the case, and case 1 has failed, then the point cannot be less than the
                        # threshold distance away from the wall (and consequently the half threshold distance as well)
                        self.__split_wall(clamped_vert_pos, vert1_id, vert2_id)
                        return self.walls.vertices - 1
        # Case 3
        self.walls.add_vertex(new_vert_pos)
        return self.walls.vertices - 1

    def __split_wall(self, split_pos, vert1_id, vert2_id):
        # If the wall type is an outer wall, both new split walls must be the same
        old_val = self.walls.get_edge_val(vert1_id, vert2_id)
        self.walls.add_vertex(split_pos)
        self.walls.set_edge(vert1_id, vert2_id, 0)
        self.walls.set_edge(vert1_id, self.walls.vertices - 1, old_val)
        self.walls.set_edge(vert2_id, self.walls.vertices - 1, old_val)

    def delete(self, delete_pos):
        # 3 potential cases for deletion:
        # Case 1 - Delete link
        # Case 2 - Delete vertex (and all walls connected to it and all vertices that are only connected to those walls)
        # Case 3 - delete wall (and all vertices that are only connected to it)
        for link_num in range(len(self.links)):
            # Case 1
            if Vec2D.squared_distance_between(self.links[link_num].position, delete_pos) < self.HALF_THRESHOLD_DIST:
                self.links.pop(link_num)
                # Should only delete one element at a time
                return
        # Start at 4 as corner vertices cannot be deleted
        for vert_id in range(4, self.walls.vertices):
            # Case 2
            if (Vec2D.squared_distance_between(self.walls.vertex_positions[vert_id], delete_pos) <
                    self.HALF_THRESHOLD_DIST):
                to_check = []
                edge_vertex = False
                edge1_vert = -1
                edge2_vert = -1
                for vert2_id in range(self.walls.vertices):
                    if self.walls.get_edge_val(vert_id, vert2_id) == 2:
                        if edge_vertex:
                            # There will always be 2 outer edges connected to an edge vertex
                            edge2_vert = vert2_id
                        else:
                            edge_vertex = True
                            edge1_vert = vert2_id
                    elif self.walls.get_edge(vert_id, vert2_id):
                        to_check.append(vert2_id)
                if edge_vertex:
                    # If edge vertex, edge must be repaired
                    self.walls.set_edge(edge1_vert, edge2_vert, 2)
                self.walls.delete_vertex(vert_id)
                to_check.sort()
                # As vertices are deleted, the next vertex indices must be decreased to their new one
                index_offset = 0
                for each in to_check:
                    index_offset += int(self.__check_if_remove(each-index_offset-int(each > vert_id)))
                return
        for vert1_id in range(self.walls.vertices):
            # Case 3
            for vert2_id in range(vert1_id):
                # Outer edges cannot be deleted
                if self.walls.get_edge_val(vert1_id, vert2_id) == 1:
                    # Gets closest point on the line of the edge
                    clamped_vert_pos = Vec2D.get_closest_point(self.walls.vertex_positions[vert1_id],
                                                               self.walls.vertex_positions[vert2_id], delete_pos)
                    if (clamped_vert_pos is not None and Vec2D.squared_distance_between(clamped_vert_pos, delete_pos) <
                            self.HALF_THRESHOLD_DIST):
                        # If this is not the case, and case 1 has failed, then the delete position cannot be less than
                        # the threshold distance away from the wall (and consequently the half threshold distance as
                        # well)
                        self.walls.set_edge(vert1_id, vert2_id, 0)
                        offset = int(self.__check_if_remove(vert1_id))
                        self.__check_if_remove(vert2_id - offset * int(vert1_id < vert2_id))
                        return

    def __check_if_remove(self, vert_id):
        # Checks if there are no edges connected to the given vertex
        # Returns if it is removed or not
        for vert2_id in range(self.walls.vertices):
            if self.walls.get_edge(vert_id, vert2_id):
                return False
        else:
            self.walls.delete_vertex(vert_id)
            return True

    def add_link(self, link_id, position):
        self.links.append(Link(link_id, position))
