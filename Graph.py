from SquareMatrix import SquareMat
from Queue import DynamicQueue, DynamicPriorityQueue
from Stack import DynamicStack
from Vector2D import Vec2D


class Graph:
    def __init__(self, vertex_positions=None):
        if vertex_positions is None:
            # Avoids mutable default
            vertex_positions = []
        # Edges are stored in an adjacency matrix with 1 representing an edge and 0 representing no edge
        self._adjacency_matrix = SquareMat(len(vertex_positions))
        # Technically the abstract data structure of a graph does not involve storing positions of vertices, but for
        # every graph in my program, vertex positions are required, so it does not make much sense to add an extra layer
        # of composition here
        self.vertex_positions = vertex_positions

    @property
    def vertices(self):
        # Returns the number of vertices in the graph
        return len(self.vertex_positions)

    @property
    def edges(self):
        # Returns the number of edges in the graph
        count = 0
        for vert1_id in range(self.vertices):
            for vert2_id in range(self.vertices):
                if self.get_edge(vert1_id, vert2_id):
                    count += 1
        return count

    def add_vertex(self, position):
        self._adjacency_matrix.expand()
        self.vertex_positions.append(position)

    def get_edge(self, vert1_id, vert2_id):
        if self._adjacency_matrix.get_item(vert1_id, vert2_id) < 0:
            # Negative weights also count as non-existent
            return False
        else:
            return bool(self._adjacency_matrix.get_item(vert1_id, vert2_id))

    def set_edge(self, vert1_id, vert2_id, new_edge_val, bi_directional=True):
        self._adjacency_matrix.set_item(vert1_id, vert2_id, new_edge_val, mirrored=bi_directional)

    def create_edge(self, vert1_pos, vert2_pos, new_edge_val=True):
        self.add_vertex(vert1_pos)
        self.add_vertex(vert2_pos)
        self.set_edge(self.vertices - 2, self.vertices - 1, new_edge_val)

    def delete_vertex(self, vert_id):
        self._adjacency_matrix.delete_row_column(vert_id)
        self.vertex_positions.pop(vert_id)

    def bfs(self, vert1_id, vert2_id):
        # Breadth-first search; finds the shortest path in an unweighted graph from vert1 to vert2
        prev = [-1 for _ in range(self.vertices)]
        distances = [-1 for _ in range(self.vertices)]
        to_visit = DynamicQueue()
        to_visit.enqueue(vert1_id)
        while not to_visit.empty:
            current_vert = to_visit.dequeue()
            for next_vert in range(self.vertices):
                if distances[next_vert] == -1 and self.get_edge(current_vert, next_vert):
                    distances[next_vert] = distances[current_vert] + 1
                    prev[next_vert] = current_vert
                    if next_vert == vert2_id:
                        # vert2 has been found
                        return prev, distances
                    to_visit.enqueue(next_vert)
        # No valid path
        return None

    def dfs(self, vert1_id):
        # Depth-first search; finds all accessible vertices from vert1
        # Technically, a modified bfs would work perfectly well instead, but a DFS is generally more space efficient
        # than a BFS, though in their respective worst-cases, they are identical
        found_verts = [False for _ in range(self.vertices)]
        found_verts[vert1_id] = True
        to_visit = DynamicStack()
        to_visit.push(vert1_id)
        while not to_visit.empty:
            current_vert = to_visit.pop()
            for next_vert in range(self.vertices):
                if not found_verts[next_vert] and self.get_edge(current_vert, next_vert):
                    to_visit.push(next_vert)
                    found_verts[next_vert] = True
        return found_verts


class WeightedGraph(Graph):
    # Inherits from graph
    def __init__(self, vertex_positions=None, heuristic_scale=1):
        super().__init__(vertex_positions)
        self.__heuristic_scale = heuristic_scale

    def get_edge_val(self, vert1_id, vert2_id):
        # Does not override get_edge as bool(any numeric!=0) is True and so rule with 0 representing no edge holds true
        # This means BFS and DFS methods are still valid for weighted graphs
        return self._adjacency_matrix.get_item(vert1_id, vert2_id)

    def set_edge(self, vert1_id, vert2_id, new_edge_val, bi_directional=True, update_heuristic_scale=False):
        self._adjacency_matrix.set_item(vert1_id, vert2_id, new_edge_val, mirrored=bi_directional)
        if update_heuristic_scale:
            new_heuristic_scale = new_edge_val/Vec2D.distance_between(self.vertex_positions[vert1_id],
                                                                      self.vertex_positions[vert2_id])
            if new_heuristic_scale < self.__heuristic_scale:
                self.__heuristic_scale = new_heuristic_scale

    def create_edge(self, vert1_pos, vert2_pos, new_edge_val=1):
        self.add_vertex(vert1_pos)
        self.add_vertex(vert2_pos)
        self.set_edge(len(self.vertex_positions) - 2, len(self.vertex_positions) - 1, new_edge_val)

    def heuristic(self, vert1_id, vert2_id):
        return (Vec2D.distance_between(self.vertex_positions[vert1_id], self.vertex_positions[vert2_id])
                * self.__heuristic_scale)

    def dijkstra(self, vert1_id, vert2_id):
        # Dijkstra graph search; finds the shortest path in a weighted graph from vert1 to vert2
        # Distance of -1 represents infinite as negative distances should be impossible
        distances = [-1 for _ in range(self.vertices)]
        distances[vert1_id] = 0
        prev = [0 for _ in range(self.vertices)]
        to_visit = DynamicPriorityQueue()
        # As queue is ordered with by minimum priority, distances can be used directly as priorities
        to_visit.enqueue(vert1_id, 0)
        while not to_visit.empty:
            current_vert = to_visit.dequeue()
            if distances[vert2_id] != -1 and distances[vert2_id] < distances[current_vert]:
                # As to_visit is ordered by distance and negative distances are impossible, there cannot be a shorter
                # path to vert2
                return prev, distances
            for next_vert in range(self.vertices):
                if self.get_edge(current_vert, next_vert):
                    dist = distances[current_vert] + self.get_edge_val(current_vert, next_vert)
                    if distances[next_vert] == -1 or dist < distances[next_vert]:
                        # Either no route to that vertex has been found previously or a shorter route than the previous
                        # one has been found
                        prev[next_vert] = current_vert
                        distances[next_vert] = dist
                        to_visit.decrease_priority_or_enqueue(next_vert, dist)
        if distances[vert2_id] != -1:
            return prev, distances
        else:
            # No valid path
            return None

    def a_star(self, vert1_id, vert2_id):
        # A* graph search; finds the shortest path in a weighted graph from vert1 to vert2 as long as the heuristic
        # is admissible
        # Very similar to Dijkstra but uses a heuristic for some speed-up
        # Distance of -1 represents infinite as negative distances should be impossible
        distances = [-1 for _ in range(self.vertices)]
        distances[vert1_id] = 0
        prev = [0 for _ in range(self.vertices)]
        to_visit = DynamicPriorityQueue()
        # As queue is ordered with by minimum priority, distances can be used directly as priorities
        to_visit.enqueue(vert1_id, 0)
        while not to_visit.empty:
            current_vert = to_visit.dequeue()
            if distances[vert2_id] != -1 and distances[vert2_id] < distances[current_vert] + self.heuristic(
                    current_vert, vert2_id):
                # As to_visit is ordered by distance and negative distances are impossible, there cannot be a shorter
                # path to vert2 as long as the heuristic is admissible
                return prev, distances
            for next_vert in range(self.vertices):
                if self.get_edge(current_vert, next_vert):
                    dist = distances[current_vert] + self.get_edge_val(current_vert, next_vert)
                    if distances[next_vert] == -1 or dist < distances[next_vert]:
                        # Either no route to that vertex has been found previously or a shorter route than the previous
                        # one has been found
                        prev[next_vert] = current_vert
                        distances[next_vert] = dist
                        to_visit.decrease_priority_or_enqueue(next_vert, dist + self.heuristic(next_vert, vert2_id))
                    # Or if a faster route has been found to that vertex already, nothing needs to be done
        if distances[vert2_id] != -1:
            return prev, distances
        else:
            # No valid path
            return None

    def greedy(self, vert1_id, vert2_id):
        # Greedy best-first graph search; finds a path in a weighted graph from vert1 to vert2
        # Can be more efficient than A* or Dijkstra in many cases as it always explores vertices closer to the goal,
        # instead of having to check for other alternate paths
        distances = [-1 for _ in range(self.vertices)]
        distances[vert1_id] = 0
        prev = [0 for _ in range(self.vertices)]
        to_visit = DynamicPriorityQueue()
        to_visit.enqueue(vert1_id, self.heuristic(vert1_id, vert2_id))
        while not to_visit.empty:
            current_vert = to_visit.dequeue()
            for next_vert in range(self.vertices):
                if self.get_edge(current_vert, next_vert) and distances[next_vert] == -1:
                    # New vertex has been found
                    prev[next_vert] = current_vert
                    distances[next_vert] = distances[current_vert] + self.get_edge_val(current_vert, next_vert)
                    to_visit.enqueue(next_vert, self.heuristic(next_vert, vert2_id))
                    if next_vert == vert2_id:
                        return prev, distances

            print(current_vert)
        # No valid path
        return None
