from math import sqrt


class Vec2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @property
    def magnitude(self):
        return sqrt(self.squared_magnitude)

    @property
    def squared_magnitude(self):  # When above is not necessary, avoid a sqrt
        return self.x ** 2 + self.y ** 2

    def scalar_multiply(self, factor):
        # Returns the vector multiplied by a scalar - DOES NOT mutate the original vector
        # This is so multiple vector expressions can be performed in a single line concisely without the need for
        # temporary variables
        return Vec2D(self.x * factor, self.y * factor)

    def normalise(self):
        # Returns a vector with the same direction but a magnitude of 1
        return self.scalar_multiply(1 / self.magnitude)

    @staticmethod
    def lerp(vec1, vec2, lerp_factor):
        # Linearly interpolates between vec1 and vec2
        return Vec2D.add(vec1.scalar_multiply(1-lerp_factor), vec2.scalar_multiply(lerp_factor))

    @staticmethod
    def bi_lerp(vec1, vec2, vec3, vec4, lerp_factor1, lerp_factor2):
        # Bilinear interpolation
        return Vec2D.lerp(Vec2D.lerp(vec1, vec2, lerp_factor1), Vec2D.lerp(vec3, vec4, lerp_factor1), lerp_factor2)

    @staticmethod
    def dot_prod(vec1, vec2):
        # Returns the dot product of 2 vectors
        return vec1.x * vec2.x + vec1.y * vec2.y

    @staticmethod
    def component_wise_prod(vec1, vec2):
        # Returns the component-wise product of 2 vectors
        return Vec2D(vec1.x * vec2.x, vec1.y * vec2.y)

    @staticmethod
    def component_wise_div(vec1, vec2):
        # Returns the vector
        return Vec2D(vec1.x / vec2.x, vec1.y / vec2.y)

    @staticmethod
    def add(vec1, vec2):
        return Vec2D(vec1.x + vec2.x, vec1.y + vec2.y)

    @staticmethod
    def sub(vec1, vec2):
        return Vec2D(vec1.x - vec2.x, vec1.y - vec2.y)

    @staticmethod
    def perp_dot_prod(vec1, vec2):
        # Returns the perpendicular product of 2 vectors
        return vec1.x * vec2.y - vec1.y * vec2.x

    @staticmethod
    def squared_distance_between(vec1, vec2):
        # Returns the squared distance between two position vectors
        return Vec2D.sub(vec1, vec2).squared_magnitude

    @staticmethod
    def distance_between(vec1, vec2):
        # Returns the distance between two position vectors
        return Vec2D.sub(vec1, vec2).magnitude

    @staticmethod
    def point_in_polygon(point_pos, polygon):
        # Finds if a Vec2D "point_pos" is in a list of Vec2Ds "polygon" that represent a convex polygon (where adjacent
        # points represent sides)
        prev_sign = 0
        for i in range(len(polygon)):
            vert1_pos = polygon[i]
            # Vert 2 must wrap back to first vertex
            vert2_pos = polygon[(i + 1) % len(polygon)]
            edge_vec = Vec2D.sub(vert2_pos, vert1_pos)
            point_vec = Vec2D.sub(point_pos, vert1_pos)
            if prev_sign == 0:
                prev_sign = Vec2D.perp_dot_prod(edge_vec, point_vec)
            elif prev_sign * Vec2D.perp_dot_prod(edge_vec, point_vec) < 0:
                # If the signs are opposite, the product will be negative and therefore < 0 and the point will not be
                # inside the polygon
                return False
        return True

    @staticmethod
    def intersect(line1_start, line1_end, line2_start, line2_end):
        # Finds if line segment 1 and line segment 2 intersect by examining a series of cases
        # First return value is if they intersect, second is none unless there is a single intersection point, in which
        # case, it is that point
        # This is definitely the shortest number of lines of code check presence of a line intersection, but should be
        # quite efficient due to short-circuiting
        line1_min_x = min(line1_start.x, line1_end.x)
        line1_max_x = max(line1_start.x, line1_end.x)
        line2_min_x = min(line2_start.x, line2_end.x)
        line2_max_x = max(line2_start.x, line2_end.x)
        if line1_min_x >= line2_max_x or line1_max_x <= line2_min_x:
            # X ranges of line segments do not overlap so no intersection
            return False, None
        line1_min_y = min(line1_start.y, line1_end.y)
        line1_max_y = max(line1_start.y, line1_end.y)
        line2_min_y = min(line2_start.y, line2_end.y)
        line2_max_y = max(line2_start.y, line2_end.y)
        if line1_min_y >= line2_max_y or line1_max_y <= line2_min_y:
            # Y ranges of line segments do not overlap so no intersection
            return False, None
        line1_vec = Vec2D.sub(line1_end, line1_start)
        line2_vec = Vec2D.sub(line2_end, line2_start)
        if line1_vec.x == 0 or line2_vec.x == 0:
            # Line segments are vertical, x and y ranges already confirmed to intersect so lines must intersect
            return True, None
        # Rise over run
        line1_grad = line1_vec.y / line1_vec.x
        line2_grad = line2_vec.y / line2_vec.x
        # Extrapolate from start pos to x=0
        line1_intercept = line1_start.y - line1_start.x * line1_grad
        line2_intercept = line2_start.y - line2_start.x * line2_grad
        if line1_grad == line2_grad:
            # Line segments are parallel
            if line1_intercept == line2_intercept:
                # Must intersect as ranges overlap
                return True, None
            else:
                # Otherwise, cannot intersect
                return False, None
        # Found by equating line equations y = mx + c and solving for x
        intersect_x = (line2_intercept - line1_intercept) / (line1_grad - line2_grad)
        # Find x range of valid solution
        min_x = max(line1_min_x, line2_min_x)
        max_x = min(line1_max_x, line2_max_x)
        if intersect_x < min_x or intersect_x > max_x:
            # Intersection is not within the bounds of the line segments
            return False, None
        # Substitute into line1 equation
        intersect_y = line1_grad * intersect_x + line1_intercept
        # Find y range of valid solution
        min_y = max(line1_min_y, line2_min_y)
        max_y = min(line1_max_y, line2_max_y)
        if intersect_y < min_y or intersect_y > max_y:
            # Intersection is not within the bounds of the line segments
            return False, None
        # The solution is valid
        return True, Vec2D(intersect_x, intersect_y)

    @classmethod
    def get_closest_point(cls, wall_start_pos, wall_end_pos, point_pos):
        # Returns the closest point to the line defined by the wall start and end position vectors if it is within the
        # line segment, otherwise returns None
        min_x = min(wall_start_pos.x, wall_end_pos.x)
        max_x = max(wall_start_pos.x, wall_end_pos.x)
        min_y = min(wall_start_pos.y, wall_end_pos.y)
        max_y = max(wall_start_pos.y, wall_end_pos.y)
        point = cls.__get_closest_point_vector(wall_start_pos, wall_end_pos, point_pos)
        if min_x <= point.x <= max_x and min_y <= point.y <= max_y:
            return point
        else:
            return None

    @staticmethod
    def __get_closest_point_vector(wall_start_pos, wall_end_pos, point_pos):
        # Finds closest point on the line wall_start_pos to wall_end_pos to point_pos
        # IMPORTANT - treats the wall as an infinite line NOT a line segment.
        if wall_end_pos.x - wall_start_pos.x == 0:
            # Line is vertical
            return Vec2D(wall_start_pos.x, point_pos.y)
        elif wall_end_pos.y - wall_start_pos.y == 0:
            # Line is horizontal
            return Vec2D(point_pos.x, wall_start_pos.y)
        wall_vec = Vec2D.sub(wall_end_pos, wall_start_pos)
        # Rise over run
        grad = wall_vec.y / wall_vec.x
        # Extrapolate from wall start pos to x = 0
        intercept = wall_start_pos.y - grad * wall_start_pos.x
        # Negative reciprocal
        perp_grad = -1 / grad
        # Extrapolate from point pos to x = 0
        perp_intercept = point_pos.y - perp_grad * point_pos.x
        intersection_x = (perp_intercept - intercept) / (grad - perp_grad)
        # Substitute into wall equation
        intersection_y = grad * intersection_x + intercept
        return Vec2D(intersection_x, intersection_y)
