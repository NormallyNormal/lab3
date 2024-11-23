import math
from collections import deque

# -1 if negative, 1 otherwise
def sign(num):
    return -1 if num < 0 else 1

# Magnitude of a 2d vector
def magni2d(x, y):
    return math.sqrt(x * x + y * y)

def get_angle_to_point(x, y, x_goal, y_goal):
    dx = x_goal - x
    dy = y_goal - y
    angle = math.atan2(dy, dx)
    return angle / 3.1415 * 180

def lines_intersect(line1, line2):
    def cross_product(v1, v2):
        return v1[0] * v2[1] - v1[1] * v2[0]

    def is_point_on_segment(p, seg_start, seg_end):
        x, y = p
        x1, y1 = seg_start
        x2, y2 = seg_end
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # Direction vectors
    line1_dir = (x2 - x1, y2 - y1)
    line2_dir = (x4 - x3, y4 - y3)
    diff = (x3 - x1, y3 - y1)

    # Calculate the cross product of direction vectors
    denom = cross_product(line1_dir, line2_dir)
    if denom == 0:  # Lines are parallel
        return False

    # Parametric intersection t for infinite line1
    t = cross_product(diff, line2_dir) / denom

    # Parametric intersection u for bounded line2
    u = cross_product(diff, line1_dir) / denom

    # u must be in the range [0, 1] for intersection to occur on the bounded line
    if 0 <= u <= 1:
        # Calculate the intersection point
        intersect_point = (x1 + t * line1_dir[0], y1 + t * line1_dir[1])
        # Confirm intersection point lies on bounded line
        return is_point_on_segment(intersect_point, (x3, y3), (x4, y4))

    return False

def crossing_mline(old_x, old_y, x, y):
    return lines_intersect(((50, 0), (250, 250)), ((old_x, old_y), (x, y)))

def looping_minimal_angle_difference(target_angle, current_angle):
    angle_difference = (target_angle - current_angle) % 360
    if angle_difference > 180:
        angle_difference -= 360
    if angle_difference < -180:
        angle_difference += 360
    return angle_difference

# Determines when a sequence of inputs start increasing
class TrendDetector:
    def __init__(self, window_size=5, threshold=3):
        self.window_size = window_size
        self.threshold = threshold
        self.numbers = deque()
    
    def add_number(self, num):
        self.numbers.append(num)
        # Cull if the list is too big
        if len(self.numbers) > self.window_size:
            self.numbers.popleft()
        if len(self.numbers) < self.window_size:
            return False
        numbers_list = list(self.numbers)
        # Check how many times the sequence of the last N inputs increase
        increases = sum(1 for i in range(1, len(numbers_list)) if numbers_list[i] > numbers_list[i - 1])
        return increases >= self.threshold
