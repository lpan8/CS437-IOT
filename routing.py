from point import Point
import heapq
from typing import *
import numpy as np
import object_dist_mapping_working_bck as map
import speed as sp
import time
import picar_4wd as fc

T = TypeVar('T')

ANGLE_RANGE = 180

#STEP indicates the angle steps
STEP = 12
us_step = STEP
#curr angle -90 indicates right most position of car
current_angle = -90
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

class Route:
    @staticmethod
    def heuristic(a: Point, b: Point) -> float:
        return abs(a.x - b.x) + abs(a.y - b.y)

    @staticmethod
    def neighbors(map, current: Point) -> [Point]:
        directions = [Point(1, 0), Point(0, 1), Point(-1, 0), Point(0, -1)]
        neighbors = [current + direction for direction in directions]
        rows, columns = np.shape(map)

        # Find neighbors within bounds and where there is no obstacle
        filtered = filter(lambda neighbor: 0 <= neighbor.x < columns and 0 <= neighbor.y < rows and \
                                           map[neighbor.y][neighbor.x] == 0, neighbors)
        return filtered

    @staticmethod
    def search(map, start: Point, goal: Point):
        frontier = PriorityQueue()
        frontier.put(start, 0)

        came_from: Dict[Point, Optional[Point]] = {}
        cost_so_far: Dict[Point, float] = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current: Point = frontier.get()

            if current == goal:
                break

            for next in Route.neighbors(map, current):
                parent = came_from.get(current, None)
                # Favor straight lines for calculating costs
                new_cost = cost_so_far[current] + 1

                if parent:
                    # straight line
                    if next.x == current.x == parent.x or next.y == current.y == parent.y:
                        new_cost = cost_so_far[current] + 1
                    else:
                        new_cost = cost_so_far[current] + 2

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + Route.heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        return came_from, cost_so_far

def route():
    speed4 = sp.Speed(25)
    speed4.start()
    # time.sleep(2)
    #fc.forward(30)
    #fc.backward(30)
    x = 0
    for i in range(20):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        print("%smm/s"%speed,x)
    print("%smm"%x)
    speed4.deinit()
    #fc.stop()


if __name__ == '__main__':

    #map = np.array([[0, 1, 1, 1, 1, 1],
    #                 [0, 0, 0, 0, 0, 0],
    #                 [0, 1, 1, 1, 0, 0],
    #                 [0, 1, 0, 1, 1, 0],
    #                 [0, 1, 0, 1, 1, 0]])

    map = map.meas_dist_fill_dist_angle_bitmap(int(ANGLE_RANGE/STEP))
    print(map)
    new_map = np.full(np.shape(map), -1)

    start = Point(0, 0)  # starting position
    end = Point(35, 35)  # ending position

    came_from, cost_so_far = Route.search(map, start, end)
    for point, cost in cost_so_far.items():
        new_map[point.y][point.x] = cost
    path = np.zeros([70,2],dtype=int)
    #print(new_map)
    #print("before end",last_point)
    last_point = end
   i =69
    while last_point is not None:
        last_point = came_from[last_point]
        #print("in loop and i ",last_point,i)
        if i != 0:
            path[i,0]=last_point.x
            path[i,1]=last_point.y
            i-=1
    print("path",path)
    #
    #route()
    speed4 = sp.Speed(25)
    speed4.start()

    init_x = path[0,0]
    init_y = path[0,1]
    for x in range(36):
        if x != 0:
            if path[x,1] - path[x+1,1] == 0:
                continue
            else:
                cur_x = path[x,0]-init_x
                cur_y = path[x,1]-init_y
                print("cur_x,cur_y",cur_x,cur_y)
                fc.forward(30)
                rel_dist = 0
                while True:
                    time.sleep(0.1)
                    speed = speed4()
                    rel_dist += speed * 0.1
                    print("%smm/s"%speed,x)

                    if ( rel_dist > (cur_x * 4) ):
                        fc.turn_left(70)
                        time.sleep(0.8)
                        fc.stop()
                        print("rel dis",rel_dist)
                        break
                init_x = cur_x
                init_y = cur_y
                rel_dist=0
                while True:
                    fc.forward(30)
                    time.sleep(0.1)
                    speed = speed4()
                    rel_dist += speed * 0.1
                    if ( rel_dist > 34 * 4 ):
                        fc.stop()
                        break
            break
    speed4.deinit()

