from point import Point
import heapq
from typing import *
import numpy as np
import picar_4wd as fc
import time
import random
import argparse
import sys
import time

T = TypeVar('T')
power = 30

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


if __name__ == '__main__':

    # map = np.array([[0, 1, 1, 1, 1, 1],
    #                  [0, 0, 0, 0, 0, 0],
    #                  [0, 1, 1, 1, 0, 0],
    #                  [0, 1, 0, 1, 1, 0],
    #                  [0, 1, 0, 1, 1, 0]])

    
    map = np.array([[0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    new_map = np.full(np.shape(map), -1)

    start = Point(10, 0)  # starting position
    end = Point(10, 15)  # ending position

    came_from, cost_so_far = Route.search(map, start, end)
    for point, cost in cost_so_far.items():
        new_map[point.y][point.x] = cost

    print(new_map)
    last_point = end
    print(last_point)

    while last_point is not None:
        last_point = came_from[last_point]
        print(last_point)