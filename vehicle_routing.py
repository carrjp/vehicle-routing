import argparse
import io
import math

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

MAX_DRIVING_TIME_MINUTES = 720
MAX_ROUTING_TIME_SECONDS = 29
MAX_DRIVERS = 1_000
SPAN_COST_COEFFICIENT = 0


class Point:
    """A 2D location representing a pickup and/or dropoff point."""

    def __init__(self, id: int, x: int, y: int):
        self.id = id
        self.x = x
        self.y = y

    def to_string(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

    def to_tuple(self):
        return self.x, self.y


class Load:
    """A load to be picked up and delivered."""

    def __init__(self, id: int, pickup: Point, dropoff: Point):
        self.id = id
        self.pickup = pickup
        self.dropoff = dropoff


def euclidean_distance(x1, y1, x2, y2):
    """Computes Euclidean distance given the x and y coordinates of two points."""
    xDiff = x1 - x2
    yDiff = y1 - y2
    return math.sqrt(xDiff * xDiff + yDiff * yDiff)


def create_data_model(loads: list[Load]) -> dict:
    """Creates model of all data related to input loads including adjacency matrix, pickup and dropoff IDs, etc."""
    data = {
        "pickups_dropoffs": [],
        "distance_matrix": [],
        "num_vehicles": MAX_DRIVERS,
        "vehicle_capacities": [1] * MAX_DRIVERS,
        "demands": [0],
        "depot": 0
    }
    ids_points = {0: (0.0, 0.0)}
    for load in loads:
        pickup_dropoff = []
        for is_dropoff, point in enumerate([load.pickup, load.dropoff]):
            ids_points[point.id] = point.to_tuple()
            pickup_dropoff.append(point.id)
            if is_dropoff:
                data["demands"].append(-1)
            else:
                data["demands"].append(1)
        data["pickups_dropoffs"].append(pickup_dropoff)

    for i in range(len(ids_points)):
        distances = []
        for j in range(len(ids_points)):
            distances.append(math.ceil(euclidean_distance(*ids_points[i], *ids_points[j])))
        data["distance_matrix"].append(distances)

    return data


def get_distance_of_schedule_with_return_home(route):
    """Compute the total time it takes for a driver to complete all the loads in their assigned route."""
    distance = 0.0
    home = Point(0, 0, 0)
    current_location = home
    for load in route:
        # to pickup
        distance += euclidean_distance(*current_location.to_tuple(), *load.pickup.to_tuple())
        current_location = load.pickup
        # to dropoff
        distance += euclidean_distance(*current_location.to_tuple(), *load.dropoff.to_tuple())
        current_location = load.dropoff
    distance += euclidean_distance(*current_location.to_tuple(), *home.to_tuple())
    return distance


def print_solution(data, manager, routing, solution, loads, verbose=False):
    """Prints solution to console."""
    points_loads = {}
    for load in loads:
        for point in [load.pickup, load.dropoff]:
            points_loads[point.id] = load

    if verbose:
        print(f"Objective: {solution.ObjectiveValue()}")

    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0

        route = []
        route_loads = []
        while not routing.IsEnd(index):
            if manager.IndexToNode(index):
                point = manager.IndexToNode(index)
                load = points_loads[point]
                if load.id not in route:
                    route.append(load.id)
                    route_loads.append(load)

            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"{route}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        plan_output += f'Fact-checked distance: {get_distance_of_schedule_with_return_home(route_loads)}'
        if route:
            if verbose:
                print(plan_output)
            print(route)


def get_loads_from_file(filePath: str) -> list[Load]:
    """Parses the contents of the input file and returns a list of Loads."""
    f = open(filePath, "r")
    problemStr = f.read()
    f.close()
    return get_loads_from_problem_str(problemStr)


def get_point_from_point_str(points_ids: dict, pointStr: str):
    """Creates a point and updates the points IDs dictionary given an input string."""
    pointStr = pointStr.replace("(", "").replace(")", "")
    splits = pointStr.split(",")
    x = float(splits[0])
    y = float(splits[1])
    if (x, y) not in points_ids:
        id = len(points_ids) + 1
        points_ids[(x, y)] = id
    id = points_ids[(x, y)]
    return Point(id, x, y)


def get_loads_from_problem_str(problemStr: str) -> list[Load]:
    """Parses the input string into a list of Loads."""
    loads = []
    points_ids = {}
    buf = io.StringIO(problemStr)
    gotHeader = False
    while True:
        line = buf.readline()
        if not gotHeader:
            gotHeader = True
            continue
        if len(line) == 0:
            break
        line = line.replace("\n", "")
        splits = line.split()
        id = int(splits[0])
        pickup = get_point_from_point_str(points_ids, splits[1])
        dropoff = get_point_from_point_str(points_ids, splits[2])
        loads.append(Load(id, pickup, dropoff))
    return loads


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_problem", help="Path to vehicle routing problem")
    parser.add_argument("--verbose", default=False, action="store_true", help="Print additional routing information")
    args = parser.parse_args()

    loads: list[Load] = get_loads_from_file(args.path_to_problem)

    # Instantiate the data problem.
    data = create_data_model(loads)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]


    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        MAX_DRIVING_TIME_MINUTES,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(SPAN_COST_COEFFICIENT)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]


    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Define Transportation Requests.
    for request in data["pickups_dropoffs"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.FromSeconds(MAX_ROUTING_TIME_SECONDS)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution, loads, args.verbose)
    else:
        print(routing.status())
