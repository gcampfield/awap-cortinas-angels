import networkx as nx
import random
from base_player import BasePlayer
from settings import *
from copy import deepcopy

def bstar_betta_have_my_money(order):
    order.get_money()-DECAY_FACTOR*(GAME_LENGTH-order.get_time_created())

class Player(BasePlayer):
    """
    You will implement this class for the competition. DO NOT change the class
    name or the base class.
    """

    # You can set up static state here
    has_built_station = False

    def __init__(self, state):
        """
        Initializes your Player. You can set up persistent state, do analysis
        on the input graph, engage in whatever pre-computation you need. This
        function must take less than Settings.INIT_TIMEOUT seconds.
        --- Parameters ---
        state : State
            The initial state of the game. See state.py for more information.
        """

        self.current_graph = deepcopy(state.get_graph().copy())
        self.paths = []
        self.stations = []
        self.tracked_orders = []
        self.orders = {}

        for node in state.get_graph().nodes():
            self.orders[node] = 0

        return

    def optimal_new_station(self):
        cur_max = -1
        i_max = -1
        for i in range(0, len(self.orders)):
            if self.orders[i] > cur_max and i not in self.stations:
                cur_max = self.orders[i]
                i_max = i
        return i_max

    def station_cost(self):
        cost = 1000
        for station in self.stations:
            cost = cost * 1.5
        return cost

    # Checks if we can use a given path
    def path_is_valid(self, state, path):
        graph = state.get_graph()
        for i in range(0, len(path) - 1):
            if graph.edge[path[i]][path[i + 1]]['in_use']:
                return False
        return True

    def get_graph(self):
        return self.current_graph

    def add_path(self, state, path):
        self.paths.append((state.get_time(), path))
        for i in range(0, len(path)-1):
            self.current_graph.remove_edge(path[i], path[i+1])

    def remove_paths(self, state):
        for path in self.paths:
            if state.get_time() >= path[0]+len(path[1]):
                print ">>> REMOVE_PATH: path = "+str(path)
                for i in range(0, len(path[1])-1):
                    self.current_graph.add_edge(path[1][i], path[1][i+1])


    def step(self, state):
        """
        Determine actions based on the current state of the city. Called every
        time step. This function must take less than Settings.STEP_TIMEOUT
        seconds.
        --- Parameters ---
        state : State
            The state of the game. See state.py for more information.
        --- Returns ---
        commands : dict list
            Each command should be generated via self.send_command or
            self.build_command. The commands are evaluated in order.
        """

        # We have implemented a naive bot for you that builds a single station
        # and tries to find the shortest path from it to first pending order.
        # We recommend making it a bit smarter ;-)

        commands = []

        self.remove_paths(state)

        # add serviced_orders to order weight graph
        for order in state.get_pending_orders():
            if order.id not in self.tracked_orders:
                self.orders[order.get_node()] += 1
                self.tracked_orders.append(order.id)

        if state.get_money() >= self.station_cost() and len(self.stations) <= 5-1:
            new_station = self.optimal_new_station()
            print ">>> BUILD_COMMAND: node = "+str(new_station)
            self.stations.append(new_station)
            commands.append(self.build_command(new_station))

        paths = []
        pending_orders = state.get_pending_orders()

        for order in sorted(pending_orders, key=bstar_betta_have_my_money):
            if order not in state.get_active_orders():
                cur_min = []
                for station in self.stations:
                    path = nx.shortest_path(self.get_graph(), station, order.get_node())
                    if self.path_is_valid(state, path) and (len(path) < len(cur_min) or cur_min == []):
                        cur_min = path
                if cur_min != []:
                    commands.append(self.send_command(order, cur_min))
                    self.add_path(state, cur_min)
                    print ">>> SEND_COMMAND: path = "+str(cur_min)

        return commands
