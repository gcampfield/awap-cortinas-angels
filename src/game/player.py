import networkx as nx
import random
from base_player import BasePlayer
from settings import *
from math import sqrt

class Player(BasePlayer):
    """
    You will implement this class for the competition. DO NOT change the class
    name or the base class.
    """


    def __init__(self, state):
        """
        Initializes your Player. You can set up persistent state, do analysis
        on the input graph, engage in whatever pre-computation you need. This
        function must take less than Settings.INIT_TIMEOUT seconds.
        --- Parameters ---
        state : State
            The initial state of the game. See state.py for more information.
        """
        self.has_built_station = False
        self.build_cost = INIT_BUILD_COST
        self.size = len(state.get_graph().nodes())
        self.heatmap = [1 for _ in range(self.size)]
        self.last_order = -1
        self.heatdepth = int(sqrt(self.size) * 0.5)
        self.stations = set()
        self.money = state.money

    # Checks if we can use a given path
    def path_is_valid(self, state, path):
        graph = state.get_graph()
        for i in range(0, len(path) - 1):
            if graph.edge[path[i]][path[i + 1]]['in_use']:
                return False
        return True

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

        graph = state.get_graph()
        self.money = state.money

        commands = []

        # Station Building
        if not self.has_built_station:
            middle = sqrt(self.size) // 2 * sqrt(self.size) + sqrt(self.size) // 2
            starting_set = graph.neighbors(middle) + [int(middle)]
            start = self.best_of_best(graph, starting_set)
            station = start
            commands.append(self.build_command(station))
            self.build_cost *= BUILD_FACTOR
            self.stations.add(station)
            self.dec_heatmap(graph, station)
            self.has_built_station = True
        else:
            top = [(node, self.heatmap[node] * len(graph.neighbors(node))) for node in range(int(sqrt(self.size)))]
            for node in graph.nodes()[int(sqrt(self.size)):]:
                value = self.heatmap[node] * len(graph.neighbors(node))
                index = self.min_index(top)
                if value > top[index][1]:
                    top.append((node, value))
                    top.pop(index)
            maxValue = 0
            bestNode = None
            for node, _ in top:
                s = sum([len(nx.shortest_path(graph, s, node)) for s in self.stations])
                if s > maxValue:
                    maxValue = s
                    bestNode = node

            # if self.worth_it(station):    
            if self.money > self.build_cost:
                self.stations.add(bestNode)
                self.build_cost *= BUILD_FACTOR
                commands.append(self.build_command(bestNode))
                self.dec_heatmap(graph, bestNode)

        # Order Routing
        pending_orders = state.get_pending_orders()
        if len(pending_orders) != 0:
            #If there's a new order
            if pending_orders[-1].id > self.last_order:
                self.last_order = pending_orders[-1].id
                self.inc_heatmap(graph, pending_orders[-1].node)

            order = random.choice(pending_orders)
            paths = [nx.shortest_path(graph, station, order.get_node()) for station in self.stations]
            paths = [path for path in paths if self.path_is_valid(state, path)]
            if len(paths) > 0:
                paths.sort(key=len)
                commands.append(self.send_command(order, paths[0]))

        return commands

    def min_index(self, l):
        value = l[0][1]
        index = 0
        for i in range(len(l)):
            if l[i][0] < value:
                value = l[i][0]
                index = i
        return index

    def inc_node_heat(self, graph, node, visited, depth):
        """
        Update node heat recursively.
        """
        if depth < 0: return
        if node in visited: return
        self.heatmap[node] += depth
        visited.add(node)
        for neighbor in graph.neighbors(node):
            self.inc_node_heat(graph, neighbor, visited, depth-1)

    def inc_heatmap(self, graph, node):
        """
        Update the heat map based on the last order submitted.
        """
        self.inc_node_heat(graph, node, set(), self.heatdepth)

    def dec_node_heat(self, graph, node, visited, depth):
        """
        Update node heat recursively.
        """
        if depth < 0: return
        if node in visited: return
        self.heatmap[node] -= depth
        visited.add(node)
        for neighbor in graph.neighbors(node):
            self.dec_node_heat(graph, neighbor, visited, depth-1)

    def dec_heatmap(self, graph, station):
        """
        Update the heat map based on the new station
        """
        self.dec_node_heat(graph, station, set(), self.heatdepth)

    def best_of_best(self, graph, best):
        bestValue = -1
        bestNode = None
        for node in best:
            newValue = self.heatmap[node] * len(graph.neighbors(node))
            if newValue > bestValue:
                bestValue = newValue
                bestNode = node
        return bestNode
