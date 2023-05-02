# -*- coding: utf-8 -*-

def read_graph_from_file(road_map):
    graph = {}
    with open(road_map) as file:
        for line in file:
            lines, direction = line.strip().split(';')
            nodes, weight = lines.strip().split(':')
            node1, node2 = nodes.split(',')
            weight = int(weight)
            if node1 not in graph:
                graph[node1] = {}
            if node2 not in graph:
                graph[node2] = {}
            graph[node1][node2] = [weight, int(direction)]
            # graph[node2][node1] = weight
    print(graph)
    return graph


def kortaste_vag(graph, start, goal):

    distance = {node: float('inf') for node in graph}
    distance[start] = 0

    path = {}
    path[start] = [start]
    directions = dict()
    # directions[start] = [start]

    directionlist = list()
    unvisited = set(graph)  # sätter alla noder till obesökta

    while unvisited:
        current_node = None
        for node in unvisited:
            if current_node is None:
                current_node = node
            elif distance[node] < distance[current_node]:
                current_node = node  # kortaste vägen av två noder

        unvisited.remove(current_node)  # tar bort besökt nod

        if current_node == goal:
            for i in directions:
                if i in path[current_node]:
                    print(directions[i])
                    directionlist.append(directions[i])
            return path[current_node], directionlist

        # Uppdaterar distans och kortaste väg för varje granne till nuvarande nod
        for neighbor, weight_and_direction in graph[current_node].items():
            new_distance = distance[current_node] + weight_and_direction[0]
            if new_distance < distance[neighbor]:
                distance[neighbor] = new_distance
                path[neighbor] = path[current_node] + [neighbor]

                directions[neighbor] = weight_and_direction[1]
                # print(path)

    # Returnera None om det inte finns väg från start till slut
    return None


road_map = 'road_map.txt'
graph = read_graph_from_file(road_map)
start = input('Enter start node: ')  # nån raw för att python 3
end = input('Enter end node: ')

print('kortaste_vag: ', kortaste_vag(graph, start, end))
