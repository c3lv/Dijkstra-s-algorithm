#Dijkstra's algorithm
def Dijkstra(graph, start, end):
    #initialize the distance of each node to infinity
    distance = {node: float('inf') for node in graph.nodes}
    #set the distance of the start node to 0
    distance[start] = 0
    #initialize the set of visited nodes
    visited = set()
    #initialize the set of unvisited nodes
    unvisited = set(graph.nodes)
    #while there are unvisited nodes
    while unvisited:
        #find the node with the smallest distance
        current = min(unvisited, key=lambda node: distance[node])
        #if the current node is the end node, return the distance
        if current == end:
            return distance[end]
        #remove the current node from the unvisited set
        unvisited.remove(current)
        #for each neighbor of the current node
        for neighbor in graph.neighbors(current):
            #if the distance of the neighbor is infinity
            if distance[neighbor] == float('inf'):
                #set the distance of the neighbor to the distance of the current node plus the weight of the edge
                distance[neighbor] = distance[current] + graph.weight(current, neighbor)
            #if the distance of the neighbor is greater than the distance of the current node plus the weight of the edge
            elif distance[neighbor] > distance[current] + graph.weight(current, neighbor):
                #set the distance of the neighbor to the distance of the current node plus the weight of the edge
                distance[neighbor] = distance[current] + graph.weight(current, neighbor)
    #if there are no unvisited nodes, return infinity
    return float('inf')

class Graph:
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges
    def neighbors(self, node):
        return self.edges[node]
    def weight(self, node1, node2):
        return self.edges[node1][node2]

if __name__ == '__main__':
    #test case 1
    nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
    edges = {'A': {'B': 10, 'C': 20},
             'B': {'A': 10, 'C': 30, 'D': 35},
             'C': {'A': 20, 'B': 30, 'D': 25, 'E': 50},
             'D': {'B': 35, 'C': 25, 'E': 30, 'F': 15},
             'E': {'C': 50, 'D': 30, 'F': 20},
             'F': {'D': 15, 'E': 20, 'G': 25},
             'G': {'F': 25}}
    graph = Graph(nodes, edges)
    print(Dijkstra(graph, 'A', 'C'))
    #test case 2
    nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
    edges = {'A': {'B': 10, 'C': 20},
             'B': {'A': 10, 'C': 30, 'D': 35},
             'C': {'A': 20, 'B': 30, 'D': 25, 'E': 50},
             'D': {'B': 35, 'C': 25, 'E': 30, 'F': 15},
             'E': {'C': 50, 'D': 30, 'F': 20},
             'F': {'D': 15, 'E': 20, 'G': 25},
             'G': {'F': 25}}
    graph = Graph(nodes, edges)
    print(Dijkstra(graph, 'A', 'G'))
