import heapq

def djikstras(graph, start, end):
  pq = [(0 ,start)]
  visited = set()
  while pq:
    (cost, curr_node) = heapq.heappop(pq)
    if curr_node == end:
      return cost
    if curr_node in visited:
      continue
    visited.add(curr_node)
    for(next_node, edge_cost) in graph[curr_node]:
      if next_node not in visited:
        heapq.heappush(pq, (edge_cost + cost, next_node))
  return float("inf")

graph = {
  'A' : [('B', 5), ('C', 1)],
  'B' : [('A', 5), ('B', 2), ('D', 1)],
  'C' : [('A', 1), ('B', 2), ('D', 4)],
  'D' : [('B', 1), ('C', 4)]
}

shortest_distance = djikstras(graph, 'B', 'D')
print("The shortest Distance between B and D is ", shortest_distance)