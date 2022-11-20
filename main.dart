import 'package:algo/graphs/graph.dart';
import 'package:algo/priority_queue/priority_queue.dart';

void main() {
  final graph = AdjacencyList<String>();
  final a = graph.createVertex('A');
  final b = graph.createVertex('B');
  final c = graph.createVertex('C');
  final d = graph.createVertex('D');
  final e = graph.createVertex('E');
  final f = graph.createVertex('F');
  final g = graph.createVertex('G');
  final h = graph.createVertex('H');
  graph.addEdge(a, b, weight: 8, edgeType: EdgeType.directed);
  graph.addEdge(a, f, weight: 9, edgeType: EdgeType.directed);
  graph.addEdge(a, g, weight: 1, edgeType: EdgeType.directed);
  graph.addEdge(g, c, weight: 3, edgeType: EdgeType.directed);
  graph.addEdge(c, b, weight: 3, edgeType: EdgeType.directed);
  graph.addEdge(c, e, weight: 1, edgeType: EdgeType.directed);
  graph.addEdge(e, b, weight: 1, edgeType: EdgeType.directed);
  graph.addEdge(e, d, weight: 2, edgeType: EdgeType.directed);
  graph.addEdge(b, e, weight: 1, edgeType: EdgeType.directed);
  graph.addEdge(b, f, weight: 3, edgeType: EdgeType.directed);
  graph.addEdge(f, a, weight: 2, edgeType: EdgeType.directed);
  graph.addEdge(h, g, weight: 5, edgeType: EdgeType.directed);
  graph.addEdge(h, f, weight: 2, edgeType: EdgeType.directed);
  final dijkstra = Dijkstra(graph);
  final allPaths = dijkstra.shortestPaths(a);
  final path = dijkstra.shortestPath(a, d);
  final shortestPathList = dijkstra.shortestPathsLists(a);
  print(shortestPathList);
}

class Pair<T> extends Comparable<Pair<T>> {
  Pair(this.distance, [this.vertex]);

  final Vertex<T>? vertex;
  final double distance;

  @override
  int compareTo(Pair<T> other) {
    if (distance == other.distance) return 0;
    if (distance > other.distance) return 1;
    return -1;
  }

  @override
  String toString() => '($distance, $vertex)';
}

/// Time complexity is O(E) since you need to visit every edge.
/// Enqueue and dequeue from a heap base priority queue is
/// a logarithmic time complexity, thus operation would be O(log
/// E). Repeating that for every edge would thus be O(E log E).
class Dijkstra<E> {
  Dijkstra(this.graph);

  final Graph<E> graph;

  /// Generates shortest paths.
  Map<Vertex<E>, Pair<E>?> shortestPaths(Vertex<E> source) {
    // The priority queue will allow you to visit the shortest
    // route next in each pass.
    final queue = PriorityQueue<Pair<E>>(priority: Priority.min);
    // The visited isn't strictly necessary, but using it will
    // prevent you from unnecessarily checking vertices that
    // you've already visited before.
    final visited = <Vertex<E>>{};
    final paths = <Vertex<E>, Pair<E>?>{};

    // Initialize every vertex in the graph with a null
    // distance-vertex pair.
    for (var vertex in graph.vertices) {
      paths[vertex] = null;
    }

    // Initialize the algorithm with the source vertex.
    // This is where the search will start from, so the
    // distance to this vertex is zero.
    // The `queue` holds the current vertex while `paths` stores
    // a reference to the previous vertex.
    queue.enqueue(Pair(0, source));
    visited.add(source);
    // Since the source vertex doesn't have a previous vertex,
    // using `Pair(0)` causes the previous vertex to default to
    // null.
    paths[source] = Pair(0);
    // Each loop handle visiting a new vertex.
    // The queue holds the vertices that are known but haven't
    // been visited yet. As long as the queue isn't empty,
    // you're not done exploring.
    while (!queue.isEmpty) {
      final currentPair = queue.dequeue()!;
      // Later on, you'll decrease the distances of certain paths
      // as you find shorter routes.
      // However, if you update the distance in paths, you should
      // really update the same distance in queue.
      // When the old distance-vertex pair comes through, the
      // code will ignore it.
      final savedDistance = paths[currentPair.vertex]!.distance;
      if (currentPair.distance > savedDistance) continue;
      // Add the current vertex to the visited set so that you
      // can skip over it later.
      visited.add(currentPair.vertex!);
      // Looping over outgoing edges.
      for (var edge in graph.edges(currentPair.vertex!)) {
        // If you’ve previously visited the destination vertex, then ignore it and go on
        final neighbor = edge.destination;

        if (visited.contains(neighbor)) continue;
        final weight = edge.weight ?? double.infinity;
        final totalDistance = currentPair.distance + weight;

        final knowDistance = paths[neighbor]?.distance ?? double.infinity;

        if (totalDistance < knowDistance) {
          paths[neighbor] = Pair(totalDistance, currentPair.vertex);
          queue.enqueue(Pair(totalDistance, neighbor));
        }
      }
    }
    return paths;
  }

  /// Providing paths as an argument is an optimization if you
  /// need to call `shortestPath` multiple times.
  List<Vertex<E>> shortestPath(Vertex<E> source, Vertex<E> destination,
      {Map<Vertex<E>, Pair<E>?>? paths}) {
    // {A: (0.0, null), B: (6.0, E), C: (4.0, G), D: (7.0, E), E: (5.0, C), F: (9.0, A), G: (1.0, A), H: null}
    final allPath = paths ?? shortestPaths(source);
    // Ensure that a path actually exists.
    if (!allPath.containsKey(destination)) return [];
    var current = destination;
    final path = <Vertex<E>>[current];
    // Build the path by working backward from the destination.
    while (current != source) {
      final previous = allPath[current]?.vertex;
      if (previous == null) return [];
      path.add(previous);
      current = previous;
    }
    return path.reversed.toList();
  }
}

extension<E> on Dijkstra<E> {
  /// To get the shortest paths from the source vertex to every
  /// other vertex in the graph
  Map<Vertex<E>, List<Vertex<E>>> shortestPathsLists(
    Vertex<E> source,
  ) {
    final Map<Vertex<E>, List<Vertex<E>>> result = {};
    final allPath = shortestPaths(source);
    for (var vertex in graph.vertices) {
      final path = shortestPath(source, vertex, paths: allPath);
      result[vertex] = path;
    }
    return result;
  }
}
