import java.util.List;
import java.util.LinkedList;
import java.util.Map;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.Set;

public class GraphAlgorithms {
    /**
     * Find the shortest distance between the start vertex and all other
     * vertices given a weighted graph.
     * You should use the adjacency list representation of the graph.
     *
     * Assume the adjacency list contains adjacent nodes of each node in the
     * order they should be visited. There are no negative edge weights in the
     * graph.
     *
     * If there is no path from from the start vertex to a given vertex,
     * have the distance be INF as seen in the graphs class.
     *
     * @throws IllegalArgumentException if graph or start vertex is null
     * @param graph the graph to search
     * @param start the starting vertex
     * @return map of the shortest distance between start and all other
     * vertices
     */
    public static Map<Vertex, Integer> dijkstraShortestPath(Graph graph,
                                                            Vertex start) {
        if (graph == null) {
            throw new java.lang.IllegalArgumentException(
                "Graph can't be null.");
        }
        if (start == null) {
            throw new java.lang.IllegalArgumentException(
                "Starting vertex can't be null.");
        }

        Map<Vertex, Integer> distances = new HashMap<>();
        Map<Vertex, Integer> shortestPaths = new HashMap<>();
        PriorityQueue<VertexDistancePair> vertexQueue =
            new PriorityQueue<>();
        Map<Vertex, VertexDistancePair> vertices = new HashMap<>();

        for (Vertex vertex : graph.getVertices()) {
            if (vertex.compareTo(start) == 0) {
                distances.put(vertex, 0);
            } else {
                distances.put(vertex, Graph.INF);
            }
            VertexDistancePair pair = new VertexDistancePair(vertex,
                distances.get(vertex));
            vertexQueue.add(pair);
            vertices.put(vertex, pair);
        }

        while (vertexQueue.size() != 0) {
            VertexDistancePair entry = vertexQueue.poll();
            int distance = entry.getDistance();
            Vertex curr = entry.getVertex();
            shortestPaths.put(curr, distance);
            vertices.remove(curr);
            Map<Vertex, Integer> adjacentVertices = graph.getAdjacencies(curr);
            if (adjacentVertices != null) {
                for (Vertex adjacentVertex : adjacentVertices.keySet()) {
                    if (shortestPaths.get(adjacentVertex) == null) {
                        int weight = adjacentVertices.get(adjacentVertex);
                        if (distances.get(curr) + weight
                                < distances.get(adjacentVertex)) {
                            distances.put(adjacentVertex,
                                distances.get(curr) + weight);
                            VertexDistancePair newDistance =
                                vertices.get(adjacentVertex);
                            vertexQueue.remove(newDistance);
                            newDistance.setDistance(distances.get(
                                adjacentVertex));
                            vertexQueue.add(newDistance);
                            vertices.put(adjacentVertex, newDistance);
                        }
                    }
                }
            }
        }

        return shortestPaths;
    }

    /**
     * Run Floyd Warshall on the given graph to find the length of all of the
     * shortest paths between vertices.
     *
     * You will also detect if there are negative cycles in the
     * given graph.
     *
     * You should use the adjacency matrix representation of the graph.
     *
     * If there is no path from from the start vertex to a given vertex,
     * have the distance be INF as seen in the graphs class.
     *
     * @throws IllegalArgumentException if graph is null
     * @param graph the graph
     * @return the distances between each vertex. For example, given
     * {@code arr} represents the 2D array, {@code arr[i][j]} represents the
     * distance from vertex i to vertex j. Return {@code null} if there is a
     * negative cycle.
     */
    public static int[][] floydWarshall(Graph graph) {
        if (graph == null) {
            throw new java.lang.IllegalArgumentException(
                "Graph can't be null.");
        }

        int[][] floydWarshall
            = new int[graph.getVertices().size()][graph.getVertices().size()];
        boolean test = false;

        for (Vertex i : graph.getVertices()) {
            for (Vertex j : graph.getVertices()) {
                int distance;
                if (i == j) {
                    distance = 0;
                } else if (graph.getAdjacencies(i).containsKey(j)) {
                    distance = graph.getAdjacencies(i).get(j);
                } else {
                    distance = Graph.INF;
                }
                floydWarshall[i.getId()][j.getId()] = distance;
            }
        }
        for (Vertex k : graph.getVertices()) {
            for (Vertex i : graph.getVertices()) {
                for (Vertex j : graph.getVertices()) {
                    if (floydWarshall[i.getId()][j.getId()]
                            > (floydWarshall[i.getId()][k.getId()]
                            + floydWarshall[k.getId()][j.getId()])) {
                        floydWarshall[i.getId()][j.getId()]
                            = floydWarshall[i.getId()][k.getId()]
                            + floydWarshall[k.getId()][j.getId()];
                    }
                }
            }
        }
        for (Vertex i : graph.getVertices()) {
            if (floydWarshall[i.getId()][i.getId()] < 0) {
                return null;
            }
        }
        return floydWarshall;
    }

    /**
     * A topological sort is a linear ordering of vertices with the restriction
     * that for every edge uv, vertex u comes before v in the ordering.
     *
     * You should use the adjacency list representation of the graph.
     * When considering which vertex to visit next while exploring the graph,
     * choose the next vertex in the adjacency list.
     *
     * You should start your topological sort with the smallest vertex
     * and if you need to select another starting vertex to continue
     * with your sort (like with a disconnected graph),
     * you should choose the next smallest applicable vertex.
     *
     * @throws IllegalArgumentException if the graph is null
     * @param graph a directed acyclic graph
     * @return a topological sort of the graph
     */
    public static List<Vertex> topologicalSort(Graph graph) {
        if (graph == null) {
            throw new java.lang.IllegalArgumentException(
                "Graph can't be null.");
        }
        List<Vertex> topological = new LinkedList<>();
        List<Vertex> order = new LinkedList<>();
        Set<Vertex> vertices = graph.getVertices();
        Map<Vertex, Integer> numEdges = new HashMap<>();

        for (Vertex vertex : vertices) {
            Map<Vertex, Integer> adjacentVertices =
                graph.getAdjacencies(vertex);
            if (adjacentVertices != null) {
                numEdges.put(vertex, adjacentVertices.keySet().size());
            } else {
                numEdges.put(vertex, 0);
            }
        }

        while (!vertices.isEmpty()) {
            Vertex minVertex = (Vertex) (vertices.toArray()[0]);
            for (Vertex vertex : vertices) {
                if (vertex.getId() < minVertex.getId()) {
                    minVertex = vertex;
                }
            }
            topological = topologicalHelper(minVertex, topological, graph);
            vertices.removeAll(topological);
        }
        return topological;
    }

    /**
     * Helper method for sorting a graph topologically. Goes through the graph
     * in the natural order.
     *
     * @param current the current vertex
     * @param topological the current list containing the vertices sorted
     * topologically
     * @param graph the graph that is being sorted topologically
     * @return the list containing the vertices sorted topologically
     */
    private static List<Vertex> topologicalHelper(Vertex current,
        List<Vertex> topological, Graph graph) {
        Map<Vertex, Integer> adjacentVertices = graph.getAdjacencies(current);
        if (adjacentVertices != null) {
            for (Vertex adjacentVertex : adjacentVertices.keySet()) {
                if (!topological.contains(adjacentVertex)) {
                    topological = topologicalHelper(adjacentVertex,
                        topological, graph);
                }
            }
        }
        if (!topological.contains(current)) {
            topological.add(0, current);
        }
        return topological;
    }


    /**
     * A class that pairs a vertex and a distance. Hint: might be helpful
     * for Dijkstra's.
     */
    private static class VertexDistancePair implements
        Comparable<VertexDistancePair> {
        private Vertex vertex;
        private int distance;

        /**
         * Creates a vertex distance pair
         * @param vertex the vertex to store in this pair
         * @param distance the distance to store in this pair
         */
        public VertexDistancePair(Vertex vertex, int distance) {
            this.vertex = vertex;
            this.distance = distance;
        }

        /**
         * Gets the vertex stored in this pair
         * @return the vertex stored in this pair
         */
        public Vertex getVertex() {
            return vertex;
        }

        /**
         * Sets the vertex to be stored in this pair
         * @param vertex the vertex to be stored in this pair
         */
        public void setVertex(Vertex vertex) {
            this.vertex = vertex;
        }

        /**
         * Gets the distance stored in this pair
         * @return the distance stored in this pair
         */
        public int getDistance() {
            return distance;
        }

        /**
         * Sets the distance to be stored in this pair
         * @param distance the distance to be stored in this pair
         */
        public void setDistance(int distance) {
            this.distance = distance;
        }

        @Override
        public int compareTo(VertexDistancePair v) {
            return (distance < v.getDistance() ? -1
                                         : distance > v.getDistance() ? 1 : 0);
        }
    }
}
