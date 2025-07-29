package graph;

import heap.Heap;
import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;
import java.io.File;
import java.io.FileNotFoundException;

/**
 * Provides an implementation of Dijkstra's single-source shortest paths
 * algorithm.
 * Sample usage:
 * Graph g = // create your graph
 * ShortestPaths sp = new ShortestPaths();
 * Node a = g.getNode("A");
 * sp.compute(a);
 * Node b = g.getNode("B");
 * LinkedList<Node> abPath = sp.getShortestPath(b);
 * double abPathLength = sp.getShortestPathLength(b);
 */
public class ShortestPaths {
    // stores auxiliary data associated with each node for the shortest
    // paths computation:
    private HashMap<Node, PathData> paths;

    /**
     * Compute the shortest path to all nodes from origin using Dijkstra's
     * algorithm. Fill in the paths field, which associates each Node with its
     * PathData record, storing total distance from the source, and the
     * backpointer to the previous node on the shortest path.
     * Precondition: origin is a node in the Graph.
     */
    public void compute(Node origin) {
        paths = new HashMap<Node, PathData>();
        Heap<Node, Double> frontierHeap = new Heap<Node, Double>();
        frontierHeap.add(origin, 0.0);
        PathData orginPathData = new PathData(0, null);
        paths.put(origin, orginPathData);

        // while there are still reachable nodes
        while (frontierHeap.size() > 0) {
            Node currNode = frontierHeap.poll();
            Double currentDist = paths.get(currNode).distance;
            if (currNode.getNeighbors() != null) {
                // for each neighbor in the current nodes neighbor
                for (Map.Entry<Node, Double> nodeEntry : currNode.getNeighbors().entrySet()) {
                    // Set the neighbor nodes info
                    Node neighborNode = nodeEntry.getKey();
                    double edgeWeight = nodeEntry.getValue();

                    // come up with "new" distance to node (or first if its the first time seeing
                    // this neighbor)
                    double newDist = currentDist + edgeWeight;

                    // if paths doesnt already contain the neighbor node, or if the new distance is
                    // shorter than what we originally had the neighbornode dist as
                    if (!paths.containsKey(neighborNode) || newDist < paths.get(neighborNode).distance) {
                        // add the neighbor node to the paths with the new PathData, including previous
                        // node (current) and its new distance
                        paths.put(neighborNode, new PathData(newDist, currNode));
                        // Update distance in frontierHeap if its already there, if not add it into the
                        // heap, so its neighbors can be checked
                        if (frontierHeap.contains(neighborNode)) {
                            frontierHeap.changePriority(neighborNode, newDist);
                        } else {
                            frontierHeap.add(neighborNode, newDist);
                        }
                    }
                }
            }
        }
    }

    /**
     * Returns the length of the shortest path from the origin to destination.
     * If no path exists, return Double.POSITIVE_INFINITY.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called.
     */
    public double shortestPathLength(Node destination) {
        if (paths.containsKey(destination)) {
            return paths.get(destination).distance;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    /**
     * Returns a LinkedList of the nodes along the shortest path from origin
     * to destination. This path includes the origin and destination. If origin
     * and destination are the same node, it is included only once.
     * If no path to it exists, return null.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called.
     */
    public LinkedList<Node> shortestPath(Node destination) {
        if (!paths.containsKey(destination)) {
            return null;
        }

        LinkedList<Node> pathList = new LinkedList<>();
        Node currentNode = destination;

        while (currentNode != null) {
            pathList.addFirst(currentNode);
            currentNode = paths.get(currentNode).previous;
        }
        return pathList;
    }

    /**
     * Inner class representing data used by Dijkstra's algorithm in the
     * process of computing shortest paths from a given source node.
     */
    class PathData {
        double distance; // distance of the shortest path from source
        Node previous; // previous node in the path from the source

        /** constructor: initialize distance and previous node */
        public PathData(double dist, Node prev) {
            distance = dist;
            previous = prev;
        }
    }

    /**
     * Static helper method to open and parse a file containing graph
     * information. Can parse either a basic file or a DB1B CSV file with
     * flight data. See GraphParser, BasicParser, and DB1BParser for more.
     */
    protected static Graph parseGraph(String fileType, String fileName) throws FileNotFoundException {
        // create an appropriate parser for the given file type
        GraphParser parser;
        if (fileType.equals("basic")) {
            parser = new BasicParser();
        } else if (fileType.equals("db1b")) {
            parser = new DB1BParser();
        } else {
            throw new IllegalArgumentException(
                    "Unsupported file type: " + fileType);
        }

        // open the given file
        parser.open(new File(fileName));

        // parse the file and return the graph
        return parser.parse();
    }

    public static void main(String[] args) {
        // read command line args
        String fileType = args[0];
        String fileName = args[1];
        String origCode = args[2];

        String destCode = null;
        if (args.length == 4) {
            destCode = args[3];
        }

        // parse a graph with the given type and filename
        Graph graph;
        try {
            graph = parseGraph(fileType, fileName);
        } catch (FileNotFoundException e) {
            System.out.println("Could not open file " + fileName);
            return;
        }
        graph.report();

        // TODO 4: create a ShortestPaths object, use it to compute shortest
        // paths data from the origin node given by origCode.

        ShortestPaths shortestPaths = new ShortestPaths();
        Node origiNode = graph.getNode(origCode);
        shortestPaths.compute(origiNode);

        if (destCode == null) {

            // TODO 5:
            // If destCode was not given, print each reachable node followed by the
            // length of the shortest path to it from the origin.
            for (Map.Entry<Node, PathData> nodeEntry : shortestPaths.paths.entrySet()) {
                System.out.println(nodeEntry.getKey().toString() + " " + nodeEntry.getValue().distance);
            }
        } else {
            // TODO 6:
            // If destCode was given, print the nodes in the path from
            // origCode to destCode, followed by the total path length
            // If no path exists, print a message saying so.

            LinkedList<Node> path = shortestPaths.shortestPath(graph.getNode(destCode));

            if (path == null) {
                System.out.println("No path exists");
            } else {
                for (Node n : path) {
                    System.out.print(n.toString() + " ");
                }
                System.out.println("length: " + shortestPaths.shortestPathLength(graph.getNode(destCode)));

            }

        }
    }
}
