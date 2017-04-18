/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private int numVertices; //  number of vertices
	private int numEdges; // number of edges
	private Map<GeographicPoint, ArrayList<Edge>> graphMap; 
	// the key of this map is an intersection/vertex
	// the value associated with the key is an ArrayList of edges
	// Edge is an added class that include the starting location, end location, 
	// roadName, roadType and length of the edge
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{	
		// TODO: Implement in this constructor in WEEK 2
		numVertices = 0;
		numEdges = 0;
		graphMap = new HashMap<GeographicPoint, ArrayList<Edge>>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return graphMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (location == null || graphMap.containsKey(location)) {
			return false;
		}
		
		graphMap.put(location, new ArrayList<Edge>());
		numVertices++;
		
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if (from == null || to == null || 
			length < 0) {
			throw new IllegalArgumentException();
		}
		
		if (!graphMap.containsKey(from) || !graphMap.containsKey(to)) {
			throw new IllegalArgumentException();
		}
		Edge edge = new Edge(from, to, roadName, roadType, length);
		graphMap.get(from).add(edge);
		numEdges++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			return null;
		}
		
		if (!graphMap.containsKey(start) || !graphMap.containsKey(goal)) {
			return null;
		}
		
		Queue<GeographicPoint> que = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		que.offer(start);
		visited.add(start);
		boolean find = false;
		while(!que.isEmpty()) {
			GeographicPoint cur = que.poll();
			nodeSearched.accept(cur);
			if (cur.equals(goal)) {
				find = true;
				break;
			}
			for (Edge edge : graphMap.get(cur)) {
				GeographicPoint neighbor = edge.getNeighbor();
				if (!visited.contains(neighbor)) {
					que.offer(neighbor);
					visited.add(neighbor);
					parentMap.put(neighbor, cur);
				}
			}
		}
		
		if (!find) {
			return null;
		}
			
		return constructPath(start, goal, parentMap);
		
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			return null;
		}
		
		if (!graphMap.containsKey(start) || !graphMap.containsKey(goal)) {
			return null;
		}
		
		PriorityQueue<NodeDistancePair> pq = new PriorityQueue<NodeDistancePair>(1, new Comparator<NodeDistancePair>(){
			@Override
			public int compare(NodeDistancePair pair1, NodeDistancePair pair2) {
				if (pair1.length < pair2.length) {
					return -1;
				} else if (pair1.length > pair2.length) {
					return 1;
				} else {
					return 0;
				}
			}
		});
		Map<GeographicPoint, Double> distances = new HashMap<>(); 
		pq.offer(new NodeDistancePair(start, 0.0));
		distances.put(start, 0.0);
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		boolean find = false;
		int count = 0;
		
		while(!pq.isEmpty()) {
			if (visited.contains(pq.peek().node)) {
				pq.poll();
				continue;
			}
			GeographicPoint cur = pq.peek().node;
			double curLength = pq.peek().length;
			pq.poll();
			visited.add(cur);
			nodeSearched.accept(cur);
			count++;
			if (cur.equals(goal)) {
				find = true;
				break;
			}
			for (Edge edge : graphMap.get(cur)) {
				GeographicPoint neighbor = edge.getNeighbor();
				if (!visited.contains(neighbor)) {
					double length = curLength + cur.distance(neighbor);
					if (distances.containsKey(neighbor)) {
						if (length < distances.get(neighbor)) {
							pq.remove(new NodeDistancePair(neighbor, distances.get(neighbor)));
							distances.put(neighbor, length); //update distance for neighbor
							parentMap.put(neighbor, cur); //update the parent node for neighbor
						}
					} 
					else {
						distances.put(neighbor, length);
						parentMap.put(neighbor, cur);
					}
					pq.offer(new NodeDistancePair(neighbor, distances.get(neighbor)));
				}
			}
		}
		
		if (!find) {
			return null;
		}
		//System.out.println("Number of node visited in Dijkstra is: " + count + "\n");
		return constructPath(start, goal, parentMap);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start == null || goal == null) {
			return null;
		}
		
		if (!graphMap.containsKey(start) || !graphMap.containsKey(goal)) {
			return null;
		}
		
		PriorityQueue<NodeDistancePair> pq = new PriorityQueue<NodeDistancePair>(1, new Comparator<NodeDistancePair>(){
			@Override
			public int compare(NodeDistancePair pair1, NodeDistancePair pair2) {
				if (pair1.length < pair2.length) {
					return -1;
				} else if (pair1.length > pair2.length) {
					return 1;
				} else {
					return 0;
				}
			}
		});
		Map<GeographicPoint, Double> distances1 = new HashMap<>(); //distance from source to a current node
		Map<GeographicPoint, Double> distances2 = new HashMap<>(); // an underestimated distance from current node to goal
		pq.offer(new NodeDistancePair(start, 0.0 + start.distance(goal)));
		distances1.put(start, 0.0);
		distances2.put(start, start.distance(goal));
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		boolean find = false;
		int count = 0;
		while(!pq.isEmpty()) {
			if (visited.contains(pq.peek().node)) {
				pq.poll();
				continue;
			}
			GeographicPoint cur = pq.peek().node;
			double curLength1 = distances1.get(cur);
			pq.poll();
			visited.add(cur);
			nodeSearched.accept(cur);
			count++;
			if (cur.equals(goal)) {
				find = true;
				break;
			}
			for (Edge edge : graphMap.get(cur)) {
				GeographicPoint neighbor = edge.getNeighbor();
				if (!visited.contains(neighbor)) {
					double length1 = curLength1 + edge.getLength();
					double length2 = neighbor.distance(goal);
					
					if (distances1.containsKey(neighbor)) {
						if (length1 + length2 < distances1.get(neighbor) + distances2.get(neighbor)) {
							distances1.put(neighbor, length1); //update distance1 for the neighbor
							distances2.put(neighbor, length2); //update distance2 for the neighbor
							parentMap.put(neighbor, cur); //update the parent node for neighbor
						}
					} 
					else {
						distances1.put(neighbor, length1); 
						distances2.put(neighbor, length2);
						parentMap.put(neighbor, cur);
					}
					pq.offer(new NodeDistancePair(neighbor, distances1.get(neighbor) + distances2.get(neighbor)));
				}
			}
		}
		
		if (!find) {
			return null;
		}
		//System.out.print("Number of node visited in Astar is: " + count + "\n");
		return constructPath(start, goal, parentMap);
	}

	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, 
												  Map<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> path = new ArrayList<>();
		GeographicPoint tmp = goal;
		while(!tmp.equals(start)) {
			path.add(0,tmp);
			tmp = parentMap.get(tmp);
		}
		path.add(0, tmp);
		return path;
	}
	
	
	
	
	
	public static void main(String[] args)
	{
		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		//System.out.println(testroute + "\n");
		//System.out.println(testroute2 + "\n");*/
		// You can use this method for testing.  
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println(route + "\n");
		System.out.println(route2 + "\n");
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
