/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.Collections;
//import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

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
	//TODO: Add your member variables here in WEEK 3
	private int numEdges;
	private int numVertices;
	//private List<Edge> Edges;
	private Set<Node> Vertices; 
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		this.numEdges = 0;
		this.numVertices = 0;
//		this.Edges = new ArrayList<Edge>();
		this.Vertices = new HashSet<Node>();
	}
	
	// find Node from HashSet
	public Node findNode(GeographicPoint location) {
		for (Node n : this.Vertices) {
			if (n.equals(location)) {
				return n;
			}
		}
		return null;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return this.numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> res = new HashSet<GeographicPoint>();
		for (Node n: this.Vertices) {
		//	res.addAll(this.Vertices);
			res.add((GeographicPoint) n);
		}
		return res;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return this.numEdges;
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
		// TODO: Implement this method in WEEK 3
		if (location != null) {
			// check: Have we this location as Vertice yet?
			if (findNode(location)==null) {
				Node n = new Node(location);
				if (this.Vertices.add(n)) {
					this.numVertices++;
					return true;
				}
			}
		}
		return false;
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 3
		
		// check arguments
		if (from != null & to != null & length >= 0 & roadName != null & roadType != null) {
			
			// Both GeographicPoints have already been added to the graph
			if (findNode(from)!=null & findNode(to)!=null) {
				
				Edge newEdge = new Edge(from, to, roadName, roadType, length);
				
			//	if (this.Edges.add(newEdge)) {
					Node vert = findNode(from);
					if (vert.addEdge(newEdge)) {
						this.numEdges++;
					}else {
						System.out.println("Error: I can't add Edge "+newEdge.toString());
					}
			//	}
				
			} else {
				throw new IllegalArgumentException("Error adding Edge: GeographicPoint must be into set of Vertexes.");
			}
		} else {
			throw new IllegalArgumentException("Error: Illegal Edge's argument for addEdge.");
		}
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
		// TODO: Implement this method in WEEK 3
		//Initialize:
		Queue<GeographicPoint> myQue = new  LinkedList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		//HashMap <n's element ,  n's parent>
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		
		//Enqueue "start" onto the queue and add to visited
		myQue.add(start);
		visited.add(start);
		
		//while queue is not empty:
		while ( !myQue.isEmpty()){
			//dequeue node curr from top of queue
			GeographicPoint curr = myQue.poll();
			
			//if curr == goal return parent map
			if (curr.equals(goal)) {
				return hashToList(start, goal, parent);
			}
			
			//for each of curr's neighbors, n
			Node vert = findNode(curr);
			for (GeographicPoint n : vert.getNeighbors()) {
				// n, not in visited set:
				if (!visited.contains(n)) {

					// Hook for visualization.  See writeup./
					//nodeSearched.accept(next.getLocation());
					nodeSearched.accept(n);
					
					//add n to visited set
					visited.add(n);
					
					//add curr as n's parent in parent map
					parent.put(n, curr);
					
					//enqueue n onto the queue
					myQue.add(n);
				}
			}
		}
		// If we get here then there's no path
		return null;
	}
	
	
	/*
	 * Helper method which convert hashMapToList
	 * 
	 *  @param start The starting location
	 *  @param goal The goal location
	 *  @param parent HashMap <n's element ,  n's parent>
	 *  @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> hashToList(GeographicPoint goal, 
		     GeographicPoint start, HashMap<GeographicPoint, GeographicPoint> parent){
		
		List<GeographicPoint> res = new LinkedList<GeographicPoint>();
		
		GeographicPoint curr = start;
		
		while ( ! goal.equals(curr)){
			res.add(curr);
			curr = parent.get(curr);
		}
		res.add(goal);
		Collections.reverse(res);
		return res;
	}
	
	/*
	 * Helper method which convert hashMap <Node, Node> ToList
	 * 
	 *  @param start The starting location
	 *  @param goal The goal location
	 *  @param parent HashMap <n's element ,  n's parent>
	 *  @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> hashNodesToList(Node goal, 
		     Node start, HashMap<Node, Node> parent){
		
		List<GeographicPoint> res = new LinkedList<GeographicPoint>();
		
		Node curr = start;
		
		while (  goal.compareTo(curr) !=0 ){
			res.add((GeographicPoint) curr);
			curr = parent.get(curr);
		}
		res.add((GeographicPoint) goal);
		Collections.reverse(res);
		return res;
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
		// TODO: Implement this method in WEEK 4
		return	searchHelper(0, start, goal, nodeSearched);
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
		// TODO: Implement this method in WEEK 4
		return	searchHelper(1, start, goal, nodeSearched);
	}

	/** Find the path from start to goal using A-Star search or Dijkstra search
	 * 
	 * @param kindOfSort = 0  for Dijkstra search
	 * @param kindOfSort != 0 for A-Star search 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> searchHelper(int kindOfSort, GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		int countP = 0;
		// TODO: Implement this method in WEEK 4
		Node startN = findNode(start);
		if (startN == null) {
			System.out.println("Error: can't find in graph: " + start.toString());
			return null;
		}
		Node goalN = findNode(goal);
		if (goalN == null) {
			System.out.println("Error: can't find in graph: " + goal.toString());
			return null;
		}
		//Search(start, goal):
		//	  Initialize: Priority queue (PQ), visited HashSet,
		//	              parent HashMap, and distances to infinity.
		TreeMap<Double, Node> pq = new TreeMap<Double, Node>();
		HashSet<Node> visited = new HashSet<>();
		HashMap<Node, Node> parent = new HashMap<>();

		// prepare all graph for search
		for (Node n : this.Vertices) {
			n.resetDistance();
		}
		// Enqueue {S,distance} onto the PQ
		startN.setDistance(0.00001, startN.dist(goal, kindOfSort));
		pq.put(startN.getDistance(), startN);

		// while PQ is not empty:
		while (!pq.isEmpty()) {

			// dequeue node curr from front of queue
			Map.Entry<Double, Node> nodeFromPQ = pq.pollFirstEntry();
			Node curr = nodeFromPQ.getValue();
			//if (kindOfSort==0) {countP++; System.out.println(String.valueOf(countP)+" Dijkstra point:"+curr.toString());}
			//if (kindOfSort!=0) {countP++; System.out.println(String.valueOf(countP)+" A-star point:"+curr.toString());}
			
			Double currDistance = curr.getDistFromStart();

			// Hook for visualization. See writeup.
			// nodeSearched.accept(next.getLocation());
			nodeSearched.accept((GeographicPoint) curr);

			// if (curr is not visited)
			if (!visited.contains(curr)) {

				// add curr to visited set
				visited.add(curr);

				// if (curr == goal) return parent map
				if (curr.equals((GeographicPoint) goal)) {
					return hashNodesToList(startN, goalN, parent);
				}

				// for each of curr's neighbors, n
				for (GeographicPoint nGP : curr.getNeighbors()) {

					Node n = findNode(nGP);

					// n, not in visited set:
					if (!visited.contains(n)) {

						// if path through curr to n is shorter
						double distToN = currDistance + curr.getEdge(n).getLength();

						if ((distToN + n.dist(goal, kindOfSort)) < n.getDistance()) {

							// update n's distance
							n.setDistance(distToN, n.dist(goal, kindOfSort));

							// update curr as n's parent in parent map
							parent.remove(n);
							parent.put(n, curr);

							// enqueue {n,distance} into the PQ
							pq.put(n.getDistance(), n);

						}
					}
				}
			}
		}
		// if we get here then there's no path from start to goal

		return null;
	}	
	
	/** Find the path from start to goal through between points 
	 * This path must be optimized by distance (TSP - the Travelling Salesperson Problem)
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param betWeen There are list of GeographicPoints betWeen which we must visit on our route. 
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> TSSearch(GeographicPoint start, 
											 GeographicPoint goal, List<GeographicPoint> betWeen, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 6
		List<GeographicPoint> result = new LinkedList<GeographicPoint>();
		List<GeographicPoint> betWeenTemp = betWeen; // here we need deep copy.
		// TODO add sorting/mixing method for this list to produce lists for route optimization
		
		betWeenTemp.add(0, start);
		betWeenTemp.add(goal);
		int n = 2;
		while (n <= betWeenTemp.size()) {
			result.addAll(searchHelper(1, betWeenTemp.get(n-2), betWeenTemp.get(n-1), nodeSearched));
			result.remove(result.size()-1); // remove last, it will be added later again 
			n++;
		}
		result.add(goal);
		// TODO add method for graduation of competition betweenLists
		
		return	result;
	}

	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
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
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
