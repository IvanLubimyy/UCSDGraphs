package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

@SuppressWarnings("serial")
public class  Node extends GeographicPoint {
	private List<Edge> Edges;
	private double distance; //from start node to this
	private double distanceToGoal; // from current node to goal
	
	public Node(GeographicPoint e)
	{
		super(e.getX(), e.getY());
		this.Edges = new LinkedList<Edge>();
		//after initialization Node has lowest priority (max distance)
		this.distance = 1000000.;
		this.distanceToGoal=1000000.;
	}

	public Node(double latitude, double longitude)
	{
		super(latitude, longitude);
		this.Edges = new LinkedList<Edge>();
		//after initialization Node has lowest priority (max distance)
		this.distance = 1000000.;
		this.distanceToGoal=1000000.;
	}
	
	// if distance max, then priority is min for this node
	public int compareTo(Node other) {
    	double diff = other.distance-this.distance;
    	if (diff > 0) {
    		return 1;
    	}
    	if (diff < 0) {
    		return -1;
    	}
        return 0;
    }
	
	// get current+predicted distance of the Node in search
	public double getDistance() {
		return this.distance+this.distanceToGoal;
	}
	
	// get current distance of the Node in search
	public double getDistFromStart() {
		return this.distance;
	}
	
	// set current distance of the Node in search 
	public void setDistance(double dist, double distTo) {
		this.distance = dist;
		this.distanceToGoal = distTo;
	}
	
	// reset current distance of the Node for new search 
	public void resetDistance() {
		this.distance = 1000000.;
		this.distanceToGoal = 1000000.;
	}
	
	// get Edges of this node
	public List<Edge> getEdges(){
		return this.Edges;
	}

	// get specific Edge of this node
	public Edge getEdge(Node other){
		for (Edge e : this.Edges) {
			if(other.equals(e.getGp2())) {
				return e;
			}
		}
		return null;
	}
	
	
	// get Neighbors of this node
	public List<GeographicPoint> getNeighbors(){
		List<GeographicPoint> res = new LinkedList<GeographicPoint>();
		for (Edge n : this.Edges) {
			res.add(n.getGp2());
		}
		return res; 
	}
	
	// add Edge to this node
	public boolean addEdge(Edge e) {
			return this.Edges.add(e);
	}
	/** The node equals GeographicPoint if they have the same
	*longitude and latitude
	 */
	
	public boolean equals(GeographicPoint o)
	{
		if (o == null) {
			return false;
		}
		if (this.getX() == o.getX() & this.getY() == o.getY()) {
			return true;
		}
		
		return false;
	}
	
	// calculate distance from this node to other GeographicPoint
	// int n = 0  - for Dijkstra search return 0;
	// int n != 0 - for A - star search return actual distance
	public double dist(GeographicPoint other, int n) {
		if (n == 0) {
			return 0.;
		}
		return this.distance(other);
	}
}
