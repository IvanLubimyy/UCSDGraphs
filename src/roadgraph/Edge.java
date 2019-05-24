package roadgraph;

import geography.GeographicPoint;

/*
 * A class to represent a Edge in a graph which is a RoadMap.
 * The Edge is one-way road from one GeographicPoint to another with 
 * name ("roadName", e.g. "Main street"), 
 * type ("roadType", e.g. "residential") and
 * length ("length" in km).
 */
public class Edge {
	private GeographicPoint end1;
	private GeographicPoint end2;
	private String roadName;
	private String roadType;
	private double length;
	
	public Edge(GeographicPoint from, GeographicPoint to, String rName,
			String rType, double rLength) throws IllegalArgumentException {

		if (rLength >= 0 & rName != null & rType != null) {
			
			this.end1 = from;
			this.end2 = to;
			this.roadName = rName;
			this.roadType = rType;
			if (length==0.) {
				this.length = end1.distance(end2);
			}else {
				this.length = rLength;
			}
			
		}
		else {
			throw new IllegalArgumentException("Error: Illegal Edge's argument.");
		}
	}

	// get the length of the road
	public double getLength() {
		return this.length;
	}
	
	// get the name of the road	
	public String getRoadName() {
		return this.roadName;
	}

	// get the type of the road	
	public String getRoadType() {
		return this.roadType;
	}
	
	// get start GeographicPoint
	public GeographicPoint getGp1() {
		return this.end1;
	}
	
	// get end GeographicPoint
	public GeographicPoint getGp2() {
		return this.end2;
	}
	
	// shows text presentation of Edge
	public String toString() {
		return "" + this.end1.toString() + ", " + this.end2.toString() + ", "
				  + this.roadName + ", " + this.roadType + ", " + this.length;
	}
}
