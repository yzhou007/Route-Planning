package roadgraph;

import geography.GeographicPoint;

public class Edge {
	private GeographicPoint start; // starting location
	private GeographicPoint end; // end location
	private String roadName; // name of the road
	private String roadType; // type of the road
	private double length; // length of the road

	public Edge(GeographicPoint start, GeographicPoint end, String roadName, String roadType, double length) {
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	public GeographicPoint getNeighbor() {
		return end;
	}

	public String getRodaName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getLength() {
		return length;
	}

}
