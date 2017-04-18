package roadgraph;
import geography.*;

public class NodeDistancePair {
	GeographicPoint node;
	double length;
	public NodeDistancePair(GeographicPoint node, double length) {
		this.node = node;
		this.length = length;
	}
}
