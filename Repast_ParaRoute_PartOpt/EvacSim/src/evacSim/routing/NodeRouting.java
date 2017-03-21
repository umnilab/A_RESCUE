package evacSim.routing;


import java.util.ArrayList;
import java.util.List;

import org.jgrapht.*;
import org.jgrapht.graph.*;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.GraphPath;

import com.vividsolutions.jts.geom.Coordinate;

import repast.simphony.context.space.graph.ContextJungNetwork;
import repast.simphony.space.projection.ProjectionEvent;
import repast.simphony.space.projection.ProjectionListener;
import repast.simphony.space.graph.EdgeCreator;
import repast.simphony.space.graph.JungNetwork;
import repast.simphony.space.graph.Network;
import repast.simphony.space.graph.RepastEdge;
import edu.uci.ics.jung.graph.Graph;
import evacSim.citycontext.Junction;

public class NodeRouting<T> implements ProjectionListener<T> {
	private Network<T> network;
	// old Floyd-Warshall node-based routing
	//private NodeBasedRouting<T,RepastEdge<T>> nbr;
	// new on-demand node-based routing
	private onDemandNodeBasedRouting<T,RepastEdge<T>> nbr;
	
	public NodeRouting(Network<T> network){
		this.network = network;
		
		Graph<T, RepastEdge<T>> graphA = null;
		WeightedGraph<T, RepastEdge<T>> graphB = null;

		if (network instanceof JungNetwork)
			graphA = ((JungNetwork) network).getGraph();
		else if (network instanceof ContextJungNetwork)
			graphA = ((ContextJungNetwork) network).getGraph();

		JungToJgraph<T> converter = new JungToJgraph<T>();
		graphB = converter.convertToJgraph(graphA);
		nbr = new onDemandNodeBasedRouting<T, RepastEdge<T>>(graphB);
	}
	
	/**
	 * Creates routing info using the Node Based routing algorithm
	 */
	public void calcRoute() {
		Graph<T, RepastEdge<T>> graphA = null;
		WeightedGraph<T, RepastEdge<T>> graphB = null;

		if (network instanceof JungNetwork)
			graphA = ((JungNetwork) network).getGraph();
		else if (network instanceof ContextJungNetwork)
			graphA = ((ContextJungNetwork) network).getGraph();

		JungToJgraph<T> converter = new JungToJgraph<T>();
		graphB = converter.convertToJgraph(graphA);
		
		// load new network
		System.out.println("Transform the network");
		nbr.updateNetwork(graphB);
		// update the routing matrix
		System.out.println("Update routing matrix");
		nbr.updateRouteMatrix();
		System.out.println("Finish updating routing matrix");
		/* print junction IDs in the path */
		//nbr.printPath();
	}
	
	public int getNextJuncID(int currJuncID, int destJuncID){
		return nbr.getNextNode(currJuncID, destJuncID);
	}
	
	
	public void projectionEventOccurred(ProjectionEvent<T> evt) {
		// TODO Auto-generated method stub
	}
	
	/**
	 * Removes this as a projection listener when this ShortestPath is garbage
	 * collected.
	 */
	@Override
	public void finalize() {
		if (network != null)
			network.removeProjectionListener(this);
	}

}
