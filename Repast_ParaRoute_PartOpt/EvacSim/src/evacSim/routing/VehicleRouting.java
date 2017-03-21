package evacSim.routing;


import java.util.ArrayList;
import java.util.List;

import org.jgrapht.*;
import org.jgrapht.graph.*;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.GraphPath;

import com.vividsolutions.jts.geom.Coordinate;

import repast.simphony.context.space.graph.ContextJungNetwork;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.projection.ProjectionEvent;
import repast.simphony.space.projection.ProjectionListener;
import repast.simphony.space.graph.EdgeCreator;
import repast.simphony.space.graph.JungNetwork;
import repast.simphony.space.graph.Network;
import repast.simphony.space.graph.RepastEdge;
import repast.simphony.space.graph.ShortestPath;
import edu.uci.ics.jung.graph.Graph;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.CityContext;
import evacSim.citycontext.Junction;
import evacSim.citycontext.Road;

public class VehicleRouting {
	private Network<Junction> network;
	private WeightedGraph<Junction, RepastEdge<Junction>> transformedNetwork = null;
	private CityContext cityContext;
	
	public VehicleRouting(Network<Junction> network){
		this.cityContext = ContextCreator.getCityContext();
		this.network = network;
		
		Graph<Junction, RepastEdge<Junction>> graphA = null;

		if (network instanceof JungNetwork)
			graphA = ((JungNetwork) network).getGraph();
		else if (network instanceof ContextJungNetwork)
			graphA = ((ContextJungNetwork) network).getGraph();

		JungToJgraph<Junction> converter = new JungToJgraph<Junction>();
		this.transformedNetwork = converter.convertToJgraph(graphA);
	}
	
	/**
	 * Creates routing info using the Node Based routing algorithm
	 */
	public void calcRoute() {
		Graph<Junction, RepastEdge<Junction>> graphA = null;

		if (network instanceof JungNetwork)
			graphA = ((JungNetwork) network).getGraph();
		else if (network instanceof ContextJungNetwork)
			graphA = ((ContextJungNetwork) network).getGraph();

		JungToJgraph<Junction> converter = new JungToJgraph<Junction>();
		this.transformedNetwork = converter.convertToJgraph(graphA);
	}
	
	/* Perform the routing computation */
	public List<Road> computeRoute(Road currentRoad, Road destRoad,
			Junction currJunc, Junction destJunc) {
		List<Road> roadPath_;
		List<RepastEdge<Junction>> shortestPath;
		shortestPath = null;
		
		// Get the edges that make up the shortest path
		int K = GlobalVariables.K_VALUE;
		double theta = GlobalVariables.THETA_LOGIT;
		
		if (GlobalVariables.K_SHORTEST_PATH) {
			// Find the k-shortest path
			KShortestPaths<Junction, RepastEdge<Junction>> ksp = new KShortestPaths<Junction, RepastEdge<Junction>>(transformedNetwork, currJunc, K);
			List<GraphPath<Junction, RepastEdge<Junction>>> kshortestPath = ksp.getPaths(destJunc);
			
			List<Double> pathLength = new ArrayList<Double>();
			List<Double> pathProb = new ArrayList<Double>();
			List<Double> cumProb = new ArrayList<Double>();
			double total = 0.0;

			for (GraphPath<Junction, RepastEdge<Junction>> kpath : kshortestPath) {
				pathLength.add(kpath.getWeight());
			}
			for (int i = 0; i < kshortestPath.size(); i++) {
				total = total + Math.exp(-theta * pathLength.get(i));
			}

			// calculate the probability
			for (int i = 0; i < kshortestPath.size(); i++) {
				double prob = Math.exp(-theta * pathLength.get(i)) / total;
				pathProb.add(prob);
				if (i == 0)
					cumProb.add(i, prob);
				else
					cumProb.add(i, cumProb.get(i - 1) + prob);
			}

			// find the path to go
			int k = 0;
			double random = Math.random();
			for (int i = 0; i < kshortestPath.size(); i++) {
				if (random < cumProb.get(i)) {
					k = i;
					break;
				}
			}
			shortestPath = kshortestPath.get(k).getEdgeList();
			
		} else if (GlobalVariables.SINGLE_SHORTEST_PATH) {
			/* Old thread unsafe implementation using shortest path algorithm from Repast library
			ShortestPath<Junction> p = new ShortestPath<Junction>(network);
			shortestPath = p.getPath(currJunc, destJunc);
			p.finalize();*/
			
			/* New implementation using JGraphT */
			DijkstraShortestPath<Junction, RepastEdge<Junction>> sp = new DijkstraShortestPath<Junction, RepastEdge<Junction>>(transformedNetwork, currJunc, destJunc);
			shortestPath = sp.getPathEdgeList();
		}

		
		// Find the roads which are associated with these edges
		double shortestPathLength = 0.0f;
		roadPath_ = new ArrayList<Road>();
		roadPath_.add(currentRoad);
		for (RepastEdge<Junction> edge : shortestPath) {
			int linkID = cityContext.getLinkIDFromEdge(edge);
			Road road = cityContext.findRoadWithLinkID(linkID);
//			System.out.println("linkID: " + linkID + " Path-" + road.getID());
			roadPath_.add(road);
			shortestPathLength = shortestPathLength + edge.getWeight();
		}
		shortestPath = null;
		
		return roadPath_;
	}
	

}
