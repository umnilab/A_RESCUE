package evacSim.routing;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import evacSim.citycontext.Road;
import evacSim.citycontext.Zone;
import evacSim.routing.RouteV;
import evacSim.vehiclecontext.Vehicle;

import org.jgrapht.alg.interfaces.*;
//import org.jgrapht.alg.interfaces.MatchingAlgorithm.Matching;
import org.jgrapht.alg.matching.GreedyWeightedMatching;
import org.jgrapht.alg.matching.KuhnMunkresMinimalWeightBipartitePerfectMatching;
//import org.jgrapht.alg.matching.MaximumWeightBipartiteMatching;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

/*
author: Jiawei Xue;
date: April 16; May 2, 2020;
This is a system optimum guidance for the evacuation vehicles faced with the
shelter-full situations. There 3 different routing (matching) strategies:
	1) Hungarian algorithm (exact algorithm)
	2) LEDA maximum weight bipartite matching algorithm (exact algorithm)
	3) Greedy weighted matching algorithm (1/2-approximation)
reference: https://jgrapht.org/javadoc/org/jgrapht/alg/interfaces/MatchingAlgorithm.html

For instance, say we have 5 shelters with capacities [100,100,100,200,200].
At time t=2000, the remaining capacities are [50,50,50,100,100] and the 
relocation demand is [60,55,55,70,95].
*/

public class SOShelterRouting {
	private ArrayList<Zone> shelters;
	private ArrayList<Integer> shelterCapacity;
	private ArrayList<Integer> shelterRemaining;
	private ArrayList<Integer> relocateDemand;
	// the sum of all entries in reLocateDemand
	private Integer demandSum;
	// distance matrix. The distance between location i and j is d_{i,j}
	private ArrayList<ArrayList<Double>> dMatrix;
	// the maximal value of dMatrix
	private Double maxDistance;

	public SOShelterRouting(ArrayList<Zone> shelters) {
		this.shelters = shelters;
		int nShelters = shelters.size();
		demandSum = 0;
		maxDistance = 0.0;
		shelterCapacity = new ArrayList<Integer>(nShelters);
		shelterRemaining = new ArrayList<Integer>(nShelters);
		relocateDemand = new ArrayList<Integer>(nShelters);
		dMatrix = new ArrayList<ArrayList<Double>>();
		for (int i = 0; i < nShelters; i++) {
			shelterCapacity.add(shelters.get(i).getCapacity());
			shelterRemaining.add(shelters.get(i).getCapacity());
			relocateDemand.add(0);
			ArrayList<Double> dMatrixRow = new ArrayList<Double>(nShelters);
			for (int j = 0; j < nShelters; j++) {
				dMatrixRow.add(0.0);
			}
			dMatrix.add(dMatrixRow);
		}
	}
	
	/** Return the (constant) capacities (in persons) of the shelters. */
	public ArrayList<Integer> getCapacity() {
		return shelterCapacity;
	}
	
	/** Update and return the remaining (available) spaces (in persons) of
	 * shelters at call time. */
	public ArrayList<Integer> updateRemaining() {
		int remaining;
		for (int i = 0; i < shelters.size(); i++) {
			remaining = shelters.get(i).getCapacity() - shelters.get(i).getOccupancy();
			shelterRemaining.set(i, remaining);
		}
		return shelterRemaining;
	}
	
	/** Update and return the no. of people trying to relocate among shelters. */
	public ArrayList<Integer> updateRelocateDemand() {
		// reset the total demand
		demandSum = 0;
		for (int i = 0; i < shelters.size(); i++) {
			Queue<Vehicle> waiting = shelters.get(i).getWaiting();
			if (waiting == null) {
				continue;
			}
			relocateDemand.set(i, waiting.size());
			demandSum += waiting.size();
		}
		return relocateDemand;
	}
	
	/** Update and return the distances between shelters at call time. */
	public ArrayList<ArrayList<Double>> updateDistMatrix() {
		double maxDist = 0.0;
		for (int i = 0; i < shelters.size(); i++) {
			for (int j = i+1; j < shelters.size(); j++) {
				Zone orig = shelters.get(i);
				Zone dest = shelters.get(j);
				if (orig.getDownJunc() == null) {
					orig.setDownJunc(RouteV.getNearestDownStreamJunction(
							null, orig.getRoad()));
				}
				if (dest.getDownJunc() == null) {
					dest.setDownJunc(RouteV.getNearestDownStreamJunction(
							null, dest.getRoad()));
				}
				Map<Double,List<Road>> shortPath = RouteV.vbr.computeRoute(
						orig.getRoad(), dest.getRoad(),
						orig.getDownJunc(), dest.getDownJunc());
				// get the distance of this path
				// (by iterating over the only key-value-pair)
				for (double dist : shortPath.keySet()) {
					dMatrix.get(i).set(j, dist);
					dMatrix.get(j).set(i, dist);
					if (dist > maxDist) maxDist = dist;
				}
			}
		}	
		maxDistance = maxDist;
		return dMatrix;
	}
	
	/**
	 * Get the matching with the updated data of this object at the current 
	 * time's shelter values.
	 * @param algo:
	 * 		routing algorithm to be used: one of ['hungarian','leda','greedy']
	 */
	public ArrayList<ArrayList<Integer>> getMatching(String algorithm) {
		ArrayList<ArrayList<Integer>> plan = null;
		
		// update the shelter info
		updateRemaining();
		updateRelocateDemand();
		updateDistMatrix();
		
//		System.out.println("Remaining capacities: " + shelterRemaining);
//		int remainingSum = 0;
//		for (int i = 0; i < shelterRemaining.size(); i++) {
//			remainingSum += shelterRemaining.get(i);
//		}
//		System.out.println("Total remaining cap: " + remainingSum);
//		System.out.println("Relocation demands: " + relocateDemand);
//		System.out.println("Total relocation demand: " + demandSum);
		
		// get the plan if there's any demand
		if (demandSum > 0) {
			if (algorithm.equals("hungarian")) {
				plan = hungarianRouting();
			} else if (algorithm.equals("leda")) {
				plan = ledaRouting();
			} else if (algorithm.equals("greedy")) {
				plan = greedyRouting();
			} else {
				throw new IllegalArgumentException("Algorithm should be one" + 
						" of 'hungarian', 'leda', 'greedy'");
			}
		}
		return plan;
	}
	
	/**
	 * Get the matching & distribute it to the vehicles
	 */
	public void assignMatching(String algorithm) {
		// first get the matching matrix
		ArrayList<ArrayList<Integer>> matching = getMatching(algorithm);
		if (demandSum == 0) {
//			System.out.println("Zero relocation demand.");
			return;
		}
		if (matching == null) {
			System.err.println("Null matching in shelter relocation for non-zero demand.");
			return;
		}
//		System.out.println("Matching: " + matching);
		
		// for each row in matching (i.e. shelter whose demand is to be relocated elsewhere)
		for (int i = 0; i < matching.size(); i++) {
			Zone origin = shelters.get(i);
			Queue<Vehicle> relocateVehicles = origin.getWaiting();
			ArrayList<Integer> matchingRow = matching.get(i);
			
			// check if the matching returns correct number for this shelter
			if (getSum(matchingRow) == relocateVehicles.size()) {
				// for each shelter, get its number of assigned vehicles
				for (int j = 0; j < matchingRow.size(); j++) {
					// get the target shelter
					Zone dest = shelters.get(j);
					// get the no. of vehicles assigned from i to j
					int numAssigned = matchingRow.get(j);
					// map each of this number to the corresponding vehicle
					for (int k = 0; k < numAssigned; k++) {
						// pop the vehicle from the relocation queue
						Vehicle veh = relocateVehicles.remove();
						// check if current destination of vehicle is same as this shelter
						int curDestID = veh.getDestinationID();
						if (curDestID != origin.getIntegerId()) {
							System.err.println("Shelter reassignment does not match"
								+ " for vehicle " + veh.getVehicleID() + " (current"
								+ "plan destination: " + curDestID +
								", current shelter: " + origin.getIntegerId() + ")");
						} else {
							// change the vehicle's plan to the new destination
							veh.setNewHouse(dest.getIntegerId());
						}
					}
				}
			} else {
				System.err.println("Matching sizes do not match for shelter " +
						origin + ", required: " + relocateVehicles.size() +
						", assigned:" + getSum(matchingRow));
//				System.out.print("...");
			}
		}
	}
	
	// ALGORITHM 1: Hungarian routing
	public ArrayList<ArrayList<Integer>> hungarianRouting(){
		// step 1: build the network
		SimpleWeightedGraph<String, DefaultWeightedEdge> matchingGraph = new 
				SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		// step 2: define the nodes
		ArrayList<String> leftNode = new ArrayList<String>();
		ArrayList<String> rightNode = new ArrayList<String>();
		/* we set each demand as a node in the left, and each capacity as a node in the right.
		Here, the dummy nodes are introduced in the left to make the number of nodes in the left side to be equal to right side.
		*/
		// 2.1 add the demand nodes
		for (int i = 0; i < demandSum; i++) {
			String str1 = "de"; //demand node
			String str2 = Integer.toString(i+1);   //de1,de2,...,de335
			String combine = str1 + str2;
			leftNode.add(combine);		
		}
		// 2.2 add the dummy nodes
		for (int i = 0; i < getSum(shelterRemaining) - demandSum ; i++) {
			String str1 = "du";  //dummy node
			String str2 = Integer.toString(i+1);   //du1,du2,...,du1340
			String combine = str1 + str2;
			leftNode.add(combine);		
		}		
		// 2.3 add the supply nodes
		for (int i = 0; i < getSum(shelterRemaining); i++) {
			String str1 = "su";  //supply node
			String str2 = Integer.toString(i+1);   //su1,su2,...,su1675
			String combine = str1 + str2;
			rightNode.add(combine);		
		}
		// 2.4 add nodes into the network
		Set<String> leftNodeSet = new HashSet<String>(leftNode);
		Set<String> rightNodeSet = new HashSet<String>(rightNode);
		for (int i = 0; i < leftNode.size();i++){
			matchingGraph.addVertex(leftNode.get(i));
		}
		for (int i = 0; i < rightNode.size();i++){
			matchingGraph.addVertex(rightNode.get(i));
		}
		// 3. add the edges
		// add the edges between left nodes and supply node
		for (int i = 0; i < leftNode.size(); i++) {
			for (int j = 0; j < rightNode.size(); j++) {
				String str1 = leftNode.get(i);
				String str2 = rightNode.get(j);
				matchingGraph.addEdge(str1, str2);
				DefaultWeightedEdge edge = matchingGraph.getEdge(str1, str2);
				if (i< demandSum){
					int locationI = getLocation(relocateDemand, i+1);  // locationI: start form 0
					int locationJ = getLocation(shelterRemaining,j+1);  // locationJ: start from 0.		
					matchingGraph.setEdgeWeight(edge, dMatrix.get(locationI).get(locationJ)); // the weight between demand node and supply node is dij.
				}
				else {
					matchingGraph.setEdgeWeight(edge, 0.0); // the weight between the dummy node and the supply node is 0.0.
				}
			}
		}
		// step 4: return the plan	
		KuhnMunkresMinimalWeightBipartitePerfectMatching<String, DefaultWeightedEdge> Hmatching =
				new KuhnMunkresMinimalWeightBipartitePerfectMatching<String, DefaultWeightedEdge>(
						matchingGraph,leftNodeSet,rightNodeSet);
		MatchingAlgorithm.Matching<String, DefaultWeightedEdge> matching = Hmatching.getMatching();
		return returnPlan(matchingGraph, matching);
	}

	// ALGORITHM 2: Leda algorithm
	public ArrayList<ArrayList<Integer>> ledaRouting() {
		// step 1: build the network
		SimpleWeightedGraph<String, DefaultWeightedEdge> matchingGraph = new SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		// step 2: define the node
		ArrayList<String> leftNode = new ArrayList<String>();
		ArrayList<String> rightNode = new ArrayList<String>();
		// 2.1 add the demand nodes
		for (int i = 0; i < demandSum; i++) {
			String str1 = "de"; //demand node
			String str2 = Integer.toString(i+1);   //de1,de2,...,de335
			String combine = str1 + str2;
			leftNode.add(combine);	
		}
		// 2.2 add the supply nodes
		for (int i = 0; i < getSum(shelterRemaining); i++) {
			String str1 = "su";  //supply node
			String str2 = Integer.toString(i+1);   //su1,su2,...,su1675
			String combine = str1 + str2;
			rightNode.add(combine);		
		}	
		// 2.4 add the nodes into the network
		//Set<String> leftNodeSet = new HashSet<String>(leftNode);
		//Set<String> rightNodeSet = new HashSet<String>(rightNode);
		for (int i = 0; i < leftNode.size();i++){
			matchingGraph.addVertex(leftNode.get(i));
		}
		for (int i = 0; i < rightNode.size();i++){
			matchingGraph.addVertex(rightNode.get(i));
		}
		// step 3: define the edges
		for (int i = 0; i < leftNode.size(); i++) {
			for (int j = 0; j < rightNode.size(); j++) {
				String str1 = leftNode.get(i);
				String str2 = rightNode.get(j);
				matchingGraph.addEdge(str1, str2);
				DefaultWeightedEdge edge = matchingGraph.getEdge(str1, str2);
				int locationI = getLocation(relocateDemand, i+1);  // start form 0
				int locationJ = getLocation(shelterRemaining,j+1);  // locationJ: start from 0.		
		        matchingGraph.setEdgeWeight(edge, maxDistance - dMatrix.get(locationI).get(locationJ)); // the weight between demand node and supply node is dij.
			}
		}
		// step 4: return the plan
		boolean normalizeEdgeCosts = false;
		GreedyWeightedMatching<String, DefaultWeightedEdge> Gmacthing = new GreedyWeightedMatching<String, DefaultWeightedEdge>(matchingGraph,normalizeEdgeCosts);
		MatchingAlgorithm.Matching<String, DefaultWeightedEdge> matching = Gmacthing.getMatching();
		return returnPlan(matchingGraph, matching);
	}

	// ALGORITHM 3: greedy algorithm
	public ArrayList<ArrayList<Integer>> greedyRouting() {
		// step 1: build the network
		SimpleWeightedGraph<String, DefaultWeightedEdge> matchingGraph = new SimpleWeightedGraph<String, DefaultWeightedEdge>(DefaultWeightedEdge.class);
		// step 2: define the node
		ArrayList<String> leftNode = new ArrayList<String>();
		ArrayList<String> rightNode = new ArrayList<String>();
		// 2.1 add the demand nodes
		for (int i = 0; i < demandSum; i++) {
			String str1 = "de"; //demand node
			String str2 = Integer.toString(i+1);   //de1,de2,...,de335
			String combine = str1 + str2;
			leftNode.add(combine);	
		}
		// 2.2 add the supply nodes
		for (int i = 0; i < getSum(shelterRemaining); i++) {
			String str1 = "su";  //supply node
			String str2 = Integer.toString(i+1);   //su1,su2,...,su1675
			String combine = str1 + str2;
			rightNode.add(combine);		
		}	
		// 2.4 add the nodes into the network
		//Set<String> leftNodeSet = new HashSet<String>(leftNode);
		//Set<String> rightNodeSet = new HashSet<String>(rightNode);
		for (int i = 0; i < leftNode.size();i++){
			matchingGraph.addVertex(leftNode.get(i));
		}
		for (int i = 0; i < rightNode.size(); i++){
			matchingGraph.addVertex(rightNode.get(i));
		}
		// step 3: define the edges
		for (int i = 0; i < leftNode.size(); i++) {
			for (int j = 0; j < rightNode.size(); j++) {
				String str1 = leftNode.get(i);
				String str2 = rightNode.get(j);
				matchingGraph.addEdge(str1, str2);
				DefaultWeightedEdge edge = matchingGraph.getEdge(str1, str2);
				int locationI = getLocation(relocateDemand, i+1);  // start form 0
				int locationJ = getLocation(shelterRemaining,j+1);  // locationJ: start from 0.
				double distance = dMatrix.get(locationI).get(locationJ);
		        matchingGraph.setEdgeWeight(edge, maxDistance - distance); // the weight between demand node and supply node is dij.
			}
		}
		// step 4: return the plan
		boolean normalizeEdgeCosts = false;
		GreedyWeightedMatching<String, DefaultWeightedEdge> Gmacthing = new GreedyWeightedMatching<String, DefaultWeightedEdge>(matchingGraph,normalizeEdgeCosts);
		MatchingAlgorithm.Matching<String, DefaultWeightedEdge> matching = Gmacthing.getMatching();
		ArrayList<ArrayList<Integer>> plan = returnPlan(matchingGraph, matching);
		return plan;
	}

	/** 
	 * Return to the location of index
	 * @param countList
	 * @param number
	 * @return
	 * for example, input:[10,20,5], 15; return 1. (because the 15th elements
	 * are located at the second set)
	 * input:[10,20,5], 32; return 2. (because the 32-th elements are located
	 * at the third set)
	 */
	public int getLocation(ArrayList<Integer> countList, int number) {
		// start from 0
		int index = -1;
		int pointer = 0;
		for (int i = 0; i < countList.size(); i++) {
			pointer += countList.get(i);
			if (pointer >= number) {
				index = i;
				break;
			}
		}
		return index;
	}

	/**
	 * Decode the matchingResult.
	 * @param matchingGraph: weighted bipartite graph used to create the matching
	 * @param matching: the matching object obtained from a routing algorithm
	 * @return result: an NxN matrix (N shelters) of distribution of relocated demand
	 */
	public ArrayList<ArrayList<Integer>> returnPlan(
			SimpleWeightedGraph<String, DefaultWeightedEdge> matchingGraph,
			MatchingAlgorithm.Matching<String, DefaultWeightedEdge> matching
			) {
		// for instance, result is a 5 by 5 matrix. There are {i,j} people relocating from i to j.
		// initialize result.
		ArrayList<ArrayList<Integer>> result = new ArrayList<ArrayList<Integer>>();
		for (int i = 0; i < shelterRemaining.size(); i++) {
			ArrayList<Integer> row = new ArrayList<Integer>();
			for (int j = 0; j< shelterRemaining.size(); j++) {
				row.add(0);
			}
			result.add(row);
		}
		Set<DefaultWeightedEdge> matchingEdges = matching.getEdges();
		for(DefaultWeightedEdge edge : matchingEdges){
			String str1 = matchingGraph.getEdgeSource(edge);
			String str2 = matchingGraph.getEdgeTarget(edge);
			String str1First = str1.substring(0,2);  // for instance, de
			int str1Number = Integer.parseInt(str1.substring(2));   // for instance, 12
			String str2First = str2.substring(0,2);  // for instance, su
			int str2Number = Integer.parseInt(str2.substring(2));   // for instance, 14
			if (str1First.equals("de") && str2First.equals("su")){
				int locationI = getLocation(relocateDemand, str1Number);  // start form 0
				int locationJ = getLocation(shelterRemaining, str2Number); // start from 0.
				result.get(locationI).set(locationJ, result.get(locationI).get(locationJ)+1);
				// increase the entry in result matrix by 1.
			}
			if (str1First.equals("su") && str2First.equals("de")) {
//				System.out.println(str2Number);
				int locationI = getLocation(relocateDemand, str2Number);  // start form 0
				int locationJ = getLocation(shelterRemaining, str1Number); // start from 0.
//				System.out.println(locationI);
//				System.out.println(locationJ);
				result.get(locationI).set(locationJ, result.get(locationI).get(locationJ)+1); // increase the entry in result matrix by 1.
			}
		}
		return result;
	}

	/** Get the sum of all entries of an ArrayList. */
	public int getSum(ArrayList<Integer> xList) {
		int xSum = 0;
		for (int i = 0; i < xList.size(); i++) {
			xSum += xList.get(i);
		}
		return xSum;
	}
}
