package evacSim.routing;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import org.apache.log4j.Logger;

import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.*;
import evacSim.vehiclecontext.Vehicle;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.operation.distance.DistanceOp;

import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.graph.Network;
import repast.simphony.space.graph.RepastEdge;

public class RouteV {
	private static Logger logger = ContextCreator.logger;
	public static Geography<Vehicle> vehicleGeography;
	public static Geography<Junction> junctionGeography;
	public static Network<Junction> roadNetwork;
	public static Geography<Road> roadGeography;
	public static CityContext cityContext;

	// geometry factory used for creating Geometries
	public static GeometryFactory geomFac;
	public static VehicleRouting vbr;
	// time that the routing information will stay valid
	public static int validRouteTime;

	/* buffers used for efficiency (so don't have to search for objects in
	 * entire space), not sure if these values are any good */
	// used when searching for a point on a road
	public static double little_buffer_distance;
	// used when searching nearby objects
	public static double big_buffer_distance; 

	/* Initialize route object */
	public static void createRoute() throws Exception {
		vehicleGeography = ContextCreator.getVehicleGeography();
		junctionGeography = ContextCreator.getJunctionGeography();
		roadNetwork = ContextCreator.getRoadNetwork();
		roadGeography = ContextCreator.getRoadGeography();
		cityContext = ContextCreator.getCityContext();
		geomFac = new GeometryFactory();
		vbr = new VehicleRouting(roadNetwork);
		little_buffer_distance = 0.0001;
		big_buffer_distance = 100;
		validRouteTime = (int) RepastEssentials.GetTickCount();
	}

	/**
	 * Update the node based routing object, update the next nearest node matrix
	 * */
	public static void updateRoute() throws Exception {
		vbr.calcRoute();
		validRouteTime = (int) RepastEssentials.GetTickCount();
	}
	
	public static int getValidTime(){
		return validRouteTime;
	}
	
	/**
	 * Perform vehicle routing: returns a path
	 * @param veh The vehicle to be routed
	 * @param destCoord
	 * @param checkZone
	 * @return
	 * @throws Exception
	 * @author Xue: Oct 2019, the return type is the HashMap, please see the
	 * computeRoute() in the VehicleRouting class
	 * @author LZ,RV: added the parameter "checkZone" which makes the function work
	 * distinctly for Vehicle.setNextRoad() and CityContext.getClosestShelter()
	 * */
	public static Map<Double,Queue<Road>> vehicleRoute(Vehicle veh,
			Zone destZone) throws Exception {

		// Find origin and destination junctions & resolving their road segments
//		Coordinate currentCoord = vehicleGeography.getGeometry(veh).getCoordinate();
//
//		if (!onRoad(currentCoord)) {
//			Coordinate nearestRoadCoord = getNearestRoadCoord(currentCoord);
//			currentCoord = nearestRoadCoord;
//		}
		Road currentRoad = veh.getRoad();
		Road destRoad = destZone.getRoad(); //cityContext.findRoadAtCoordinates(destCoord, true);
		
		Junction curDownJunc = currentRoad.getJunctions().get(1);
		// downstream junction of the destination junction along that road
		Junction destDownJunc = getNearestDownStreamJunction(destRoad);

// <<<<<<< HEAD
		if (curDownJunc.getID() == destDownJunc.getID()) {
			if (veh.getVehicleID() == GlobalVariables.Global_Vehicle_ID) {
				logger.info("Destination road reached " + destRoad.getLinkid()
					+ " from current road: " + currentRoad.getLinkid());
			}
			Map<Double, Queue<Road>> empty = new HashMap<Double, Queue<Road>>();
			empty.put(0.0, new ArrayDeque<Road>());
// =======
// 		if (curDownJunc.getID() == destDownJunc.getID() || (currentRoad.getLinkid()==104819 || currentRoad.getLinkid()==101235)) {
// //			logger.info("Destination road reached " + destRoad.getLinkid()
// //					+ " from current road: " + currentRoad.getLinkid());
// 			Map<Double, Queue<Road>> empty = new HashMap<Double, Queue<Road>>();
// 			empty.put(0.0, new ArrayDeque<Road>());
// >>>>>>> 702f235241382c0a02715a83d3a38396cf90e3d1
			return empty;
		}
		// Set the time that the routing is computed
		veh.setLastRouteTime((int) RepastEssentials.GetTickCount());
		
		return vbr.computeRoute(currentRoad, destRoad, curDownJunc, destDownJunc);
	}
	
	/** 
	 * RV: Normal shortest route between any two points at current time;
	 * needed for shortest path between zones without involvement of any vehicle
	 * */
	public static Map<Double, Queue<Road>> nonVehicleRouting(
			Coordinate origCoord, Coordinate destCoord) {
		// resolve the nearest roads & their downstream junctions
		Road origRoad = cityContext.findRoadAtCoordinates(origCoord);
		Road destRoad = cityContext.findRoadAtCoordinates(destCoord);
		Junction origDownJunc = RouteV.getNearestDownStreamJunction(origRoad);
		Junction destDownJunc = RouteV.getNearestDownStreamJunction(destRoad);
		
		return vbr.computeRoute(origRoad, destRoad, origDownJunc, destDownJunc);
	}

	public static void printRoute(List<Road> path) {
		System.out.print("Route:");
		for (Road r : path) {
			System.out.print(" " + r.getLinkid());
		}
	}
	
	/**
	 * Search all roads in the vicinity, looking for the point which is
	 * nearest the person
	 * */
	public static Coordinate getNearestRoadCoord(Coordinate coord) {
		double minDist = Double.MAX_VALUE;
		// Road nearestRoad = null;
		Coordinate nearestPoint = null;
		Point coordGeom = geomFac.createPoint(coord);
		for (Road road : roadGeography.getObjectsWithin(coordGeom.buffer(
				big_buffer_distance).getEnvelopeInternal())) {
			// XXXX: BUG:if an agent is on a really long road, the long road
			// will not be found by
			// getObjectsWithin because it is not within the buffer
			DistanceOp distOp = new DistanceOp(coordGeom,
					roadGeography.getGeometry(road));
			double thisDist = distOp.distance();
			if (thisDist < minDist) {
				minDist = thisDist;
				// nearestRoad = road;
				// Two coordinates returned by closestPoints(), need to find the
				// one which isn''t the
				// coord parameter
				for (Coordinate c : distOp.nearestPoints()) {
					if (!c.equals(coord)) {
						nearestPoint = c;
						break;
					}
				}
			}
		}

		return nearestPoint;
	}

	/**
	 * Gets the nearest junction to the current coordinate on the road that the
	 * coordinate lies on.
	 * @param coord: the coordinate we are interested in
	 * @param road: the road which this coordinate is situated on
	 * @return the junction which is closest to the coordinate
	 */
	public static Junction getNearestJunction(Coordinate coord, Road road) {
		// Find the associated edge in road network
		RepastEdge<?> edge;
		Junction j1 = null;
		Junction j2 = null;
		edge = cityContext.getEdgeFromIDNum(road.getID());
		// Find which Junction connected to the edge is closest to the
		// coordinate.
		j1 = (Junction) edge.getSource();
		j2 = (Junction) edge.getTarget();

		Geometry coordGeom = geomFac.createPoint(coord);
		Geometry geom1 = geomFac.createPoint(junctionGeography.getGeometry(j1)
				.getCoordinate());
		Geometry geom2 = geomFac.createPoint(junctionGeography.getGeometry(j2)
				.getCoordinate());
		DistanceOp dist1 = new DistanceOp(geom1, coordGeom);
		DistanceOp dist2 = new DistanceOp(geom2, coordGeom);

		if (dist1.distance() < dist2.distance())
			return j1;
		else
			return j2;
	}

	public static Junction getNearestUpStreamJunction(Road road) {

		// Find the associated edge in road network
		RepastEdge<?> edge;
		Junction j1 = null;
		edge = cityContext.getEdgeFromIDNum(road.getID());

		// Find the downstream junction
		j1 = (Junction) edge.getSource();
		return j1;
	}

	public static Junction getNearestDownStreamJunction(
			Road road) {
		// Find the associated edge in road network
		RepastEdge<?> edge;
		Junction j1 = null;
		edge = cityContext.getEdgeFromIDNum(road.getID());

		// Find the downstream junction
		j1 = (Junction) edge.getTarget();

		return j1;
	}
	
	/**
	 * Test if a coordinate is part of a road segment.
	 * @param coord: the coordinate which we want to test
	 * @return true if the coordinate is part of a road segment
	 */
	private static boolean onRoad(Coordinate coord) {
		return RoadContext.onRoad(coord);
	}
}
