package evacSim.routing;

import java.util.List;

import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.*;
import evacSim.vehiclecontext.Vehicle;
import cern.colt.Arrays;

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
	public static Geography<Vehicle> vehicleGeography;
	public static Geography<Junction> junctionGeography;
	public static Network<Junction> roadNetwork;
	public static Geography<Road> roadGeography;
	public static CityContext cityContext;

	public static GeometryFactory geomFac; // Used for creating Geometries
	public static VehicleRouting vbr;
	public static int validRouteTime; // The time that the routing information will stay valid

	// Buffers used for efficiency (so don't have to search for objects in
	// entire space), not sure if these values are any good
	public static double little_buffer_distance; // Used when searching for a
													// point on a road
	public static double big_buffer_distance; // Used when searching nearby
												// objects

	/* Initialize route object */
	public static void createRoute() throws Exception {
	//	System.out.println("Creating Vehicle-based routing");
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

	/* Update the node based routing object, update the next nearest node matrix */
	public static void updateRoute() throws Exception {
//		System.out.println("Update Route");
		vbr.calcRoute();
		validRouteTime = (int) RepastEssentials.GetTickCount();
//		System.out.println("Finish update Route");
	}
	
	public static int getValidTime(){
		return validRouteTime;
	}
	
	/* Perform vehicle routing: returns a path
	 * The routing uses K-shortest path */
	public static List<Road> vehicleRoute(Vehicle vehicle, Coordinate destination)
		throws Exception {
		int time = (int) RepastEssentials.GetTickCount();
		/* The first part resolves the origin and destination road and junctions */
		/*
		 * See if the current position and the destination are on road segments.
		 */
		Coordinate currentCoord = vehicleGeography.getGeometry(vehicle)
				.getCoordinate();

		/* destination coordinate of the vehicle */
		Coordinate destCoord = vehicle.getDestCoord();

		Coordinate nearestRoadCoord;

		if (!onRoad(currentCoord)) {
			nearestRoadCoord = getNearestRoadCoord(currentCoord);
			currentCoord = nearestRoadCoord;
		}
		Road currentRoad = vehicle.getRoad();
		Road destRoad = cityContext.findRoadAtCoordinates(destCoord, true);
		
		/* current downstream junction of the road the vehicle is on */
		Junction curDownstreamJunc = vehicle.getRoad().getJunctions().get(1);
		/*
		 * current downstream junction of the destination junction the vehicle
		 * is on
		 */
		Junction destDownstreamJunc = getNearestDownStreamJunction(destCoord,
				destRoad);

		if (curDownstreamJunc.getID() == destDownstreamJunc.getID()) {
			if (vehicle.getVehicleID() == GlobalVariables.Global_Vehicle_ID) {
				System.out.println("Destination road reached " + destRoad.getLinkid() +" from current road: " + currentRoad.getLinkid());
			}
			
			return null;
		}
		
		// Set the time that the routing is computed
		vehicle.setLastRouteTime(time);
		
//		/* Debugging */
//		List<Road> path = vbr.computeRoute(currentRoad, destRoad, curDownstreamJunc, destDownstreamJunc);
//		System.out.println("Vehicle: " + vehicle.getId());
//		printRoute(path);
		
		return vbr.computeRoute(currentRoad, destRoad, curDownstreamJunc, destDownstreamJunc);
	}

	public static void printRoute(List<Road> path){
		System.out.print("Route:");
		for (Road r : path) {
			System.out.print(" " + r.getLinkid());
		}
		System.out.println();
	}
	
	
	public static Coordinate getNearestRoadCoord(Coordinate coord) {
		double time = System.currentTimeMillis();
		// Search all roads in the vicinity, looking for the point which is
		// nearest the person
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
				for (Coordinate c : distOp.closestPoints()) {
					if (!c.equals(coord)) {
						nearestPoint = c;
						break;
					}
				}
			} // end if

		} // for nearRoads

		return nearestPoint;
	}

	/**
	 * Gets the nearest junction to the current coordinate on the road that the
	 * coordinate lies on.
	 * 
	 * @param coord
	 *            The coordinate we are interested in
	 * @param road
	 *            The road which this coordinate is situated on
	 * @return the Junction which is closest to the coordinate.
	 */
	public static Junction getNearestJunction(Coordinate coord, Road road) {
		double time = System.currentTimeMillis();
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

	private static Junction getNearestUpStreamJunction(Coordinate coord, Road road) {
		double time = System.currentTimeMillis();

		// Find the associated edge in road network
		RepastEdge<?> edge;
		Junction j1 = null;
		edge = cityContext.getEdgeFromIDNum(road.getID());

		// Find the downstream junction
		j1 = (Junction) edge.getSource();
		return j1;
	}

	private static Junction getNearestDownStreamJunction(Coordinate coord,
			Road road) {
		double time = System.currentTimeMillis();

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
	 * 
	 * @param coord
	 *            The coordinate which we want to test
	 * @return True if the coordinate is part of a road segment
	 */
	private static boolean onRoad(Coordinate coord) {
		return RoadContext.onRoad(coord);
	}
}
