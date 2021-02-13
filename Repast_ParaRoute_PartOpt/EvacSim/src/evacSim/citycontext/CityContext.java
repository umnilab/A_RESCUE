package evacSim.citycontext;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.Queue;

import org.apache.log4j.Logger;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.operation.distance.DistanceOp;
import repast.simphony.context.DefaultContext;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.graph.Network;
import repast.simphony.space.graph.RepastEdge;
import repast.simphony.essentials.RepastEssentials;
import evacSim.*;
import evacSim.routing.RouteV;
import evacSim.vehiclecontext.*;

public class CityContext extends DefaultContext<Object> {
	
	Logger logger = ContextCreator.logger;

	// These are used so we can keep a link between Roads (in the RoadGeography)
	// and Edges in the RoadNetwork

	// Stores the linkIDs of Repast edges (Edge as key)
	private HashMap<RepastEdge<?>, Integer> edgeLinkID_KeyEdge;
	// Stores the TOIDs of edges (Edge as key)
	private HashMap<RepastEdge<?>, Integer> edgeIdNum_KeyEdge;
	// Stores the TOIDs of edges (TOID as key)
	private HashMap<Integer, RepastEdge<?>> edgeIDs_KeyIDNum;

	private HashMap<Integer, Lane> lane_KeyLaneID;
	private HashMap<Integer, Road> road_KeyLinkID;
	private HashMap<Integer, Junction> junction_KeyJunctionID;

	/*
	 * Cache the nearest road Coordinate to every house for efficiency (agents
	 * usually/always need to get from the centroids of houses to/from the
	 * nearest road.
	 */
	private Map<Coordinate, Coordinate> nearestRoadCoordCache;
	
	/* 
	 * RV: Bottom-left-most point on the road network, used to shift origin of
	 * coordinates for storage reduction in JSON for visualization
	 */
	private Coordinate origin;

	/**
	 * Constructs a CityContextContext and creates a Geography (called
	 * RoadNetworkGeography) which is part of this context.
	 */
	public CityContext() {
		super("CityContext"); // Very important otherwise repast complains
		this.edgeLinkID_KeyEdge = new HashMap<RepastEdge<?>, Integer>();
		this.edgeIdNum_KeyEdge = new HashMap<RepastEdge<?>, Integer>();
		this.edgeIDs_KeyIDNum = new HashMap<Integer, RepastEdge<?>>();
		this.lane_KeyLaneID = new HashMap<Integer, Lane>();
		this.junction_KeyJunctionID = new HashMap<Integer, Junction>();
		this.road_KeyLinkID = new HashMap<Integer, Road>();
		this.origin = new Coordinate(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	}

	public void createSubContexts() {
		this.addSubContext(new RoadContext());
		this.addSubContext(new JunctionContext());
		this.addSubContext(new LaneContext());
		this.addSubContext(new ZoneContext());
	}

	/**
	 * Actually creates the road network. Runs through the RoadGeography and
	 * generate nodes from the end points of each line. These nodes are added to
	 * the RoadNetwork (part of the JunctionContext) as well as to edges.
	 */
	public void buildRoadNetwork() {
		// Get the geographies and the network

		// Get lane geography
		Geography<Lane> laneGeography = ContextCreator.getLaneGeography();
		Iterable<Lane> laneIt = laneGeography.getAllObjects();

		// Get road geography
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		Geography<Junction> junctionGeography = ContextCreator
				.getJunctionGeography();
		JunctionContext junctionContext = ContextCreator.getJunctionContext();
		Network<Junction> roadNetwork = ContextCreator.getRoadNetwork();

		// Create a GeometryFactory so we can create points/lines from
		// the junctions and roads (this is so they can be displayed
		// on the same display to check if the network has been created
		// successfully)
		GeometryFactory geomFac = new GeometryFactory();

		Iterable<Road> roadIt = roadGeography.getAllObjects();

		for (Road road : roadIt) {
			this.road_KeyLinkID.put(road.getLinkid(), road);
			// road.printShpInput();
			// road.calcLength();
			Geometry roadGeom = roadGeography.getGeometry(road);
			// NM: First coord
			// (XXXX - check coorinates are in this order)
			Coordinate c1 = roadGeom.getCoordinates()[0];
			// Last coord
			Coordinate c2 = roadGeom.getCoordinates()[roadGeom.getNumPoints() - 1];
			
			/* RV: Set the coordinate of the origin as the bottom left most
			 * corner of the road network */
			if (c1.x < origin.x) origin.x = c1.x;
			if (c2.x < origin.x) origin.x = c2.x;
			if (c1.y < origin.y) origin.y = c1.y;
			if (c1.y < origin.y) origin.y = c2.y;

			// Create Junctions from these coordinates and add them to the
			// JunctionGeography (if they haven't been created already)
			Junction junc1, junc2;
			if (!junction_KeyJunctionID.containsKey(road.getFn())) {
				junc1 = new Junction(c1, road.getFn());
				this.junction_KeyJunctionID.put(road.getFn(), junc1);
				junctionContext.add(junc1);
				Point p1 = geomFac.createPoint(c1);
				junctionGeography.move(junc1, p1);
			} else
				junc1 = junction_KeyJunctionID.get(road.getFn());

			if (!junction_KeyJunctionID.containsKey(road.getTn())) {
				junc2 = new Junction(c2, road.getTn());
				this.junction_KeyJunctionID.put(road.getTn(), junc2);
				junctionContext.add(junc2);
				Point p2 = geomFac.createPoint(c2);
				junctionGeography.move(junc2, p2);
			} else
				junc2 = junction_KeyJunctionID.get(road.getTn());

			// Tell the road object who it's junctions are
			road.addJunction(junc1);
			road.addJunction(junc2);

			// Tell the junctions about this road
			junc1.addRoad(road);
			junc2.addRoad(road);

			RepastEdge<Junction> edge = new RepastEdge<Junction>(junc1, junc2,
					true, road.getLength()); // KD: replace weight

			// Store the road's TOID in a dictionary (one with edges as keys,
			try {
				this.edgeLinkID_KeyEdge.put(edge, road.getLinkid());
				this.edgeIdNum_KeyEdge.put(edge, road.getID());
				this.edgeIDs_KeyIDNum.put(road.getID(), edge);
			} catch (Exception e) {
				logger.error(e.getMessage());
			}
			if (!roadNetwork.containsEdge(edge)) {
				roadNetwork.addEdge(edge);
			} else {
				System.err
						.println("CityContext: buildRoadNetwork: for some reason this edge that has just been created already exists in the RoadNetwork!");
			}

		} // for road

		roadIt = roadGeography.getAllObjects(); // not sure why this is needed

		// Assign the lanes to each road
		for (Lane lane : laneIt) {
			this.lane_KeyLaneID.put(lane.getLaneid(), lane);
			for (Road road : roadIt) {
				if (road.getLinkid() == (long) lane.getLink()) {
					lane.setRoad(road);
				}
			}
			// lane.printShpInput();
		}
		
		for (Road r : roadIt) {
			// r.sortLanes();
			roadMovementFromShapeFile(r);
			laneConnectionsFromShapeFile(r);
			//r.setSpeedProfile();
		}
		
		for (Zone z : ContextCreator.getZoneGeography().getAllObjects()){
			z.setRoad();
		}
	}

	/**
	 * Get road movement from shapefile and put in an arraylist
	 */
	public void roadMovementFromShapeFile(Road road) {
		ArrayList<Integer> dsLinkIds = new ArrayList<Integer>();
		dsLinkIds.add(road.getLeft());
		dsLinkIds.add(road.getThrough());
		dsLinkIds.add(road.getRight());
		dsLinkIds.add(road.getTlinkid());
		Road opRoad = this.road_KeyLinkID.get(road.getTlinkid());
		road.setOppositeRoad(opRoad);

		for (int dsRoadId : dsLinkIds) {
			if (dsRoadId != 0) {
				Road dsRoad = this.road_KeyLinkID.get(dsRoadId);
				road.addDownStreamMovement(dsRoad);
			}
		}
	}

	public Road getRoadfromID(int roadId_) {
		// Road road = null;
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		Iterable<Road> roadIt = roadGeography.getAllObjects();

		for (Road road : roadIt) {
			if (road.getLinkid() == roadId_) {
				return road;
			}
		}
		return null;
	}

	public Lane getLanefromID(int laneID) {
		// Lane lane = null;
		Geography<Lane> laneGeography = ContextCreator.getLaneGeography();
		Iterable<Lane> laneIt = laneGeography.getAllObjects();
		for (Lane lane : laneIt) {
			if (lane.getLaneid() == laneID) {
				return lane;
			}
		}
		return null;
	}

	public void laneConnectionsFromShapeFile(Road road) {
		// ArrayList<Road> dsRoads = new ArrayList<Road>();
		ArrayList<Integer> dsLaneIds = new ArrayList<Integer>();
		int nLanes = road.getnLanes(); // number of lanes in current road
		Lane curLane, dsLane;

		/*
		 * logger.info("Road " + road.getLinkid() + " has "+
		 * nLanes+" lanes and is " + road.getLength() + " meters long");
		 */

		for (int i = 0; i < nLanes; i++) {
			curLane = road.getLanes().get(i);
			dsLaneIds.clear();
			dsLaneIds.add(curLane.getLeft());
			dsLaneIds.add(curLane.getThrough());
			dsLaneIds.add(curLane.getRight());
			/*
			 * logger.info("Lane: " + curLane.getLaneid() + " from road "
			 * + curLane.road_().getIdentifier() +
			 * " has downstream connections: ");
			 */
			for (double dsLaneId : dsLaneIds) {
				// logger.info("Connection " + dsLaneId);
				if (dsLaneId != 0) {
					dsLane = this.lane_KeyLaneID.get((int) dsLaneId);
					/*
					 * logger.info("Connection " + dsLane.getLaneid() +
					 * " Repast ID: " + dsLane.getID());
					 */
					curLane.addDnLane(dsLane);
					dsLane.addUpLane(curLane);
				}
			}
		}

		// add u-connected lanes
		if (road.getOppositeRoad() != null) {
			curLane = road.getLanes().get(0);
			if(curLane.getLength()>GlobalVariables.MIN_UTURN_LENGTH){ //LZ: Greater then 100m, otherwise the highway entrance can be link to its opposite direction...
				dsLane = road.getOppositeRoad().getLanes().get(0);
				curLane.addDnLane(dsLane);
				dsLane.addUpLane(curLane);
			}
		}
	}

	/**
	 * We update node based routing while modify road network
	/* TODO: change if want to incorporate other routing method
	 * */
	public void modifyRoadNetwork() {
//		logger.info("Modifying road network! Tick: "
//				+ System.currentTimeMillis());
		int tickcount;
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		// Network<Junction> roadNetwork = ContextCreator.getRoadNetwork();
		Iterable<Road> roadIt = roadGeography.getAllObjects();
		for (Road road : roadIt) {
			road.setTravelTime();
			// Geometry roadGeom = roadGeography.getGeometry(road);
			Junction junc1 = road.getJunctions().get(0);
			Junction junc2 = road.getJunctions().get(1);
			ContextCreator.getRoadNetwork().getEdge(junc1, junc2)
					.setWeight(road.getTravelTime());
		}

		// At beginning, initialize route object
		tickcount = (int) RepastEssentials.GetTickCount();
		logger.info("Tick: " + tickcount);
		if (tickcount < 1) {
			try {
				RouteV.createRoute();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// Update next node routing matrix
		try {
			RouteV.updateRoute();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Gets the ID String associated with a given edge. Useful for linking
	 * RepastEdges with spatial objects (i.e. Roads).
	 */
	public int getLinkIDFromEdge(RepastEdge<Junction> edge) {
		int id = 0;
		try {
			id = this.edgeLinkID_KeyEdge.get(edge);
		} catch (Exception e) {
			System.err
					.println("CityContext: getIDDromEdge: Error, probably no id found for edge "
							+ edge.toString());
			logger.error(e.getStackTrace());
		}
		return id;
	}

	public int getIdNumFromEdge(RepastEdge<Junction> edge) {
		int id = 0;
		try {
			id = this.edgeIdNum_KeyEdge.get(edge);
		} catch (Exception e) {
			System.err
					.println("CityContext: getIdNumFromEdge: Error, probably no id found for edge "
							+ edge.toString());
			logger.error(e.getStackTrace());
		}
		return id;
	}

	/**
	 * Get the repast edge with the given Road ID number This is the easiest way
	 * to find an edge using the road info
	 * @param ID of the road
	 */
	public RepastEdge<?> getEdgeFromIDNum(int id) {
		RepastEdge<?> edge = null;
		try {
			edge = this.edgeIDs_KeyIDNum.get(id);
		} catch (Exception e) {
			System.err
					.println("CityContext: getEdgeDromID: Error, probably no edge found for id "
							+ id);
			logger.error(e.getStackTrace());
		}
		return edge;
	}

	/**
	 * Returns the road which is crosses the given coordinates (Actually it just
	 * returns thenearest road to the coords)
	 */
	public int findRoadIDAtCoordinates(Coordinate coord)
			throws NullPointerException {
		if (coord == null) {
			throw new NullPointerException(
					"CityContext: findRoadAtCoordinates: ERROR: the input coordinate is null");
		}
		GeometryFactory geomFac = new GeometryFactory();
		Geography<?> roadGeography = ContextCreator.getRoadGeography();
		// Use a buffer for efficiency
		Point point = geomFac.createPoint(coord);
		Geometry buffer = point.buffer(GlobalVariables.XXXX_BUFFER);
		double minDist = Double.MAX_VALUE;
		int nearestRoadID = 0;
		// Road nearestRoad = null;
		for (Road road : roadGeography.getObjectsWithin(
				buffer.getEnvelopeInternal(), Road.class)) {
			DistanceOp distOp = new DistanceOp(point,
					roadGeography.getGeometry(road));
			double thisDist = distOp.distance();
			if (thisDist < minDist) {
				minDist = thisDist;
				nearestRoadID = road.getLinkid();
			} // if thisDist < minDist
		} // for nearRoads
		if (nearestRoadID == 0) {
			System.err
					.println("CityContext: findRoadAtCoordinates (Coordinate coord): ERROR: couldn't find a road at these coordinates:\n\t"
							+ coord.toString());
		}
		return nearestRoadID;
	}

	/**
	 * Returns the road which is crosses the given coordinates (Actually it just
	 * returns the nearest road to the coords)
	 */
	public Road findRoadAtCoordinates(Coordinate coord)
			throws NullPointerException {
		if (coord == null) {
			throw new NullPointerException(
					"CityContext: findRoadAtCoordinates: ERROR: the input coordinate is null");
		}
		GeometryFactory geomFac = new GeometryFactory();
		Geography<?> roadGeography = ContextCreator.getRoadGeography();
		// Use a buffer for efficiency
		Point point = geomFac.createPoint(coord);
		Geometry buffer = point.buffer(GlobalVariables.XXXX_BUFFER);
		double minDist = Double.MAX_VALUE;
		Road nearestRoad = null;
		for (Road road : roadGeography.getObjectsWithin(
				buffer.getEnvelopeInternal(), Road.class)) {
			DistanceOp distOp = new DistanceOp(point,
					roadGeography.getGeometry(road));
			double thisDist = distOp.distance();
			if (thisDist < minDist) {
				minDist = thisDist;
				nearestRoad = road;
			} // if thisDist < minDist
		} // for nearRoads
		if (nearestRoad == null) {
			System.err
					.println("CityContext: findRoadAtCoordinates (Coordinate coord): ERROR: couldn't find a road at these coordinates:\n\t"
							+ coord.toString());
		}
		return nearestRoad;
	}

	public Road findRoadAtCoordinates(Coordinate coord, boolean toDest)
			throws NullPointerException {
		if (coord == null) {
			throw new NullPointerException(
					"CityContext: findRoadAtCoordinates: ERROR: the input coordinate is null");
		}
		GeometryFactory geomFac = new GeometryFactory();
		Geography<?> roadGeography = ContextCreator.getRoadGeography();
		// Use a buffer for efficiency
		Point point = geomFac.createPoint(coord);
		Geometry buffer = point.buffer(GlobalVariables.XXXX_BUFFER);
		double minDist = Double.MAX_VALUE;
		Road nearestRoad = null;

		// New code when nearest road was found based on distance from junction
		// Requires the direction variable
		for (Road road : roadGeography.getObjectsWithin(
				buffer.getEnvelopeInternal(), Road.class)) {
			if (toDest) {
				Coordinate roadToNode = road.getJunctions().get(1)
						.getCoordinate();
				Point pointToNode = geomFac.createPoint(roadToNode);
				DistanceOp distOp = new DistanceOp(point, pointToNode);
				double thisDist = distOp.distance();
				if (thisDist < minDist) {
					minDist = thisDist;
					nearestRoad = road;
				} // if thisDist < minDist
			} else {
				Coordinate roadFromNode = road.getJunctions().get(0)
						.getCoordinate();
				Point pointFromNode = geomFac.createPoint(roadFromNode);
				DistanceOp distOp = new DistanceOp(point, pointFromNode);
				double thisDist = distOp.distance();
				if (thisDist < minDist) {
					minDist = thisDist;
					nearestRoad = road;
				} // if thisDist < minDist
			}

		} // for nearRoads

		if (nearestRoad == null) {
			System.err
					.println("CityContext: findRoadAtCoordinates (Coordinate coord, boolean toDest): ERROR: couldn't find a road at these coordinates:\n\t"
							+ coord.toString());
		}
		return nearestRoad;
	}

	public Road findRoadWithID(int id) {
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		for (Road road : roadGeography.getAllObjects()) {
			if (road.getLinkid() == id)
				return road;
		}
		System.err
				.println("CityContext: findRoadWithID: Error, couldn't find a road woth id: "
						+ id);
		return new Road();
	}

	public Road findRoadBetweenJunctionIDs(int junc1, int junc2) {
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		ArrayList<Junction> junctions;
		for (Road road : roadGeography.getAllObjects()) {
			junctions = road.getJunctions();
//			if (junctions.get(0).getJunctionID() == junc1 || junctions.get(0).getJunctionID() == junc2)
//				logger.info("First junction: " + junctions.get(0).getJunctionID() + " Second Junction: "+junctions.get(1).getJunctionID());

			if ((junctions.get(0).getJunctionID() == junc1
					&& junctions.get(1).getJunctionID() == junc2)) {
				return road;
			}
		}

		System.err
				.println("CityContext: findRoadBetweenJunctionIDs: Error, couldn't find a road with id: "
						+ junc1 + " to id: " + junc2);
		return null;
	}

	public Road findRoadWithLinkID(int linkID) {
		return this.road_KeyLinkID.get(linkID);
	}
	
	public Zone findHouseWithID(int id) {
		Geography<Zone> zoneGeography = ContextCreator.getZoneGeography();
		for (Zone house : zoneGeography.getAllObjects()) {
			// if (house.getId()%GlobalVariables.Total_Person_Number==id)
			if (house.getId() == id)
				return house;
		}
		System.err
				.println("CityContext: findHouseWithID: Error, couldn't find a house with id: "
						+ id);
		return null;
	}

	// /////////////////////////////////R&K////////////////////////////////////////////
	public Zone findHouseWithDestID(int destid) {
		Geography<Zone> zoneGeography = ContextCreator.getZoneGeography();
		for (Zone house : zoneGeography.getAllObjects()) {
			// if (house.getId()%GlobalVariables.Total_Person_Number==id)
			if (house.getIntegerId() == destid)
				return house;
		}
		System.err
				.println("CityContext: findHouseWithDestID: Error, couldn't find a house with id: "
						+ destid);
		return null;
	}

	// /////////////////////////////////R&K////////////////////////////////////////////
	public Road findRoadWithHouseID(int id) {
		Coordinate coord;
		Road road;
		Geography<Zone> zoneGeography = ContextCreator.getZoneGeography();
		for (Zone house : zoneGeography.getAllObjects()) {
			if (house.getId() == id) {
				coord = house.getCoord();
				road = findRoadWithID(findRoadIDAtCoordinates(coord));
				return road;
			}
		}
		System.err
				.println("CityContext: findRoadWithHouseID: Error, couldn't find a house with id: "
						+ id);
		return new Road();
	}

	public void createNearestRoadCoordCache() {
		this.nearestRoadCoordCache = new Hashtable<Coordinate, Coordinate>();
		Map<Road, Geometry> allRoads = new HashMap<Road, Geometry>();
		for (Road r : ContextCreator.getRoadGeography().getAllObjects()) {
			allRoads.put(r, ContextCreator.getRoadGeography().getGeometry(r));
		}

		for (Zone h : ContextCreator.getZoneGeography().getAllObjects()) {
			double minDist = Double.MAX_VALUE;
			Coordinate houseCoord = ContextCreator.getZoneGeography()
					.getGeometry(h).getCentroid().getCoordinate();
			Coordinate nearestPoint = null;
			Point coordGeom = ContextCreator.getZoneGeography().getGeometry(h)
					.getCentroid();
			for (Road r : allRoads.keySet()) {
				Geometry roadGeom = allRoads.get(r);
				DistanceOp distOp = new DistanceOp(coordGeom, roadGeom);
				double thisDist = distOp.distance();
				if (thisDist < minDist) {
					minDist = thisDist;
					// Two coordinates returned by closestPoints(), need to find
					// the one which isn''t the
					// coord parameter
					for (Coordinate c : distOp.nearestPoints()) {
						if (!c.equals(houseCoord)) {
							nearestPoint = c;
							break;
						}
					}
				} // if thisDist < minDist
			} // for allRoads
			this.nearestRoadCoordCache.put(houseCoord, nearestPoint);
		}// for Houses
	}

	public Coordinate getNearestRoadCoordFromCache(Coordinate c) {
		return this.nearestRoadCoordCache.get(c);
	}

	/**
	 * Return true if the roads nearest to each house have been cached.
	 */
	public boolean nearestRoadsCached() {
		return (this.nearestRoadCoordCache != null);
	}

	public static double angle(Coordinate p0, Coordinate p1) {
		double dx = p1.x - p0.x;
		double dy = p1.y - p0.y;

		return Math.atan2(dy, dx);
	}
	
	public static double squaredEuclideanDistance(Coordinate p0, Coordinate p1){
		return (p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)*(p0.y - p1.y);
	}
	
	/**
	 * LZ,RV: Returns the closest shelter for the driver upon reaching a full shelter
	 * according to current traffic conditions & strategy used (as follows):
	 * 1: driver looks for the closest shelter without knowing its availability
	 * 2: driver only looks for the closest available shelter
	 * 3: driver is recommended the best shelter by the algorithm
	 * 
	 * RV:DynaDestTest:
	 * - added the return type null when no shelter is found,
	 * - fixed the code for calculating the minimum distance (earlier, the distance was
	 * not getting updated because of a missing line),
	 * - removed the Euclidean distance calculation code
	 * 
	 * @param vehicle: vehicle to be routed
	 */
	public Zone getClosestShelter(Vehicle vehicle) throws Exception {
		// initialize
		int strategy = GlobalVariables.DYNAMIC_DEST_STRATEGY;
		double minDist = Double.MAX_VALUE;
		Zone nearestShelter = null;
		Map<Float, Queue<Road>> currentPathPlusDist = null;
//		ArrayList<Integer> visitedShelterIds = new ArrayList<Integer>();
		ArrayList<Zone> eligibleShelters = new ArrayList<Zone>();
		
		// get the shelter zones
		GeometryFactory geomFac = new GeometryFactory();
		Geography<?> zoneGeography = ContextCreator.getZoneGeography();
		
		Point point = geomFac.createPoint(vehicle.getCurrentCoord());
		Geometry buffer = point.buffer(GlobalVariables.XXXX_BUFFER); // use a buffer for efficiency
		Iterable<Zone> zones = zoneGeography.getObjectsWithin(buffer.getEnvelopeInternal(), Zone.class);

//		// get the zones that have been previously visited
//		ArrayList<Plan> plans = vehicle.getHouse().getActivityPlan();
//		for (int i = 0; i < plans.size(); i++) { // includes the current destination
//			visitedShelterIds.add(plans.get(i).getLocation());
//		}
		// prepare the list of candidate shelters depending on the strategy
		for (Zone zone : zones) {
			// consider only shelters
			if (zone.getType() == 1) {
				// consider only unvisited shelters
				if (!vehicle.getVisitedShelters().containsKey(zone.getIntegerId())) {
					// include all shelters, irrespective of occupancy
					if (strategy == 1) {
						eligibleShelters.add(zone);
					}
					// else if strategy is 2 or 3, include only shelters with
					// available capacity
					else if (zone.getOccupancy() < zone.getCapacity()) {
						eligibleShelters.add(zone);
					}
				}
			}
		}
		// if there is no candidate shelter, return null
		if (eligibleShelters.size() == 0) {
			return null;
		}
		// perform the routing for each candidate shelter and pick the closest one
		for (Zone shelter : eligibleShelters) {
			// LZ,RV: get the path to this shelter acc. to (non-Euclidean) selfish routing
			currentPathPlusDist = RouteV.vehicleRoute(vehicle, shelter);
			try {
				// get the cost of the shortest path (i.e. key of the single key-value-pair HashMap)
				for (double dist : currentPathPlusDist.keySet()) {
					// check if this path is the shortest by far
					if (dist < minDist) {
						nearestShelter = shelter;
						minDist = dist;
					}
				}
			} catch (NullPointerException e) {
				e.printStackTrace();
			}
		}
		return nearestShelter;
	}
	
	public void loadDemandOfNextHour() {
		if(ContextCreator.getZoneContext().dataset.getHousesByHour().size()>0) {
			logger.info("Loading demand for the next hour...");
			ContextCreator.getZoneContext().loadDemandofNextHour();
			ContextCreator.getVehicleContext().createVehicleContextFromActivityModels();
		}
	}
	
	/**
	 * RV:DynamicDest: Main caller of the SO routing scheduling (shelter matching)
	 * */
	public void soShelterRoutingSchedule() {

		ContextCreator.getZoneContext().soShelterMatcher.
			assignMatching(GlobalVariables.SO_SHELTER_MATCHING_ALGO);
	}
	
	public Coordinate getOrigin() {
		return this.origin;
	}
}
