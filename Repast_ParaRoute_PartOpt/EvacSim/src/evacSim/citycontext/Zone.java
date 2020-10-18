package evacSim.citycontext;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import com.vividsolutions.jts.geom.Coordinate;

import evacSim.ContextCreator;
import evacSim.vehiclecontext.Vehicle;
import repast.simphony.space.gis.Geography;

public class Zone {
	// /////////////////////////////////////////RODRIGO/////////////////////////////////
	private int id;
	private int totalevacuationvehicles;
	private ArrayList<House> houses;
	private ArrayList<Integer> destination;
	private ArrayList<Integer> time;
	private int integerID;
	private String charID;
	
	// LZ,RV: Dynamic destination
	private int type; // 0 for normal zone, 1 for shelter
	private String name;
	private int capacity; // how many people can this shelter hold
	private int occupancy; // how many people are currently in this shelter
	// RV: map of excess vehicles waiting to be sheltered along with their occupancy
	private Queue<Vehicle> waiting;
	private Coordinate coord;
	private Road road; // road closest to this zone (used in route calculations)
	private Road departureRoad; //road with the starting point closest to the zone (used in route calculations)
//	private Coordinate nearestCoord; // nearest point on the nearest road
	private Junction downJunc; // downstream junction of its road
	// RV: for snapshot
	private int lastRecordedOccupancy = 0;
	
	// RV: Constructor for shelters only (after reading shape file)
	public Zone() {
		id = ContextCreator.generateAgentID();
		type = 1;
		houses = new ArrayList<House>();
	}
	
	// LZ: this changed
	public Zone(int integerID) {
		this.integerID = integerID;
		id = ContextCreator.generateAgentID();
		houses = new ArrayList<House>();
		type = 0;
		name = null;
		occupancy = 0;
		capacity = 0;
		waiting = new LinkedList<Vehicle>();
		System.out.print(".");
	}
	
	public Zone(int integerID, int type, int capacity) {
		this(integerID);
		this.type = type;
		this.capacity = capacity;
	}

	@Override
	public String toString() {
		return "<Zone" + this.integerID + ">";
	}

	// getters & setters
	
	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public String getCharID() {
		return charID;
	}

	public void setCharID(String charID) {
		this.charID = charID;
	}
	
	public int getIntegerId() {
		return this.integerID;
	}
	
	public void setIntegerId(int id) {
		this.integerID = id;
	}
	
	public int getType(){
		return this.type;
	}
	
	public void setType(int type) {
		this.type = type;
	}
	
	public String getName() {
		return name;
	}
	
	public void setName(String name) {
		this.name = name;
	}
	
	// geometric properties
	
	/**
	 * RV: Set the coordinate, closest road & its downstream junction
	 * (used in routing functions)
	 * */
	public void setGeometry(Geography<Zone> zoneGeography) {
		this.coord = zoneGeography.getGeometry(this).getCentroid().getCoordinate();
//		RepastEdge<?> edge = cityContext.getEdgeFromIDNum(road.getID());
//		downJunc = (Junction) edge.getTarget();
	}
	
	public void setRoad(){
		CityContext cityContext = ContextCreator.getCityContext();
		road = cityContext.findRoadAtCoordinates(coord, true); //destRoad
		departureRoad = cityContext.findRoadAtCoordinates(coord, false); //departureRoad
	}
	
	public Coordinate getCoord() {
		return coord;
	}
	
	public Road getRoad() {
		return road;
	}
	
	public Road getdepartureRoad() {
		return departureRoad;
	}
	
	public Junction getDownJunc() {
		return downJunc;
	}
	
	public void setDownJunc(Junction junc) {
		this.downJunc = junc;
	}

	/* Rodrigo: Specific for evacuation zones */
	
	public int getEvacuatingDemand() {
		return this.totalevacuationvehicles;
	}

	public ArrayList<Integer> getDestinationPerVehicle() {
		return this.destination;
	}

	public ArrayList<Integer> getEvacuationTimePerVehicle() {
		return this.time;
	}
	
	public void setEvacuationDemand() {
		this.destination = new ArrayList<Integer>();
		this.time = new ArrayList<Integer>();
		for (int i = 0; i < this.houses.size(); i++) {
			House h = this.houses.get(i);
			int thishouseholdevacuate = h.getEvacuate();
			if (thishouseholdevacuate == 1) {
				int evacuationdestination = h.getDestZone();
				int evacuationtime = h.getEvacuationTime();
				this.destination.add(evacuationdestination);
				this.time.add(evacuationtime);
			}
		}

		this.totalevacuationvehicles = this.destination.size();
	}
	
	public ArrayList<House> getHouses() {
		return this.houses;
	}
	
	public void setHouses(ArrayList<House> houses) {
		this.houses = houses;
	}

	public void printHouses() {
		System.out.println("Zone number of houses in zone "
				+ this.houses.size());
		for (int i = 0; i < this.houses.size(); i++) {
			House h = this.houses.get(i);
			System.out.println("Zone House " + h.getId() + ": is in Zone = "
					+ h.getZoneId() + ", Evacuates? " + h.getEvacuate()
					+ ", at evacuation time = " + h.getEvacuationTime()
					+ ", to destZone: " + h.getDestZone());
		}
	}
	
	/* LZ,RV: Specific for shelters */
	
	public int getCapacity() {
		return capacity;
	}
	
	public void setCapacity(int cap) {
		capacity = cap;
	}
	
	public int getOccupancy() {
		return occupancy;
	}
	
	public void setOccupancy(int occupied) {
		occupancy = occupied;
	}
	
	public int getAvailability() {
		return this.capacity - this.occupancy;
	}
	
	public int getLastRecordedOccupancy() {
		return lastRecordedOccupancy;
	}

	public void setLastRecordedOccupancy(int value) {
		this.lastRecordedOccupancy = value;
	}
	
	public Queue<Vehicle> getWaiting() {
		if (waiting == null) {
			waiting = new LinkedList<Vehicle>();
		}
		return waiting;
	}
	
	public void addWaiting(Vehicle veh) {
		if (waiting == null) {
			waiting = new LinkedList<Vehicle>();
		}
		waiting.add(veh);
	}
	
	/** 
	 * Update shelter capacity when receiving evacuees. In the case of SO
	 * shelter routing, add the vehicle to the queue of vehicles waiting to be
	 * relocated when the shelter does not have enough space.
	 * */
	public boolean receiveEvacuee() {
		if (getType() != 1) { // if this is not a shelter
			try {
				throw new NoSuchMethodException(
						"Only shelters can receive evacuees!");
			} catch (NoSuchMethodException e) {
				e.printStackTrace();
			}
		}
		if (occupancy < capacity) {
			occupancy++;
			return true;
		}
		else {
			return false;
		}
	}
}
