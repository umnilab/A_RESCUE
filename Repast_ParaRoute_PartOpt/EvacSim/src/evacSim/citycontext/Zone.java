package evacSim.citycontext;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

import evacSim.ContextCreator;

public class Zone {
	private int id;
	private Coordinate coord;

	// /////////////////////////////////////////RODRIGO/////////////////////////////////
	private int totalevacuationvehicles;
	private ArrayList<House> houses;
	private ArrayList<Integer> destination;
	private ArrayList<Integer> time;
	private int integerID;
	private String charID;

	public Zone() {
		this.id = ContextCreator.generateAgentID();
		this.houses = new ArrayList<House>();
	}

	public void setHouses(ArrayList<House> houses) {
		this.houses = houses;
	}

	public ArrayList<House> getHouses() {
		return this.houses;
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

	public int getId() {
		return id;
	}

	public Coordinate getCoord() {
		return ContextCreator.getZoneGeography().getGeometry(this)
				.getCentroid().getCoordinate();
	}

	public void setId(int id) {
		this.id = id;
	}

	// /////////////////////////////////////////RODRIGO/////////////////////////////////

	public int getIntegerID() {
		return integerID;
	}

	public void setIntegerID(int integerID) {
		this.integerID = integerID;
	}

	public String getCharID() {
		return charID;
	}

	public void setCharID(String charID) {
		this.charID = charID;
	}

	// SPECIFIC FUNCTIONS FOR ZONES
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

	public int getEvacuatingDemand() {
		return this.totalevacuationvehicles;
	}

	public ArrayList<Integer> getDestinationPerVehicle() {
		return this.destination;
	}

	public ArrayList<Integer> getEvacuationTimePerVehicle() {
		return this.time;
	}
}
