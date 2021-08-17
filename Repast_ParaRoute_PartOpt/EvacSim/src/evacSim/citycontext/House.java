package evacSim.citycontext;

import java.util.ArrayList;
import com.vividsolutions.jts.geom.Coordinate;
import evacSim.ContextCreator;

public class House {

	private int id;
	private Zone zone;
	private ArrayList<Plan> activityplan;
	private int zoneid;
	private int evacuate;
	private int evacuationtime;
	private int destzone;
	private int numPeople;

	public House() {
		this.id = ContextCreator.generateAgentID();
		this.zoneid = 0;
		this.evacuate = 0;
		this.evacuationtime = 0;
		this.destzone = 0;
		this.numPeople = 0;
	}

	public House(int ID, int zone) {
		this.id = ID;
		this.zoneid = zone;
		this.activityplan = new ArrayList<Plan>();
	}
	
	@Override
	public String toString() {
		return "House<" + id + ", " + zoneid + ">";
	}
	
	/* Getters */
	
	public int getId() {
		return id;
	}

	public Zone getZone() {
		return this.zone;
	}

	public int getZoneId() {
		return zoneid;
	}

	public int getNumPeople() {
		return this.numPeople;
	}

	public int getEvacuate() {
		return evacuate;
	}
	
	public Coordinate getCoord() {
		return ContextCreator.getZoneGeography().getGeometry(this)
				.getCentroid().getCoordinate();
	}

	public int getEvacuationTime() {
		return this.evacuationtime;
	}

	public int getDestZone() {
		return this.destzone;
	}
	
	public ArrayList<Plan> getActivityPlan() {
		return this.activityplan;
	}
	
	/* Setters */
	
	public void setId(int id) {
		this.id = id;
	}

	public void setZone(Zone z) {
		this.zone = z;
	}
	
	public void setActivityPlan(ArrayList<Integer> locations, ArrayList<Integer> durations) {
		for (int i = 0; i < locations.size(); i++) {
			Plan p = new Plan(locations.get(i), durations.get(i));
			this.activityplan.add(p);
		}
		// LZ: Set the id of the destination zone.
		this.destzone = locations.get(locations.size()-1);
	}
	
	/* Other methods */
	
	public void addActivityPlan(Plan plan) {
		this.activityplan.add(plan);
		this.destzone = plan.getLocation();
	}

	public void removePlan(Plan p) {
		this.activityplan.remove(p);
	}

}