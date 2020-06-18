package evacSim.citycontext;

import java.util.ArrayList;
import java.util.Random;

import com.vividsolutions.jts.geom.Coordinate;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;

public class House {

	private int id;
	private Zone zone;
	private ArrayList<Plan> activityplan;
	private Coordinate coord;
	private int zoneid;
	private int evacuate;
	private int evacuationtime;
	private int destzone;

	public House() {
		this.id = ContextCreator.generateAgentID();
		this.zoneid = 0;
		this.evacuate = 0;
		this.evacuationtime = 0;
		this.destzone = 0; // Unfinished?

	}

	public House(int ID, int zone) {
		this.id = ID;
		this.zoneid = zone;
		this.activityplan = new ArrayList<Plan>();
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

	public int getZoneId() {
		return zoneid;
	}

	public void setZone(Zone z) {
		this.zone = z;
	}

	public Zone getZone() {
		return this.zone;
	}

	public int getEvacuate() {
		return evacuate;
	}

	public void setActivityPlan(ArrayList<Integer> locations,
			ArrayList<Float> durations) {
		for (int i = 0; i < locations.size(); i++) {
			Plan p = new Plan(locations.get(i), durations.get(i));
			this.activityplan.add(p);
		}
		
		this.destzone = locations.get(locations.size()-1); //LZ: Set the id of the destination zone.
		//System.out.println(" Size of plan list: " +this.activityplan.size());
	}

	public ArrayList<Plan> getActivityPlan() {
		return this.activityplan;
	}

	public void removePlan(Plan p) {
		this.activityplan.remove(p);
	}

	public int getEvacuationTime() {
		return this.evacuationtime;
	}

	public int getDestZone() {
		return this.destzone;
	}
}