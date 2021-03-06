package evacSim.data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.apache.log4j.Logger;

import com.vividsolutions.jts.geom.Coordinate;

import evacSim.ContextCreator;
import evacSim.NetworkEventObject;
import evacSim.citycontext.Plan;
import evacSim.citycontext.Road;
import evacSim.citycontext.Zone;
import evacSim.vehiclecontext.Vehicle;
import jdk.nashorn.internal.runtime.regexp.joni.exception.ValueException;

/**
 * evacSim.data.TickSnapshot
 * 
 * The tick "snapshot" is the basic bundle of data for the EvacSim output
 * data buffer.  All pieces of data collected about the simulation during
 * one model tick will be packaged into one of these objects and placed
 * into the simulation data buffer as one single item.
 * 
 * Each piece of code within the program which wishes to act upon the 
 * simulation output data will read one tick snapshot from the buffer, 
 * process it, and then read the next tick snapshot from the buffer.
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 28 June 2017
 */
public class TickSnapshot {
	
	private Logger logger = ContextCreator.logger;
    
    /** The number of the time step of this snapshot of the simulation. */
    private final int tickNumber;
    
    /** The collection of vehicle data gathered during this time tick.
     *  Access of this object is package-level so that DataCollector can modify it. */
    private List<VehicleSnapshot> vehicles;
    
    /** The collection of road data gathered during this time tick.
     *  Access of this object is package-level so that DataCollector can modify it. */
    private List<RoadSnapshot> roads;
    
    /** The collection of shelter data gathered during this time tick.
     *  Access of this object is package-level so that DataCollector can modify it. */
    private List<ShelterSnapshot> shelters;
    
    /** The collection of the generated and arrived vehicle data
     * Access of this object is package-level so that DataCollector can modify it.
     */
    private List<NewVehSnapshot> newVehs;
    private List<ArrVehSnapshot> arrVehs;
    
    /** HG: The collection of event data gathered during this time tick. */
    private List<ArrayList<NetworkEventObject>> events;

    /**
     * Additional subclasses for snapshots of specific simulation objects.
     * These have package-level access.
     */
    
    private class VehicleSnapshot {
    	/** The number identifying this vehicle within the simulation. */
        int id;  
        
        /** Coordinates of the vehicle in the previous epoch when the
         * snapshot was recorded for visualization interpolation. */
        @SuppressWarnings("unused")
		double prevX, prevY;
     
        /** Current coordinates of the vehicle. */
        double curX, curY;
        
        /** Current speed of the vehicle. */
        float speed;
        
        /** Current bearing of the vehicle. */
        double bearing;
        
//        /** IDs of the origin & destination zones of the vehicle. */
//        int origID, destID;
//        
//        /** Coordinates of the vehicle's origin & destination. */
//        double origX, origY;
//        double destX, destY;
        
//        /** Vehicle is traveling on the last segment of its path, so close to destination. */
//        int nearlyArrived;
//        
//        /** Vehicle routing class. */
//        int vehicleClass;
//        
        /** The road ID of the vehicle within the simulation. */
        int roadID;
        
        	VehicleSnapshot(Vehicle veh, Coordinate coord) {
        		id = veh.getVehicleID();
        		prevX = veh.getPreviousEpochCoord().x;
        		prevY = veh.getPreviousEpochCoord().y;
        		curX = coord.x;
        		curY = coord.y;
//        		origX = veh.getOriginalCoord().x;
//        		origY = veh.getOriginalCoord().y;
//        		destX = veh.getDestCoord().x;
//        		destY = veh.getDestCoord().y;
        		speed = veh.currentSpeed();
        		bearing = veh.getBearing();
//        		nearlyArrived = veh.nearlyArrived();
//        		vehicleClass = veh.getVehicleClass();
        		roadID = veh.getRoad().getID();
        		
        		// validity checks
        		if (id < 0) {
        			throw new ValueException("Vehicle ID cannot be negative.");
        		}
        		if (roadID < 0) {
        			throw new ValueException("Road ID cannot be negative.");
			}
        		if (Double.isNaN(curX) || Double.isInfinite(curX) ||
        				Double.isNaN(curY) || Double.isInfinite(curY)) {
                throw new ValueException("Current coordinate invalid for " + veh);
            }
//        		if (Double.isNaN(origX) || Double.isInfinite(origX) ||
//            		Double.isNaN(origY) || Double.isInfinite(origY)) {
//                throw new ValueException("Origin coordinate invalid for " + veh);
//            }
//            if (Double.isNaN(destX) || Double.isInfinite(destX) ||
//            		Double.isNaN(destY) || Double.isInfinite(destY)) {
//                throw new ValueException("Destination coordinate invalid for " + veh);
//            }
            if (Float.isNaN(speed) || Float.isInfinite(speed)) {
                throw new ValueException("Speed invalid for " + veh);
            }
            
            if (Double.isNaN(bearing) || Double.isInfinite(bearing)) {
                throw new ValueException("Bearing invalid for " + veh);
            }
    		
    		// add this object to the map of vehicle snapshots in the tick snapshot
    		vehicles.add(this); //Comment this out and see what would happen
        	}
        	
        	/** Return the string of this vehicle's attributes to be exported to JSON */
        	String getJSONLine() {
            // format 1 includes previous coordinate (for smoother visualization)
//        		jsonLine = String.format("%d,%.6f,%.6f,%.6f,%.6f,%.3f",
//        				id, prevX, prevY, curX, curY, speed);
        		
        		// format 2 excludes previous coordinate since it is not necessary
//        		return String.format("%d,%.6f,%.6f,%.3f", id, curX, curY, speed);
        		
        		/* format 3 shifts the origin of the coordinates & removes the decimal
        		 * point upto a precision of 6 decimal places to save some storage space.
        		 * Also converts speed from m/s to mm/s & converted as integer.
        		 */
//        		Coordinate origin = ContextCreator.getCityContext().getOrigin();
//        		int shiftedCurX = (int) ((curX - origin.x) * 1e6);
//        		int shiftedCurY = (int) ((curY - origin.y) * 1e6);
//        		int speed_mms = (int) (speed * 1e3);
//        		int bearing = (int) Math.round(this.bearing * 1e4);
//        		String output = id + "," + shiftedCurX + "," + shiftedCurY + "," + speed_mms + "," + bearing;
        		String output = String.format("%d,%.5f,%.5f,%.2f,%.2f",
        				id, curX, curY, speed, bearing); // 5 digits to reach precision of 1 meter.
        		if (output == null || output.isEmpty()) {
        			logger.error("empty/null string for " + id);
        		}
        		return output;
        	}
        
        	/** Return the string of this vehicles's attributes to be exported to CSV */
        	String getCSVLine() {
//        		return String.format("%d,%d,%d,%.6f,%.6f,%.3f",
//        				id, origID, destID, curX, curY, speed);
        		return String.format("%d, %.6f,%.6f,%.3f",
        				id, curX, curY, speed);
        	}
        
    }

    
    private class RoadSnapshot {
    	
    	/** Unique ID of the road */
    	final int id;
    	
    	/** Current time mean speed (unit most likely meters/second) */
    	// TODO: change this to space mean speed as this is a snapshot in space
    	final double speed;
    	
    	/** Number of vehicles currently on the road,
    	 * including the junction */
    	final int nVehicles;
    	
    	RoadSnapshot(Road road) {
    		id = road.getLinkid();
    		speed = road.calcSpeed();
    		nVehicles = road.getVehicleNum(); //LZ: Oct 19, replaced getNumVehicles with this.
    		
    		// validity checks
    		if (id < 0) {
    			throw new ValueException("Road ID negative for " + id);
    		}
    		if (speed < 0) {
    			throw new ValueException("Speed is negative for " + id);
    		}
    		if (nVehicles < 0) {
    			throw new ValueException(
    					"No. of vehicles on road is negative for " + id);
    		}
    		if (Double.isNaN(speed) || Double.isInfinite(speed)) {
    			throw new NumberFormatException("Speed is NaN for " + id);
    		}
    		
    		// add this object to the map of road snapshots in the tick snapshot
    		roads.add(this);
    	}
    	
    	/** Return the string of this road's attributes to be exported to JSON */
    	String getJSONLine() {
    		return String.format("%d,%d,%.3f", id, nVehicles, speed);
    	}
    
    }
    
    
    private class ShelterSnapshot {

    	/** Unique ID of the shelter */
    	final int id;
    	
    	/** Current no. of spaces occupied by incoming persons (1 space = 1 person) */
    	final int occupancy;
    	
    	ShelterSnapshot(Zone shelter) {
    		// check if this is a shelter or a non-shelter zone
    		if (shelter.getType() != 1) {
    			throw new ValueException(
    					"Shelter's zone type must be 1. Error in " + shelter);
    		}
    		// get the attributes
    		id = shelter.getIntegerId();
    		occupancy = shelter.getOccupancy();
    		int capacity = shelter.getCapacity();
    		
    		// validity check
    		if (occupancy < 0) {
    			throw new ValueException(
    					"Shelter availability is negative for " + shelter);
    		} else if (occupancy > capacity) {
    			throw new ValueException(
    					"Shelter availability is greater than capacity " + capacity +
    					" for " + shelter);
    		}
    		
    		// add this object to the map of shelter snapshots in the tick snapshot
    		shelters.add(this);
    	}
    	
    	/** Return the string of this shelter's attributes to be exported to JSON */
    	String getJSONLine() {
    		return String.format("%d,%d", id, occupancy);
    	}
    	
    }
    
    private class NewVehSnapshot{
    	final int id;
    	/** IDs of the origin & destination zones of the vehicle. */
    	final int origID, destID;
        
        /** Coordinates of the vehicle's origin & destination. */
//    	final double origX, origY;
//    	final double destX, destY;
        
    	NewVehSnapshot(Vehicle veh){
        	id = veh.getVehicleID();
//        	origX = veh.getOriginalCoord().x;
//    		origY = veh.getOriginalCoord().y;
//    		destX = veh.getDestCoord().x;
//    		destY = veh.getDestCoord().y;
    		// resolve the current origin and destination ID
            ArrayList<Plan> plans = veh.getHouse().getActivityPlan();
        		origID = plans.get(0).getLocation();
        		destID = plans.get(1).getLocation();
        		
        	newVehs.add(this);
        }
        
        /** Return the string of this vehicle's attributes to be exported to JSON */
    	String getJSONLine() {
    		//return String.format("%d,%d,%d,%.4f,%.4f,%.4f,%.4f", id, origID, destID, origX, origY, destX, destY);
    		return String.format("%d,%d,%d", id, origID, destID); //LZ 12/01/2020, just return IDs
    	}
    }
    
    private class ArrVehSnapshot{
    	final int id;
        
    	ArrVehSnapshot(Vehicle veh){
        	id = veh.getVehicleID();
        	arrVehs.add(this);
        }
        
        /** Return the string of this vehicle's attributes to be exported to JSON */
    	String getJSONLine() {
    		return String.format("%d", id);
    	}
    }
    
    
    /**
     * Creates the tick snapshot with the given simulation tick number and
     * initializes all the data structures for holding the data gathered
     * during the tick.
     * 
     * @param tickNumber the tick number within the simulation.
     * @throws IllegalArgumentException if the tick number given is invalid.
     */
    public TickSnapshot(double tickNumber) {
    	
        // verify the given tick number is valid and set it
        if (tickNumber < 0.0) {
            throw new IllegalArgumentException("Invalid tick number");
        }
        this.tickNumber = (int) tickNumber;
        
        // setup the map for holding the vehicle data
        this.vehicles =  Collections.synchronizedList(new ArrayList<VehicleSnapshot>());
        	
        // RV: also add the data of the roads and shelters
        this.roads = Collections.synchronizedList(new ArrayList<RoadSnapshot>());
        this.shelters = Collections.synchronizedList(new ArrayList<ShelterSnapshot>());
        
     // LZ: also add the data of the new and killed vehicles
        this.newVehs = Collections.synchronizedList(new ArrayList<NewVehSnapshot>());
        this.arrVehs = Collections.synchronizedList(new ArrayList<ArrVehSnapshot>());
        
        /* HG: setup the map for holding the event data.
         * Two sub-array lists (for starting events and ending events)
         * is created in a large array list */
        this.events = Collections.synchronizedList(new ArrayList<ArrayList<NetworkEventObject>>(3));
        for (int i=0;i<3;i++){
        		this.events.add(new ArrayList<NetworkEventObject>());
	    }
    }
    
    
    /** Record the snapshot of a vehicle from the DataCollector class. */
    public void recordVehicleSnapshot(Vehicle veh, Coordinate coord) {
    	new VehicleSnapshot(veh, coord);
    }
    
    /** Record the snapshot of a road from the DataCollector class. */
    public void recordRoadSnapshot(Road road) {
    	new RoadSnapshot(road);
    }
    
    /** Record the snapshot of a shelter from the DataCollector class. */
    public void recordShelterSnapshot(Zone shelter) {
    	new ShelterSnapshot(shelter);
    }
    
    /** Record the snapshot of a generated vehicle from the DataCollector class. */
    public void recordNewVehSnapshot(Vehicle veh) {
    	new NewVehSnapshot(veh);
    }
    
    /** Record the snapshot of a killed vehicle from the DataCollector class. */
    public void recordArrVehSnapshot(Vehicle veh) {
    	new ArrVehSnapshot(veh);
    }
    
    /** Returns the model time step for the tick this snapshot represents. */
    public int getTickNumber() {
        return this.tickNumber;
    }
    
    /** Returns whether or not anything was recorded in the snapshot. */
    public boolean isEmpty() {
        return (vehicles.isEmpty() && roads.isEmpty() && shelters.isEmpty() && newVehs.isEmpty() && arrVehs.isEmpty());
    }
    
    /**
     * Create an array of tick lines to be exported to JSON (not CSV) where
     * each line has comma-separated attributes of the simulation object type
     * 
     * @param objectType type of simulation object ("vehicle"/"road"/"shelter")
     * to be used to provide the attributes in the current tick.
     * @return lines: the array of strings each containing info of an object
     */
    public ArrayList<String> createJSONTickLines(String objectType) {
    	ArrayList<String> lines = new ArrayList<String>();
    	
    	// for vehicles
    	if (objectType == "vehicle") {
	        for (VehicleSnapshot veh : vehicles) {
	            if (veh == null) continue;
	            lines.add(veh.getJSONLine());
	        }
    	}
    	// for roads
    	if (objectType == "road") {
	        for (RoadSnapshot road : roads) {
	            if (road == null) continue;
	            lines.add(road.getJSONLine());
	        }
    	}
    	// for shelters
    	if (objectType == "shelter") {
	        for (ShelterSnapshot shelt : shelters) {
	            if (shelt == null) continue;
	            lines.add(shelt.getJSONLine());
	        }
    	}
    	// for entered vehicles
    	if (objectType == "newVeh"){
    		for(NewVehSnapshot veh: newVehs){
    			if(veh == null) continue;
    			lines.add(veh.getJSONLine());
    		}
    	}
    	// for arrived vehicles
    	if (objectType == "arrVeh"){
    		for(ArrVehSnapshot veh: arrVehs){
    			if(veh == null) continue;
    			lines.add(veh.getJSONLine());
    		}
    	}
    	return lines;
    }
    
    /**
     * Create an array of tick lines to be exported to CSV (not JSON) where
     * each line has comma-separated attributes of each vehicle in the tick snapshot.
     * 
     * @return lines: the array of strings each containing info of an object
     */
    public ArrayList<String> createCSVTickLines() {
    	
    	ArrayList<String> lines = new ArrayList<String>();
    	for (VehicleSnapshot veh: vehicles) {
            if (veh == null) continue;
            lines.add(veh.getCSVLine());
        }
    	return lines;
    }
    
    /**
     * HG: Stores the current state of the given event to the tick snapshot.
     * 
     * @param event the event for which a snapshot is being recorded.
     * @param type whether it is starting or end of the event. 1: starting, 2: ending
     * @throws Throwable if an error occurs trying to record the event.
     */
    public void logEvent(NetworkEventObject event,
            int type) throws Throwable {
    	
    	// make sure the given event object is valid
        if (event == null) {
            throw new IllegalArgumentException("No event given.");
        }
        
        //Add event to the arraylist
        if (type == 1) {//if it is event starting
            this.events.get(0).add(event); 
        } else if (type == 2) {//if it is event ending
        		this.events.get(1).add(event);
        } else {//if external event has been added to queue
        		this.events.get(2).add(event);
        }
    }
    
    /** Returns a list of vehicle IDs stored in the tick snapshot. */
    // LZ: Oct 21, update the implementation
    public List<VehicleSnapshot> getVehicleList() {
        if (this.vehicles == null || this.vehicles.isEmpty()) {
            return null;
        }
        ArrayList<Integer> vID = new ArrayList<Integer>();
        for(VehicleSnapshot v: this.vehicles){
        	vID.add(v.id);
        }
        return this.vehicles;
    }
    
    /** HG: Returns a list of events stored in the tick snapshot. */
    public List<ArrayList<NetworkEventObject>> getEventList() {
        if (this.events == null || this.events.isEmpty()) {
            return null;
        }
        
        return this.events;
    }
    
    /**
     * Retrieves the matching vehicle from the snapshot list or null if this
     * vehicle has not yet been recorded within this tick.
     * 
     * @param id: the identity of the vehicle for which a snapshot is requested.
     * @return the vehicle snapshot for the given id or null if not found.
     */
    public VehicleSnapshot getVehicleSnapshot(int id) {
        // check the map exists and is not empty
        if (this.vehicles == null || this.vehicles.isEmpty()) {
            return null;
        }
        
        // attempt to pull out the vehicle from the map with the given id
        VehicleSnapshot snapshot = this.vehicles.get(id);
        
        // return the found vehicle snapshot or null if nothing found
        return snapshot;
    }
      
}
