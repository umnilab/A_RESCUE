package evacSim.data;


import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
//import java.util.LinkedList;

import com.vividsolutions.jts.geom.Coordinate;

import evacSim.NetworkEventObject;
import evacSim.citycontext.Plan;
//import evacSim.citycontext.Road;
import evacSim.vehiclecontext.Vehicle;


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
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 28 June 2017
 */
public class TickSnapshot {
    
    /** The number of the time step of this snapshot of the simulation. */
    private final double tickNumber;
    
    /** The collection of vehicle data gathered during this time tick. */
    private HashMap<Integer, VehicleSnapshot> vehicles;
    
    /** RV:DynaDestTest: String containing relevant details of vehicles 
     * in this tick for testing purposes */
    public ArrayList<String> dynamicDestTestVehDetails = new ArrayList<String>();
    
    /** HG: The collection of event data gathered during this time tick. */
    private ArrayList<ArrayList<NetworkEventObject>> events;
    
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
        this.tickNumber = tickNumber;
        
        // setup the map for holding the vehicle data
        this.vehicles = new HashMap<Integer, VehicleSnapshot>();
        
        // HG: setup the map for holding the event data. Two subarraylists (for starting events and ending events) is created in a large arraylist
        this.events = new ArrayList<ArrayList<NetworkEventObject>>(3);
        for (int i=0;i<3;i++){
	    	this.events.add(new ArrayList<NetworkEventObject>());
	    }
    }
    
    
    /**
     * Stores the current state of the given vehicle to the tick snapshot.
     * If this vehicle has already been logged in the current snapshot, its
     * values will be overwritten.  Each vehicle can only report one snapshot
     * of current values per simulation tick.
     * 
     * @param vehicle the vehicle from which data values are being recorded.
     * @param coordinate the current vehicle position in the simulation.
     * @throws Throwable if there is an error getting data from the vehicle.
     */
    public void logVehicle(Vehicle vehicle, 
                           Coordinate coordinate) throws Throwable {
        if (vehicle == null) {
            return;
        }
        if (coordinate == null) {
            return;
        }
        
        // pull out values from the vehicle & coord we need to capture
        int id = vehicle.getVehicleID();
        double prev_x = vehicle.getPreviousEpochCoord().x;
        double prev_y = vehicle.getPreviousEpochCoord().y;
        double x = coordinate.x;
        double y = coordinate.y;
        float speed = vehicle.currentSpeed();
        double originalX =  vehicle.getOriginalCoord().x;
        double originalY = vehicle.getOriginalCoord().y;
        double destX = vehicle.getDestCoord().x;
        double destY = vehicle.getDestCoord().y;
        int nearlyArrived = vehicle.nearlyArrived();
        int vehicleClass = vehicle.getVehicleClass();
        int roadID = vehicle.getRoad().getLinkid();
        //int departure = vehicle.getDepTime();
        //int arrival = vehicle.getEndTime();
        //float distance = vehicle.accummulatedDistance_;
        //double z = coordinate.z;
        
        // TODO: perform any checks on the extracted values
        
        //HGehlot: Check if there is already a vehicleSnapshot in this tick due to visualization interpolation recording.
        //If so, then use the previous coordinates from the recorded snapshot because we had set the previous coordinates to the current coordinates 
        //when we ended the function recVehSnaphotForVisInterp() in vehicle class.
        //LZ: Not sure why this is necessary, this may require a lock of vehicles
//        if(this.getVehicleSnapshot(id)!=null){
//        	prev_x = this.getVehicleSnapshot(id).prev_x;
//        	prev_y = this.getVehicleSnapshot(id).prev_y;
//        }
        
        // create a snapshot for the vehicle and store it in the map
        VehicleSnapshot snapshot = new VehicleSnapshot(id, prev_x, prev_y,
        											   x, y, speed,
        											   originalX, originalY,
                                                       destX, destY, 
                                                       nearlyArrived,
                                                       vehicleClass,
                                                       roadID            
                                                       //departure,
                                                       //arrival,
                                                       //distance,    
                                                       );
        this.vehicles.put(id, snapshot);
        // RV:DynaDestTest
        storeDynamicDestTestDetails(vehicle, coordinate);
    }
    
    /** RV:DynaDestTest: This code is only for testing the dynamic destination feature.
     * It stores the info of a given vehicle relevant to get its trajectory. This is
     * different from VehicleSnapShot in that it does not get a lot of other info. */
    private void storeDynamicDestTestDetails(Vehicle veh, Coordinate coord) 
    		throws Throwable {
    	// resolve the current origin and destination ID
        ArrayList<Plan> plans = veh.getHouse().getActivityPlan();
        int origID = plans.get(0).getLocation();
        int destID = plans.get(1).getLocation();
        // store the vehicle ID, coordinates, speed & current destination
        this.dynamicDestTestVehDetails.add(String.format("%d,%d,%d,%.6f,%.6f,%.3f",
        		veh.getVehicleID(), origID, destID, coord.x, coord.y, veh.currentSpeed()));
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
    
      
    /**
     * Returns a list of vehicle IDs stored in the tick snapshot.
     * 
     * @return a list of vehicle IDs stored in the tick snapshot.
     */
    public Collection<Integer> getVehicleList() {
        if (this.vehicles == null || this.vehicles.isEmpty()) {
            return null;
        }
        
        return this.vehicles.keySet();
    }
    
    
    /**
     * HG: Returns a list of events stored in the tick snapshot.
     * 
     * @return a list of events stored in the tick snapshot.
     */
    public ArrayList<ArrayList<NetworkEventObject>> getEventList() {
        if (this.events == null || this.events.isEmpty()) {
            return null;
        }
        
        return this.events;
    }
    
    /**
     * Retrieves the matching vehicle from the snapshot list or null if this
     * vehicle has not yet been recorded within this tick.
     * 
     * @param id the identity of the vehicle for which a snapshot is requested.
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
    
    
    /**
     * Returns whether or not anything was recorded in the snapshot.
     * 
     * @return whether or not anything was recorded in the snapshot.
     */
    public boolean isEmpty() {
        return this.vehicles.isEmpty();
    }
    
    
    /**
     * Returns the model time step for the tick this snapshot represents.
     * 
     * @return the model time step for the tick this snapshot represents.
     */
    public double getTickNumber() {
        return this.tickNumber;
    }
}
