package evacSim.data;


import java.util.Collection;
import java.util.HashMap;

import com.vividsolutions.jts.geom.Coordinate;

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
        double x = coordinate.x;
        double y = coordinate.y;
        double z = coordinate.z;
        float speed = vehicle.currentSpeed();
        int departure = vehicle.getDepTime();
        int arrival = vehicle.getEndTime();
        float distance = vehicle.accummulatedDistance_;
        
        // TODO: perform any checks on the extracted values
        
        // create a snapshot for the vehicle and store it in the map
        VehicleSnapshot snapshot = new VehicleSnapshot(id, 
                                                       x, y, z, 
                                                       speed,
                                                       departure,
                                                       arrival,
                                                       distance);
        this.vehicles.put(id, snapshot);
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
