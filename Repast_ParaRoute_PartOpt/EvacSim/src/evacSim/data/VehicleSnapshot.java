package evacSim.data;


import com.vividsolutions.jts.geom.Coordinate;

import evacSim.vehiclecontext.Vehicle;


/**
 * evacSim.data.VehicleSnapshot
 * 
 * This class is the simple data object for capturing the state 
 * of an EvacSim vehicle at a particular point in time.
 * 
 * This object is immutable and composed of simple data variables.
 * It should be trivial to serialize and reconstruct this object.
 * All values are available directly as public members and through
 * "get" methods.
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 28 June 2017
 */
public class VehicleSnapshot {
    
    /** The number identifying this vehicle within the simulation. */
    final public int id;
    
    /** The X-axis (longitude) position within the simulation. */
    final public double x;
    
    /** The Y position of the vehicle within the simulation. */
    final public double y;
    
    /** The Z position of the vehicle within the simulation. */
    final public double z;
    
    /** The current speed of the vehicle with the simulation. */
    final public float speed;
    
    /** The start time of the vehicle's current trip in the simulation. */
    final public int departure;
    
    /** The end time of the vehicle's current trip in the simulation. */
    final public int arrival;
    
    /** The total distance traveled by the vehicle in the simulation. */
    final public float distance;
    
    /** Vehicle is traveling on the last segment of its path, so close to destination. */
    final public boolean nearlyArrived;
    
    /** Vehicle routing class. */
    final public String vehicleClass;
    
    /**
     * Construct the vehicle snapshot from the given vehicle and position.
     * 
     * @param vehicle the vehicle for which a snapshot is being made.
     * @param coordinate the vehicle's current position in the simulation.
     * @throws Throwable if the supplied vehicle object is not valid.
     */
    public VehicleSnapshot(Vehicle vehicle, 
                           Coordinate coordinate) throws Throwable {
        this(vehicle.getVehicleID(),
             coordinate.x,
             coordinate.y,
             coordinate.z,
             vehicle.currentSpeed(),
             vehicle.getDepTime(),
             vehicle.getEndTime(),
             vehicle.accummulatedDistance_,
             vehicle.nearlyArrived(),
        	 vehicle.getClass().getName().toString());
    }
    
    
    /**
     * Construct the vehicle snapshot with the given ID and position.
     * 
     * @param id the identifier of the vehicle within the simulation.
     * @param x the x-axis position (longitude) within the simulation.
     * @param y the y-axis position (latitude) within the simulation.
     * @param z the z-axis position (altitude) within the simulation.
     * @param speed the current vehicle speed within the simulation.
     * @param departure the time the vehicle started moving for current trip.
     * @param arrival the expected time of arrival for vehicle's current trip.
     * @param distance the distance traveled so far for the vehicle's trip.
     * @throws Throwable if one of the supplied values is invalid.
     */
    public VehicleSnapshot(int id, 
                           double x, 
                           double y,
                           double z,
                           float speed,
                           int departure,
                           int arrival,
                           float distance,
                           boolean nearlyArrived,
                           String vehicleClass) throws Throwable {
        // all values are passed in as primitaves instead of objects,
        // so the compiler won't allow any to be null, no need to check
        
        // do basic validity checks against the values provided
        if (id < 0) {
            throw new Exception("Vehicle ID cannot be negative.");
        }
        if (Double.isNaN(x) || Double.isInfinite(x)) {
            throw new NumberFormatException("X-axis value is invalid.");
        }
        if (Double.isNaN(y) || Double.isInfinite(y)) {
            throw new NumberFormatException("Y-axis value is invalid.");
        }
        // the model doesn't use the Z-axis
        //if (Double.isNaN(z) || Double.isInfinite(z)) {
        //    throw new NumberFormatException("Z-axis value is invalid.");
        //}
        if (Float.isNaN(speed) || Float.isInfinite(speed)) {
            throw new NumberFormatException("Speed value is invalid.");
        }
        if (Float.isNaN(distance) || Float.isInfinite(distance)) {
            throw new NumberFormatException("Distance value is invalid.");
        }
        
        // TODO: check the position values are within range

        // TODO: check the speed value is reasonable for a ground vehicle
        
        // TODO: check the departure and arrival timestamps are valid
        
        // TODO: check the distance traveled value is valid
        
        // store the values in the object
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = 0.0d;
        this.speed = speed;
        this.departure = departure;
        this.arrival = arrival;
        this.distance = distance;
        this.nearlyArrived = nearlyArrived;
        this.vehicleClass = vehicleClass;
    }
    
    
    /**
     * Returns the identify of the vehicle within the simulation.
     * 
     * @return the identify of the vehicle within the simulation.
     */
    public int getId() { return this.id; }
    
    
    /**
     * Returns the X-axis (longitude?) position within the simulation.
     * 
     * @return the X-axis (longitude?) position within the simulation.
     */
    public double getX() { return this.x; }
    
    
    /**
     * Returns the Y-axis (latitude?) position within the simulation.
     *  
     * @return the Y-axis (latitude?) position within the simulation.
     */
    public double getY() { return this.y; }
    
    
    /**
     * Returns the Z-axis (altitude) position within the simulation.
     * 
     * @return the Z-axis (altitude) position within the simulation.
     */
    public double getZ() { return this.z; }
    
    
    /**
     * Returns the current speed of the vehicle within the simulation.
     * 
     * @return the current speed of the vehicle within the simulation.
     */
    public float getSpeed() { return this.speed; }
    
    
    /**
     * Returns the time of departure of the vehicle for the current trip.
     * 
     * @return the time of departure of the vehicle for the current trip.
     */
    public int getDeparture() { return this.departure; }
    
    
    /**
     * Returns the expected arrival time of the vehicle for the current trip.
     * 
     * @return the expected arrival time of the vehicle for the current trip.
     */
    public int getArrival() { return this.arrival; }
    
    
    /**
     * Returns the total distance traveled by the vehicle so far on this trip.
     * 
     * @return the total distance traveled by the vehicle so far on this trip.
     */
    public float getDistance() { return this.distance; }
    
    
    /**
     * Returns the total distance traveled by the vehicle so far on this trip.
     * 
     * @return the total distance traveled by the vehicle so far on this trip.
     */
    public boolean getNearlyArrived() { return this.nearlyArrived; }
    
    /**
     * Returns the routing class of the vehicle.
     * 
     * @return the routing class of the vehicle.
     */
    public String getvehicleClass() { return this.vehicleClass; }
}
