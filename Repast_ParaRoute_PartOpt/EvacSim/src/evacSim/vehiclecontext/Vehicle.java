package evacSim.vehiclecontext;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.operation.distance.DistanceOp;

import java.util.ArrayList;
import java.util.List;

import org.opengis.referencing.operation.MathTransformFactory;
import org.apache.commons.lang3.ArrayUtils;
import org.geotools.factory.FactoryFinder;
import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.GeodeticCalculator;
import org.geotools.referencing.ReferencingFactoryFinder;
import org.geotools.referencing.operation.matrix.GeneralMatrix;
import org.opengis.referencing.operation.MathTransform;
import org.geotools.referencing.CRS;
import org.geotools.referencing.crs.DefaultGeographicCRS;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.MathTransformFactory;
import org.opengis.referencing.operation.TransformException;

import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.lang.Math;
import java.util.concurrent.locks.ReentrantLock;

import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.CityContext;
import evacSim.citycontext.House;
import evacSim.citycontext.Junction;
import evacSim.citycontext.Lane;
import evacSim.citycontext.Plan;
import evacSim.citycontext.Road;
import evacSim.citycontext.Zone;
import evacSim.routing.RouteV;

public class Vehicle {
	private int id;
	private int vehicleID_;
	// private int startTime; //duplicated
	private int deptime;
	private int endTime;
	private int destinationZoneId;
	private int evactime;
	private int destRoadID;
	private int lastRouteTime; // The time of getting the last routing information

	private Coordinate originalCoord;
	private Coordinate destCoord;

	private float length;
	private float distance_;// distance from downstream junction
	private float currentSpeed_;
	private float accRate_;
	private float desiredSpeed_; // in meter/sec
	private static int regime_;
	private float maxAcceleration_; // in meter/sec2
	private float normalDeceleration_; // in meter/sec2
	private float maxDeceleration_; // in meter/sec2
	private float distanceToNormalStop_; // assuming normal dec is applied
	private float lastStepMove_;
	public float accummulatedDistance_;

	private double travelPerTurn;

	private boolean reachDest;
	private boolean reachActLocation;
	private boolean moveVehicle = true;// BL: to flag a vehicle for moving in
	// the next step
	private boolean onlane = false;
	private boolean atOrigin = true;

	// Need to use a better way in the future: currently use the house object to
	// load information to vehicle
	private House house;

	private Road road;
	private Road nextRoad_;
	private Lane lane;
	private Lane nextLane_;
	private Zone destZone; // RMSA
	
	/* Zhan: For vehicle based routing */
	private List<Road> roadPath; // The route is always started with the current road, whenever entering the next road, the current road will be popped out

	private List<Coordinate> coordMap;

	private Geography<Lane> laneGeography;
	private Geography<Vehicle> vehicleGeography;

	private Vehicle leading_; // leading vehicle in the lane
	private Vehicle trailing_; // trailing vehicle in the lane
	private Vehicle macroLeading_; // BL: leading vehicle on the road (all lane
	// combined)
	private Vehicle macroTrailing_; // BL: trailing vehicle on the road (all lane
	// combined)

	// BL: Variables for lane changing model
	private Lane targetLane_; // BL: this is the correct lane that vehicle
	// should change to.
	private Lane tempLane_;// BL: this is the adjection lane toward the target
	// lane direction
	private boolean correctLane; // BL: to check if the vehicle is in the
	// correct lane
	private boolean nosingFlag;// BL: if a vehicle in MLC and it can't find gap
	// acceptance then nosing is true.
	private boolean yieldingFlag; // BL: the vehicle need to yield if true

	GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
			.getLaneGeography().getCRS());
	MathTransformFactory mtFactory = ReferencingFactoryFinder.getMathTransformFactory(null);
	
	// For adaptive network partitioning
	private int Nshadow; // Number of current shadow roads in the path
	private ArrayList<Road> futureRoutingRoad;
	
	// Create a lock variable, this is to enforce concurrency within vehicle update computation
//	private ReentrantLock lock;

	public Vehicle(House h) {
		this.id = ContextCreator.generateAgentID();
		// if (ContextCreator.debug)
		// System.out.println("Generating vehicle Id: " + this.getId());
		this.house = h;

		this.length = GlobalVariables.DEFAULT_VEHICLE_LENGTH;
		this.travelPerTurn = GlobalVariables.TRAVEL_PER_TURN;
		this.maxAcceleration_ = GlobalVariables.MAX_ACCELERATION;
		this.maxDeceleration_ = GlobalVariables.MAX_DECELERATION;
		this.normalDeceleration_ = -0.5f;

		this.endTime = 0;
		// this.atOrigin = true;
		this.reachDest = false;
		this.reachActLocation = true;
		this.accRate_ = 0;
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.nextRoad_ = null;
		this.laneGeography = ContextCreator.getLaneGeography();
		this.coordMap = new ArrayList<Coordinate>();
		this.destRoadID = 0;
		// upload the vehicle into the queue of the corresponding link
		this.lastStepMove_ = 0;
		this.vehicleID_ = h.getId();
		this.accummulatedDistance_ = 0;
		this.roadPath = null;
		this.lastRouteTime = -1;
//		this.lock = ContextCreator.lock;
		this.setNextPlan();
		
		// For adaptive network partitioning
		this.Nshadow = 0;
		this.futureRoutingRoad = new ArrayList<Road>();
	}

	public void setNextPlan() {
		Plan current = this.house.getActivityPlan().get(0);

		Plan next = this.house.getActivityPlan().get(1);
		int destinationZone = next.getLocation();
		this.destinationZoneId = destinationZone;
		float duration = current.getDuration();
		int deptime = (int) ((duration * 3600)
				/ GlobalVariables.SIMULATION_STEP_SIZE + RepastEssentials
				.GetTickCount());
		this.setDepTime(deptime);
		CityContext cityContext = (CityContext) ContextCreator.getCityContext();
		this.destZone = cityContext.findHouseWithDestID(destinationZone);
		this.destCoord = this.destZone.getCoord();
		this.originalCoord = cityContext.findHouseWithDestID(
				current.getLocation()).getCoord();
		this.destRoadID = cityContext.findRoadAtCoordinates(this.destCoord,
				true).getLinkid();
		this.atOrigin = true;
		this.house.removePlan(current);
	}

	/**
	 * SH: This function enters the vehicles into the network
	 */

	public int enterNetwork(Road road) {
		// BL: Temporarily comment
		// out to test the route when vehicle re-enter network from activity
		// location
		Lane firstlane = road.firstLane();
		float roadlen = (float) firstlane.getLength();
		Vehicle front = firstlane.lastVehicle();
		this.distance_ = 0;
		if (front != null) {
			float gap = roadlen - (front.distance() + front.length());
			if (gap < this.length) {
				return 0;
			}
		}

		float capspd = road.calcSpeed();// calculate the initial speed

		currentSpeed_ = capspd; // have to change later
		this.setRoad(road);
		this.append(firstlane);
		this.setCoordMap(firstlane);
		this.appendToRoad(this.road);
		this.setNextRoad();
		this.assignNextLane();
		this.road.removeVehicleFromNewQueue(this);
		return (1);
	}

	public Road nextRoad() {
		return this.nextRoad_;
	}
	
	/* Clear the legacy impact from the shadow vehicles and future routing vehicles.
	 * Performed before next routing computation. */
	public void clearShadowImpact() {
		if (this.roadPath != null) {
			if (this.Nshadow > this.roadPath.size())
				this.Nshadow = this.roadPath.size();
			if (this.Nshadow > 0) {
				for (int i = 0; i < this.Nshadow; i++) {
					Road r = this.roadPath.get(i);
					r.decreaseShadowVehicleNum();
				}
			}
			this.Nshadow = 0;
			// Clear future routing road impact
			for (Road r : this.futureRoutingRoad) {
				r.decreaseFutureRoutingVehNum();
			}
			this.futureRoutingRoad.clear();
		}
	}
	
	/* Remove shadow vehicle count after the vehicle leaves the road */
	public void removeShadowCount(Road r) {
		if (this.Nshadow > 0) {
			r.decreaseShadowVehicleNum();
			this.Nshadow--;
		}
		
		// Remove the future routing road impact
		if (this.futureRoutingRoad.contains(r)){
			r.decreaseFutureRoutingVehNum();
			this.futureRoutingRoad.remove(r);
		}
	}
	
	/* Set shadow vehicles and future routing road */
	public void setShadowImpact() {
		this.Nshadow = GlobalVariables.N_SHADOW;
		if (this.roadPath.size() < this.Nshadow)
			this.Nshadow = this.roadPath.size();
		if (this.Nshadow > 0) {
			double cumulativeTT = 0.0;
			int foundFutureRoutingRoad = 0; // Future routing road count: number of road found in shadow roads
			for (int i=0; i < this.Nshadow; i++) {
				// Increase the shadow vehicle count: include current road
				Road r = this.roadPath.get(i);
				// Set the shadow vehicle count
				r.incrementShadowVehicleNum();
				cumulativeTT += r.getTravelTime();
				// Found the road with cumulative TT greater than than network refresh interval, use it as the future routing road
				if (foundFutureRoutingRoad < GlobalVariables.PART_REFRESH_MULTIPLIER) {
					if (cumulativeTT >= GlobalVariables.SIMULATION_NETWORK_REFRESH_INTERVAL * GlobalVariables.SIMULATION_STEP_SIZE){
						this.futureRoutingRoad.add(r);
						r.incrementFutureRoutingVehNum();
						// Update the future routing road count
						foundFutureRoutingRoad += 1;
						// Reset the cumulative TT
						cumulativeTT = 0.0;
					}
				}
			}
		} else {
			this.Nshadow = 0;
		}
		
	}
	
	public void setNextRoad() {
		try {
			if (!this.atOrigin) {
				if (this.road.getLinkid() == this.destRoadID) {
					this.nextRoad_ = null;
					return;
				}
				
				if (this.lastRouteTime < RouteV.getValidTime()) {
					// The information are outdated, needs to be recomputed
					// Check if the current lane connects to the next road in the new path
					List<Road> tempPath = RouteV.vehicleRoute(this, this.destCoord);
					if (this.checkNextLaneConnected(tempPath.get(1))){
						// If the next road is connected to the current lane, then we assign the path, otherwise, we use the old path
						// Clear legacy impact
						this.clearShadowImpact();
						this.roadPath = tempPath;
						this.setShadowImpact();
						this.lastRouteTime = (int) RepastEssentials.GetTickCount();
						this.nextRoad_ = this.roadPath.get(1);
					} else {
						// New Route will cause blocking, use the old path
						// Remove the current road from the path
						this.removeShadowCount(this.roadPath.get(0));
						this.roadPath.remove(0);
						this.nextRoad_ = this.roadPath.get(1);
					}
//					System.out.println("Debug 1: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
				} else {
					// Route information is still valid
					// Remove the current road from the path
					this.removeShadowCount(this.roadPath.get(0));
					this.roadPath.remove(0);
					this.nextRoad_ = this.roadPath.get(1);
				}	

//				if (nextRoad != null)
//					if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
//						System.out.println("Next Road ID for Vehicle: "
//								+ this.getVehicleID() + " is "
//								+ nextRoad.getLinkid());
				
				/*
				 * if (nextRoad.getLinkid() != this.road.getLinkid()) {
				 * System.out.println("Next Road ID for Vehicle: " +
				 * this.getVehicleID() + " is " + nextRoad.getLinkid());
				 * this.nextRoad_ = nextRoad; } else {
				 * System.out.println("No next road found for Vehicle " +
				 * this.vehicleID_ + " on Road " + this.road.getLinkid());
				 * this.nextRoad_ = null; }
				 */

			} else {
				// Clear legacy impact
				this.clearShadowImpact();
				// Compute new route
				this.roadPath = RouteV.vehicleRoute(this, this.destCoord);	
				this.setShadowImpact();
				this.lastRouteTime = (int) RepastEssentials.GetTickCount();
				this.atOrigin = false;
				this.nextRoad_ = roadPath.get(1);
//				System.out.println("Debug 2: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("No next road found for Vehicle "
					+ this.vehicleID_ + " on Road " + this.road.getLinkid());
			this.nextRoad_ = null;
		}
		
	}

	// BL: Append a vehicle to vehicle list in plane
	public void append(Lane plane) {
		this.lane = plane;
		Vehicle v = plane.lastVehicle();
		if (v != null) {
			this.leading(v);
			v.trailing(this);
		} else {
			plane.firstVehicle(this);
			this.leading(null);
		}
		this.trailing(null);
		plane.lastVehicle(this);
		this.onlane = true;
	}

	public void setCoordMap(Lane plane) {
		Coordinate lastCoordinate = vehicleGeography.getGeometry(this)
				.getCoordinate();
		if (plane != null) {
			coordMap.clear();
			Coordinate[] coords = laneGeography.getGeometry(plane)
					.getCoordinates();
			Coordinate start, end;
			start = coords[0];
			end = coords[coords.length - 1];
			if (!start.equals(getNearestCoordinate(lastCoordinate, start, end))) {
				ArrayUtils.reverse(coords);
				if (this.id == GlobalVariables.Global_Vehicle_ID
						&& !GlobalVariables.Debug_On_Road)
					System.out
							.println("Reversed the coordinates for this vehicle's coordinate map");
			}
			for (Coordinate coord : coords) {
				this.coordMap.add(coord);
			}
			this.distance_ = (float) plane.getLength() - lastStepMove_ / 2;
		} else {
			this.coordMap.add(destCoord);
			this.distance_ = (float) distance(lastCoordinate,
					this.coordMap.get(0))
					- lastStepMove_ / 2;
		}
	}

	/**
	 * update the coordinates of vehicle to the lane coordinate
	 * 
	 * @param: lane
	 */
	private void updateCoordMap(Lane lane) {
		double adjustdist_ = 0;
		Coordinate[] coords = laneGeography.getGeometry(lane).getCoordinates();
		coordMap.clear();

		Coordinate juncCoordinate, nextLaneCoordinate, closeVehCoordinate;
		juncCoordinate = coords[coords.length - 1];

		// SH Temp
		// Geography<Vehicle> vehicleGeography;
		// vehicleGeography = ContextCreator.getVehicleGeography();

		Coordinate vcoordinate = vehicleGeography.getGeometry(this)
				.getCoordinate();

		for (int i = 0; i < coords.length - 1; i++) {
			nextLaneCoordinate = getNearestCoordinate(juncCoordinate,
					coords[i], coords[i + 1]);
			// nextLaneCoordinate = coords[i];
			closeVehCoordinate = getNearestCoordinate(juncCoordinate,
					vcoordinate, nextLaneCoordinate);
			if (!closeVehCoordinate.equals(vcoordinate))
				coordMap.add(nextLaneCoordinate);
		}
		// BL: update distance due to the additional length covers by lane width
		if (coordMap.size() > 0) {
			adjustdist_ = distance(vcoordinate, coordMap.get(0));
			double cos = (double) GlobalVariables.LANE_WIDTH / adjustdist_;
			cos = Math.acos(cos);
			adjustdist_ = adjustdist_ * (1 - Math.sin(cos));
			if (this.id == 178) {
				System.out.println("Distance adjusted for vehicle: " + this.id
						+ " is " + adjustdist_);
			}
		}
		this.distance_ += (float) adjustdist_;
	}

	public void calcState() {
		double time = System.currentTimeMillis();
		// SH-- right now there is only one function we may also invoke lane
		// changing decision here
		// SHinvoke accelerating decision
		this.makeAcceleratingDecision();

		double time2 = System.currentTimeMillis();

		if (this.road.getnLanes() > 1 && this.onlane) {
			this.makeLaneChangingDecision();
		}
	}

	/*
	 * -------------------------------------------------------------------- The
	 * Car-Following model calculates the acceleration rate based on interaction
	 * with other vehicles. The function returns a the most restrictive
	 * acceleration (deceleration if negative) rate among the rates given by
	 * several constraints. This function updates accRate_ at the end.
	 * --------------------------------------------------------------------
	 */

	public void makeAcceleratingDecision() {
		// int id = GlobalVariables.Global_Vehicle_ID;
		Vehicle front = this.vehicleAhead();
		float aZ = this.accRate_; /* car-following */
		float acc = this.maxAcceleration(); /* returned rate */
		/*
		 * BL: vehicle will have acceleration rate based on car following if it
		 * is not in yielding or nosing state
		 */
		if (!this.nosingFlag && !this.yieldingFlag) {
			aZ = this.calcCarFollowingRate(front);
		} else if (this.nosingFlag) {
			aZ = this.nosing();
		} else if (this.yieldingFlag) {
			aZ = this.yielding();
		}

		if (aZ < acc)
			acc = aZ; // car-following rate

		if (acc < maxDeceleration_) {
			acc = maxDeceleration_;
		}

		accRate_ = acc;
	}

	public float calcCarFollowingRate(Vehicle front) {
		// SH-if there is no front vehicle the car will be in free flow regime
		// and have max acceleration

		if (front == null) {
			regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			return (this.maxAcceleration_);
		}

		float acc;

		float space = gapDistance(front);
		// if (ContextCreator.debug)
		// System.out.println("Gap with front vehicle :"+ space);

		float speed = currentSpeed_ == 0f ? 0.00001f : currentSpeed_;

		float headway = 2.0f * space / (speed + currentSpeed_);
		float hupper, hlower;

		float AlphaDec = GlobalVariables.ALPHA_DEC;
		float BetaDec = GlobalVariables.BETA_DEC;
		float GammaDec = GlobalVariables.GAMMA_DEC;

		float AlphaAcc = GlobalVariables.ALPHA_ACC;
		float BetaAcc = GlobalVariables.BETA_ACC;
		float GammaAcc = GlobalVariables.GAMMA_ACC;

		// float dens = this.road.density();

		hupper = GlobalVariables.H_UPPER;
		hlower = GlobalVariables.H_LOWER;

		// There will be three regimes emergency/free-flow/car-following regime
		// depending on headway

		// Emergency regime
		if (headway < hlower) {
			float dv = currentSpeed_ - front.currentSpeed_;

			if (dv < GlobalVariables.SPEED_EPSILON) { // the leader is
				// decelerating
				acc = front.accRate_ + 0.25f * normalDeceleration_;
			} else {
				if (space > 0.01) {
					acc = front.accRate_ - 0.5f * dv * dv / space;
				} else {
					float dt = GlobalVariables.SIMULATION_STEP_SIZE;
					float v = front.currentSpeed_ + front.accRate_ * dt;
					space += 0.5f * (front.currentSpeed_ + v) * dt;
					acc = brakeToTargetSpeed(space, v);
				}
			}
			acc = Math.min(normalDeceleration_, acc);
			regime_ = GlobalVariables.STATUS_REGIME_EMERGENCY;
		}
		// Free-flow regime
		else if (headway > hupper) { // desired speed model will do
			if (space > distanceToNormalStop_) {
				acc = maxAcceleration_;
				regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			} else {
				float dt = GlobalVariables.SIMULATION_STEP_SIZE;
				float v = front.currentSpeed_ + front.accRate_ * dt;
				space += 0.5 * (front.currentSpeed_ + v) * dt;
				acc = brakeToTargetSpeed(space, v);
				regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			}
		}
		// SH: We are using Herman model
		else {
			float dv = front.currentSpeed_ - currentSpeed_;
			if (dv < 0) {
				acc = dv * AlphaDec * (float) Math.pow(currentSpeed_, BetaDec)
						/ (float) (Math.pow(space, GammaDec));

			} else if (dv > 0) {
				acc = dv * AlphaAcc * (float) Math.pow(currentSpeed_, BetaAcc)
						/ (float) (Math.pow(space, GammaAcc));
			} else { // uniform speed
				acc = 0.0f;
			}
			regime_ = GlobalVariables.STATUS_REGIME_CARFOLLOWING;
		}
		return acc;
	}

	public float brakeToTargetSpeed(float s, float v) {
		if (s > GlobalVariables.FLT_EPSILON) {
			float v2 = v * v;
			float u2 = currentSpeed_ * currentSpeed_;
			float acc = (v2 - u2) / s * 0.5f;
			return acc;
		} else {
			float dt = GlobalVariables.SIMULATION_STEP_SIZE;
			if (dt <= 0.0)
				return maxAcceleration_;
			return (v - currentSpeed_) / dt;
		}
	}

	public Vehicle vehicleAhead() {
		if (leading_ != null) {
			return leading_;
		} else if (nextLane_ != null) {
			if (nextLane_.lastVehicle() != null)
				return nextLane_.lastVehicle();
			else
				return null;
		} else {
			return null;
		}
	}

	public Junction nextJuction() {
		Junction nextJunction;
		if (this.nextRoad() == null)
			return null;
		if (this.nextRoad().getJunctions().get(0).getID() == this.road
				.getJunctions().get(1).getID())
			nextJunction = this.road.getJunctions().get(1);
		else
			nextJunction = this.road.getJunctions().get(0);
		return nextJunction;
	}

	public float gapDistance(Vehicle front) {
		float headwayDistance;
		if (front != null) { /* vehicle ahead */
			if (this.lane.getID() == front.lane.getID()) { /* same lane */
				headwayDistance = this.distance_ - front.distance()
						- front.length();
			} else { /* different lane */
				headwayDistance = this.distance_
						+ ((float) front.lane.length() - front.distance() - front
								.length());
			}
		} else { /* no vehicle ahead. */
			headwayDistance = Float.MAX_VALUE;
		}
		return (headwayDistance);
	}

	public void makeLaneChangingDecision() {
		if (this.distFraction() < 0.5) { 
			// Halfway to the downstream intersection, only mantatory LC allowed, check the correct lane
			if (this.isCorrectLane() != true) { // change lane if not in correct
				// lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					System.out.println("Vehicle " + this.getId()
							+ "has no lane to change");
					System.out.println("this vehicle is on road "
							+ this.road.getID() + " which has "
							+ this.road.getnLanes()
							+ " lane(s) and I am on lane "
							+ this.road.getLaneIndex(this.lane));
				}
			}
		} else {
			if (this.distFraction() > 0.75) {
				// First 25% in the road, do discretionary LC with 50% chance
				double laneChangeProb1 = GlobalVariables.RandomGenerator
						.nextDouble();
				// The vehicle is at beginning of the lane, it is free to change lane
//				Lane tarLane = this.findBetterLane();
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					if (laneChangeProb1 < 1.0)
						this.discretionaryLC(tarLane);
				}
			} else {
				// First 25%-50% in the road, we do discretionary LC but only to correct lanes with 100% chance
				double laneChangeProb2 = GlobalVariables.RandomGenerator
						.nextDouble();
				// The vehicle is at beginning of the lane, it is free to change lane
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					if (laneChangeProb2 < 1.0)
						this.discretionaryLC(tarLane);
				}
				
			}
		}
	}
	
	public void makeLaneChangingDecision_oldCode() { //OLD CODE
		// if not using dynamic routing alg, then check the correct lane
		if (this.distFraction() > 0.70) {
			double laneChangeProb = GlobalVariables.RandomGenerator
					.nextDouble();
			// The vehicle is at beginning of the lane, it is free to change
			// lane
			Lane tarLane = this.findBetterLane();
			if (tarLane != null) {
				// TODO: input a PDF of normal distribution
				if (laneChangeProb > 0.5)
					this.discretionaryLC(tarLane);
			}
		} else if (this.distFraction() > 0.50) {
			// vehicle is half the lane length to downstream node
			// only DLC if the target lane is also correct lane
			// otherwise only MLC
			if (this.isCorrectLane() != true) {
				// change lane if not in correct lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					System.out.println("Vehicle " + this.getId()
							+ "has no lane to change");
				}
			} else {
				// if the vehicle is already in correct lane.
				// it seek for another connected lane with better traffic
				// condition
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					this.discretionaryLC(tarLane);
				}
			}
		} else {
			// when the vehicle is close to downstream node, only DLC is
			// accepted
			if (this.isCorrectLane() != true) {
				// change lane if not in correct lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					System.out.println("Vehicle " + this.getId()
							+ "has no lane to change");
				}
			}
		}
	}
	

	/*
	 * Calculate new location and speed after an iteration based on its current
	 * location, speed and acceleration. The vehicle will be removed from the
	 * network if it arrives its destination.
	 */
	public void travel() {
		double time = System.currentTimeMillis();
		this.endTime++;
		try {
			if (!this.reachDest && !this.reachActLocation) {
				this.move(); // move the vehicle towards their destination
				// BL: if the vehicle travel too fast, it will change the
				// marcroList of the road.
				this.advanceInMacroList();
				// up to this point this.reachDest == false;
				if (this.nextRoad() == null) {
					this.checkAtDestination();
				}
			}

		} catch (Exception e) {
			System.err.println("Vehicle " + this.getVehicleID()
					+ " had an error while travelling on road: "
					+ this.road.getLinkid() + "with next road: "
					+ this.nextRoad().getLinkid());
			e.printStackTrace();
			RunEnvironment.getInstance().pauseRun();
		}
	}

	public void move() {
		Coordinate currentCoord = null;
		Coordinate target = null;
		float dx = 0;
		double maxMove = GlobalVariables.FREE_SPEED
				* GlobalVariables.SIMULATION_STEP_SIZE;
		boolean travelledMaxDist = false; // True when traveled maximum dist
		// this iteration
		double time = System.currentTimeMillis(); // used for debugging
		double duration = 0.0f;
		
		/*
		 * For debuging: print out the current road and next junction ID of the
		 * vehicle
		 */
		// System.out.println("Vehicle: " + this.vehicleID_ + " on Road: "
		// + this.road.getID() + " next Road: " + this.nextRoad().getLinkid());

		/*
		 * Calling the static method in Signal class If you comment out this
		 * line it would be just vehicle movement without signals
		 */
		while (!travelledMaxDist) {
			GeometryFactory geomFac = new GeometryFactory();
			float oldpos = distance(); // have to check
			float step = GlobalVariables.SIMULATION_STEP_SIZE;
			if (currentSpeed_ < GlobalVariables.SPEED_EPSILON
					&& accRate_ < GlobalVariables.ACC_EPSILON) {
				return; // does not move
			}
			float oldv = currentSpeed_; // Velocity at the beginning
			float oldd = distance_; // Position at the beginning
			// distance can be traveled in last time interval based on
			// the speed and the acceleration rate calculated earlier.

			// Maximum distance it can travel in last time interval
			float dv = accRate_ * step;

			if (dv > -currentSpeed_) { // still moving at the end of the cycle
				dx = currentSpeed_ * step + 0.5f * dv * step;

			} else { // stops before the cycle end
				dx = -0.5f * currentSpeed_ * currentSpeed_ / accRate_;
			}

			// Solve the crash problem
			Vehicle front = this.vehicleAhead();
			if (front != null) {
				float gap = gapDistance(front);
				if (gap > this.length()) {
					dx = Math.min(dx, gap - this.length());
				} else {
					dx = 0.0f;
				}
			}
			// actual acceleration rate applied in last time interval.
			accRate_ = 2.0f * (dx - oldv * step) / (step * step);

			// update speed
			currentSpeed_ += accRate_ * step;
			if (currentSpeed_ < GlobalVariables.SPEED_EPSILON) {
				currentSpeed_ = 0.0f;
				accRate_ = 0.0f; // no back up allowed
			} else if (currentSpeed_ > this.road.getFreeSpeed()
					&& accRate_ > GlobalVariables.ACC_EPSILON) {
				currentSpeed_ = (float) this.road.getFreeSpeed();
				accRate_ = (currentSpeed_ - oldv) / step;
			}

			if (dx < 0.0f) {
				lastStepMove_ = 0;
				return;
			}
			/*
			 * check if the vehicle's current pos. is under some threshold, i.e.
			 * it will move to the next road 1. search for the junction 2.
			 * searchfor the road connected to the junction which has the same
			 * id with next road id3. find the first vehicle of all the lanes of
			 * those roads4. find the minimum of the distances of these vehicles
			 * and this vehicle.5. if
			 */
			// Hack for intersection
			if (distance_ < maxMove && !onlane) {
				this.moveVehicle = true;
				/*
				 * System.out.println(this.id + " " + this.road.getIdentifier()
				 * + " " + this.distance_ + " " + moveVehicle);
				 */
				ArrayList<Road> allRoads_ = new ArrayList<Road>();
				ArrayList<Road> allApproaches_ = new ArrayList<Road>();
				if (this.nextJuction() != null) {
					for (Road r : this.nextJuction().getRoads()) {
						if (r.getID() != this.road.getID()
								&& r.getID() != this.nextRoad().getID()) {
							allRoads_.add(r);
						}
					}
					if (allRoads_.size() != 0) {
						for (Road r : allRoads_) {
							if (this.nextJuction().getID() == r.getJunctions()
									.get(1).getID()) {
								allApproaches_.add(r);
							}
						}
						if (allApproaches_.size() != 0) {
							for (Road r : allApproaches_) {
								for (Lane l : r.getLanes()) {
									if (l.firstVehicle() != null) {
										double dis = l.firstVehicle()
												.distance();
										if (dis < this.distance())
											this.moveVehicle = false;
									}
								}
							}
						}
					}
				}
			}

			if (this.moveVehicle == false) {
				lastStepMove_ = 0;
				return;
			}
			// update position
			distance_ -= dx;

			double distTravelled = 0; // The distance traveled so far

			// Current location
			currentCoord = vehicleGeography.getGeometry(this).getCoordinate();

			/*
			 * TODO: Solve the no coordinate problem (10/23/2012) need to recall
			 * last coordinate if it was deleted.
			 */

			target = this.coordMap.get(0);

			Geometry currentGeom = geomFac.createPoint(currentCoord);
			Geometry targetGeom = geomFac.createPoint(target);

			// SH
			// new block of code for using new distance
			double[] distAndAngle = new double[2];

			time = System.currentTimeMillis();
			this.distance(currentCoord, target, distAndAngle);
			duration = System.currentTimeMillis() - time;

			double distToTarget = distAndAngle[0];

			// If we can get all the way to the next coords on the route then
			// just go there
			if (distTravelled + distToTarget < dx) {
				distTravelled += distToTarget;
//				this.lock.lock();
				vehicleGeography.move(this, targetGeom);
//				this.lock.unlock();
				// this.accummulatedDistance_+=ContextCreator.convertToMeters(distToTarget);
				// if(this.vehicleID_ == GlobalVariables.Global_Vehicle_ID)
				// System.out.println("distToTarget(move)= "+ContextCreator.convertToMeters(distToTarget));
				// this.route.remove();
				Coordinate coor = this.coordMap.get(0);
				this.coordMap.remove(0);
				if (this.coordMap.isEmpty()) {
					if (this.nextRoad() != null) {
						if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID) {
							System.out
									.println("+++++++++ I am moving but coordinate map is empty+++++++++++++");
							System.out.println("My next road is: "
									+ this.nextRoad().getLinkid());
						}
						if (this.onlane) {
							this.coordMap.add(coor);
							if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID) {
								System.out.println("Vehicle: "
										+ this.getVehicleID()
										+ " is at the end of the road "
										+ this.getRoad().getLinkid()
										+ " and appending to a junction");
							}
							if (this.appendToJunction(nextLane_) == 0)
								break;
						} else {
							if (this.changeRoad() == 0)
								break;
						}

					} else {
						// Lane nextLane = null;
						if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID) {
							System.out
									.println("+++++++++ I am moving but coordinate map is empty and no next lane found +++++++++++++");
						}
						this.setCoordMap(this.lane);
					}
				}
			}

			// Otherwise move as far as we can towards the target along the road
			// we're on Get the angle between the two points
			// (current and target)
			// (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)
			else {
				// double angle = angle(target, currentCoord) + Math.PI;
				// angle() returns range from -PI->PI,
				// but moveByVector wants range 0->2PI
				// vehicleGeography.moveByVector(this, dx, angle);

				time = System.currentTimeMillis();

//				// Old implementation: This approach seems to be fast but still have some issues.
//				vehicleGeography.moveByVector(this, dx, distAndAngle[1]);
				
				/*// Zhan: implementation 1: adding concurrency locks
				this.lock.lock();
				vehicleGeography.moveByVector(this, dx, distAndAngle[1]);
				this.lock.unlock();*/
				
				// Zhan: implementation 2: thread safe version of the moveByVector
				this.moveVehicleByVector(dx, distAndAngle[1]);
				
				this.accummulatedDistance_ += dx;
				// SH: Trying to remove this function context creator but this
				// is not working either
				travelledMaxDist = true;
			} // else

			lastStepMove_ = dx;
//			printGlobalVehicle(dx);

		}
		return;
	}

	public void printGlobalVehicle(float dx) {
		if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID) {
			System.out.println("Next Road ID for vhielc: " + this.vehicleID_
					+ " is: " + this.nextRoad().getLinkid() + " step size is: "
					+ dx + " distance to downstream: " + this.distance_ + 
					" next lane: " + this.nextLane_
					+ " current speed: " + this.currentSpeed_);
		}
	}

	public void primitiveMove() {
		Coordinate currentCoord = null;
		Coordinate target = null;
		if (this.atDestination()) {
			return;
		}
		boolean travelledMaxDist = false; // True when traveled max dist this
		// iteration
		double time = System.currentTimeMillis(); // used for debugging

		while (!travelledMaxDist) {
			// SH Temp
			// Geography<Vehicle> vehicleGeography = ContextCreator
			// .getVehicleGeography();
			GeometryFactory geomFac = new GeometryFactory();
			double distTravelled = 0.0f;
			currentCoord = vehicleGeography.getGeometry(this).getCoordinate();

			if (this.coordMap.size() > 0) {
				target = this.coordMap.get(0);
			} else {
				lane = this.road.firstLane();
				this.setCoordMap(lane);
				target = this.coordMap.get(0);
			}

			// target = this.route.routeMap().get(0);

			Geometry currentGeom = geomFac.createPoint(currentCoord);
			Geometry targetGeom = geomFac.createPoint(target);

			// double distToTarget = DistanceOp.distance(currentGeom,
			// targetGeom);

			double[] distAndAngle = new double[2];
			this.distance(currentCoord, target, distAndAngle);
			double distToTarget = distAndAngle[0];

			if (distTravelled + distToTarget < travelPerTurn) {
				distTravelled += distToTarget;
//				this.lock.lock();
				vehicleGeography.move(this, targetGeom);
//				this.lock.unlock();
				// this.accummulatedDistance_+=ContextCreator.convertToMeters(distToTarget);
				// if(this.vehicleID_ == GlobalVariables.Global_Vehicle_ID)
				// System.out.println("distToTarget= "+ContextCreator.convertToMeters(distToTarget));
				this.coordMap.remove(0);
				// this.route.remove();
			}
			// Otherwise move as far as we can towards the target along the road
			// we're on
			// Get the angle between the two points (current and target)
			// (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)

			else {
				// double angle = angle(target, currentCoord) + Math.PI; //
				// angle()
				// returns range from -PI->PI, but moveByVector wants range
				// 0->2PI
				double distToTravel = travelPerTurn - distTravelled;
				// Need to convert distance from long/lat degrees to meters
				double distToTravelM = ContextCreator
						.convertToMeters(distToTravel);
				// System.out.println("Angle: "+angle);
				this.accummulatedDistance_ += distToTravelM;
				// if(this.vehicleID_ == GlobalVariables.Global_Vehicle_ID)
				// System.out.println("Vehicle ID: " + this.getVehicleID()
				// +" disToTravelM= "+distToTravelM);
				
				// Zhan: implementation 1: add locks to enforce concurrency
				/*this.lock.lock();
				vehicleGeography.moveByVector(this, distToTravelM,
						distAndAngle[1]);
				this.lock.unlock();*/
				
				// Zhan: implementation 2: thread safe version of the moveByVector
				this.moveVehicleByVector(distToTravelM, distAndAngle[1]);
				
				travelledMaxDist = true;
			}
		}
		return;
	}

	/**
	 * SH: This function change the vehicle from its current road to the next
	 * road.
	 * 
	 * @return
	 */

	public int changeRoad() {
		// SH- check if the vehicle has reached the destination or not
		if (this.atDestination()) {
			this.removeFromLane();
			this.removeFromMacroList();
			this.clearShadowImpact(); // ZH: Clear shadow impact if already reaches destination
			return 0; // only one will reach destination once
		} else if (this.nextRoad_ != null) {
			// BL: check if there is enough space in the next road to change to
			if (this.entranceGap(nextLane_) < 1.2 * GlobalVariables.DEFAULT_VEHICLE_LENGTH) {
				if (this.coordMap.isEmpty()) {
					Coordinate coor = null;
					Coordinate[] coords = laneGeography.getGeometry(nextLane_)
							.getCoordinates();
					Coordinate lastCoordinate = null, start, end;
					// SH Temp
					// Geography<Vehicle> vehicleGeography;
					// vehicleGeography = ContextCreator.getVehicleGeography();
					lastCoordinate = vehicleGeography.getGeometry(this)
							.getCoordinate();
					start = coords[0];
					end = coords[coords.length - 1];
					coor = this
							.getNearestCoordinate(lastCoordinate, start, end);
					coordMap.add(coor);
				}
				return 0;
			} else {
//				float maxMove = GlobalVariables.FREE_SPEED
//						* GlobalVariables.SIMULATION_STEP_SIZE;
				// if (distance_ < maxMove && !onlane) {
				if (!onlane) {
					this.setCoordMap(nextLane_);
					this.moveVehicle = true;
					this.removeFromLane();
					this.removeFromMacroList();
//					this.lock.lock();
					this.appendToRoad(this.nextRoad());
					this.append(nextLane_);
//					this.lock.unlock();
					this.setNextRoad();
					this.assignNextLane();
					// Reset the desired speed according to the new road
					this.desiredSpeed_ = (float) (this.road.getFreeSpeed());
					return 1;
				}
			}
		}
		return 0;
	}

	public int closeToRoad(Road road) {
		// SH Temp
		Coordinate currentCoord = vehicleGeography.getGeometry(this)
				.getCoordinate();
		GeometryFactory geomFac = new GeometryFactory();
		Coordinate nextCoord;

		if (this.coordMap == null)
			return 0;
		else
			nextCoord = this.coordMap.get(0);

		Geometry geom1 = geomFac.createPoint(currentCoord);
		Geometry geom2 = geomFac.createPoint(nextCoord);
		DistanceOp dist1 = new DistanceOp(geom1, geom2);

		if (dist1.distance() < GlobalVariables.TRAVEL_PER_TURN) {
			return 1;
		} else
			return 0;
	}

	public boolean atDestination() {
		return this.reachDest;
	}

	public boolean atActivityLocation() {
		return this.reachActLocation;
	}

	public boolean checkAtDestination() {
		double maxMove = GlobalVariables.FREE_SPEED
				* GlobalVariables.SIMULATION_STEP_SIZE;

		if (distance_ < maxMove) {
			this.setReachDest();
			return true;
		}
		return false;
	}

	public float maxAcceleration() {
		return maxAcceleration_;
	}

	public void appendToRoad(Road road) {
		this.road = road;
		this.appendToMacroList(road);
		this.reachActLocation = false;
	}

	public void appendToMacroList(Road road) {
		macroLeading_ = road.lastVehicle();
		macroTrailing_ = null;
		road.lastVehicle(this);
		if (macroLeading_ != null) // there is a vehicle ahead
		{
			macroLeading_.macroTrailing_ = this;
		} else {
			road.firstVehicle(this);
		}
		// after this appending, update the number of vehicles
		Vehicle pv = road.firstVehicle();
		int nVehicles_ = 0;
		while (pv != null) {
			nVehicles_++;
			pv = pv.macroTrailing_;
		}
		road.setNumberOfVehicles(nVehicles_);
	}

	public void leading(Vehicle v) {
		if (v != null)
			this.leading_ = v;
		else
			this.leading_ = null;
	}

	public Vehicle leading() {
		return leading_;
	}

	public Vehicle macroLeading() {
		return macroLeading_;
	}

	public Vehicle trailing() {
		return trailing_;
	}

	public Vehicle macroTrailing() {
		return macroTrailing_;
	}

	public void trailing(Vehicle v) {
		if (v != null)
			this.trailing_ = v;
		else
			this.trailing_ = null;
	}

	public void setDepTime(int time) {
		this.deptime = time;
	}

	public int getDepTime() {
		return this.deptime;
	}

	public void setRoad(Road road) {
		this.road = road;
		this.currentSpeed_ = (float) this.road.getFreeSpeed();
	}

	public Road getRoad() {
		return road;
	}

	public float distance() {
		return distance_;
	}

	public float distFraction() {
		if (distance_ > 0)
			return distance_ / (float) this.road.length();
		else
			return 0;
	}

	public float length() {
		return length;
	}

	public void setGeography() {
		// SH temp
		vehicleGeography = ContextCreator.getVehicleGeography();

	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public Lane getLane() {
		return lane;
	}

	public int getVehicleID() {
		return this.vehicleID_;
	}

	public int getDestinationID() {
		return this.destZone.getIntegerID();
	}

	public House getHouse() {
		return this.house;
	}

	public Coordinate getOriginalCoord() {
		return originalCoord;
	}

	public Coordinate getDestCoord() {
		return this.destCoord;
	}

	public void setOriginalCoord(Coordinate coord) {
		this.originalCoord = coord;
	}

	public void setReachDest() {
		Coordinate target = null;
		GeometryFactory geomFac = new GeometryFactory();
		this.removeFromLane();
		this.removeFromMacroList();
		target = this.destCoord;
		Geometry targetGeom = geomFac.createPoint(target);
//		this.lock.lock();
		vehicleGeography.move(this, targetGeom);
//		this.lock.unlock();
		double time = System.currentTimeMillis();
		if (this.house.getActivityPlan().size() > 1) {
			this.endTime = (int) RepastEssentials.GetTickCount();
			this.reachActLocation = true;
			this.resetVehicle();
		} else {
			this.endTime = (int) RepastEssentials.GetTickCount();
			this.reachActLocation = false;
			this.reachDest = true;
			this.killVehicle();
		}
	}
	
	public void setLastRouteTime(int routeTime) {
		this.lastRouteTime= routeTime;
	}
	

	public float currentSpeed() {
		return currentSpeed_;
	}

	public void resetVehicle() {
		this.setNextPlan();
		CityContext cityContext = (CityContext) ContextCreator.getCityContext();
		Coordinate currentCoord = vehicleGeography.getGeometry(this)
				.getCoordinate();
		Road road = cityContext.findRoadAtCoordinates(currentCoord, false); // todest=false
		// TODO: Remove one of the two add queue functions after finish
		// debugging
		// road.addVehicleToQueue(this);
		road.addVehicleToNewQueue(this);
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.coordMap = new ArrayList<Coordinate>();
		this.accummulatedDistance_ = 0;
	}

	public void killVehicle() {
		this.road = null;
		this.lane = null;
		this.laneGeography = null;
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.coordMap = null;
		this.currentSpeed_ = 0.0f;
		this.moveVehicle = false;
		this.destCoord = null;
		this.destZone = null;
		this.targetLane_ = null;
		this.tempLane_ = null;
		this.house = null;
		this.clearShadowImpact(); // ZH: clear any remaining shadow impact
	}

	/*
	 * ----------------------------------------------------------------------
	 * BL: From this we build functions that are used to incorporate lane
	 * object.
	 * ----------------------------------------------------------------------
	 */
	// BL: In one road find the front leader in the same lane of the road, if
	// there is no front, then return null.
	public Vehicle findFrontBumperLeaderInSameRoad(Lane plane) {
		Vehicle front = this.macroLeading_;
		while (front != null && front.lane != plane) {
			front = front.macroLeading_;
		}
		return front;
	}

	public void removeFromLane() {
		if (this.trailing_ != null) {
			this.lane.firstVehicle(this.trailing_);
			this.trailing_.leading(null);
		} else {
			this.lane.firstVehicle(null);
			this.lane.lastVehicle(null);
		}
		this.lane.removeVehicles();
	}

	public boolean getMoveVehicleFlag() {
		return this.moveVehicle;
	}

	// BL: remove a vehicle from the macro vehicle list in the current road
	// segment.
	public void removeFromMacroList() {
		// Current road of this vehicle
		Road pr = this.getRoad();
		// if this is not the first vehicle on the road
		if (this.macroLeading_ != null) {
			this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		} else { // this is the first vehicle on the road
			pr.firstVehicle(this.macroTrailing_);
		}
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = macroLeading_;
		} else {
			pr.lastVehicle(macroLeading_);
		}
		// pr.removeVehicleFromRoad(this);
		// TODO: need to change the above line by following line later
		// pr.removeVehicleFromRoad();
		Vehicle pv = pr.firstVehicle();
		int nVehicles_ = 0;
		while (pv != null) {
			nVehicles_++;
			pv = pv.macroTrailing_;
		}
		pr.setNumberOfVehicles(nVehicles_);
	}

	/*
	 * ------------------------------------------------------------------- BL:
	 * Advance a vehicle to the position in macro vehicle list that
	 * corresponding to its current distance. This function is invoked whenever
	 * a vehicle is moved (including moved into a downstream segment), so that
	 * the vehicles in macro vehicle list is always sorted by their position.
	 * The function is called in travel()
	 * -------------------------------------------------------------------
	 */
	/**
	 * @param: distance_ is with respect to the end of road
	 */
	// BL: changed from distance_ to realDistance_ (Jul 27/2012)
	public void advanceInMacroList() {
		// (0) check if vehicle should be advanced in the list
		if (macroLeading_ == null || this.distance_ >= macroLeading_.distance_) {
			// no macroLeading or the distance to downstream node is greater
			// than marcroLeading
			// no need to advance this vehicle in list
			return;
		}
		// (1) find vehicle's position in the list
		// now this vehicle has a macroLeading that has the higher distance to
		// downstream node.
		// that should not be the vehicle marcroLeading anymore. Need to find
		// new marcroLeading.
		Vehicle front = macroLeading_;
		while (front != null && this.distance_ < front.distance_) {
			front = front.macroLeading_;
		}
		// (2) Take this vehicle out from the list
		// this macroLeading now will be assigned to be macroLeading of this
		// vehicle marcroTrailing
		Road pr = this.road;
		this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		if (this.macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this.macroLeading_;
		} else {
			pr.lastVehicle(this.macroLeading_);
		}
		// (3) Insert this vehicle after the front
		// (3.1) Point to the front
		this.macroLeading_ = front;
		if (this.macroLeading_ != null) {
			this.macroTrailing_ = macroLeading_.macroTrailing_;
			this.macroLeading_.macroTrailing_ = this;
		} else {
			this.macroTrailing_ = pr.firstVehicle();
			pr.firstVehicle(this);
		}
		// (3.2) Point to the trailing vehicle
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this;
		} else {
			pr.lastVehicle(this);
		}
	}

	/*
	 * function: checkCorrectLane() BL: this function will check if the current
	 * lane connect to a lane in the next road if yes then it gives the
	 * checkLaneFlag true value if not then the checkLaneFlag has false value
	 * the function will be called after the vehicle updates its route i.e. the
	 * routeUpdateFlag has true value
	 */

	public boolean isCorrectLane() {
		// Lane curLane = this.lane;
		if (nextRoad() == null)
			return true;
		Lane nextLane = this.getNextLane();
		// if using dynamic shortest path then we need to check lane only after
		// the route is updated
		this.correctLane = false;
		if (nextLane.getUpLanes().size() > 0)
			for (Lane pl : nextLane.getUpLanes()) {
				if (pl.equals(this.lane)) {
					this.correctLane = true;
					break;
				}
			}
		// if (this.id == GlobalVariables.Global_Vehicle_ID)
		// System.out.println("Am I in correct lane? :" + this.correctLane);
		return this.correctLane;
	}

	// Find if the potential next road and current lane are connected
	public boolean checkNextLaneConnected(Road nextRoad) {
		boolean connected = false;
		Lane curLane = this.lane;
		Road curRoad = this.getRoad();
		
		if (nextRoad != null) {
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.road_().equals(nextRoad)) {
					// if this lane already connects to downstream road then
					// assign to the connected lane
					connected = true;
					break;
				}
			}
		}
		
		return connected;
	}
	
	
	public void assignNextLane() {
		boolean connected = false;
		Lane curLane = this.lane;
		Road curRoad = this.getRoad();

		Junction curUpJunc, curDownJunc, nextUpJunc, nextDownJunc;

		if (this.nextRoad() == null) {
			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
				System.out.println("Assign next lane: current link ID= "
						+ curRoad.getLinkid() + " current lane ID: "
						+ curLane.getLaneid() + " next link ID="
						+ this.nextRoad());
			this.nextLane_=null;
			return;
		} else {
			curUpJunc = this.road.getJunctions().get(0);
			curDownJunc = this.road.getJunctions().get(1);
			nextUpJunc = this.nextRoad_.getJunctions().get(0);
			nextDownJunc = this.nextRoad_.getJunctions().get(1);
//			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
//				System.out.println("Assign next lane for Vehicle: "
//						+ this.getVehicleID() + " --current link ID= "
//						+ curRoad.getLinkid() + " --from Node: "
//						+ curUpJunc.getJunctionID() + " --to Node: "
//						+ curDownJunc.getJunctionID() + " --next link ID= "
//						+ this.nextRoad().getLinkid() + " --from Node: "
//						+ nextUpJunc.getJunctionID() + " --to Node: "
//						+ nextDownJunc.getJunctionID());
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.road_().equals(this.nextRoad())) {
					this.nextLane_ = dl;
					// if this lane already connects to downstream road then
					// assign to the connected lane
					connected = true;
					break;
				}
			}
			if (!connected) {
				for (Lane pl : this.nextRoad().getLanes()) {
					for (Lane ul : pl.getUpLanes()) {
						if (ul.road_().getID() == curRoad.getID()) {
							this.nextLane_ = pl;
							break; // assign the next lane to the 1st connected
							// lane
						}
					}
				}
			}

			if (this.nextLane_ == null)
				System.err.println("No next lane found for vehicle: "
						+ this.vehicleID_ + " moving on the road: "
						+ this.getRoad().getLinkid() + " lane: "
						+ this.getLane().getLaneid() + " heading to location "
						+ this.getDestinationID()
						+ " while looking for next lane on road: "
						+ this.nextRoad().getLinkid() + " that has "
						+ this.nextRoad().getnLanes() + " lanes");
		}

		// this.updateCoorMapWithNewLane();
	}

	/*
	 * BL: nextLane -> get next lane of a vehicle From the next Road of a
	 * vehicle. Check each lane and see which lane connects to vehicle's current
	 * Road
	 */
	public Lane getNextLane() {
		return this.nextLane_;
	}

	/*
	 * BL: return the target lane (the lane that connect to the downstream Road)
	 */
	public Lane targetLane() {
		Road curRoad = this.getRoad();
		Lane nextLane = this.getNextLane();
		if (nextLane != null) {
			for (Lane pl : nextLane.getUpLanes()) {
				if (pl.road_().equals(curRoad)) {
					this.targetLane_ = pl;
					break;
				}
			}
		}
		return this.targetLane_;
	}

	/*
	 * BL: return the next lane that the vehicle need to change to in order to
	 * reach the correct lane
	 */
	public Lane tempLane() {
		Lane plane = this.targetLane();
		Lane tempLane_ = null;
		if (this.road.getLaneIndex(plane) > this.road.getLaneIndex(this.lane)) {
			tempLane_ = this.rightLane();
		}
		if (this.road.getLaneIndex(plane) < this.road.getLaneIndex(this.lane)) {
			tempLane_ = this.leftLane();
		}
		return tempLane_;
	}

	// get left lane
	public Lane leftLane() {
		Lane leftLane = null;
		if (this.road.getLaneIndex(this.lane) > 0) {
			leftLane = this.road.getLane(this.road.getLaneIndex(this.lane) - 1);
		}
		return leftLane;
	}

	// get right lane
	public Lane rightLane() {
		Lane rightLane = null;
		if (this.road.getLaneIndex(this.lane) < this.road.getnLanes() - 1) {
			rightLane = this.road
					.getLane(this.road.getLaneIndex(this.lane) + 1);
		}
		return rightLane;
	}

	/*
	 * BL: This function change the lane of a vehicle regardless it is MLC or
	 * DLC state. The vehicle change lane when its lead and lag gaps are
	 * acceptable. This will not change the speed of the vehicle, the only
	 * information updated in this function is as follow: remove the vehicle
	 * from old lane and add to new lane. Re-assign the leading and trailing
	 * sequence of the vehicle.
	 */
	public void changeLane(Lane plane, Vehicle leadVehicle, Vehicle lagVehicle) {
		Vehicle curLeading = this.leading();
		Vehicle curTrailing = this.trailing();
		if (curTrailing != null) {
			if (curLeading != null) {
				curLeading.trailing(curTrailing);
				curTrailing.leading(curLeading);
			} else {
				curTrailing.leading(null);
				this.lane.firstVehicle(curTrailing);
			}
		} else if (curLeading != null) {
			// this.lane.firstVehicle(curLeading);
			this.lane.lastVehicle(curLeading);
			curLeading.trailing(null);
		} else {
			this.lane.firstVehicle(null);
			this.lane.lastVehicle(null);
		}

		this.lane.removeVehicles();
		/*
		 * BL: after change the lane the vehicle updates its leading and
		 * trailing in the target lanes. and also the lead and lag vehicle have
		 * to update its leading and trailing.
		 */
		this.lane = plane;// vehicle move to the target lane
		if (leadVehicle != null) {
			this.leading_ = leadVehicle;
			this.leading_.trailing(this);
			if (lagVehicle != null) {
				this.trailing_ = lagVehicle;
				this.trailing_.leading(this);
			} else {
				this.trailing(null);
				this.lane.lastVehicle(this);
			}
		} else if (lagVehicle != null) {
			this.lane.firstVehicle(this);
			this.trailing_ = lagVehicle;
			this.trailing_.leading(this);
		} else {
			this.lane.firstVehicle(this);
			this.lane.lastVehicle(this);
			this.leading(null);
			this.trailing(null);
		}
		this.lane.addVehicles();
		// this.tempLane_ = null;
		this.updateCoordMap(this.lane);
	}

	/*
	 * BL: Following we implement mandatory lane changing. The input parameter
	 * is the temporary lane.
	 */
	public void mandatoryLC(Lane plane) {
		Vehicle leadVehicle = this.leadVehicle(plane);
		Vehicle lagVehicle = this.lagVehicle(plane);
		/*
		 * BL: Consider the condition to change the lane as follow: If there are
		 * leading and trailing vehicle then the vehicle will check for gap
		 * acceptance as usual. However, if there is no leading or no trailing,
		 * the leadGap or the lagGap should be neglected. In the case the
		 * vehicle cannot change the lane and the distance to downstream is less
		 * than some threshold then the vehicle starts nosing.
		 */
		if (leadVehicle != null) {
			if (lagVehicle != null) {
				if (this.leadGap(leadVehicle) >= this
						.critLeadGapMLC(leadVehicle)
						&& this.lagGap(lagVehicle) >= this
								.critLagGapMLC(lagVehicle)) {
					// BL: debug the error that vehicle changes the lane
					// internally but not on the GUI
					this.changeLane(this.tempLane(), leadVehicle, lagVehicle);
					this.nosingFlag = false;
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}
			} else {
				if (this.leadGap(leadVehicle) >= this
						.critLeadGapMLC(leadVehicle)) {
					this.changeLane(this.tempLane(), leadVehicle, null);
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}

			}
		} else {
			if (lagVehicle != null) {
				if (this.lagGap(lagVehicle) >= this.critLagGapMLC(lagVehicle)) {
					this.changeLane(this.tempLane(), null, lagVehicle);
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}
			} else
				this.changeLane(this.tempLane(), null, null);
		}
		// if (this.id == GlobalVariables.Global_Vehicle_ID) {
		// System.out.println("I am in MLC");
		// }
	}

	/*
	 * BL: if the vehicle with MLC state can't change the lane after some
	 * distance. The vehicle need to nose and yield the lag Vehicle of the
	 * target lane in order to have enough gap to change the lane This function
	 * is called only when nosingFlag is true and must be recalled until
	 * nosingFlag receive false value after the vehicle nosed, tag the lag
	 * vehicle in target lane to yielding status. This function will be called
	 * in makeAccelerationDecision
	 */
	public float nosing() //
	{
		float acc = 0;
		float lagGap;
		Lane tarLane = this.tempLane();
		Vehicle leadVehicle = this.leadVehicle(tarLane);
		Vehicle lagVehicle = this.lagVehicle(tarLane);
		/*
		 * BL: if there is a lag vehicle in the target lane, the vehicle will
		 * yield that lag vehicle however, the yielding is only true if the
		 * distance is less than some threshold
		 */
		if (lagVehicle != null) {
			lagGap = lagVehicle.distance_ - this.distance_
					- GlobalVariables.DEFAULT_VEHICLE_LENGTH;
			if (lagGap < GlobalVariables.minLag) {
				lagVehicle.yieldingFlag = true;
			}
			// System.out.println("Vehicle " + this.id + " has lag gap " +
			// lagGap
			// + " with lag vehicle " + lagVehicle.id);
		}
		Vehicle front = this.leading();
		/*
		 * BL: 1. if there is a lead and a lag vehicle in the target lane. the
		 * vehicle will check the lead gap before decide to decelerate. if the
		 * lead gap is large, then the subject vehicle will be assigned with the
		 * accelerate rate as in car following. 2. if there is no lead vehicle
		 * in the target lane. the subject vehicle will max accelerate.
		 */
		if (leadVehicle != null) {
			if (this.leadGap(leadVehicle) < this.critLeadGapMLC(leadVehicle)) {
				if (this.currentSpeed_ > 12.2f) {
					acc = -1.47f;// meters/sec^2
				} else if (this.currentSpeed_ > 6.1f)
					acc = -2.04f;
				else
					acc = -2.4f;
			} else {
				if (front != null)
					acc = this.calcCarFollowingRate(front);
				else
					acc = this.maxAcceleration_;
			}
		} else {
			if (front != null)
				acc = this.calcCarFollowingRate(front);
			else
				acc = this.maxAcceleration_;
		}
		this.nosingFlag = false;
		// if (this.distFraction() < 0.25 && this.onlane)
		// lagVehicle.yieldingFlag = true;
		return acc;
	}

	/*
	 * BL: While moving, the vehicle will checks if the vehicles in adjection
	 * lanes are nosing to its lane or not after some distance to the downstream
	 * node If the nosing is true then it will be tagged in yielding state to
	 * slow down.
	 */

	public float yielding() {
		float acc = 0;
		if (this.currentSpeed_ > 24.3f)
			acc = -2.44f;
		else if (this.currentSpeed_ > 18.3f)
			acc = -2.6f;
		else if (this.currentSpeed_ > 12.2f)
			acc = -2.74f;
		else if (this.currentSpeed_ > 6.1f)
			acc = -2.9f;
		else
			acc = -3.05f;
		this.yieldingFlag = false;
		return acc;
	}

	/*
	 * BL: when change lane, distance need to be adjusted with the lane width.
	 */

	// Calculate critical lead gap of the vehicle with the lead vehicle in the
	// target lane.
	public double critLeadGapMLC(Vehicle leadVehicle) {
		double critLead = 0;
		double minLead_ = GlobalVariables.minLead;
		double betaLead01 = GlobalVariables.betaLeadMLC01;
		double betaLead02 = GlobalVariables.betaLeadMLC02;
		double gama = GlobalVariables.gama;
		if (leadVehicle != null)
			critLead = minLead_
					+ (betaLead01 * this.currentSpeed() + betaLead02
							* (this.currentSpeed() - leadVehicle.currentSpeed()))
					* (1 - Math.exp(-gama * this.distance()));
		if (critLead < minLead_)
			critLead = minLead_;
		return critLead;
	}

	// BL: Calculate lead gap of the vehicle with the lead vehicle in the target
	// lane.
	public double leadGap(Vehicle leadVehicle) {
		double leadGap = 0;
		if (leadVehicle != null)
			leadGap = this.distance() - leadVehicle.distance()
					- GlobalVariables.DEFAULT_VEHICLE_LENGTH;
		else
			leadGap = this.distance();
		return leadGap;
	}

	// BL: Calculate critical lag gap of the vehicle with the lag vehicle in the
	// target lane.
	public double critLagGapMLC(Vehicle lagVehicle) {
		double critLag = 0;
		double betaLag01 = GlobalVariables.betaLagMLC01;
		double betaLag02 = GlobalVariables.betaLagMLC02;
		double gama = GlobalVariables.gama;
		double minLag_ = GlobalVariables.minLag;
		if (lagVehicle != null) {
			critLag = minLag_
					+ (betaLag01 * this.currentSpeed() + betaLag02
							* (this.currentSpeed() - lagVehicle.currentSpeed()))
					* (1 - Math.exp(-gama * this.distance()));
		}
		if (critLag < minLag_)
			critLag = minLag_;
		return critLag;
	}

	// BL: Calculate lag gap of the vehicle with the lag vehicle in the target
	// lane.
	public double lagGap(Vehicle lagVehicle) {
		double lagGap = 0;
		if (lagVehicle != null)
			lagGap = lagVehicle.distance() - this.distance()
					- GlobalVariables.DEFAULT_VEHICLE_WIDTH;
		else
			lagGap = this.road.length() - this.distance();
		return lagGap;
	}

	// BL: find the lead vehicle in target lane
	public Vehicle leadVehicle(Lane plane) {
		Vehicle leadVehicle = this.macroLeading_;
		while (leadVehicle != null && leadVehicle.lane != plane) {
			leadVehicle = leadVehicle.macroLeading_;
		}
		return leadVehicle;
	}

	// BL: find lag vehicle in target lane
	public Vehicle lagVehicle(Lane plane) {
		Vehicle lagVehicle = this.macroTrailing_;
		while (lagVehicle != null && lagVehicle.lane != plane) {
			lagVehicle = lagVehicle.macroTrailing_;
		}
		return lagVehicle;
	}

	/*
	 * BL: Following we will implement discretionary LC model At current stage,
	 * the DLC is implementing as follow: 1. If the vehicle is not close to
	 * downstream node 2. and it finds a correct lane with better traffic
	 * condition -> then it will change lane
	 */
	/*
	 * BL: if the vehicle is in correct lane then we find a better lane that is
	 * also connected to downstream line this function is called at the
	 * makeLaneChangingDecision
	 */
	public Lane findBetterLane() {
		Lane curLane = this.lane;
		Lane targetLane = null;
		Lane rightLane = this.rightLane();
		Lane leftLane = this.leftLane();
		// If left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.firstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				Lane tempLane = leftLane.betterLane(rightLane);
				targetLane = curLane.betterLane(tempLane); // get the lane that
				// has best traffic condition
			} else if (leftLane != null)
				targetLane = curLane.betterLane(leftLane);
			else if (rightLane != null) {
				targetLane = curLane.betterLane(rightLane);
			}
			// if we have a target lane, then compare the speed of
			// front bumper leader in the lane with current leader
			if (targetLane != null && !targetLane.equals(curLane)) {
				Vehicle front = this
						.findFrontBumperLeaderInSameRoad(targetLane);
				if (front == null) {
					return targetLane;
					// TODO: fix the null for leading vehicle
				} else if (this.leading_ != null
						&& this.leading_.currentSpeed_ < this.desiredSpeed_
						&& this.currentSpeed_ < this.desiredSpeed_) {
					if (front.currentSpeed_ > this.currentSpeed_
							&& front.accRate_ > 0)
						return targetLane;
				}
			}
			return null;
		}
	}

	public Lane findBetterCorrectLane() {
		Lane curLane = this.lane;
		Lane targetLane = null;
		Lane rightLane = this.rightLane();
		Lane leftLane = this.leftLane();
		// If left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.firstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				// if both left and right lanes are connected to downstream lane
				if (leftLane.isConnectToLane(this.nextLane_)
						&& rightLane.isConnectToLane(this.nextLane_)) {
					Lane tempLane = leftLane.betterLane(rightLane);
					targetLane = curLane.betterLane(tempLane); // get the lane that
					// has best traffic condition
				}
				// if only left lane connects to downstream lane
				else if (leftLane.isConnectToLane(this.nextLane_)) {
					targetLane = curLane.betterLane(leftLane);
				}
				// if only right lane connects to downstream lane
				else if (rightLane.isConnectToLane(this.nextLane_)) {
					targetLane = curLane.betterLane(rightLane);
				}
			} else if (leftLane != null
					&& leftLane.isConnectToLane(this.nextLane_))
				targetLane = curLane.betterLane(leftLane);
			else if (rightLane != null
					&& rightLane.isConnectToLane(this.nextLane_)) {
				targetLane = curLane.betterLane(rightLane);
			}
			// if we have a target lane, then compare the speed of
			// front bumper leader in the lane with current leader
			if (targetLane != null && !targetLane.equals(curLane)) {
				Vehicle front = this
						.findFrontBumperLeaderInSameRoad(targetLane);
				if (front == null) {
					return targetLane;
				} else if (this.leading_ != null
						&& this.leading_.currentSpeed_ < this.desiredSpeed_
						&& this.currentSpeed_ < this.desiredSpeed_) {
					if (front.currentSpeed_ > this.currentSpeed_
							&& front.accRate_ > 0)
						return targetLane;
					
				}
			}
			return null;
		}

	}

	// once the vehicle finds a better lane. It changes to that lane
	// discretionarily.
	public void discretionaryLC(Lane plane) {
		Vehicle leadVehicle = this.leadVehicle(plane);
		Vehicle lagVehicle = this.lagVehicle(plane);
		double leadGap = this.leadGap(leadVehicle);
		double lagGap = this.lagGap(lagVehicle);
		double critLead = this.criticalLeadDLC(leadVehicle);
		double critLag = this.criticalLagDLC(lagVehicle);
		if (leadGap > critLead && lagGap > critLag) {
			this.changeLane(plane, leadVehicle, lagVehicle);
		}
	}

	public double criticalLeadDLC(Vehicle pv) {
		double critLead = 0;
		double minLead = GlobalVariables.minLeadDLC;
		// TODO: change betaLead01 and 02 to DLC value.
		if (pv != null) {
			critLead = minLead + GlobalVariables.betaLeadDLC01
					* this.currentSpeed_ + GlobalVariables.betaLeadDLC02
					* (this.currentSpeed_ - pv.currentSpeed_);
		}
		critLead = Math.max(minLead, critLead);
		return critLead;
	}

	public double criticalLagDLC(Vehicle pv) {
		double critLag = 0;
		double minLag = GlobalVariables.minLagDLC;
		// TODO: change betaLag01 and 02 to DLC value
		if (pv != null) {
			critLag = minLag + GlobalVariables.betaLagDLC01
					* this.currentSpeed_ + GlobalVariables.betaLagDLC02
					* (this.currentSpeed_ - pv.currentSpeed_);
		}
		critLag = Math.max(minLag, critLag);
		return critLag;
	}

	/*
	 * After appending vehicle to next road, a new nextlane being assigned. The
	 * coordinate map will be updated with the last point is the starting point
	 * of the next lane
	 */
	public void updateCoorMapWithNewLane() {
		Coordinate coor1, coor2;
		if (!this.coordMap.isEmpty()) {
			int end = this.coordMap.size() - 1;
			coor1 = this.coordMap.get(end);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out
						.println("=====I already had my coordinate map but need to update for moving through junction ");
				System.out.println("which connects to the point: " + coor1);
			}
		} else {
			// SH Temp
			// Geography<Vehicle> vehicleGeography;
			// vehicleGeography = ContextCreator.getVehicleGeography();
			coor1 = vehicleGeography.getGeometry(this).getCoordinate();
		}
		if (this.nextLane_ != null) {
			Lane plane = this.nextLane_;
			Coordinate c1, c2;
			Coordinate[] coords = laneGeography.getGeometry(plane)
					.getCoordinates();
			c1 = coords[0];
			c2 = coords[coords.length - 1];
			coor2 = getNearestCoordinate(coor1, c1, c2);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out.println("The adding coordinate is from the lane: "
						+ plane.getLaneid());
			}
		} else
			coor2 = this.destCoord;
		if (!coor2.equals(coor1)) {
			this.coordMap.add(coor2);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out
						.println("@@@@@@@@@@@@@@ New coordinate added @@@@@@@@@@@@@@@@@@@");
			}
		} else {
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out.println("+++++++++++No coordinate added+++++++++ ");
			}
		}
		if (this.id == GlobalVariables.Global_Vehicle_ID
				&& !GlobalVariables.Debug_On_Road) {
			System.out
					.println("My coordinate map after update the next lane: ");
			int end = this.coordMap.size() - 1;
			System.out.println("Distance to added point: "
					+ distance(this.coordMap.get(end - 1),
							this.coordMap.get(end)));
		}
	}

	/*
	 * BL: when vehicle approach junction, need new coordinate and distance
	 */
	public int appendToJunction(Lane nextlane) {
		Coordinate lastCoordinate = null, start, end;
		lastCoordinate = vehicleGeography.getGeometry(this).getCoordinate();

		coordMap.clear();
		if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID
				&& this.nextRoad().getLinkid() == this.destRoadID) {
			System.out.println("Vehicle " + this.getVehicleID()
					+ " is appending to a junction of the final Road");
		}
		if (this.atDestination()) {
			this.removeFromLane();
			this.removeFromMacroList();
			return 0;
		} else if (nextlane != null) {
			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID
					&& this.nextRoad().getLinkid() == this.destRoadID)
				System.out.println("Vehicle " + this.getVehicleID()
						+ " has final next lane " + nextlane.getLaneid());
			Coordinate coor = null;
			Coordinate[] coords = laneGeography.getGeometry(nextlane)
					.getCoordinates();

			start = coords[0];
			end = coords[coords.length - 1];
			coor = this.getNearestCoordinate(lastCoordinate, start, end);
			coordMap.add(coor);
		} else {
			coordMap.add(this.destCoord);
		}

		this.distance_ = (float) distance(lastCoordinate, coordMap.get(0))
				- lastStepMove_ / 2;
		this.onlane = false;
		if (this.distance_ < 0) {
			if (this.changeRoad() == 0)
				return 0;
		}
		return 1;
	}

	public boolean isOnLane() {
		return onlane;
	}

	public float entranceGap(Lane nextlane) {
		float gap = 0;
		if (nextlane != null) {
			Vehicle newleader = nextlane.lastVehicle();
			if (newleader != null) {
				gap = (float) nextlane.getLength() - newleader.distance_;
			} else
				gap = 9999999;
		}
		return gap;
	}

	/**
	 * @param c
	 * @param c1
	 * @param c2
	 * @return
	 */
	private Coordinate getNearestCoordinate(Coordinate c, Coordinate c1,
			Coordinate c2) {
		/*
		 * GeometryFactory geomFac = new GeometryFactory(); Geometry coordGeom =
		 * geomFac.createPoint(c); Geometry geom1 = geomFac.createPoint(c1);
		 * Geometry geom2 = geomFac.createPoint(c2);
		 */
		double dist1 = distance(c, c1);
		double dist2 = distance(c, c2);

		if (dist1 < dist2)
			return c1;
		else
			return c2;

	}

	private double distance(Coordinate c1, Coordinate c2) {
		// GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
		// .getRoadGeography().getCRS());
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		double distance;
		try {
			distance = calculator.getOrthodromicDistance();
		} catch (AssertionError ex) {
			System.err.println("Error with finding distance");
			distance = 0.0;
		}
		return distance;
	}

	private double distance(Coordinate c1, Coordinate c2, double[] returnVals) {
		double time = System.currentTimeMillis(); // used for debugging
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		double distance;
		try {
			distance = calculator.getOrthodromicDistance();

		} catch (AssertionError ex) {
			System.err.println("Error with finding distance");
			distance = 0.0;
		}
		if (returnVals != null && returnVals.length == 2) {
			returnVals[0] = distance;
			double angle = Math.toRadians(calculator.getAzimuth()); // Angle in
			// range -PI to PI
			// Need to transform azimuth
			// (in range -180 -> 180 and where 0 points north)
			// to standard mathematical (range 0 -> 360 and 90 points north)
			if (angle > 0 && angle < 0.5 * Math.PI) { // NE Quadrant
				angle = 0.5 * Math.PI - angle;
			} else if (angle >= 0.5 * Math.PI) { // SE Quadrant
				angle = (-angle) + 2.5 * Math.PI;
			} else if (angle < 0 && angle > -0.5 * Math.PI) { // NW Quadrant
				angle = (-1 * angle) + 0.5 * Math.PI;
			} else { // SW Quadrant
				angle = -angle + 0.5 * Math.PI;
			}
			returnVals[1] = angle;
		}
		return distance;
	}
	
	/* *
	 * Thread safe version of the moveByVector, replace the one in the DefaultGeography class
	 * Creating a new Geometry point given the current location of the vehicle as well as the distance and angle.
	 * In the end, move the vehicle to the new geometry point.
	 * @return the new Geometry point
	 */
	
	public void moveVehicleByVector(double distance, double angleInRadians) {
		Geometry geom = vehicleGeography.getGeometry(this);
		if (geom == null) {
			System.err.println("Error moving object by vector");
		}

		if (angleInRadians > 2 * Math.PI || angleInRadians < 0) {
			throw new IllegalArgumentException(
					"Direction cannot be > PI (360) or less than 0");
		}
		double angleInDegrees = Math.toDegrees(angleInRadians);
		angleInDegrees = angleInDegrees % 360;
		angleInDegrees = 360 - angleInDegrees;
		angleInDegrees = angleInDegrees + 90;
		angleInDegrees = angleInDegrees % 360;
		if (angleInDegrees > 180) {
			angleInDegrees = angleInDegrees - 360;
		}
		Coordinate coord = geom.getCoordinate();
		AffineTransform transform;
		
		try {
			if (!this.vehicleGeography.getCRS().equals(DefaultGeographicCRS.WGS84)) {
				MathTransform crsTrans = CRS.findMathTransform(this.vehicleGeography.getCRS(),
						DefaultGeographicCRS.WGS84);
				Coordinate tmp = new Coordinate();
				JTS.transform(coord, tmp, crsTrans);
				this.calculator.setStartingGeographicPoint(tmp.x, tmp.y);
			} else {
				this.calculator.setStartingGeographicPoint(coord.x, coord.y);
			}
			this.calculator.setDirection(angleInDegrees, distance);
			Point2D p = this.calculator.getDestinationGeographicPoint();
			if (!this.vehicleGeography.getCRS().equals(DefaultGeographicCRS.WGS84)) {
				MathTransform crsTrans = CRS.findMathTransform(
						DefaultGeographicCRS.WGS84, this.vehicleGeography.getCRS());
				JTS.transform(new Coordinate(p.getX(), p.getY()), coord,
						crsTrans);
			}

			transform = AffineTransform.getTranslateInstance(
					p.getX() - coord.x, p.getY() - coord.y);

			MathTransform mt = mtFactory
					.createAffineTransform(new GeneralMatrix(transform));
			geom = JTS.transform(geom, mt);
		} catch (Exception ex) {
			System.err.println("Error moving object by vector");
		}
		
		vehicleGeography.move(this, geom);
	}
}
