package evacSim.vehiclecontext;

import java.util.Iterator;
import java.util.Map;
import java.util.Queue;

import org.apache.log4j.Logger;

import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.House;
import evacSim.citycontext.Road;
import evacSim.routing.RouteV;
import repast.simphony.essentials.RepastEssentials;

public class Vehicle_less_frequent_routing extends Vehicle {
	private Logger logger = ContextCreator.logger;

	public Vehicle_less_frequent_routing(House h) {
		super(h);
		this.setVehicleClass(3);//HG:Set vehicleclass to 3
	}

	public Vehicle_less_frequent_routing(House h, float maximumAcceleration, float maximumDeceleration) {
		super(h, maximumAcceleration, maximumDeceleration);
	}
	
	//Gehlot: Override original setNextRoad() method to refresh travel times with some probability
		@Override
		public void setNextRoad() {
			try {
				if (!this.atOrigin) {
					if (this.road.getLinkid() == this.destRoadID) {
						this.nextRoad_ = null;
						return;
					}
					
					//Gehlot: This probability determines how the chances with which we decide to update routes. If it is high then we update routes more frequently
					if((double) Math.random() < GlobalVariables.PROBABILITY_OF_UPDATING_ROUTING){
						this.lastRouteTime = RouteV.getValidTime();
					}
					
					if (this.lastRouteTime < RouteV.getValidTime()) {
						// The information are outdated, needs to be recomputed
						// Check if the current lane connects to the next road in the new path
						//List<Road> tempPath = RouteV.vehicleRoute(this, this.destCoord);                 
						Map<Double,Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);   //Xue, Oct 2019: change the return type of RouteV.vehicleRoute to be a HashMap, and get the tempPathNew and pathTimeNew.
						Map.Entry<Double,Queue<Road>> entry = tempPathMap.entrySet().iterator().next();	 
						
						Queue<Road> tempPathNew = entry.getValue();    // Calculate path 
						double pathTimeNew = entry.getKey();           // Calculate path time
						int currentTick = (int) RepastEssentials.GetTickCount();  //Calculate tick.
						Queue<Road> tempPath = entry.getValue();
						
						double pathTimeOldPath = this.travelTimeForPreviousRoute - (currentTick-this.previousTick) * GlobalVariables.SIMULATION_STEP_SIZE;
						//Xue: make the comparison between the previous route time and the new route time. If the absolute and relative difference are both large 
						//, the vehicle will shift to the new route (Mahmassani and Jayakrishnan(1991), System performance  and user  response  under  real-time  information  in a congested  traffic corridor).
						if (pathTimeOldPath - pathTimeNew > this.indiffBand * pathTimeOldPath) { //Rajat, Xue
							if (pathTimeOldPath - pathTimeNew > GlobalVariables.TAU) {
								tempPath = tempPathNew;                              // Update path.
								this.travelTimeForPreviousRoute = pathTimeNew;       // Update path time.
								this.previousTick =  currentTick;                    // Update tick.
							}
						}
						Iterator<Road> itr = roadPath.iterator();
						itr.next();
						if (this.checkNextLaneConnected(itr.next())){
							// If the next road is connected to the current lane, then we assign the path, otherwise, we use the old path
							// Clear legacy impact
							this.clearShadowImpact();
							this.roadPath = (Queue<Road>) tempPath;
							this.setShadowImpact();
							this.lastRouteTime = (int) RepastEssentials.GetTickCount();
							itr = this.roadPath.iterator();
							itr.next();
							this.nextRoad_ = itr.next();
						} else {
							// New Route will cause blocking, use the old path
							// Remove the current road from the path
							this.removeShadowCount(this.roadPath.peek());
							this.roadPath.poll();
							itr = this.roadPath.iterator();
							itr.next();
							this.nextRoad_ = itr.next();
						}
						
//						logger.info("Debug 1: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
					} else {
						// Route information is still valid
						// Remove the current road from the path
						this.removeShadowCount(this.roadPath.peek());
						this.roadPath.poll();
						Iterator<Road> itr = this.roadPath.iterator();
						itr.next();
						this.nextRoad_ = itr.next();
					}	

//					if (nextRoad != null)
//						if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
//							logger.info("Next Road ID for Vehicle: "
//									+ this.getVehicleID() + " is "
//									+ nextRoad.getLinkid());
					
					/*
					 * if (nextRoad.getLinkid() != this.road.getLinkid()) {
					 * logger.info("Next Road ID for Vehicle: " +
					 * this.getVehicleID() + " is " + nextRoad.getLinkid());
					 * this.nextRoad_ = nextRoad; } else {
					 * logger.info("No next road found for Vehicle " +
					 * this.vehicleID_ + " on Road " + this.road.getLinkid());
					 * this.nextRoad_ = null; }
					 */

				} else {
					// Clear legacy impact
					this.clearShadowImpact();
					// Compute new route
					//this.roadPath = RouteV.vehicleRoute(this, this.destCoord); 
					Map<Double,Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);  //return the HashMap
					Map.Entry<Double,Queue<Road>> entry = tempPathMap.entrySet().iterator().next();	 //
					Queue<Road> tempPath = entry.getValue(); //get the route  
					this.roadPath = (Queue<Road>) tempPath;  //store the route
					
					this.setShadowImpact();
					this.lastRouteTime = (int) RepastEssentials.GetTickCount();
					this.atOrigin = false;
					Iterator<Road> itr = this.roadPath.iterator();
					itr.next();
					this.nextRoad_ = itr.next();
//					logger.info("Debug 2: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
				}
			} catch (Exception e) {
				e.printStackTrace();
				logger.info("No next road found for Vehicle "
						+ this.vehicleID_ + " on Road " + this.road.getLinkid());
				this.nextRoad_ = null;
			}
			
		}
		
}