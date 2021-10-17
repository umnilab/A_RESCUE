package evacSim.vehiclecontext;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
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
		this.setVehicleClass(3); // HG: Set vehicleclass to 3
	}

	public Vehicle_less_frequent_routing(House h, float maximumAcceleration, 
			float maximumDeceleration) {
		super(h, maximumAcceleration, maximumDeceleration);
	}
	
	/* HG: Override original setNextRoad() method to refresh travel times 
	 * with some probability */
	@Override
	public void setNextRoad() {
		try {
			if (!this.atOrigin) {
				if (this.road.getLinkid() == this.destRoadID) {
					this.nextRoad_ = null;
					return;
				}
				/* HG: This probability determines how the chances with 
				 * which we decide to update routes. If it is high then 
				 * we update routes more frequently */
				if((double) Math.random() < GlobalVariables.PROBABILITY_OF_UPDATING_ROUTING){
					this.lastRouteTick = RouteV.getValidTime();
				}
				if (this.lastRouteTick < RouteV.getValidTime()) {
					/* JX, Oct 2019: change the return type of RouteV
					 * .vehicleRoute to be a HashMap, and get the 
					 * tempPathNew and pathTimeNew. */
					Map<Float, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);
					Entry<Float, Queue<Road>> entry = tempPathMap.entrySet().iterator().next();	 
					// calculate path
					Queue<Road> tempPathNew = entry.getValue();
					// calculate path time
					float pathTimeNew = entry.getKey();
					// calculate tick
					int currentTick = (int) RepastEssentials.GetTickCount();
					Queue<Road> tempPath = entry.getValue();
					
					double pathTimeOldPath = this.travelTimeForPreviousRoute - (
							currentTick-this.previousTick) * GlobalVariables.SIMULATION_STEP_SIZE;
					/* JX, RV: make the comparison between the previous route 
					 * time and the new route time. If the absolute and 
					 * relative difference are both large, the vehicle will 
					 * shift to the new route (Mahmassani and Jayakrishnan 
					 * (1991), System performance and user response under 
					 * real-time information in a congested traffic corridor). 
					 */
					if (pathTimeOldPath - pathTimeNew > this.indiffBand * pathTimeOldPath) {
						if (pathTimeOldPath - pathTimeNew > GlobalVariables.TAU) {
							tempPath = tempPathNew; // update path
							this.travelTimeForPreviousRoute = pathTimeNew; // update path time
							this.previousTick =  currentTick; // update tick
						}
					}
					Iterator<Road> itr = roadPath.iterator();
					itr.next();
					if (this.checkNextLaneConnected(itr.next())){
						/* If the next road is connected to the current 
						 * lane, then we assign the path, otherwise, we 
						 * use the old path */
						// Clear legacy impact
						this.clearShadowImpact();
						this.roadPath = (Queue<Road>) tempPath;
						this.setShadowImpact();
						this.lastRouteTick = (int) RepastEssentials.GetTickCount();
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
				} else {
					// Route information is still valid
					// Remove the current road from the path
					this.removeShadowCount(this.roadPath.peek());
					this.roadPath.poll();
					Iterator<Road> itr = this.roadPath.iterator();
					itr.next();
					this.nextRoad_ = itr.next();
				}	
			} else {
				this.clearShadowImpact(); // clear legacy impact
				// Compute new route
				Map<Float, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);
				Entry<Float, Queue<Road>> entry = tempPathMap.entrySet().iterator().next();
				Queue<Road> tempPath = entry.getValue(); // get the route
				this.roadPath = (Queue<Road>) tempPath;  // store the route
				
				this.setShadowImpact();
				this.lastRouteTick = (int) RepastEssentials.GetTickCount();
				this.atOrigin = false;
				Iterator<Road> itr = this.roadPath.iterator();
				itr.next();
				this.nextRoad_ = itr.next();
			}
		} catch (Exception e) {
			e.printStackTrace();
			logger.info("No next road found for Vehicle "
					+ this.vehicleID_ + " on Road " + this.road.getLinkid());
			this.nextRoad_ = null;
		}
	}
}