package evacSim.vehiclecontext;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import java.util.Queue;

import evacSim.ContextCreator;
import evacSim.citycontext.House;
import evacSim.citycontext.Road;
import evacSim.routing.RouteV;
import repast.simphony.essentials.RepastEssentials;

// HG: This is a subclass of Vehicle class
public class Vehicle_predefinedroutes extends Vehicle {
	private Logger logger = ContextCreator.logger;

	public Vehicle_predefinedroutes(House h) {
		super(h);
		this.setVehicleClass(2); // HG: Set vehicleclass to 2
	}

	public Vehicle_predefinedroutes(House h, float maximumAcceleration,
			float maximumDeceleration) {
		super(h, maximumAcceleration, maximumDeceleration);
	}
	
	/* HG: Override original setNextRoad() method to use predefined 
	 * routes rather than periodically updating routes */ 
	@Override
	public void setNextRoad() {
		try {
			if (!this.atOrigin) {
				if (this.road.getLinkid() == this.destRoadID) {
					this.nextRoad_ = null;
					return;
				}
				if (this.lastRouteTick < RouteV.getValidTime()) {
					//return the HashMap
					Map<Float, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);
					Entry<Float, Queue<Road>> entry = tempPathMap.entrySet().iterator().next();
					Queue<Road> tempPath = entry.getValue(); //get the path
					Iterator<Road> iter = tempPath.iterator();
					iter.next();
					if (this.checkNextLaneConnected(iter.next())){
						// If the next road is connected to the current lane, 
						// then we assign the path, otherwise, we use the old 
						// path. Clear legacy impact.
						this.clearShadowImpact();
						this.roadPath = (Queue<Road>) tempPath;
						this.setShadowImpact();
						// HG: Predefined routes are used as routing will 
						// not happen again after the intial time
						this.lastRouteTick = Integer.MAX_VALUE;
						Iterator<Road> itr = this.roadPath.iterator();
						itr.next();
						this.nextRoad_ = itr.next();
					} else {
						// New Route will cause blocking, use the old path
						// Remove the current road from the path
						this.removeShadowCount(this.roadPath.peek());
						this.roadPath.poll();
						Iterator<Road> itr = this.roadPath.iterator();
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
				// Clear legacy impact
				this.clearShadowImpact();
				// Compute new route
				Map<Float, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);
				Entry<Float, Queue<Road>> entry = tempPathMap.entrySet().iterator().next();
				Queue<Road> tempPath = entry.getValue(); //get the value
				this.roadPath = tempPath;  //get the path
				
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
