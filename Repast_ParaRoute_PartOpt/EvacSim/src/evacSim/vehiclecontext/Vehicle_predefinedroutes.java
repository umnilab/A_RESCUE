package evacSim.vehiclecontext;

import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;

import evacSim.citycontext.House;
import evacSim.citycontext.Road;
import evacSim.routing.RouteV;
import repast.simphony.essentials.RepastEssentials;

//Gehlot: This is a subclass of Vehicle class
public class Vehicle_predefinedroutes extends Vehicle {

	public Vehicle_predefinedroutes(House h) {
		super(h);
		this.setVehicleClass(2);//HG:Set vehicleclass to 2
	}

	public Vehicle_predefinedroutes(House h, float maximumAcceleration, float maximumDeceleration) {
		super(h, maximumAcceleration, maximumDeceleration);
	}
	
	//Gehlot: Override original setNextRoad() method to use predefined routes rather than periodically updating routes 
	@Override
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
					// List<Road> tempPath = RouteV.vehicleRoute(this, this.destCoord); //
					
					Map<Double, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destCoord);  //return the HashMap
					Entry<Double, Queue<Road>> entry = tempPathMap.entrySet().iterator().next();	 //get the entry
					Queue<Road> tempPath = entry.getValue(); //get the path
					Iterator<Road> iter = tempPath.iterator();
					iter.next();
					if (this.checkNextLaneConnected(iter.next())){
						// If the next road is connected to the current lane, then we assign the path, otherwise, we use the old path
						// Clear legacy impact
						this.clearShadowImpact();
						this.roadPath = (Queue<Road>) tempPath;
						this.setShadowImpact();
						this.lastRouteTime = Integer.MAX_VALUE; //Gehlot: Predefined routes are used as routing will not happen again after the intial time
						Iterator<Road> itr = this.roadPath.iterator();
						itr.next();
						this.nextRoad_ = itr.next();
//						System.out.println("initial routing done for "+this.vehicleID_+"lastroutetime= "+this.lastRouteTime);//Gehlot: for debugging
					} else {
						// New Route will cause blocking, use the old path
						// Remove the current road from the path
						this.removeShadowCount(this.roadPath.peek());
						this.roadPath.poll();
						Iterator<Road> itr = this.roadPath.iterator();
						itr.next();
						this.nextRoad_ = itr.next();
					}
//					System.out.println("Debug 1: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
				} else {
					// Route information is still valid
					// Remove the current road from the path
					this.removeShadowCount(this.roadPath.peek());
					this.roadPath.poll();
					Iterator<Road> itr = this.roadPath.iterator();
					itr.next();
					this.nextRoad_ = itr.next();
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
				// this.roadPath = RouteV.vehicleRoute(this, this.destCoord);	  //
				Map<Double,Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destCoord);  //get the HashMap
				Map.Entry<Double,Queue<Road>> entry = tempPathMap.entrySet().iterator().next();	 //get the entry
				Queue<Road> tempPath = entry.getValue(); //get the value
				this.roadPath = tempPath;  //get the path
				
				this.setShadowImpact();
				this.lastRouteTime = (int) RepastEssentials.GetTickCount();
				this.atOrigin = false;
				Iterator<Road> itr = this.roadPath.iterator();
				itr.next();
				this.nextRoad_ = itr.next();
//				System.out.println("Debug 2: Vehicle: " + this.getId() + " current road: " + this.road.getLinkid() + " next road: " + this.nextRoad_.getLinkid());
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("No next road found for Vehicle "
					+ this.vehicleID_ + " on Road " + this.road.getLinkid());
			this.nextRoad_ = null;
		}
		
	}
	
}
