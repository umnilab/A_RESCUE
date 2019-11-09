package evacSim.vehiclecontext;

import java.util.List;
import java.util.Map;

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
					
					Map<Double,List<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destCoord);  //return the HashMap
					Map.Entry<Double,List<Road>> entry = tempPathMap.entrySet().iterator().next();	 //get the entry
					List<Road> tempPath = entry.getValue(); //get the path
					
					if (this.checkNextLaneConnected(tempPath.get(1))){
						// If the next road is connected to the current lane, then we assign the path, otherwise, we use the old path
						// Clear legacy impact
						this.clearShadowImpact();
						this.roadPath = tempPath;
						this.setShadowImpact();
						this.lastRouteTime = Integer.MAX_VALUE; //Gehlot: Predefined routes are used as routing will not happen again after the intial time
						this.nextRoad_ = this.roadPath.get(1);
//						System.out.println("initial routing done for "+this.vehicleID_+"lastroutetime= "+this.lastRouteTime);//Gehlot: for debugging
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
				// this.roadPath = RouteV.vehicleRoute(this, this.destCoord);	  //
				Map<Double,List<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destCoord);  //get the HashMap
				Map.Entry<Double,List<Road>> entry = tempPathMap.entrySet().iterator().next();	 //get the entry
				List<Road> tempPath = entry.getValue(); //get the value
				this.roadPath = tempPath;  //get the path
				
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
	
}
