package evacSim.vehiclecontext;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.vehiclecontext.Vehicle;
import evacSim.citycontext.*;

public class VehicleContext extends DefaultContext<Vehicle> {

	public VehicleContext() {
		super("VehicleContext");
		
		Logger logger = ContextCreator.logger;
		
		logger.info("VehicleContext creation");
		GeographyParameters<Vehicle> geoParams = new GeographyParameters<Vehicle>();
		 geoParams.setCrs("EPSG:32618");
		Geography<Vehicle> vehicleGeography = GeographyFactoryFinder
				.createGeographyFactory(null).createGeography(
						"VehicleGeography", this, geoParams);
//		Geography<Zone> zoneGeography;
//		zoneGeography = ContextCreator.getZoneGeography();
		//createVehicleContextFromManualDemand(zoneGeography,vehicleGeography);
		createVehicleContextFromActivityModels();
		}

	
//	public void createVehicleContextFromActivityModels(
//			Geography<Zone> zoneGeography, Geography<Vehicle> vehicleGeography) {
//		double propPreDetermined = GlobalVariables.PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES;
//		double propLessFreq = GlobalVariables.PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES;
//		
//		for (Zone z : zoneGeography.getAllObjects()) {
//			Geometry hgeom = zoneGeography.getGeometry(z);
//			Coordinate coord = hgeom.getCoordinate();
//			for (House h : z.getHouses()) {
//				GeometryFactory fac = new GeometryFactory();
//				Vehicle v;
//				
//				//TODO: Code a mechanism to generate vehicles with different parameters (like max acceleration)
//				if (GlobalVariables.ENABLE_MULTICLASS_ROUTING){//Gehlot: Generate multi-class vehicles
//					if ((double) Math.random() > propPreDetermined + propLessFreq) {
//						v = new Vehicle(h);
//					} else if ((double) Math.random() < propPreDetermined/(propPreDetermined + propLessFreq)) {
//						v = new Vehicle_predefinedroutes(h);
//					} else {
//						v = new Vehicle_less_frequent_routing(h);
//					}
//				} else {
//					v = new Vehicle(h);
//				}
////				logger.info(v + "_" + v.getHouse().getActivityPlan());
//				
//				this.add(v);
//				v.setOriginalCoord(coord);
//				v.setCurrentCoord(coord); 
//				
//				Road road = ContextCreator.getCityContext().findRoadAtCoordinates(coord, false);
//				if(road.getLinkid() == 104819){
//					road =  ContextCreator.getCityContext().findRoadWithLinkID(104818);
//				}
//				if(road.getLinkid() == 101235){
//					road =  ContextCreator.getCityContext().findRoadWithLinkID(101236);
//				}
//				road.addVehicleToNewQueue(v);
//			}
//		}
//	}
	
	public void createVehicleContextFromActivityModels() {
		Geography<Zone> zoneGeography=ContextCreator.getZoneGeography();
//		Geography<Vehicle> vehicleGeography=ContextCreator.getVehicleGeography();
		double propPreDetermined = GlobalVariables.PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES;
		double propLessFreq = GlobalVariables.PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES;
		for (Zone z : zoneGeography.getAllObjects()) {
			Geometry hgeom = zoneGeography.getGeometry(z);
			Coordinate coord = hgeom.getCoordinate();
			for (House h : z.getHouses()) {
//				GeometryFactory fac = new GeometryFactory();
				Vehicle v;
				//TODO: Code a mechanism to generate vehicles with different parameters (like max acceleration)
				if (GlobalVariables.ENABLE_MULTICLASS_ROUTING){//Gehlot: Generate multi-class vehicles
					if ((double) Math.random() > (propPreDetermined + propLessFreq)) {
						v = new Vehicle(h); 
					} else if ((double) Math.random() < propPreDetermined/(propPreDetermined + propLessFreq)) {
						v = new Vehicle_predefinedroutes(h);
					} else {
						v = new Vehicle_less_frequent_routing(h);
					}
				} else {
					v = new Vehicle(h);
				}
//				GlobalVariables.NUM_GENERATED_VEHICLES++;
				// this.add(v);
				
				//v.setEvacuationTime(evactime);
//				this.add(v); 
				
				// Add randomness to origin and destination coord, this improves the speed of adding new vehicles
//				if(GlobalVariables.ENABLE_RANDOM_VEHICLE_INITIALIZATION){
//					coord.x = coord.x - 3e-4 + 6e-4*rd.nextDouble();
//					coord.y = coord.y - 3e-4 + 6e-4*rd.nextDouble();
//				}
				v.setOriginalCoord(coord);
				v.setCurrentCoord(coord); // LZ: instead of creating geometry, just storing the coordinates
//				Point geom = fac.createPoint(coord);
//				vehicleGeography.move(v, geom); //This is time consuming
				Road road = z.getdepartureRoad();
				if(road.getLinkid() == 104819){
					road =  ContextCreator.getCityContext().findRoadWithLinkID(104818);
				}
				if(road.getLinkid() == 101235){
					road =  ContextCreator.getCityContext().findRoadWithLinkID(101236);
				}
				road.addVehicleToNewQueue(v);
			}
		}
	}
	
	
//	public void createVehicleContextFromManualDemand(
//			Geography<Zone> zoneGeography, Geography<Vehicle> vehicleGeography) {
//		/***
//		 * Reads manually from CSV file or Database depending on the Global
//		 * Variable VEHICLES_PER_ZONE_DATABASE in config file
//		 */
//		VehicleCreatorFromManualData vehicleCreator;
//
////		vehicleCreator = new VehicleCreatorFromManualData(
////					GlobalVariables.VEHICLES_CSV, GlobalVariables.READING_CSV);
//		vehicleCreator = new VehicleCreatorFromManualData();
//
//		for (Zone z : zoneGeography.getAllObjects()) {
////			System.out
////					.println("VehicleContext: createVehicleContextFromManualDemand: zone: "
////							+ z.getIntegerID());
//
//			Geometry hgeom = zoneGeography.getGeometry(z);
//			Coordinate coord = hgeom.getCoordinate();
//
//			// Creates a temporal array list of vehicles
//			ArrayList<Vehicle> evacuatingVehicles = new ArrayList<Vehicle>();
//			// gets the ID of the current zone
//			int zoneId = z.getIntegerID();
//			// sets the temporal array list of vehicles as the array of vehicles
//			// evacuating from this zone
//			if (vehicleCreator.getVehiclesByZone(zoneId) != null) {
//				evacuatingVehicles = vehicleCreator.getVehiclesByZone(zoneId);
//				/*
//				 * logger.info(
//				 * "VehicleContext: createVehicleContextFromManualDemand: Evacuating vehicles size: "
//				 * + evacuatingVehicles.size());
//				 */
//				for (int i = 0; i < evacuatingVehicles.size(); i++) {
//					GeometryFactory fac = new GeometryFactory();
//					Vehicle v = evacuatingVehicles.get(i);
//					this.add(v);
//					v.setOriginalCoord(coord);
//					Point geom = fac.createPoint(coord);
//					vehicleGeography.move(v, geom);
//
//				}
//			}
//		}
//	}
}