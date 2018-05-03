package evacSim.vehiclecontext;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Point;

import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters; //import repast.simphony.space.gis.House; //From Rodrigo
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.vehiclecontext.Vehicle;
import evacSim.citycontext.*;

public class VehicleContext extends DefaultContext<Vehicle> {
	public VehicleContext() {
		super("VehicleContext");
		System.out.println("VehicleContext creation");
		
		GeographyParameters<Vehicle> geoParams = new GeographyParameters<Vehicle>();
		// geoParams.setCrs("EPSG:32618");
		Geography<Vehicle> vehicleGeography = GeographyFactoryFinder
				.createGeographyFactory(null).createGeography(
						"VehicleGeography", this, geoParams);
		Geography<Zone> zoneGeography;
		zoneGeography = ContextCreator.getZoneGeography();

		//createVehicleContextFromManualDemand(zoneGeography,vehicleGeography);
		createVehicleContextFromActivityModels(zoneGeography,vehicleGeography);
		}

	
	public void createVehicleContextFromActivityModels(
			Geography<Zone> zoneGeography, Geography<Vehicle> vehicleGeography) {
		for (Zone z : zoneGeography.getAllObjects()) {
			Geometry hgeom = zoneGeography.getGeometry(z);
			Coordinate coord = hgeom.getCoordinate();
			for (House h : z.getHouses()) {
				GeometryFactory fac = new GeometryFactory();
				Vehicle v;
				if (GlobalVariables.ENABLE_MULTICLASS_VEHICLES){//Gehlot: Generate multi-class vehicles
					if((double) Math.random() < GlobalVariables.RATIO_OF_ORIGINALCLASS){
						v = new Vehicle(h);
					}else{
						if(GlobalVariables.ENABLE_MULTICLASS_VEHICLES_PREDEFINEDROUTE){//Gehlot: generate a vehicle with predefined routes
							v = new VehicleType2_predefinedroutes(h);
						}else{ //Gehlot: generate a vehicle with different parameters
							v = new Vehicle(h,GlobalVariables.MAX_ACCELERATION_VTYPE3,GlobalVariables.MAX_DECELERATION_VTYPE3);
						}
					}
				}else{
					v = new Vehicle(h);
				}
				
				//v.setEvacuationTime(evactime);
				this.add(v);
				v.setOriginalCoord(coord);
				Point geom = fac.createPoint(coord);
				vehicleGeography.move(v, geom);
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
//				 * System.out.println(
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