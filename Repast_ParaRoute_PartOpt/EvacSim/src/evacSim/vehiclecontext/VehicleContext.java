package evacSim.vehiclecontext;

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
		@SuppressWarnings("unused")
		Geography<Vehicle> vehicleGeography = GeographyFactoryFinder
		.createGeographyFactory(null).createGeography(
				"VehicleGeography", this, geoParams);
		//		Geography<Zone> zoneGeography;
		//		zoneGeography = ContextCreator.getZoneGeography();
		createVehicleContextFromActivityModels();
	}

	public void createVehicleContextFromActivityModels() {
		Geography<Zone> zoneGeography=ContextCreator.getZoneGeography();
		double propPreDetermined = GlobalVariables.PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES;
		double propLessFreq = GlobalVariables.PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES;
		for (Zone z : zoneGeography.getAllObjects()) {
			Geometry hgeom = zoneGeography.getGeometry(z);
			Coordinate coord = hgeom.getCoordinate();
			for (House h : z.getHouses()) {
				Vehicle v;
				//TODO: Code a mechanism to generate vehicles with different
				// parameters (like max acceleration)
				if (GlobalVariables.ENABLE_MULTICLASS_ROUTING){
					// HG: Generate multi-class vehicles
					if ((double) Math.random() > (propPreDetermined + propLessFreq)) {
						v = new Vehicle(h); 
					} else if ((double) Math.random() < propPreDetermined/
							(propPreDetermined + propLessFreq)) {
						v = new Vehicle_predefinedroutes(h);
					} else {
						v = new Vehicle_less_frequent_routing(h);
					}
				} else {
					v = new Vehicle(h);
				}
				v.setOriginalCoord(coord);
				// LZ: instead of creating geometry, just storing the coordinates
				v.setCurrentCoord(coord);
				Road road = z.getdepartureRoad();
				if (road.getLinkid() == 104819) {
					road =  ContextCreator.getCityContext().findRoadWithLinkID(104818);
				}
				if (road.getLinkid() == 101235) {
					road =  ContextCreator.getCityContext().findRoadWithLinkID(101236);
				}
				road.addVehicleToNewQueue(v);
			}
		}
	}

}