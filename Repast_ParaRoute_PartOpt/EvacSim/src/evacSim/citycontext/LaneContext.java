/*
�Copyright 2008 Nick Malleson

This file is part of RepastCity.

RepastCity is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

RepastCity is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with RepastCity.  If not, see <http://www.gnu.org/licenses/>.
 */
package evacSim.citycontext;

import java.io.File;
import java.net.URI;
import java.util.Map;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.vividsolutions.jts.geom.Coordinate;

import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters;
import repast.simphony.space.gis.ShapefileLoader;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;

public class LaneContext extends DefaultContext<Lane> {

	// NM: Cache every coordinate which forms a road so that Route.onRoad() is
	// quicker.
	private static Map<Coordinate, ?> coordCache;

	public LaneContext() {
		super("LaneContext");
				
		/*
		 * GIS projection for spatial information about Lanes. 
		 */
		ContextCreator.logger.info("LaneContext: building lane context and projections");
		
		GeographyParameters<Lane> geoParams = new GeographyParameters<Lane>();
		//geoParams.setCrs("EPSG:32618");
		Geography<Lane> laneGeography = GeographyFactoryFinder
				.createGeographyFactory(null).createGeography("LaneGeography",
						this, geoParams);

		/* Read in the data and add to the context and geography */
		File laneFile = null;
		ShapefileLoader<Lane> laneLoader = null;
		
		/* CSV file for data attribute */
        String fileName = GlobalVariables.LANES_CSV;
		
		try {
			laneFile = new File(GlobalVariables.LANES_SHAPEFILE);
			URI uri=laneFile.toURI();
			laneLoader = new ShapefileLoader<Lane>(Lane.class,
					uri.toURL(), laneGeography, this);
			
			BufferedReader br = new BufferedReader(new FileReader(fileName));
			while (laneLoader.hasNext()) {
				Lane lane = laneLoader.next();
				String line=br.readLine();
				String[] result=line.split(",");
				lane=setAttribute(lane,result);
			}

		} catch (java.net.MalformedURLException e) {
			System.out
					.println("ContextCreator: malformed URL exception when reading roadshapefile. Check the 'roadLoc' parameter is correct");
			e.printStackTrace();
		} catch (FileNotFoundException e){
			System.out
			.println("ContextCreator: No road csv file found");
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();}

	}
	
	public Lane setAttribute(Lane l, String[] att){
		l.setLaneid(Integer.parseInt(att[5]));
		l.setLink(Integer.parseInt(att[1]));
		l.setLeft(Integer.parseInt(att[2]));
		l.setThrough(Integer.parseInt(att[3]));
		l.setRight(Integer.parseInt(att[4]));
		return l;
	}
}
