package evacSim.citycontext;

import java.lang.Iterable;

import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.context.space.graph.NetworkFactory;
import repast.simphony.context.space.graph.NetworkFactoryFinder;
import repast.simphony.space.gis.GeographyParameters;

import java.util.HashMap;

import org.apache.log4j.Logger;

import com.vividsolutions.jts.geom.Coordinate;

import evacSim.ContextCreator;

/**
 * Context which holds junction objects and also the RoadNetwork.
 * 
 * @author Nick Malleson
 */
public class JunctionContext extends DefaultContext<Junction>
{
	private Logger logger = ContextCreator.logger;

   public JunctionContext()
   {

      super("JunctionContext");
      ContextCreator.logger.info("JunctionContext creation");

      /* Create a Network projection for the road network--->Network Projection */
      NetworkFactory netFac =
               NetworkFactoryFinder.createNetworkFactory(new HashMap<String, Object>());
      netFac.createNetwork("RoadNetwork", this, true);

      /* Create a Geography to store junctions in spatially-->Junction Geography */
      GeographyParameters<Junction> geoParams = new GeographyParameters<Junction>();
      //geoParams.setCrs("EPSG:32618");
      GeographyFactoryFinder.createGeographyFactory(null).createGeography("JunctionGeography",
               this, geoParams);

   }

   /**
    * Runs through all the junctions in the context. If it finds one with 
    * coordinates which are the same as the Junction passed to this functions 
    * it returns true.
    */
   public boolean existsInContext(Junction j)
   {
      Iterable<Junction> it = this.getObjects(Junction.class);
      for (Junction junc : it)
      {
         if (junc.equals(j)) return true;
      }
      return false;
   }

   public Junction getJunctionWithCoordinates(Coordinate c)
   {
      Iterable<Junction> it = this.getObjects(Junction.class);
      for (Junction junc : it)
      {
         if (junc.getCoordinate().equals(c)) return junc;
      }
      logger.error("JunctionContext: getJunctionWithCoordinates: error, junction not found. ");
      logger.error("Coordinates: " + c.toString());
      return null;
	}
   
   public Junction getJunctionFromID(int id){
	   Iterable<Junction> it = this.getObjects(Junction.class);
	      for (Junction junc : it)
	      {
	         if (junc.getJunctionID() == id) return junc;
	      }
	      System.err.print("JunctionContext: getJunctionFromID: error, junction not found. ");
	      logger.error("Junction ID: %d\n" + id);
	      return null;
   }

}
