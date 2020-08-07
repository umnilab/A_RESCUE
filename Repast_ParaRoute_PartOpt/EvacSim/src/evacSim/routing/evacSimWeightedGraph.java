package evacSim.routing;

import org.jgrapht.*;
import org.jgrapht.graph.*;
import repast.simphony.space.graph.RepastEdge;
//import evacSim.ContextCreator;

public class evacSimWeightedGraph<V, E> extends SimpleDirectedWeightedGraph<V, E>
{

   public evacSimWeightedGraph(EdgeFactory ef)
   {
      super(ef);
      // TODO Auto-generated constructor stub
   }

   public evacSimWeightedGraph(Class<? extends E> edgeClass)
   {
      this(new ClassBasedEdgeFactory<V, E>(edgeClass));
   }

   @Override
   public void setEdgeWeight(E e, double weight)
   {
      // super.setEdgeWeight(e, weight);
      ((RepastEdge) e).setWeight(weight);
   }

   @Override
   public double getEdgeWeight(E e)
   {
      // super.setEdgeWeight(e, weight);
      return ((RepastEdge) e).getWeight();
   }
}
