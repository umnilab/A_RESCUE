package evacSim.citycontext;

import java.util.Comparator;

class LaneComparator implements Comparator<Lane>
{
   // parameter are of type Object, so we have to downcast it to vehicle objects
   public int compare(Lane l1, Lane l2)
   {
      int laneId1, laneId2;
//      laneId1 = Math.abs(l1.getLaneid());
//      laneId2 = Math.abs(l2.getLaneid());
      laneId1 = l1.getLaneid();
      laneId2 = l2.getLaneid();
 
      if (laneId1 < laneId2)
         return -1;
      else if (laneId1 > laneId2)
         return 1;
      else
         return 0;
   }
}
