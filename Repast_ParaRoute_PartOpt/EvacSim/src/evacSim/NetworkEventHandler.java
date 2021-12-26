package evacSim;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
//import java.util.LinkedList;
import java.util.Map;
//import java.util.Queue;
import java.util.TreeMap;
import com.vividsolutions.jts.geom.Coordinate;

import org.apache.log4j.Logger;

import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import au.com.bytecode.opencsv.CSVReader;
import evacSim.citycontext.Lane;
import evacSim.citycontext.LaneContext;
//import evacSim.citycontext.CityContext;
import evacSim.citycontext.Road;
import evacSim.data.DataCollector;
import evacSim.network.ConnectionManager;
import evacSim.vehiclecontext.Vehicle;

/* Author: Xianyuan Zhan and Hemant Gehlot
 * Schedules and handles the supplyside Events to be executed
 * */

public class NetworkEventHandler {
	private Logger logger = ContextCreator.logger;
	// Queue that store unhappened events
	// Queue operations see https://docs.oracle.com/javase/7/docs/api/java/util/Queue.html
	// A sorted list storing the running events, sorted following the order of the end time
	// We use a treeMap and use the end time as the key
	// TreeMap usage see: https://docs.oracle.com/javase/7/docs/api/java/util/TreeMap.html
	private TreeMap<Integer, ArrayList<NetworkEventObject>> runningQueue;
	
	// Connection manager maintains the socket server for remote programs
	@SuppressWarnings("unused")
	private static final ConnectionManager manager = ConnectionManager.getInstance();
	
	
	// Constructor: initialize everything
	public NetworkEventHandler() {
		runningQueue = new TreeMap<Integer, ArrayList<NetworkEventObject>>();
		readEventFile();
	}
	
	/**
	 * Implement the file read and insert to the event queue.
	 * Note that the queue is first in first out, so the event file should 
	 * order event start time in ascending order, earliest comes first.
	 * @author Hemant
	 */
	public void readEventFile() {
		File eventFile = new File(GlobalVariables.EVENT_FILE);
		CSVReader csvreader = null;
		String[] nextLine;
		int startTime = 0;
		int endTime = 0;
		int eventID = 0;
		int roadID = 0;
		double value1 = 0.0d;
		double value2 = 0.0d;
		
		try {
			csvreader = new CSVReader(new FileReader(eventFile));
			// This is used to avoid reading the header (Data is assumed to
			// start from the second row)
			boolean readingheader = true;

			// This while loop is used to read the CSV iterating through the row
			while ((nextLine = csvreader.readNext()) != null) {
				// Do not read the first row (header)
				if (readingheader) {
					readingheader = false;
					
				} else {
					startTime = Math.round(Integer.parseInt(
							nextLine[0])/GlobalVariables.SIMULATION_STEP_SIZE);
					endTime = Math.round(Integer.parseInt(
							nextLine[1])/GlobalVariables.SIMULATION_STEP_SIZE);
					// Make StartTime and endTime divisible by EVENT_CHECK_FREQUENCY
					startTime = startTime - (
							startTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
					endTime = endTime - (endTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
					eventID = Integer.parseInt(nextLine[2]);
					roadID = Integer.parseInt(nextLine[3]);
					value1 = Double.parseDouble(nextLine[4]);
					value2 = Double.parseDouble(nextLine[5]);
					NetworkEventObject EventObject = new NetworkEventObject(
							startTime, endTime, eventID, roadID, value1, value2); 
					GlobalVariables.newEventQueue.add(EventObject);
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	// To be scheduled at every tick in Context Creator
	public void checkEvents() {
		// Get current tick
		int tickcount = (int) RepastEssentials.GetTickCount();
		// check new events
		checkNewEvents(tickcount);
		// terminate old events
		terminateRunningEvents(tickcount);
	}
	
	// Check new events from the event stack and add it into the running queue
	public void checkNewEvents(int tickcount) {
		boolean flag = true; // identify last usable event
		while (flag) {
			NetworkEventObject e = GlobalVariables.newEventQueue.peek();
			if (e != null) {
				if (e.startTime <= tickcount) {
					// Make the event happen
					NetworkEventObject event = this.setEvent(e, true);
					
					//HG: store event information in data buffer 
					try {
							DataCollector.getInstance().recordEventSnapshot(e, 1);//Here 1 value denotes starting of event
					}
					catch (Throwable t) {
					    // could not log the event starting in data buffer!
					    DataCollector.printDebug("ERR" + t.getMessage());
					}
					
					if (event != null) {
						// Add it into the running queue and remove the event from the newEventQueue and 
						if (this.runningQueue.containsKey(e.endTime)) {
							// If running queue already contains a endTime entry, simply append it to the arrayList
							this.runningQueue.get(e.endTime).add(e);
						} else {
							// Create a new arrayList that contains e
							ArrayList<NetworkEventObject> runningEvents = new ArrayList<NetworkEventObject>();
							runningEvents.add(e);
							this.runningQueue.put(e.endTime, runningEvents);
						}
					}
					GlobalVariables.newEventQueue.remove(); 
				} else {
					// If the event has start time later than current tick, no need to check the rest
					flag = false;
				}
			} else {
				flag = false;
			}
		}
	}
	
	// Check the running events and remove it from the running queue
	public void terminateRunningEvents(int tickcount) {
		if (this.runningQueue.containsKey(tickcount)) {
			// If there are events to be ended at this tick, we terminate it
			ArrayList<NetworkEventObject> terminateEvents = this.runningQueue.get(tickcount);
			// We terminate every events in the set
			for (NetworkEventObject e : terminateEvents) {
				this.setEvent(e, false);
				
				//HG: store event information in data buffer 
				try {
						DataCollector.getInstance().recordEventSnapshot(e, 2);//Here 2 value denotes ending of event
				}
				catch (Throwable t) {
				    // could not log the event ending in data buffer!
				    DataCollector.printDebug("ERR" + t.getMessage());
				}
			}
			this.runningQueue.remove(tickcount);
			terminateEvents.clear();
		}
	}
	
	// Make the event work in the simulation, do actual work. If mode = true, set the event; if false, terminate the event
	public NetworkEventObject setEvent(NetworkEventObject event, boolean setEvent) {
		if (event == null)
			return null;
		if (logger == null)
			logger = ContextCreator.logger;
		// Iterate over the roads to identify the correct road object
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		Iterable<Road> roadIterable = roadGeography.getAllObjects();
		for (Road road : roadIterable) {
			if (road.getLinkid() == event.roadID) {
				// If the event needs to be set
				if (setEvent) {
					// If there is no event running, set this event, depending on its ID
					if (!road.checkEventFlag()) {
						switch (event.eventID) {
							case 1: // Change speed limit, set the event from file
								return setEvent1(road, event);
							case 2: // Change speed limit and the speed of the opposite link
								return setEvent2(road, event);
							case 3: // Reverse the road direction
								return setEvent3(road, event);
							default:
								break;
						}
					} else { // Terminate the existing event on this road
						NetworkEventObject conflictEvent = null;
						for (Map.Entry<Integer, ArrayList<NetworkEventObject>> entry: runningQueue.entrySet()) {
							ArrayList<NetworkEventObject> tempEventList = entry.getValue();
							// Iterate through the event list
							for (NetworkEventObject e : tempEventList) {
								if (e.roadID == event.roadID) {
									// We found a conflicting event
									conflictEvent = e;
									break;
								}
							}
						}
						// terminate the conflict event
						if (conflictEvent != null) {
							// restore the event
							NetworkEventObject tempEvent = setEvent(conflictEvent, false);
							// Clear the running queue
							runningQueue.get(conflictEvent.endTime).remove(conflictEvent);
							// Set the new event
							tempEvent = setEvent(event, true);
							return tempEvent;
						}
					}
				} else {
					// Restore the event from file
					switch (event.eventID) {
						case 1: // restore speed limit
							return restoreEvent1(road, event);
						case 2:
							return restoreEvent2(road, event);
						case 3: // Recover the link reversal
							restoreEvent3(road);
						default: break;
					}
				}
				break;
			}
		}
		return null;
	}
	
	private NetworkEventObject setEvent1(Road road, NetworkEventObject event) {
		road.setDefaultFreeSpeed();
		road.updateFreeFlowSpeedEvent((float) event.value1);
		road.setEventFlag();
		// if opposite link has speed
		if((float) event.value2>=0){
			Road oppRoad = road.getOppositeRoad();
			if(oppRoad != null) {
				oppRoad.setDefaultFreeSpeed();
				oppRoad.updateFreeFlowSpeedEvent((float) event.value2);
			}
		}
		return event;
	}
	
	private NetworkEventObject setEvent2(Road road, NetworkEventObject event) {
		road.setDefaultFreeSpeed();
		road.updateFreeFlowSpeedEvent((float) event.value1);
		road.setEventFlag();
		// if opposite link has speed
		if((float) event.value2>=0) {
			Road oppRoad = road.getOppositeRoad();
			if (oppRoad != null) {
				oppRoad.setDefaultFreeSpeed();
				oppRoad.updateFreeFlowSpeedEvent((float) event.value2);
			}
		}
		return event;
	}
	
	/** 
	 * Move the vehicles from one lane to another lane & from this
	 * road to opposite road, and update the position of the reversed
	 * lane vehicles on the new lane and update the macroList of the
	 * opposite road.
	 * @param fromLane the lane from which vehicles are to be transfered
	 * @param toLane the lane to which the vehicles are to be transfered
	 * @author Rajat
	 * */
	private void transferVehiclesInLaneReversal(
			Lane fromLane, Lane toLane) {
		Vehicle veh = fromLane.getLastVehicle();
//		toLane.setFirstVehicle(veh);
//		toLane.setLastVehicle(fromLane.getFirstVehicle());
		while (veh != null) { // for each vehicle on the old lane
			// remove this vehicle from previous road and lane
			veh = veh.moveToNewLane(toLane);
		}
	}
	
	/**
	 * Event type 3: Reverse all lanes of the given road. This is done by 
	 * first creating duplicate dummy lanes from the opposite (through after 
	 * reversal) road and zeroing the speed of the existing lanes, effectively 
	 * rendering them useless. Then, vehicles of the old lanes are moved to 
	 * their new (previously opposite) road and their positions are updated in 
	 * the macroList.
	 * @author Rajat
	 */
	private NetworkEventObject setEvent3(Road road, NetworkEventObject event) {
		logger.info("Reversing " + road);
		LaneContext laneContext = ContextCreator.getLaneContext();
		Road oppRoad = road.getOppositeRoad();
		// do nothing if there is not a road in the opposite direction
		if (oppRoad == null) {
			logger.warn("Skipping "+road+": no opposite road");
			return event;
		}
		if (road.getLanes().size() != oppRoad.getLanes().size()) {
			logger.warn("Skipping "+road+" because no. of lanes " +  
					"does not equal that on opposite "+oppRoad);
			return event;
		}
		// Want to reverse road to have the same direction as oppRoad
		// (1) Set the speed of the current link to be (close to) zero
		road.updateFreeFlowSpeedEvent(0.01f);
		road.updateSpeedEvent(0.01f);
		road.setNumberOfVehicles(0);
		ContextCreator.getCityContext().modifyRoadNetwork();
		// (2) Add certain amount of lanes to the opposite link
		for (int i=0; i < road.getLanes().size(); i++) { // for each lane on target road
			Lane oldLane = road.getLane(i);
			Lane newLane = new Lane();
			laneContext.add(newLane);
			newLane.setLength(oldLane.getLength());
//			Coordinate[] laneCoords = ContextCreator.getLaneGeography()
//					.getGeometry(oldLane).getCoordinates();
//			for (int j = laneCoords.length - 1; j >= 0; j--) {
//				
//			}
			// RV: set references between new and old lanes
			newLane.setPreReversalLane(oldLane);
			oldLane.setPostReversalLane(newLane);
			// use the negative index of the old lane's index as this one's
			newLane.setLaneid(-oldLane.getLaneid());
			newLane.setRoad(oppRoad);
			// assign the lane attributes of the opposite road to this road's new lanes
			// get the paired lane based on the same index in the two roads
			Lane oppLane = oppRoad.getLane(i);
			newLane.setLeft(oppLane.getLeft());
			newLane.setThrough(oppLane.getThrough());
			newLane.setRight(oppLane.getRight());
			/* (3) Move the vehicles from old lane to new lane & from this 
			 * road to opposite road, and update the position of the reversed 
			 * lane vehicles on the new lane and update the macroList of the 
			 * opposite road */
			Vehicle veh = oldLane.getLastVehicle();
			while (veh != null) // for each vehicle on the old lane
				veh = veh.moveToNewLane(newLane);
		}
		return event;
	}
	
	/**
	 * Restore a previously reversed road to its original direction.
	 * Here, the input road is the one whose lanes were dissolved and 
	 * alternate lanes created on its opposite road.
	 * @author Rajat
	 */
	private void restoreEvent3(Road road) {
		logger.info("Undoing reversal on road ID " + road.getLinkid());
		LaneContext laneContext = ContextCreator.getLaneContext();
		Road oppRoad = road.getOppositeRoad();
		// do nothing if there is not a road in the opposite direction
		if (oppRoad == null) {
			return;
		}
		// Want to reverse road to have the same direction as oppRoad
		// (1) Reset the free flow speed
		road.updateFreeFlowSpeedEvent((float) road.getDefaultFreeSpeed());
		road.updateSpeedEvent((float) road.getDefaultFreeSpeed());
		road.setNumberOfVehicles(0);
		road.setTravelTime();
		ContextCreator.getCityContext().modifyRoadNetwork();
		// (2) Remove the added lanes
		for (Lane postLane : road.getLanes()) { // for each lane on target road
			Lane preLane = postLane.getPreReversalLane();
			if (preLane == null) {
				logger.warn("Pre-reversal lane for " + postLane.getLaneid() 
				+ " not found.");
				continue;
			}
			preLane.setPostReversalLane(null);
			/* (3) Move the vehicles from old lane to new lane & from this 
			 * road to opposite road, and update the position of the reversed 
			 * lane vehicles on the new lane and update the macroList of the 
			 * opposite road */
			transferVehiclesInLaneReversal(preLane, postLane);
			// (4) Remove this lane from the list of lanes
			laneContext.remove(postLane);
		}
	}
	
	private NetworkEventObject restoreEvent1(Road road, NetworkEventObject event) {
		// To be moved into a buffer variable in the road
		road.updateFreeFlowSpeedEvent((float) road.getDefaultFreeSpeed());
		road.restoreEventFlag();
		if ((float) event.value2 >= 0) {
			Road oppRoad = road.getOppositeRoad();
			if (oppRoad != null) {
				oppRoad.updateFreeFlowSpeedEvent((float) oppRoad.getDefaultFreeSpeed());
			}
		}
		return event;
		
	}
	
	private NetworkEventObject restoreEvent2(Road road, NetworkEventObject event) {
		// To be moved into a buffer variable in the road
		road.updateFreeFlowSpeedEvent((float) road.getDefaultFreeSpeed());
		road.restoreEventFlag();
		if ((float) event.value2 >= 0) {
			Road oppRoad = road.getOppositeRoad();
			if(oppRoad != null) {
				oppRoad.updateFreeFlowSpeedEvent((float) oppRoad.getDefaultFreeSpeed());
			}
		}
		return event;
	}

}
