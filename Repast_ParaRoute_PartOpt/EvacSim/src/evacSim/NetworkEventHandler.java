	package evacSim;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.lang.IllegalStateException;
//import java.util.LinkedList;
import java.util.Map;
//import java.util.Queue;
import java.util.TreeMap;
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
	
	
	public void readEventFile(){
		// Hemant: implement the file read and insert to the event queue
		// Note queue is first in first out, so the eventfile should order event start time in ascending order, earliest comes first
		
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
					startTime = Math.round(Integer.parseInt(nextLine[0])/GlobalVariables.SIMULATION_STEP_SIZE);
					endTime = Math.round(Integer.parseInt(nextLine[1])/GlobalVariables.SIMULATION_STEP_SIZE);
					// Make StartTime and endTime divisable by EVENT_CHECK_FREQUENCY
					startTime = startTime - (startTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
					endTime = endTime - (endTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
					eventID = Integer.parseInt(nextLine[2]);
					roadID = Integer.parseInt(nextLine[3]);
					value1 = Double.parseDouble(nextLine[4]);
					value2 = Double.parseDouble(nextLine[5]);
					//System.out.println("starttime = "+startTime+ "endtime ="+endTime+"eventID = " + eventID+"roadID = "+ roadID+"value1 =" + value1+ "value2 =" + value2);
					
					NetworkEventObject EventObject = new NetworkEventObject(startTime, endTime, eventID, roadID, value1, value2); 
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
					    // could not log the event strating in data buffer!
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
		// Iterate over the roads to identify the correct road object
		LaneContext laneContext = ContextCreator.getLaneContext();
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		Iterable<Road> roadIterable = roadGeography.getAllObjects();
		for (Road road : roadIterable) {
			if (road.getLinkid() == event.roadID) {
				// Found the road, and we do the change
				if (setEvent) {
					switch (event.eventID) {
						case 1: // Change speed limit, set the event from file
							if (!road.checkEventFlag()) {
								road.setDefaultFreeSpeed();
								road.updateFreeFlowSpeed_event((float) event.value1);
								road.setEventFlag();
								// if opposite link has speed
								if((float) event.value2>=0){
									Road oppRoad = road.getOppositeRoad();
									if(oppRoad != null) {
										oppRoad.setDefaultFreeSpeed();
										oppRoad.updateFreeFlowSpeed_event((float) event.value2);
									}
								}
								return event;
							} else {
								// If there is another event running on the same road, we need to terminate it
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
						case 2: // Change speed limit and the speed of the opposite link
							if (!road.checkEventFlag()) {
								road.setDefaultFreeSpeed();
								road.updateFreeFlowSpeed_event((float) event.value1);
								road.setEventFlag();
								// if opposite link has speed
								if((float) event.value2>=0){
									Road oppRoad = road.getOppositeRoad();
									if(oppRoad != null) {
										oppRoad.setDefaultFreeSpeed();
										oppRoad.updateFreeFlowSpeed_event((float) event.value2);
									}
								}
								return event;
							} else {
								// If there is another event running on the same road, we need to terminate it
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
						case 3: // Reverse the link
							if (!road.checkEventFlag()) { // if there is no event going on currently
								Road oppRoad = road.getOppositeRoad();
								// do nothing if there is not a road in the opposite direction
								if (oppRoad == null) {
									continue;
								}
								// Want to reverse road to have the same direction as oppRoad
								// Step 1: Set the speed of the current link to be (close to) zero
								road.updateFreeFlowSpeed_event((float) 1);
								// Step 2: Add certain amount of lanes to the opposite link
								for (int i=0; i < road.getLanes().size(); i++) {
									Lane oldLane = road.getLane(i);
									Lane newLane = new Lane();
									laneContext.add(newLane);
									// RV: set references between new and old lanes
									newLane.setPreReversalLane(oldLane);
									oldLane.setPostReversalLane(newLane);
									// use the -ve index of the old lane's index as this one's
									newLane.setLaneid(-oldLane.getLaneid());
									// new lane's road is the old lane's opposite road
									newLane.setRoad(oppRoad);
									// assign the lane attributes of the opposite road to this road's new lanes
									// throw error if the no. of lanes on the current & opposite road don't match
									if (road.getLanes().size() == oppRoad.getLanes().size()) {
										// get the paired lane based on the same index in the two roads
										Lane oppLane = oppRoad.getLane(i);
										newLane.setLeft(oppLane.getLeft());
										newLane.setThrough(oppLane.getThrough());
										newLane.setRight(oppLane.getRight());
									} else {
										throw new IllegalStateException(
												"No. of lanes on reversed lane do not match with opposite road's lanes");
									}
									// Step 3: Move the vehicle from this link to the opposite one
									Vehicle firstVehicle = oldLane.firstVehicle();
								}
								// Step 4: Update the topology of the network
								ContextCreator.getCityContext().modifyRoadNetwork();
							}
							else {
								// If there is another event running on the same road, we need to terminate it
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
							
						// Other cases to be implemented here, if there are any
						default: break;
					}
				} else {
					// Restore the event from file
					switch (event.eventID) {
						case 1: // restore speed limit
							road.updateFreeFlowSpeed_event((float) road.getDefaultFreeSpeed()); // To be moved into a buffer variable in the road
							road.restoreEventFlag();
							if ((float) event.value2 >= 0) {
								Road oppRoad = road.getOppositeRoad();
								if(oppRoad != null) {
									oppRoad.updateFreeFlowSpeed_event((float) oppRoad.getDefaultFreeSpeed());
								}
							}
							return event;
						// Other cases to be implemented later
						case 2:
							road.updateFreeFlowSpeed_event((float) road.getDefaultFreeSpeed()); // To be moved into a buffer variable in the road
							road.restoreEventFlag();
							if ((float) event.value2 >= 0) {
								Road oppRoad = road.getOppositeRoad();
								if(oppRoad != null) {
									oppRoad.updateFreeFlowSpeed_event((float) oppRoad.getDefaultFreeSpeed());
								}
							}
							return event;
						case 3: // Recover the link reversal
							Road oppRoad = road.getOppositeRoad();
							if (oppRoad == null) continue;
							for(Lane l: road.getLanes()) {
								// Use negative value for the reversed lane
								Lane new_l = ContextCreator.getCityContext().getLanefromID(-l.getLaneid());
								// Step1: Move the vehicle from this lane to previously existing lanes 
								Vehicle first_v = l.firstVehicle();
								// Step2: Remove the additional lanes of the reversed link
								ContextCreator.getLaneContext().remove(new_l);
								oppRoad.removeLane(new_l);
							}
							// Step3: Recover the speed of this link
							road.updateFreeFlowSpeed_event((float) road.getDefaultFreeSpeed()); 
							road.restoreEventFlag();
							// Step4: Update the network topology
							ContextCreator.getCityContext().modifyRoadNetwork();
						default: break;
					}
				}
				break;
			}
		}
		
		
		return null;
	}
	
}
