package evacSim;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.TreeMap;
import repast.simphony.essentials.RepastEssentials;

/* Author: Xianyuan Zhan and Hemant Gehlot
 * Schedules and handles the supplyside Events to be executed
 * */

public class NetworkEventHandler {
	// Queue that store unhappened events
	// Queue operations see https://docs.oracle.com/javase/7/docs/api/java/util/Queue.html
	private Queue<NetworkEventObject> newEventQueue;
	// A sorted list storing the running events, sorted following the order of the end time
	// We use a treeMap and use the end time as the key
	// TreeMap usage see: https://docs.oracle.com/javase/7/docs/api/java/util/TreeMap.html
	private TreeMap<Integer, ArrayList<NetworkEventObject>> runningQueue;
	
	// Constructor: initialize everything
	public NetworkEventHandler() {
		newEventQueue = new LinkedList<NetworkEventObject>();
		runningQueue = new TreeMap<Integer, ArrayList<NetworkEventObject>>();
		readEventFile();
	}
	
	
	public void readEventFile(){
		// Hemant: implement the file read and insert to the event queue
		// Note queue is first in first out, so the eventfile should order event start time in ascending order, earliest comes first
		File eventFile = new File(GlobalVariables.EVENT_FILE);
		
		// TODO
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
			NetworkEventObject e = this.newEventQueue.peek();
			if (e != null) {
				if (e.startTime <= tickcount) {
					// Make the event happen
					this.setEvent(e, true);
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
					this.newEventQueue.remove();
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
			}
			this.runningQueue.remove(tickcount);
			terminateEvents.clear();
		}
	}
	
	// Make the event work in the simulation, do actual work. If mode = true, set the event; if false, terminate the event
	public void setEvent(NetworkEventObject event, boolean mode) {
		// TODO
	}
	
}
