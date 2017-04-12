package evacSim;

/* Author: Xianyuan Zhan
 * Event object that holds supply side event information
 * */
public class NetworkEventObject {
	// Fields in the event data file
	public int startTime;
	public int endTime;
	public int eventID;
	public int roadID;
	public double value1;
	public double value2;
	// Field to be loaded from simulation
	public double defaultValue;
	// Status of the current event: true if is currently running
	public boolean isRunning;
	
	public NetworkEventObject(int startTime, int endTime, int eventID, int roadID, double value1, double value2) {
		this.startTime = startTime;
		this.endTime = endTime;
		this.eventID = eventID;
		this.roadID = roadID;
		this.value1 = value1;
		this.value2 = value2;
		// Set rest of the object to default
		this.defaultValue = -1.0;
		this.isRunning = false;
	}
	
	// Set the event if it is going to be run
	public void setEvent() {
		this.isRunning = true;
	}
	
}
