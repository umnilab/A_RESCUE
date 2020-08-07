package evacSim.citycontext;

public class Plan {
	private int location;
	private int duration;

	public Plan(int loc, int dur) {
		this.location = loc;
		this.duration = dur;
	}
	
	public Integer getLocation(){
		return location;
	}
	
	public int getDuration(){
		return duration;
	}
	
	public void setDuration(int dur) {
		duration = dur;
	}
		
	public String toString() {
		return String.format("Plan(%d,%d)", location, duration);
	}
}