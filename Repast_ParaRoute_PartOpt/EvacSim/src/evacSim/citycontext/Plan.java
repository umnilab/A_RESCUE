package evacSim.citycontext;

public class Plan {
	private Integer location;
	private Float duration;

	public Plan(Integer loc, Float dur) {
		this.location = loc;
		this.duration = dur;
	}
	public Integer getLocation(){
		return location;
	}
	public Float getDuration(){
		return duration;
	}
	public String toString() {
		return "Plan(" + location + "," + duration + ")";
	}
}