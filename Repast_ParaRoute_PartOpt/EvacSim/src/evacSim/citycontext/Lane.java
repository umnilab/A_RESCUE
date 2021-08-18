/**
 * @author Samiul Hasan and Binh Luong 
 */

package evacSim.citycontext;

import java.util.ArrayList;

import org.apache.log4j.Logger;

import evacSim.ContextCreator;
import evacSim.vehiclecontext.Vehicle;
import evacSim.GlobalVariables;

@SuppressWarnings("unused")
public class Lane {
	Logger logger = ContextCreator.logger;
	
	/** SH: An auto-generated id from ContextCreater */
	private int id;
	private int index;
	
	/** Attributes of the lane taken from the shapefile */
	private int laneid;
	private int link;
	private int left;
	private int through;
	private int right;
	private float length;
	/** SH: The road to which this lane belongs to */
	private Road road_;
	/** SH: Number of vehicle in the lane */
	private int nVehicles_;
	/** SH: The first & last vehicle on a lane */
	private Vehicle firstVehicle_; 
	private Vehicle lastVehicle_;
	/** Currently accumulated speed and density of this lane */
	private double accumulatedDensity_;
	private double accumulatedSpeed_;
	/** Current space mean speed of the lane (m/s) */
	private float speed_;
	/** Max allowed speed of this lane (m/s) */
	private float maxSpeed_;
	/** No. of upstream and downstream connections of this lane */
	private int upConnections_;
	private int downConnections_;
	/** BL: List of upstream & downstream lanes that connect to this lane */
	private ArrayList<Lane> upLanes_;
	private ArrayList<Lane> dnLanes_;
	/** LZ: Store the latest enter time of vehicles */
	private int lastEnterTick = -1;

	public Lane() {
		this.id = ContextCreator.generateAgentID();
		this.nVehicles_ = 0;
		this.lastVehicle_ = null;
		this.upLanes_ = new ArrayList<Lane>();
		this.dnLanes_ = new ArrayList<Lane>();
	}
	
	public String toString() {
		return "<Lane"+this.laneid+"@Road"+this.link+">";
	}

	/* Getters ------------------------------------------------------------- */

	public int getID() {
		return id;
	}
	
	public int getIndex() {
		return index;
	}
	
	public int getLaneid() {
		return laneid;
	}
	
	public float getLength() {
		return length;
	}
	
	public int getLink() {
		return link;
	}
	
	public Road road_() {
		return road_;
	}

	public int getNumVehicles() {
		return nVehicles_;
	}
	
	public int getLastEnterTick() {
		return lastEnterTick;
	}
	
	public int getLeft() {
		return left;
	}
	
	public int getThrough() {
		return through;
	}
	
	public int getRight() {
		return right;
	}
	
	public Lane getUpLane(int i) {
		return upLanes_.get(i);
	}

	public Lane getDnLane(int i) {
		return dnLanes_.get(i);
	}
	
	public ArrayList<Lane> getDnLanes() {
		return dnLanes_;
	}

	public ArrayList<Lane> getUpLanes() {
		return upLanes_;
	}
	
	public int index() {
		return index;
	}
	
	public float speed() {
		return speed_;
	}
	
	public float maxSpeed() {
		return maxSpeed_;
	}
	
	public double lengthRoad() {
		return road_.length();
	}

	public Vehicle firstVehicle() {
		return firstVehicle_;
	}

	public Vehicle lastVehicle() {
		return lastVehicle_;
	}
	
	public float calcMaxSpeed() {
		return maxSpeed_;
	}
	
	public int nVehicles() {
		return nVehicles_;
	}
	
	public Lane getUpStreamConnection(Road pr) {
		Lane connectLane = null;
		for (Lane ul : this.getUpLanes()) {
			if (ul.road_().equals(pr)) {
				connectLane = ul;
				break;
			}
		}
		return connectLane;
	}
	
	/**
	 * BL: Returns the last vehicle in the downstream lanes. The vehicle 
	 * closest to the upstream end is returned.
	 */
	public Vehicle lastInDnLane() {
		Vehicle last = null;
		Vehicle pv;
		double mindis = GlobalVariables.FLT_INF;
		double dis;
		Lane dlane;
		int i;
		for (i = 0; i < dnLanes_.size(); i++) {
			dlane = dnLanes_.get(i);
			pv = dlane.lastVehicle_;
			if (pv != null) {
				dis = dlane.getLength() - (pv.distance() + pv.length());
				if (dis < mindis) {
					mindis = dis;
					last = pv;
				}
			}
		}
		return (last);
	}
	
	/* Setters ------------------------------------------------------------- */
	
	public void setIndex(){
		this.index = this.road_.getLaneIndex(this);
	}
	
	public void setLaneid(int laneid) {
		this.laneid = laneid;
	}

	public void setLength(float length) {
		this.length = length;
	}

	public void setLink(int link) {
		this.link = link;
	}
	
	public void setRoad(Road road) {
		this.road_ = road;
		road.addLane(this);
	}
	
	public void setLeft(int left) {
		this.left = left;
	}

	public void setThrough(int through) {
		this.through = through;
	}

	public void setRight(int right) {
		this.right = right;
	}
	
	public void upConnection(int n) {
		this.upConnections_ = n;
	}

	public void downConnection(int n) {
		this.downConnections_ = n;
	}
	
	public void addDnLane(Lane l) {
		this.dnLanes_.add(l);
	}

	public void addUpLane(Lane l) {
		this.upLanes_.add(l);
	}

	public void updateLastEnterTick(int current_tick) {
		this.lastEnterTick = current_tick;
	}
	
	public void firstVehicle(Vehicle v) {
		if (v != null) {
			this.firstVehicle_ = v;
			v.leading(null);
		} else
			this.firstVehicle_ = null;
	}

	public void lastVehicle(Vehicle v) {
		if (v != null) {
			this.lastVehicle_ = v;
			v.trailing(null);
		} else
			this.lastVehicle_ = null;

	}
	
	/** 
	 * Add only the number of vehicle to lane, while addVehicle in road and
	 * a vehicle to arrayList.
	 */
	public void addVehicles() {
		nVehicles_++;
	}

	public void removeVehicles() {
		this.nVehicles_--;
	}
	
	/* Others -------------------------------------------------------------- */

	/**
	 * BL: Reset the accumulated speed and accumulated density to recalculate 
	 * in the next simulation step.
	 */
	public void resetStatistics() {
		accumulatedSpeed_ = 0;
		accumulatedDensity_ = 0;
	}

	public void printShpInput() {
		logger.info("Repast Lane ID: " + id + " Lane ID: "+ laneid
				+" link " + link + " L: "+ left + " T: " + through 
				+ " R: " + right + " Repast road ID "
				+ road_.getID());
	}

	/**
	 * BL: Get all the downstream lanes that connect to this lane.
	 */
	public void printLaneConnection(){
		logger.info("Road: " + this.road_.getLinkid() +
				" lane " + this.laneid + " has downstream connections: ");
		for (int i=0; i < this.dnLanes_.size(); i++) {
			logger.info("To Lane: " + this.dnLanes_.get(i).laneid +
					" of road: " + this.dnLanes_.get(i).road_.getLinkid());
		}
		logger.info("and with upstream connection: ");
		for (int i=0;i<this.upLanes_.size();i++) {
			logger.info("To Lane: " + this.upLanes_.get(i).laneid + 
					" of road: " + this.upLanes_.get(i).road_.getLinkid());
		}
	}
	
	/*
	 * BL: Following are functions dedicated for discretionary lane changing:
	 */
	public boolean isConnectToLane(Lane pl) {
		boolean connectFlag = false;
		if (pl != null) {
			for (Lane ul : pl.getUpLanes()) {
				if (ul.equals(this))
					connectFlag = true;
			}
		}
		return connectFlag;
	}

	public Lane betterLane(Lane plane) {
		if (this != null && plane != null) {
			if (this.nVehicles_ < plane.nVehicles_) {
				return this;
			} else if (this.nVehicles_ > plane.nVehicles_) {
				return plane;
			} else {
				double randomnumber = GlobalVariables.RandomGenerator
						.nextDouble();
				if (randomnumber > 0.5)
					return this;
				else
					return plane;
			}
		} else if (this != null)
			return this;
		else if (plane != null)
			return plane;
		return null;
	}
	
}
