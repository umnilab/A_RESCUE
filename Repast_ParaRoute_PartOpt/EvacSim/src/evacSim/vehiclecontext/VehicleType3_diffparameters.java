package evacSim.vehiclecontext;

import evacSim.GlobalVariables;
import evacSim.citycontext.House;

//Gehlot: This is a subclass of Vehicle class
public class VehicleType3_diffparameters extends Vehicle {

	//Gehlot: A modified constructor is used to generate a vehicle class with different parameters like acceleration, deceleration
	public VehicleType3_diffparameters(House h) {
		super(h);
		// TODO Auto-generated constructor stub
		this.maxAcceleration_ = GlobalVariables.MAX_ACCELERATION_VTYPE3;
		this.maxDeceleration_ = GlobalVariables.MAX_DECELERATION_VTYPE3;
	}

}
