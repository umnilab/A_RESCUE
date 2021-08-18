package evacSim;

import java.util.concurrent.*;

import org.json.simple.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import evacSim.ContextCreator;
import evacSim.citycontext.Road;
import evacSim.vehiclecontext.Vehicle;
import repast.simphony.essentials.RepastEssentials;

public class ThreadedScheduler {
//	private boolean roadFinishedStepping;
	private ExecutorService executor;
	private int N_Partition;
	private int N_threads;
	
	private int min_para_time;
	private int max_para_time;
	private int avg_para_time;
	@SuppressWarnings("unused")
	private int seq_time;

	public ThreadedScheduler(int N_threads) {
		this.N_threads = N_threads;
		this.executor = Executors.newFixedThreadPool(this.N_threads);
		this.N_Partition = GlobalVariables.N_Partition;
		
		this.min_para_time = 0;
		this.max_para_time = 0;
		this.avg_para_time = 0;
		this.seq_time = 0;
	}
	
	public void step(){
		for(Road r: ContextCreator.getRoadGeography().getAllObjects()){
			   r.step();
		}
        // Record vehicle trajectories
//		for(Road r: ContextCreator.getRoadGeography().getAllObjects()){
//			   Vehicle pv = r.firstVehicle();
//			   while(pv!=null){
//				   pv.recVehSnaphotForVisInterp();
//				   pv = pv.macroTrailing();
//			   }
//		   }
	}
	
	public void paraStep() {
		// Load the road partitions
		ArrayList<ArrayList<Road>> PartitionedInRoads = ContextCreator.partitioner.getPartitionedInRoads();
		// ArrayList<Road> PartitionedBwRoads = ContextCreator.partitioner.getPartitionedBwRoads();
		
		// Creates an list of tasks
		List<PartitionThread> tasks = new ArrayList<PartitionThread>();
		for (int i = 0; i< this.N_Partition; i++) {
			tasks.add(new PartitionThread(PartitionedInRoads.get(i), i));
		}
		try {
			List<Future<Integer>> futures = executor.invokeAll(tasks);
			ArrayList<Integer> time_stat = new ArrayList<Integer>();
			for (int i = 0; i < N_Partition; i++)
				time_stat.add(futures.get(i).get());
			ArrayList<Integer> time_result = minMaxAvg(time_stat);
			min_para_time = min_para_time + time_result.get(0);
			max_para_time = max_para_time + time_result.get(1);
			avg_para_time = avg_para_time + time_result.get(2);
			// Record vehicle trajectories
//		   for(Road r: ContextCreator.getRoadGeography().getAllObjects()){
//			   Vehicle pv = r.firstVehicle();
//			   while(pv!=null){
//				   pv.recVehSnaphotForVisInterp();
//				   pv = pv.macroTrailing();
//			   }
//		   }
			// Step over the boundary roads
//			double start_t = System.currentTimeMillis();
//			stepBwRoads();
//			seq_time = seq_time + (int)(System.currentTimeMillis() - start_t);
			// Generate a vehicle list, filename by the corresponding ticks
			int tickcount = (int) RepastEssentials.GetTickCount();
			if(tickcount % 6000 == 0) { //Every 30 min
				createVehicleList();
			}
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}
	
	@SuppressWarnings("unchecked")
	public void createVehicleList() {
		// get the default output directory
		String outDir = GlobalVariables.OUTPUT_DIR;

		// get the basename of the demand file -
		String basename = "";
		if (GlobalVariables.ORGANIZE_OUTPUT_BY_ACTIVITY_FNAME) {
			String[] temp = GlobalVariables.OUTPUT_DIR.split("/");
			basename = temp[temp.length - 1];
		}
		int tickcount = (int) RepastEssentials.GetTickCount();

		// create the overall file path, named after the demand filename
		String outpath = outDir + File.separatorChar + "vehicle-list-" + basename + "-" + tickcount/6000 + ".json";
		BufferedWriter bw = null;
		// check the path will be a valid file
		try {
			FileWriter fw = new FileWriter(outpath, false);
			bw = new BufferedWriter(fw);
			
			HashMap<Integer, String> storeJsonObjects = new HashMap<Integer, String>();
			
			for(Road r: ContextCreator.getRoadGeography().getAllObjects()){
				   Vehicle pv = r.firstVehicle();
				   while(pv!=null){
					   String info= String.format("%d,%d,%.5f,%.5f",
		        				pv.getHouse().getZoneId(), pv.getDestinationID(), pv.getCurrentCoord().x, pv.getCurrentCoord().y);
					   storeJsonObjects.put(pv.getVehicleID(), info);
					   pv = pv.macroTrailing();
				   }
			   }
		   JSONObject jsonObject = new JSONObject();
     	   jsonObject.putAll(storeJsonObjects);
     	   bw.write(JSONObject.toJSONString(jsonObject));
		   bw.flush();
		} catch (IOException e) {
			e.printStackTrace();
		} finally{
			try {
				bw.close();
			} catch (IOException e){
				e.printStackTrace();
			}
		}
	}
	
	public void stepBwRoads() {
		ArrayList<Road> PartitionedBwRoads = ContextCreator.partitioner
				.getPartitionedBwRoads();
//		try {
			for (Road r : PartitionedBwRoads) {
				r.step();
			}
		/*} catch (Exception ex) {
			ex.printStackTrace();
		}*/
	}
	
	public void shutdownScheduler() {
		executor.shutdown();
	}
	
	
	public ArrayList<Integer> minMaxAvg(ArrayList<Integer> values) {
	    int min = values.get(0);
	    int max = values.get(0);
	    int sum = 0;

	    for (int value : values) {
	        min = Math.min(value, min);
	        max = Math.max(value, max);
	        sum += value;
	    }

	    int avg = sum / values.size();
	    
	    ArrayList<Integer> results = new ArrayList<Integer>();
	    results.add(min);
	    results.add(max);
	    results.add(avg);
	    
	    return results;
	}
	
	public void reportTime() {
		this.min_para_time = 0;
		this.max_para_time = 0;
		this.avg_para_time = 0;
		this.seq_time = 0;
	}	
}

/* Single thread to call road's step() method */
class PartitionThread implements Callable<Integer> {
	private ArrayList<Road> RoadSet;
	private int threadID;
	
	public PartitionThread(ArrayList<Road> roadPartition, int ID) {
		this.RoadSet = roadPartition;
		this.threadID = ID;
	}
	
	public int getThreadID(){
		return this.threadID;
	}
	
	public Integer call() {
		double start_t = System.currentTimeMillis();
		try {
			for (Road r : this.RoadSet) {
				r.step();
			}
		} catch (Exception ex) {
			ex.printStackTrace();
		}
//		return this.threadID;
		return (int) (System.currentTimeMillis() - start_t);
	}
}
