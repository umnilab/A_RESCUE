package evacSim.data;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import evacSim.GlobalVariables;
import repast.simphony.context.DefaultContext;
import repast.simphony.engine.environment.RunEnvironment;

/**
 * DataCollectionContext
 * 
 * This functions as the home for the core of the data collection system
 * within EvacSim and the object through which the Repast framework will
 * ensure it is scheduled to receive the signals it needs to operate at
 * key points in the execution of the simulation.
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @date 20 sept 2017
 */
public class DataCollectionContext extends DefaultContext<Object> {
    
    /** A convenience reference to to the system-wide data collector. */
    private DataCollector collector;
    
    /** A consumer of output data from the buffer which saves it to disk. */
    private CsvOutputWriter csvOutputWriter;
    
    /** A consumer of output data from the buffer which saves it to disk. */
    private JsonOutputWriter jsonOutputWriter;
    
    /**
     * Creates the data collection framework for the program and ensures
     * it is ready to start receiving data when the simulation starts.
     */
    public DataCollectionContext() {
        // needed for the repast contexts framework to give it a name
        super("DataCollectionContext");
        
        // there is no real need to do this, but this gives us a location
        // where we know during startup this was guaranteed to be called
        // at least once to ensure that it created an instance of itself
        this.collector = DataCollector.getInstance();
        
        
        // RV: update the default output directory if multiple demand scenarios are run
		if (GlobalVariables.ORGANIZE_OUTPUT_BY_ACTIVITY_FNAME) {
			updateOutputDirectory();
		}
        
        // create the output file writer.  without specifying a filename,
        // this will generate a unique value including a current timestamp
        // and placing it in the current jre working directory.
        if (GlobalVariables.ENABLE_CSV_WRITE) {
            this.csvOutputWriter = new CsvOutputWriter();
            this.collector.registerDataConsumer(this.csvOutputWriter);
        }

        // create the JSON output file writer.  without specifying a filename,
        // this will generate a unique value including a current timestamp
        // and placing it in the current jre working directory.
        if (GlobalVariables.ENABLE_JSON_WRITE) {
            this.jsonOutputWriter = new JsonOutputWriter();
            this.collector.registerDataConsumer(this.jsonOutputWriter);
        }
        
        	// RV: Initialize the runtime recorder
        	if (GlobalVariables.ENABLE_RUNTIME_RECORD) {
        		GlobalVariables.RUNTIME_RECORD_LIST.add((double) System.currentTimeMillis());
        	}
    }
    
    
    public void startCollecting() {
        this.collector.startDataCollection();
    }
    
    
    public void stopCollecting() {
        this.collector.stopDataCollection();
    }
    
    
    public void startTick() {
        // get the current tick number from the system
        double tickNumber = 
            RunEnvironment.getInstance().getCurrentSchedule().getTickCount();
        
        // tell the data framework what tick is starting
        // TODO: figure out tick int/double issue
        this.collector.startTickCollection(tickNumber);
    }
    
    
    public void stopTick() {
        this.collector.stopTickCollection();
    }
    
	/**
	 * RV: Modify the default output directory to be a subdirectory
	 * of the default output directory, named by the basename of the
	 * demand (activity) file.
	 */
	public static void updateOutputDirectory() {
		String outDir = GlobalVariables.DEFAULT_OUTPUT_DIR;
		// get the base name of the demand file
		String[] temp1 = GlobalVariables.ACTIVITY_CSV.split("/");
        String temp2 = temp1[temp1.length - 1];
        String activityScenario = temp2.substring(0, temp2.lastIndexOf('.'));
        	if (activityScenario == null || activityScenario.length() <= 0) {
        		System.err.println("JsonOutputWriter.createDefaultPath():"
        				+ "activity file name is not valid");
        	}
        	// set this base name as a subdirectory
        	outDir = outDir + File.separatorChar + activityScenario;
        	// if the directory does not exist, create it
		try {
			Files.createDirectories(Paths.get(outDir));
		} catch (IOException e) {
			e.printStackTrace();
		}
		// set it as the default output directory
		GlobalVariables.OUTPUT_DIR = outDir;
	}
    
    /**
     * RV: Record runtime per few ticks for performance analysis (in seconds)
     * */
    public void recordRuntime() {
    	if (GlobalVariables.ENABLE_RUNTIME_RECORD) {
    		ArrayList<Double> runtimeRecorder = GlobalVariables.RUNTIME_RECORD_LIST;
    		runtimeRecorder.add((System.currentTimeMillis() - runtimeRecorder.get(0))/1000);
    		
    		// print the total no. of vehicles generated and destroyed so far,
    		// along with runtime since the last call of this function
    		System.out.println("nVehGenerated=" + GlobalVariables.NUM_GENERATED_VEHICLES
    				+ ", nVehEnteredNetwork=" + GlobalVariables.NUM_VEHICLES_ENTERED_ROAD_NETWORK
    				+ ", nVehKilled=" + GlobalVariables.NUM_KILLED_VEHICLES
    				+ ", nVehFailed=" + GlobalVariables.NUM_FAILED_VEHICLES
    				+ ", tick=" + RunEnvironment.getInstance().getCurrentSchedule().getTickCount()
    				+ ", cumRuntime=" + runtimeRecorder.get(runtimeRecorder.size() - 1));
    	} else {
    		System.out.println("nVehGenerated=" + GlobalVariables.NUM_GENERATED_VEHICLES
    				+ ", nVehEnteredNetwork=" + GlobalVariables.NUM_VEHICLES_ENTERED_ROAD_NETWORK
    				+ ", nVehKilled=" + GlobalVariables.NUM_KILLED_VEHICLES
    				+ ", nVehFailed=" + GlobalVariables.NUM_FAILED_VEHICLES
    				+ ", tick=" + RunEnvironment.getInstance().getCurrentSchedule().getTickCount());
    	}
    }
}
