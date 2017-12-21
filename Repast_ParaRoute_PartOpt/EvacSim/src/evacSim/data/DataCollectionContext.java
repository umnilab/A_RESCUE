package evacSim.data;

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
    private CsvOutputWriter outputWriter;
    
    
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
        
        // create the output file writer.  without specifying a filename,
        // this will generate a unique value including a current timestamp
        // and placing it in the current jre working directory.
        if (GlobalVariables.ENABLE_CSV_WRITE) {
            this.outputWriter = new CsvOutputWriter();
            this.collector.registerDataConsumer(this.outputWriter);
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
}
