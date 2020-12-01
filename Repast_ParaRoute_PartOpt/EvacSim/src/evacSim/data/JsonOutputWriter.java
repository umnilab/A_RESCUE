package evacSim.data;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import evacSim.GlobalVariables;
import repast.simphony.essentials.RepastEssentials;

/**
 * evacSim.data.JsonOutputWriter
 * 
 * This data consumer writes the contents of the simulation output buffer
 * to disk in the JSON format.  If no file is specified, the output will
 * be written to the same location as this program with a default name
 * of the format "EvacSim_Output_<timestamp>.json".  If this is the case,
 * a new filename with an updated timestamp will be used for each restart 
 * of the writing process.
 * 
 * The way data is stored in this file is dependent on what format is needed 
 * for web-interface/visualization. Each JSON file consists of two entries 
 * for two ticksnaphots. Each ticksnapshot starts with the tick and consists 
 * of a list of lists to stores entries for different vehicle snapshots.
 * 
 * @author Christopher Thompson (thompscs@purdue.edu) and Hemant Gehlot 
 * @version 1.0
 * @date 10 August 2017, 6 June 2019
 */
public class JsonOutputWriter implements DataConsumer {
    
    /** Whether or not existing output files should be appended. */
    private boolean append;
    
    /** Whether or not to generate a unique filename with each execution. */
    private boolean defaultFilenames;
    
    /** The file to which output data will be written. */
    private File file;
    
    /** Previous FileName. */
    private int previousFileName = 1;
    
    /** The writer which streams prepared JSON content to the output file. */
    private BufferedWriter writer;
    
    /** HG: The number of ticks written to the current JSON file. */
    private int ticksWritten;
    
    /** The number in the output series for the current JSON file.  */
    @SuppressWarnings("unused")
	private int fileSeriesNumber;
    
    /** The thread which periodically reads the buffer for data to process. */
    private Thread writingThread;
    
    /** The simulation tick currently being processed (or just processed). */
    protected double currentTick;
    
    /** Whether or not the output writer is currently consuming data. */
    private boolean consuming;
    
    /** Whether or not the output writer is paused for consuming data. */
    private boolean paused;
    
    /** HG: This HashMap stores the data of ticks for every json file. */
    private Map<String, Object> storeJsonObjects;
    
    
    /**
     * Creates the JSON output writer object using a default file name and
     * not appending the contents to 
     */
    public JsonOutputWriter() {
        this(new File(JsonOutputWriter.createDefaultFilePath()), false);
        System.out.println("Creating JSON file " + JsonOutputWriter.createDefaultFilePath());
    }
    
    
    /**
     * Creates the JSON output writer object to place output from the
     * simulation buffer into the file specified.
     * 
     * @param file the file to which output will be written.
     * @param append whether or not to append to an existing file.
     */
    public JsonOutputWriter(File file, boolean append) {
        // set whether or not we are to append to existing files
        this.append = append;
        
        // set the file to which output is written or set the flag
        // so we know to generate a unique filename with each start
        // if a filename was not explicitly given
        this.file = file;
        if (file == null) {
            this.defaultFilenames = true;
        }
        else {
            this.defaultFilenames = false;
        }
        
        // set the initial state of the writer
        this.consuming = false;
        this.paused = false;
        this.currentTick = -1.0;
        this.writer = null;
        this.writingThread = null;
        this.fileSeriesNumber = 1;
        this.ticksWritten = 0;
        this.storeJsonObjects = new HashMap<String, Object>();
        
        DataCollector.printDebug("JSON", "FILE: " + this.file.getAbsolutePath());
    }
    
    
    /**
     * Returns whether or not the writer will append to existing files.
     * 
     * @return whether or not the writer will append to existing files.
     */
    public boolean isAppending() {
        return this.append;
    }
    
    
    /**
     * Sets whether or not the writer will append to existing files.
     * 
     * @param append whether or not to append new output to existing files.
     */
    public void setAppending(boolean append) {
        this.append = append;
    }
    
    
    /**
     * Returns the file to which output is written.
     * 
     * @return the file to which output is written.
     */
    public File getOutputFile() {
        return this.file;
    }
    
    
    /**
     * Sets the file to which contents will be written.  If the writer
     * is currently writing, the file cannot be changed and this method
     * will throw an IOException.
     * 
     * @param file the file to which output will be written.
     * @throws IOException if the output file cannot be set.
     */
    public void setOutputfile(File file) throws IOException {
        if (this.consuming || this.paused) {
            throw new IOException("Cannot change file while running.");
        }

        // set the output file to the one given 
        this.file = file;
        this.fileSeriesNumber = 1;
        
        // check if we are now to use a default filename or not 
        if (this.file == null) {
            // the explicit filename was removed, so we will start generating
            // a unique filename each time the writer starts, using timestamps
            this.defaultFilenames = true;
        }
        else {
            // a specific file has been set, so we will use this each time the
            // writer starts and stops until we are told to use something else
            this.defaultFilenames = false;
        }
    }
    
    
    /**
     * Starts consuming data from the data buffer.  This opens the set
     * output file for writing and creates the thread which will monitor
     * the data buffer for new items to process and write to disk.  Calling
     * this method if the writer is already consuming data will do nothing
     * but resume the consumer if it is currently paused.  If no output
     * file has been explicitly set, a unique filename will be generated when
     * the writer starts which includes a timestamp of the time of start.
     * 
     * @throws Throwable if any problem occurred starting the data writing.
     */
    @Override
    public void startConsumer() throws Throwable {        
        if (this.consuming) {
            // the consumer is already running, so just resume if necessary
            if (this.paused) {
                this.paused = false;
            }
            return;
        }

        // set the flags to the running state
        this.consuming = true;
        this.paused = false;
        
        // create a new, unique filename if one has not been set
        if (this.defaultFilenames || this.file == null) {
            this.file = new File(JsonOutputWriter.createDefaultFilePath());
            this.previousFileName = 1;
        }
        
        // create the data writer
        this.openOutputFileWriter();
        
        // start the data consumption thread
        Runnable writingRunnable = new Runnable() {
            @Override
            public void run() {
                // get a reference to the data buffer for pulling new items
                DataCollector collector = DataCollector.getInstance();

                // loop and process data buffers until we are told to stop
                int totalCount = 0;
                int writeCount = 0;
                boolean running = true;
                while (running) {
                    // check if we are supposed to be done
                    if (!JsonOutputWriter.this.consuming) {
                        DataCollector.printDebug("JSON", "NOT CONSUMING");
                        break;
                    }
                    
                    // check if we are supposed to be paused
                    if (JsonOutputWriter.this.paused) {
                        DataCollector.printDebug("JSON", "PAUSED");
                        // we are currently paused, so we will wait our delay
                        // before performing another poll on our running state
                        try {
                        	System.out.println("Sleeping JSON consumer at t=" + RepastEssentials.GetTickCount());
                            Thread.sleep(GlobalVariables.JSON_BUFFER_REFRESH);
                            continue;
                        }
                        catch (InterruptedException ie) {
                            // we've been told to stop running
                            break;
                        }
                    }
                    
                    /* get the next item from the buffer. HGehlot: I have changed from
                     * 1 to GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ to only 
                     * send the data at the new frequency for viz interpolation
                     */
                    double nextTick = JsonOutputWriter.this.currentTick +
                    		GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ;
                    TickSnapshot snapshot = collector.getNextTick(nextTick);
                    
                    if (snapshot == null) {
                        // the buffer has no more items for us at this time
                        if (writeCount > 0) {
                            String report = "Wrote " + writeCount + 
                                " ticks to disk (" + totalCount + " total)";
                            DataCollector.printDebug("JSON", report);
                            writeCount = 0;
                        }
                        
                        // is the data collection process finished?
                        if (!collector.isCollecting() &&
                            !collector.isPaused()) {
                            // the collector is stopped so no more are coming
                            break;
                        }
                        // we will wait for our longer "the buffer is empty"
                        // delay to give it a chance to add a few new data
                        // items before we loop around and try again...
                        try {
                            Thread.sleep(GlobalVariables.JSON_BUFFER_REFRESH);
                            continue;
                        }
                        catch (InterruptedException ie) {
                            // the thread has been told to stop writing data
                            break;
                        }
                    }
                    
                    // update the currently processing tick index to this item
                    JsonOutputWriter.this.currentTick = snapshot.getTickNumber();
                    
                    // process the current item into lines in the output file
                    try {
                        JsonOutputWriter.this.writeTickSnapshot(snapshot);
                        totalCount++;
                        writeCount++;
                    }
                    catch (IOException ioe) {
                        // TODO: Handle being unable to write output lines?
                        String errMsg = "WRITE ERROR: " + ioe.getMessage();
                        DataCollector.printDebug("JSON" + errMsg);
                    }
                    
                    // wait a short delay (a few ms) to give java's thread
                    // scheduler a chance to switch contexts if necessary
                    // before we loop around and grab the next buffer item
                    try {
                        Thread.sleep(5);
                    }
                    catch (InterruptedException ie) {
                        // the thread has been told to stop wrting data
                        break;
                    }
                }
                
                // we have finished collecting data, so we will close the file
                try {
                    JsonOutputWriter.this.closeOutputFileWriter();
                }
                catch (IOException ioe) {
                    // TODO: Handle not being able to close the output file?
                }
                
                // set the data consumption flags as finished
                JsonOutputWriter.this.paused = false;
                JsonOutputWriter.this.consuming = false;
            }
        };
        this.writingThread = new Thread(writingRunnable);
        this.writingThread.start();
    }
    
    /**
     * Stops the data consumption and writing (after finishing any currently
     * processing item) and waits for the writing thread to halt.  Stopping
     * a data consumer should only be done if immediate halting of its own
     * operations is desired.  If the simulation is simply stopping, it is
     * best to allow data consumers to run to completion to make sure they
     * have finished processing any remaining items in the buffer.  This will
     * block until the writing thread has finished running and as such should
     * be called from its own thread so as to not block the entire simulation.
     * 
     * @throws Throwable if any problems occurred stopping the data writer.
     */
    @Override
    public void stopConsumer() throws Throwable {
        if (!this.consuming) {
            return;
        }
        
        // set the flags to the stopped state
        this.paused = false;
        this.consuming = false;
        
        // wait for the writer thread to halt.  setting the flags above should
        // tell the thread to stop processing new items and return on its own,
        // but we will go ahead and explicitly interrupt the thread to speed
        // this process along, as well.  then we will wait for it to complete.
        this.writingThread.interrupt();
        this.writingThread.join();
        this.writingThread = null;
        
        // reset the counter to the the initial position
        this.currentTick = -1;
        
        // dispose of anything we no longer need (like the writer)
        this.closeOutputFileWriter();
        this.writer = null;
    }
    
    
    /**
     * Signals the pausing of consumption of new items from the buffer.
     * Any item currently being written will finish, and the thread which
     * performs the actual pull of data from the buffer will check this
     * state before grabbing the next tick snapshot to know to stop.  The
     * thread is left running as this is only the paused state so it is
     * expected to resume at some point in the future.
     * 
     * @throws Throwable if any error occurs preventing the pause.
     */
    @Override
    public void pauseConsumer() throws Throwable {
        // check that we are even running
        if (!this.consuming) {
            return;
        }
        
        // set the flags to tell the consumer to stop consuming new items
        this.paused = true;
        this.consuming = true;
        
        // we do nothing to the thread or file writer directly here.  
        // the thread on its next loop will see the paused state and 
        // start checking (with a delay) for this state to change back 
        // to normal running before resuming its work.
    }
    
    
    /**
     * Stops the consumption of new items from the buffer, stops the file
     * writing after the current item has processed, and resets the file
     * writer to start again fresh in the future.  This will block and wait
     * for the writing thread to complete its current task, so it should not
     * be called from the main program thread or the whole simulation could
     * be blocked temporarily. 
     * 
     * @throws Throwable if a problem occurred restarting the file writer.
     */
    @Override
    public void resetConsumer() throws Throwable {
        // stop the file writer if it is currently operating
        this.stopConsumer();
        
        // reset the flags to the initial state
        this.paused = false;
        this.consuming = false;
        
        // reset the current tick counter back to the start
        this.currentTick = -1;
    }
    

    /**
     * Returns the current tick being processed (or just processed).
     * 
     * @return the current tick being processed (or just processed).
     */
    @Override
    public double getTick() {
        return this.currentTick;
    }
    
    
    /**
     * Sets the next tick to process from the data buffer.  The next
     * tick retrieved from the data buffer will be the first found to
     * have a value equal to or greater than this value.  If the model
     * has not yet reached this tick index value, nothing will be
     * returned until the simulation has advanced to this point.
     * 
     * @param tick the next tick to process from the data buffer.
     */
    @Override
    public void setTick(double tick) throws Throwable {
        this.currentTick = tick;        
    }
    
    
    /**
     * Creates a buffered writer for saving the JSON lines to disk and
     * opens it.  Any problem doing so will throw an IOException. 
     * 
     * @throws IOException if the file could not be opened for writing.
     */
    private void openOutputFileWriter() throws IOException {
        // check the writer doesn't already exist
        if (this.writer != null) {
            // is the writer still open?
            try {
                this.writer.flush();
                
                // there's no way to get the path to the file the current
                // writer object is using, so we have no choice but to
                // throw an error since we can't check it's the same file
                throw new Exception();
                
            }
            catch (IOException ioe) {
                // if the flush threw an exception, the writer is closed
                // and is just a stale object that wasn't destroyed.
                // we should be safe to replace it.
            	
            }
            catch (Exception e) {
                // because we're already trapping IOException from the flush,
                // we had to do the awkward step of throwing something else
                // to get out of the try so we can throw a new IOException...
                throw new IOException("JSON writer already has a file open.");
            }
        }
        
        // check the output file has been given
        if (this.file == null) {
            throw new IOException("No output file has been specified.");
        }
        
        // create the buffered writer for the file
        FileWriter fw = new FileWriter(this.file);
        this.writer = new BufferedWriter(fw);
    }
    
    
    /**
     * Closes the buffered writer for the output file.
     * 
     * @throws IOException if the output file could not be closed.
     */
    private void closeOutputFileWriter() throws IOException {
        // check the writer object exists
        if (this.writer == null) {
            return;
        }
        
        // close the file writer
        this.writer.close();
        this.writer = null;
        
        // if this was a default filename, it is intended for a single
        // use and should be thrown away once it is complete so we cannot
        // accidentally write to it again
        if (this.defaultFilenames) {
            this.file = null;
        }
    }
    
    
    /**
     * Closes the current output file and opens the next in the series of 
     * output files for this simulation execution.  The filename will be the 
     * same with the increment of the series counter.
     * 6/28/2020 LZ: modify this to use time tick information
     * 
     * @throws IOException if the new file could not be created and opened.
     */
    
    private void startNextOutputFile(int current_time) throws IOException {
        if (this.file == null ) {
            // there is no file currently open to increment!
            // TODO: figure out how to deal with this situation...
            return;
        }
        
        // determine the current filename being written
        String filename = this.file.getName();
        if (filename == null || filename.trim().length() < 1) {
            // the filename is null (is this even possible?) or is just
            // whitespace with no valid characters!
            return;
        }
        
        // if we are using default filenames, we know for certain the format
        // of the series.  it should end ".1.json", ".2.json", etc.  we can
        // easily create the next in the series this way.  if no t, we will
        // have to do a little extra checking to setup the next file.
//        String currentEnd = "." + this.fileSeriesNumber + "." + 
//                            GlobalVariables.JSON_DEFAULT_EXTENSION;
        String currentEnd = "." + this.previousFileName + ".json";
        String nextEnd = "." + (current_time/
        		GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ/2) + ".json";
        
        String newFilename = filename;
        
        newFilename = newFilename.replaceAll(currentEnd + "$", nextEnd);
        
//        if (newFilename.endsWith(currentEnd)) {
//            // the user is using a standard format filename so easy to update
//            newFilename = newFilename.replaceAll(currentEnd + "$", nextEnd);
//        }
//        else {
//            // the user is using a custom filename format so we need to
//            // do a little extra work to create the next filename...
//            String extEnd = "." + GlobalVariables.JSON_DEFAULT_EXTENSION;
//            if (newFilename.endsWith(extEnd)) {
//                newFilename = newFilename.replaceAll(extEnd + "$", nextEnd);
//            }
//            else {
//                // TODO: continue checking for other filename variants?
//            }
//        }
        
        // create the next filename in this output file series
        File nextFile = new File(this.file.getParent(), newFilename);
        
        
        // close out the current output file
        this.closeOutputFileWriter();
        
        // open the new output file for writing
        this.file = nextFile;
        FileWriter fw = new FileWriter(this.file);
        this.writer = new BufferedWriter(fw);
        
        // finally, having successfully moved to the next file, update counter
        this.fileSeriesNumber++;
        this.previousFileName = (current_time/GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ/2);
    }
    
    
    /**
     * HG: Writes the given tick snapshot to the output file.  The buffered writer is flushed at the
     * end so any cached content is immediately written out to disk.
     * 
     * @param tick the tick snapshot to be written to the output file.
     * @throws IOException if any error occurred writing the lines to disk.
     */
    @SuppressWarnings("unchecked")
	private void writeTickSnapshot(TickSnapshot tick) throws IOException {
        if (tick == null) {
            return;
        }

        // check the file has been opened
        if (this.writer == null) {
        		throw new IOException("The JSON file is not open for writing.");
        }
        
        // check if writing these lines will go over our output file limit
        // and create the next output file in the series if necessary
        if (this.ticksWritten >= GlobalVariables.JSON_TICK_LIMIT_PER_FILE) {
            this.startNextOutputFile((int) tick.getTickNumber());
            this.ticksWritten = 0;
            this.storeJsonObjects = new HashMap<String, Object>();
        }
        
        /*
         * RV: Change the structure of this hashmap to include the status of roads and shelters.
         * Previously, it was of the format:
         * { "tick 1": [[vehicle attribute1, attr2, ...], ...], "tick 2": ... }
         * Now, changing to the new format:
         * { "tick1": { "vehicles": [[veh attr1, attr2, ...], ...],
         *              "roads": [[road id, speed, nVehicles], ...],
         *              "shelters": [[shelt id, available spaces], ...] },
         *   "tick2": ...
         * }
         */

        // get the JSON representation of this tick
        String tickString = String.valueOf(tick.getTickNumber());
        
        HashMap<String, ArrayList<String>> tickData = new
        		HashMap<String, ArrayList<String>>();

        // get the 2D arrays containing data of vehicles, roads, and shelters
        ArrayList<String> vehTickArray = tick.createJSONTickLines("vehicle");
        ArrayList<String> roadTickArray = tick.createJSONTickLines("road");
        ArrayList<String> sheltTickArray = tick.createJSONTickLines("shelter");
        
        if (vehTickArray == null) {
            return; // there was no JSON output created by this tick
        }
        	
        // add the data
        tickData.put("vehicles", vehTickArray);
        tickData.put("shelters", sheltTickArray);
        tickData.put("roads", roadTickArray);
        
        this.storeJsonObjects.put(tickString, tickData);
        
        //write the the ticks to json file if only one more tick information needs to be added to the current file
        if (this.ticksWritten == GlobalVariables.JSON_TICK_LIMIT_PER_FILE - 1) {
        		JSONObject jsonObject = new JSONObject();
        		jsonObject.putAll(this.storeJsonObjects);
        		this.writer.write(JSONObject.toJSONString(jsonObject));
        }
        this.ticksWritten += 1;
        		
        // flush all of our changes now so nothing waits cached in memory
        this.writer.flush();
    }
    
    
//    /**
//     * HG: Returns the given tick snapshot as an array of arrays.
//     * 
//     * @param tick: the snapshot of the tick to convert.
//     * @param object: the type of simulation object (vehicle/road/shelter) that
//     * @return the array of array for the given tick snapshot.
//     */
//    public static ArrayList<ArrayList<String>> createTickLines(TickSnapshot tick) {
//        // check the tick snapshot exists
//        if (tick == null) {
//            return null;
//        }
//
//        // output array
//        ArrayList<ArrayList<String>> tickArray = new ArrayList<ArrayList<String>>();
//        
//        // get the list of of vehicles stored in the tick snapshot
//        Collection<Integer> vehicleIDs = tick.getVehicleList();
//        if (vehicleIDs == null || vehicleIDs.isEmpty()) {
//            return null;
//        }
//        
//        // loop through the list of vehicles and convert each to a arraylist
//        for (Integer id : vehicleIDs) {
//            if (id == null) {
//                continue;
//            }
//            
//            // retrieve the vehicle snapshot from the tick snapshot
//            VehicleSnapshot vehicle = tick.getVehicleSnapshot(id);
//            if (vehicle == null) {
//                continue;
//            }
//            
//            // get the arraylist representation of this vehicle
//            ArrayList<String> vehicleArray = JsonOutputWriter.createVehicleLine(vehicle);
//            if (vehicleArray == null) {
//                continue;
//            }
//            
//            //add the vehicle array to the tick arraylist
//            tickArray.add(vehicleArray);
//            
//        }
//        
//        return tickArray;
//        
//    }
//    
//    
//    /**
//     * HG: Returns the arraylist representation of the given vehicle snapshot. 
//     * 
//     * @param vehicle the vehicle snapshot to convert to arraylist.
//     * @return the arraylist representation of the given vehicle snapshot.
//     */
//    public static ArrayList<String> createVehicleLine(VehicleSnapshot vehicle) {
//        if (vehicle == null) {
//            return null;
//        }
//        
//        // extract the values from the vehicle snapshot
//        ArrayList<String> vehicleArray = new ArrayList<String>();
//        vehicleArray.add(Integer.toString(vehicle.getId()));
//        vehicleArray.add(vehicle.getPrevXString());
//        vehicleArray.add(vehicle.getPrevYString());
//        vehicleArray.add(vehicle.getXString());
//        vehicleArray.add(vehicle.getYString());
//        vehicleArray.add(vehicle.getSpeedString());
//        
////        vehicleArray.add(vehicle.getOriginXString());
////        vehicleArray.add(vehicle.getOriginYString());
////        vehicleArray.add(vehicle.getDestXString());
////        vehicleArray.add(vehicle.getDestYString());
////        vehicleArray.add(Integer.toString(vehicle.getNearlyArrived()));
////        vehicleArray.add(Integer.toString(vehicle.getvehicleClass()));
////        vehicleArray.add(Integer.toString(vehicle.getRoadID()));
//        //double z = vehicle.getZ();
//   
//        //int departure = vehicle.getDeparture();
//        //int arrival = vehicle.getArrival();
//        //float distance = vehicle.getDistance();
//
//
//        return vehicleArray;
//        // build the json line and return it
//        //return (id + "," + x + "," + y + "," + OriginalX + "," + OriginalY + "," + DestX + "," + DestY + "," + roadID + ","
//        //+ speed + "," +departure + "," + arrival + "," + distance + "," + nearlyArrived + "," + vehicleClass + "," + prev_x + "," + prev_y);
////        return (id + "," + prev_x + "," + prev_y + "," + x + "," + y + "," + speed + "," +
////        		originalX + "," + originalY + "," + destX + "," + destY + "," +
////                nearlyArrived + "," + vehicleClass + "," + roadID);
//        //departure + "," +
//        //arrival + "," +
//        //distance + "," +
//    }
//    
    
    /**
     * Returns a guaranteed unique absolute path for writing output
     * from the simulation within the current working directory and
     * using the filename format of "EvacSim_Output_<timestamp>.json"
     * so long as this method is not called more frequently than once
     * per second.  If there is a problem writing to this location,
     * the file will be saved to a temporary directory determined by
     * the Java library determined by the operating system upon which
     * this program is executing.
     * 
     * @return a guaranteed unique absolute path for writing output.
     */
    public static String createDefaultFilePath() {
        // get the default pieces of the filename to assemble
        String defaultFilename = GlobalVariables.DEFAULT_OUT_FNAME;
        
        // get a timestamp to use in the filename
        SimpleDateFormat formatter = 
                new SimpleDateFormat("YYYY-MM-dd-hh-mm-ss");
        String timestamp = formatter.format(new Date());
        
        // RV: use the basename of the demand file in the output file
        String[] temp1 = GlobalVariables.ACTIVITY_CSV.split("/");
        String temp2 = temp1[temp1.length - 1];
        String activityScenario = temp2.substring(0, temp2.lastIndexOf('.'));
        	if (activityScenario == null || activityScenario.length() <= 0) {
        		System.err.println("JsonOutputWriter.createDefaultPath():"
        				+ "activity file name is not valid");
        	}
        
        // build the filename
        String filename = defaultFilename + "-" + activityScenario + "-" +
        		timestamp + ".1.json";
        
        // get the default directory for placing the file
        String defaultDir = GlobalVariables.DEFAULT_OUT_PATH;
        if (defaultDir == null || defaultDir.trim().length() < 1) {
        		defaultDir = System.getProperty("user.dir");
        }
                
        /* RV: provide the output directory - either the default one or
         * in case of running multiple scenarios, organize all files into
         * a folder for each scenario based on its demand filename
         */
        String outDir;
        if (GlobalVariables.ORGANIZE_OUTPUT_BY_ACTIVITY_FNAME) {
    		outDir = defaultDir + File.separatorChar + activityScenario;
    		
    		// if the directory does not exist, create it
    		try {
    			Files.createDirectories(Paths.get(outDir));
    		} catch (IOException e) {
    			e.printStackTrace();
    		}
        } else {
        		outDir = defaultDir;
        }
        
        // build the full path to the file
        String outpath = outDir + File.separatorChar + filename;
        
        
        // check the path will be a valid file
        File outfile = new File(outpath);
        if (outfile.exists()) {
            // a file with this name somehow already exists even though
            // we've given it a timestamp of this very second at runtime.
            // we will add the hashcode for the filename string object as
            // a bit of randomization and just hope that is good enough.
            int hashCode = System.identityHashCode(filename);
            filename = defaultFilename + "_" + timestamp + "_" +
                       hashCode + ".1.json";
            outpath = defaultDir + File.pathSeparator + filename;
            outfile = new File(outpath);
        }

        try {
            outfile.createNewFile();
            if (!outfile.canWrite()) {
                throw new IOException("Can't write to file.");
            }
            outfile.delete();
        }
        catch (IOException ioe) {
            // we don't have permissions to write to the current directory
            // so we will have to fall back on saving this in the temp dir
            try {
                outfile = 
                    File.createTempFile(filename, "json");
            }
            catch (IOException ioe2) {
                // our default filename failed, and now our temp file failed.
                // for this to happen, something has to be wrong with the OS
                // or the storage medium has to be full.  we give up...
                return null;
            }
        }
        
        // return the path to whatever we decided our file would be
        return outfile.getAbsolutePath();
    }
    
}
