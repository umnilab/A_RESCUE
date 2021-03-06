package evacSim.network;


import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.NetworkEventObject;
import evacSim.data.DataCollector;
import evacSim.data.DataConsumer;
import evacSim.data.TickSnapshot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ISchedule;
import repast.simphony.essentials.RepastEssentials;

import org.apache.log4j.Logger;


/**
 * evacSim.network.Connection
 * 
 * When a request from a remote program for a network connection is received,
 * the network connection manager will produce a new WebSocket connection to
 * be managed by a new instance of this class.  This class provides both the
 * WebSocket annotated methods the Jetty library will use to handle processing
 * control of this particular socket and the delivery of messages received
 * from the remote program and the interface methods of a data consumer which
 * uses the data collection buffer built into the simulation.  A thread 
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 17 October 2017
 */
public class Connection implements DataConsumer {
    
    /** The number of instances of the connection class created so far. */
    private static int COUNTER = 0;
    
    /** An id number for referring to this particular connection in logs. */
    private int id;

    /** A local convenience reference to manager for this connection. */
    private ConnectionManager manager = ConnectionManager.getInstance();
    
    /** A local convenience reference to the collection buffer for the sim. */
    private DataCollector collector = DataCollector.getInstance();
    
    /**Prefix for event */
    private static final String EVENT_MSG = "EVENT";
    
    /**Prefix for event */
    private static final String START_MSG = "START";
    
    
    /** The address of the host to which the connection is made. */
    private String host;
    
    /** The port at which the remote host is accepting connections. */
    private int port;
    
    /** The socket which is used to connect to the remote host. */
    private MessageSocket socket;
    
    /** The unique id value assigned to this copy of the model. */
    private String modelId;
    
    
    /** A thread which periodically sends buffer contents over the socket. */
    private Thread sendingThread;
    
    /** A "heartbeat" thread which pings remote end often to keep it alive. */
    private Thread heartbeatThread;
    
    /** A thread which monitors the underlying socket for new incoming data. */
    private Thread receivingThread;
    
    
    /** Whether or not the connection is currently consuming buffer data. */
    private boolean consuming;
    
    /** Whether or not the connection is paused for consuming more data. */
    private boolean paused;
    
    /** The current tick number being processed (or just processed). */
    private double currentTick;
    
    
    /** The console logging system used across the whole simulation. */
    private Logger logger;
    
    
    /**
     * Performs any preparation needed for this object to be ready to
     * receive a new socket connection and begin processing the data in
     * the simulation data buffer.
     * 
     * @param host the address of the remote host.
     * @param port the port at which the remote host accepts connections.
     */
    public Connection(String host, int port) {
    	// Get a reference to the system-wide logging system
    	this.logger = ContextCreator.logger;
    	
    	// check the given host and port are valid
    	if (host == null || host.trim().length() < 1) {
    	   throw new IllegalArgumentException("Host address not specified.");
    	}
    	if (port < 1 || port > 65535) {
    		throw new IllegalArgumentException("Port number not valid.");
    	}
    	this.host = host;
    	this.port = port;
    	
        // increment our instance counter and save this object's number
        this.id = ++Connection.COUNTER;
        
        // set the current tick to zero.
        this.currentTick = -1.0;
        
        // set the flags to the pre-running state
        this.consuming = false;
        this.paused = false;
        
        // the heartbeat thread will be created and destroyed by
        // the annotated socket open and close methods so that it
        // runs not when this object is created or destroyed but
        // when the actual socket is created or destroyed
        this.heartbeatThread = null;
        
        // the data sending thread will be created each time the
        // data consumption start signal is sent and stopped when
        // the data consumption stop signal is sent.  the heartbeat
        // thread operates independently of this and remains running
        // whether or not the data consumption is active so long as
        // the socket is open.
        this.sendingThread = null;
        
        // the control program which created this copy of the model should
        // have assigned a unique id value for this particular copy.  this
        // value is transmitted as part of the connection establishing
        // process so the remote control program can properly route the
        // traffic of this connection to the correct location within itself.
        this.modelId = this.loadModelId();
        
        ConnectionManager.printDebug(this.id + "-CTRL", 
                                     "Connection object created.");
    }
    
    
    /**
     * Attempts to open the connect to the remote host and starts the
     * message processing loop which will handle all incoming data from
     * the remote host.
     * 
     * @throws IOException if the connection could not be established.
     */
    public void open() throws IOException {
    	// check the socket does not already exist and is not already open
    	if (this.socket != null) {
        	if (this.socket.isOpen()) {
    			// this connection already exists and is still open, so
        		// we will do nothing and just return here.
        		return;
    		}
        	else {
        		// the connection already exists but is not open.  as the
        		// connection is established when the socket object is
        		// created, this must mean it has died.  we will throw out
        		// this socket object and create a fresh connection.
        		this.socket = null;
        	}
    	}
    	
    	// TODO: perform any pre-opening tasks
    	
    	// attempt to open the connection 
    	try {
    	    this.socket = new MessageSocket(this.host, this.port);
    	}
    	catch (Throwable t) {
    		// a problem occurred attempting to establish the socket
    		throw new IOException("Could not open socket to remote host.", t);
    	}
    	
    	// perform any post-opening tasks
    	this.onConnect();
    }
    
    
    /**
     * Closes the connection and triggers any tasks that need to be handled
     * for shutting down the connection.
     * 
     * @throws IOException if an error occurred closing the connection.
     */
    public void close() throws IOException {
    	// TODO: perform any pre-closing tasks
    	
    	// attempt to close the socket
    	if (this.socket != null) {
    		this.socket.disconnect();
    	}
    	this.socket = null;
    	
    	// TOOD: perform any post-closing clean-up tasks 
    }
    
    
    /**
     * Starts consuming data from the data buffer.  This starts the thread
     * for sending data across the socket.  Calling this method if the
     * consumer is paused will do nothing but resume it.  If the signal
     * to start consuming arrives before the socket is opened yet, the
     * sending thread will wait for the socket to be open.
     * 
     * @throws Throwable if any problem occurred starting the data sending.
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
        
        // start the data consumption thread
        this.sendingThread = new Thread(new SendingRunnable());
        this.sendingThread.start();
    }

    
    /**
     * Stops the data consumption and sending (after finishing any currently
     * processing item) and waits for the sending thread to halt.  Stopping
     * a data consumer should only be done if immediately halting of its own
     * operations is desired.  If the simulation is simply stopping, it is
     * best to allow data consumers to run to completion to make sure they
     * have finished processing any remaining items in the buffer.  This will
     * block until the sending thread has finished running and as such should
     * be called from its own thread so as to not block the entire simulation.
     * 
     * @throws Throwable if any problems occurred stopping the data sender.
     */
    @Override
    public void stopConsumer() throws Throwable {
        if (!this.consuming) {
            return;
        }
        
        // set the flags to the stopped state
        this.paused = false;
        this.consuming = false;
        
        // wait for the sending thread to halt.  setting the flags above
        // should tell the thread to stop processing new items and return
        // on its own, but we will go ahead and explicitly interrupt the
        // thread to speed this process along, as well.  then we will wait
        // for it to complete.
        this.sendingThread.interrupt();
        this.sendingThread.join();
        this.sendingThread = null;
        
        // reset the tick counter to the initial position
        this.currentTick = -1;
    }

    
    /**
     * Signals the pausing of consumption of new items from the buffer.
     * Any items currently being sent will finish, and the thread which
     * performs the actual pull of data from the buffer will check this
     * state before grabbing the next tick snapshot to know to stop.
     * The thread is left running as this is only the paused state so
     * it is expected to resume at some point in the future.
     * 
     * @throws Throwable if any error occurs preventing the pause.
     */
    @Override
    public void pauseConsumer() throws Throwable {
        // check we are even running
        if (!this.consuming) {
            return;
        }
        
        // set the flags to tell the consumer to stop consuming new items
        this.paused = true;
        this.consuming = true;
        
        // we do nothing to the thread or connection directly here.
        // the thread on its next loop will see the paused state and
        // start checking (with a delay) for this state to change back
        // to normal running before resuming its work.
    }

    
    /**
     * Stops the consumption of new items from the buffer, stops the data
     * sending after the current item has been processed, and resets the
     * buffer position to start again fresh at the front of the buffer if
     * the sending of data to this connection is reestablished.  This will
     * block and wait for the sending thread to complete its current task,
     * so it should not be called from the main program thread or the whole
     * simulation could be blocked temporarily.
     * 
     * @throws Throwable if a problem occurred resetting the consumer.
     */
    @Override
    public void resetConsumer() throws Throwable {
        // stop the current network sending if it is running already
        this.stopConsumer();
        
        // reset the flags to the initial state
        this.paused = false;
        this.consuming = false;
        
        // reset the current tick counter back to the start
        this.currentTick = -1.0;
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
     * This method is invoked when the WebSocket closes and should handle
     * any necessary tasks for gracefully stopping connection related tasks
     * or attempt to restart the connection.
     * 
     * @param statusCode the status code for why the connection closed.
     * @param reason a text description of why the connection closed.
     */
    public void onClose(int statusCode, String reason) {
        // TODO: handle any tasks needed when socket closes.
        ConnectionManager.printDebug(this.id + "-CTRL", 
                                     statusCode + ": " + reason);
    }
    
    
    /**
     * This method is called whenever the connection experiences an error
     * and should handle any problems that would be consequences of the
     * exception.
     * 
     * @param t the error or exception thrown because of the problem.
     */
    public void onError(Throwable t) {
        // TODO: gracefully handle socket errors 
        ConnectionManager.printDebug(this.id + "-ERR", 
                                     t.getMessage());
    }
    
    
    /**
     * This method is called when the socket connection is established
     * to handle any tasks needing to happen when the connection is started.
     * 
     * @param session the WebSocket session object for this connection.
     */
    public void onConnect() {

        if (this.socket == null) {
            // is there any reason this would be called if socket was null?
            return;
        }
        
        ConnectionManager.printDebug(this.id + "-CTRL", 
                "Connected to " + this.host + ".");
        
        // register the connection as a data consumer
        this.collector.registerDataConsumer(this);
        
        // create the heartbeat thread to keep the connection open
        this.heartbeatThread = new Thread(new HeartbeatRunnable());
        this.heartbeatThread.start();
        
        // create the thread for receiving and processing incoming data
        this.receivingThread = new Thread(new ReceivingRunnable());
        this.receivingThread.start();
        
        // identify this simulation by sending the unique model id
        try {
            this.sendModelId();
        }
        catch (IOException ioe) {
        	// TODO: handle being unable to send the model id value
        }
    }
    
    
    /**
     * This method is called whenever the remote hosts sends a message
     * through the socket to the simulation.  This should be a command
     * from the remote viewer to alter the simulation in some fashion or 
     * perhaps a request for a more detailed or specific piece of data from
     * the simulation.  This method should perform any minimal processing
     * required to decide the intended recipient of this message within the
     * simulation program and then hand the message over to that component. 
     * 
     * @param message the contents of the message from the remote host.
     */
    public void onMessage(String message) {
        ConnectionManager.printDebug(this.id + "-RECV", message);
        
        if(message.startsWith(START_MSG)){
        	try{//HG and XQ: If you receive any message from visualization in starting, then change the sleep variable to 1.
        		GlobalVariables.SIMULATION_SLEEPS = 1;
        	}catch (NumberFormatException nfe) {
                // one of the values is malformed during parsing
                logger.error("ERROR IN RECEIVED NETWORK DATA", nfe);
            }
            catch (Throwable t) {
                // something went wrong creating the snapshot object
            	logger.error("ERROR IN RECEIVED NETWORK DATA", t);
            }
        }
        else if (message.startsWith(EVENT_MSG)){
        	try{
        		NetworkEventObject event = ParseString(message);
        		if(event.eventID == 2){//HG: check if the event type is external blocking of road then insert it in global event queue. 1 = Predefined blockage of road, 2 = External blocking of road
        			insertExternalEvent(event);
        			//HG: store event adding information in data buffer 
    				try {
    						DataCollector.getInstance().recordEventSnapshot(event, 3);//Here 3 value denotes adding of external event to global queue
    				}
    				catch (Throwable t) {
    				    // could not log the event ending in data buffer!
    				    DataCollector.printDebug("ERR" + t.getMessage());
    				}
        		}
        	}catch (NumberFormatException nfe) {
                // one of the values is malformed during parsing
                logger.error("ERROR IN PARSING EVENT NETWORK MESSAGE", nfe);
            }
            catch (Throwable t) {
                // something went wrong creating the snapshot object
            	logger.error("ERROR CREATING EVENT SNAPSHOT FOR NETWORK", t);
            }
            
        }
        else{
        	logger.error("Unknown Information");
        }

    }
    
    
    /**
     * Sends the unique model id generated for this copy of the simulation
     * to the remote listener.
     * 
     * @throws IOException if a problem occured sending the message
     */
    private void sendModelId() throws IOException {
    	// create the message for communicating the model id
    	String[] msg = { "MODEL_ID", this.modelId };
    	
    	// send it over the socket
    	if (this.socket != null) {
    	    this.socket.send(msg);
    	}
    }
    
    
    /**
     * Sends the given tick snapshot over the connection as a message after
     * converting it into a String suitable for the body of a socket message. 
     * 
     * @param tick the tick snapshot to be written to the output file.
     * @throws IOException if any error occurred sending the tick.
     */
    private void sendTickSnapshot(TickSnapshot tick) throws IOException {
        if (tick == null) {
            return;
        }
        
        // get the message representation of this tick
        String message = Connection.createTickMessage(tick);
        if (message.trim().length() < 1) {
            // there was no data in the tick to send
            return;
        }
        
        // check the connection is open and ready for sending
        if (this.socket == null || !this.socket.isOpen()) {
            throw new IOException("Socket is not open.");
        }
        
        // send the message over the socket
        this.socket.send(new String[] { message });
    }
    
    
    /**
     * Returns the socket message representation of the given tick.
     * 
     * @param tick the snapshot of a tick to convert to a socket message.
     * @return the socket message representation of the given tick.
     */
    public static String createTickMessage(TickSnapshot tick) {
        // check if the tick snapshot even exists
        if (tick == null) {
            return null;
        }
        double tickNum = tick.getTickNumber();
                
        // open the socket tick message by saying which tick this is
        ArrayList<String> lines = new ArrayList<String>();
        lines.add("TICK," + tickNum);

        // get the data representation of the tick
        ArrayList<String> tickData = tick.createCSVTickLines();
        for (String dataLine : tickData) {
        	lines.add(dataLine);
        }

        // HG: loop through the list of starting events and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(0)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_START," + line;
            lines.add(line);
        }
        
        // HG: loop through the list of ending events and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(1)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_END," + line;
            lines.add(line);
        }
        
     // HG: loop through the list of external event feedbacks being added and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(2)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_FROM_REMOTE_ADDED," + line;
            lines.add(line);
        }
        
        // join the array of lines to one socket message to return
        return String.join("\n", lines);
    }

    /**
    /**
     * Returns the socket message representation of the given tick.
     * 
     * @param tick the snapshot of a tick to convert to a socket message.
     * @return the socket message representation of the given tick.
     * /
    public static String createTickMessage(TickSnapshot tick) {
        // check if the tick snapshot even exists
        if (tick == null) {
            return null;
        }
        double tickNum = tick.getTickNumber();
        
        // get the list of vehicles stored in the tick snapshot
        Collection<Integer> vehicleIDs = tick.getVehicleList();
        if (vehicleIDs == null || vehicleIDs.isEmpty()) {
            return null;
        }
        
        // open the socket tick message by saying which tick this is
        ArrayList<String> lines = new ArrayList<String>();
        lines.add("TICK," + tickNum);
        
        // loop through the list of vehicles and convert each to a string
        for (Integer id : vehicleIDs) {
            if (id == null) {
                continue;
            }
            
            // retrieve the vehicle snapshot from the tick snapshot
            VehicleSnapshot vehicle = tick.getVehicleSnapshot(id);
            if (vehicle == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createVehicleMessage(vehicle);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "V," + line;
            lines.add(line);
        }

        // HG: loop through the list of starting events and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(0)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_START," + line;
            lines.add(line);
        }
        
     // HG: loop through the list of ending events and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(1)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_END," + line;
            lines.add(line);
        }
        
     // HG: loop through the list of external event feedbacks being added and convert each to a string
        for (NetworkEventObject event : tick.getEventList().get(2)) {
            if (event == null) {
                continue;
            }
            
            // get the string representation of the vehicle
            String line = Connection.createEventMessage(event);
            if (line == null) {
                continue;
            }
            
            // prepend the tick number, data type token, and add to array
            line = "EVENT_FROM_REMOTE_ADDED," + line;
            lines.add(line);
        }
        
        // join the array of lines to one socket message to return
        return String.join("\n", lines);
    }
    
    
    /**
     * Returns the socket message representation of the given vehicle.
     * 
     * @param vehicle the snapshot of a vehicle to convert into a message.
     * @return the socket message representation of the given vehicle.
     * /
    public static String createVehicleMessage(VehicleSnapshot vehicle) {
        // check if the vehicle snapshot even exists
        if (vehicle == null) {
            return null;
        }
        
        // extract all the values from the vehicle snapshot
        /*int id = vehicle.getId();
        double prev_x = vehicle.getPrevX();
        double prev_y = vehicle.getPrevY();
        double x = vehicle.getX();
        double y = vehicle.getY();
        float speed = vehicle.getSpeed();
        double originalX = vehicle.getOriginX();
        double originalY = vehicle.getOriginY();
        double destX = vehicle.getDestX();
        double destY = vehicle.getDestY();
        int nearlyArrived = vehicle.getNearlyArrived();
        int vehicleClass = vehicle.getvehicleClass();
        int roadID = vehicle.getRoadID();* /    
        int id = vehicle.id;
        double prev_x = vehicle.getPrevX();
        double prev_y = vehicle.getPrevY();
        double x = vehicle.getX();
        double y = vehicle.getY();
        float speed = vehicle.getSpeed();
        double originalX = vehicle.getOriginX();
        double originalY = vehicle.getOriginY();
        double destX = vehicle.getDestX();
        double destY = vehicle.getDestY();
        int nearlyArrived = vehicle.getNearlyArrived();
        int vehicleClass = vehicle.getvehicleClass();
        int roadID = vehicle.getRoadID();
        //int departure = vehicle.getDeparture();
        //int arrival = vehicle.getArrival();
        //float distance = vehicle.getDistance();

        // put them together into a string for the socket and return it
        return id + "," +
        	   prev_x + "," +
        	   prev_y + "," +
               x + "," +
               y + "," +
               speed + "," +
               originalX + "," +
               originalY + "," +
               destX + "," +
               destY + "," +
               nearlyArrived + "," +
               vehicleClass + "," +      
               roadID ;
               //departure + "," +
               //arrival + "," +
               //distance + "," +
    } */
    
    
    /**
     * Returns the socket message representation of the given vehicle.
     * 
     * @param vehicle the snapshot of a vehicle to convert into a message.
     * @return the socket message representation of the given vehicle.
     */
    public static String createEventMessage(NetworkEventObject event) {
        // check if the event even exists
        if (event == null) {
            return null;
        }
        
        // extract all the values from the event 
        int startTime = event.startTime;
		int endTime = event.endTime;
		int eventID = event.eventID;
		int roadID = event.roadID;
 
        // put them together into a string for the socket and return it
        return startTime + "," +
        	endTime + "," +
        	eventID + "," +
        	roadID ;
    }
    
    
    /**
     * Returns the unique id value created for this copy of the model.  This
     * value has been generated external to this program and inserted into
     * the appropriate place in the model's data files so that it can be used
     * at runtime to help the control program managing this model identify
     * among all the connections it is handling which copy of the model is
     * at the other end of each of these sockets.
     * 
     * @return the unique id value created for this copy of the model.
     */
    public String loadModelId() {
    	String unknownId = "UNKNOWN";
    	
    	// get the path to the id file
	    String modelIdPath = GlobalVariables.MODEL_ID_FILE;
	    if (modelIdPath == null || modelIdPath.trim().length() < 1) {
	    	// the config file does not contain the location of id file
	    	return unknownId;
	    }
    	
    	// load the contents of the id file
	    File modelIdFile = new File(modelIdPath);
	    if (!modelIdFile.isFile()) {
	    	// the model id file does not actually exist where expected
	    	return unknownId;
	    }
	    
	    try {
	        FileReader fr = new FileReader(modelIdFile);
	        BufferedReader br = new BufferedReader(fr);
	        String id = br.readLine();
	        br.close();
	        
	        if (id == null || id.trim().length() < 1) {
	        	// the file had no contents to read
	        	return unknownId;
	        }
	        
	        return id;
	    }
	    catch (Throwable t) {
	    	// we were given a path to a valid file, but something went
	    	// wrong trying to read the first line to get the id value
	    	return unknownId;
	    }    	
    }
    
      
    /**
     * This is the control portion of the body of the thread which runs
     * periodically to pull items from the data collection buffer and
     * send them over the socket to the remote program.
     */
    private class SendingRunnable implements Runnable {
        
        public SendingRunnable() {
            ConnectionManager.printDebug(id + "-SEND", 
                                         "Created stream thread.");
        }
        
        @Override
        public void run() {        	
            // wait for session to exist if it hasn't been created yet
            while (socket == null) {
                try {
                    Thread.sleep(GlobalVariables.NETWORK_BUFFER_RERESH);
                }
                catch (InterruptedException ie) {
                    ConnectionManager.printDebug(id + "-SEND",
                                                 "Send thread stop.");
                    return;
                }
            }
            
            // loop and process buffered data until we are told to stop
            int totalCount = 0;
            int sendCount = 0;
            boolean running = true;
            while (running) {
                // check if we are supposed to be done
                if (!Connection.this.consuming) {
                    ConnectionManager.printDebug(id + "-SEND",
                                                 "Not consuming.");
                    break;
                }
                
                // check if we are supposed to be paused
                if (Connection.this.paused) {
                    ConnectionManager.printDebug(id + "-SEND", 
                                                 "Paused.");
                    // we are currently paused, so we will wait our delay
                    // before performing another poll on our running state
                    try {
                        Thread.sleep(GlobalVariables.NETWORK_BUFFER_RERESH);
                    }
                    catch (InterruptedException ie) {
                        break;
                    }
                }
                
                // get the next item from the buffer. HGehlot: I have changed from 1 to GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ to only send the data at the new frequency for viz interpolation
                double nextTick = Connection.this.currentTick + GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ;
                TickSnapshot snapshot = collector.getNextTick(nextTick);
                if (snapshot == null) {
                    // the buffer has no more items for us at this time
                    if (sendCount > 0) {
                        String report = "Sent " + sendCount +
                           " ticks to remote host (" + totalCount + " total)";
                        ConnectionManager.printDebug(id + "-SEND", report);
                        sendCount = 0;
                    }
                    
                    // is the data collection process finished?
                    if (!collector.isCollecting() && !collector.isPaused()) {
                        // the collector is stopped so no more are coming
                        break;
                    }
                    
                    // we will wait a little while before looping around
                    // to check if anything new has arrived in the buffer
                    try {
                        Thread.sleep(GlobalVariables.NETWORK_BUFFER_RERESH);
                    }
                    catch (InterruptedException ie) {
                        // while waiting, the thread was told to stop
                        ConnectionManager.printDebug(id + "-SEND", "Stopped.");
                        break;
                    }
                    
                    // loop around to try again
                    continue;
                }
                
                // update the currently processing tick index to this item
                Connection.this.currentTick = snapshot.getTickNumber();
                
                // process the current item into a socket message and send it
                try {
                    Connection.this.sendTickSnapshot(snapshot);
                    totalCount++;
                    sendCount++;
                }
                catch (Throwable t) {
                    // TODO: Handle being unable to send tick message?
                    ConnectionManager.printDebug(id + "-SEND", 
                                                 t.getMessage());
                }
                
                // wait a short delay (a few ms) to give java's thread
                // scheduler a chance to switch contexts if necessary
                // before we loop around and grab the next buffer item
                try {
                    Thread.sleep(5);
                }
                catch (InterruptedException ie) {
                    // thread has been told to stop writing network data
                    ConnectionManager.printDebug(id + "-SEND", "Stopping.");
                    break;
                }
            }
            
            ConnectionManager.printDebug(id + "-SEND", "SEND THREAD EXITING");
            
            // TODO: handle any post-consuming cleanup tasks for the thread
            
            // set the data consumption flags as finished
            Connection.this.paused = false;
            Connection.this.consuming = false;
            
            // halt the status and receiving threads
            // TODO: find a better place to perform this step?
            Thread heartbeatThread = Connection.this.heartbeatThread;
            Thread receivingThread = Connection.this.receivingThread;
            try {
                if (heartbeatThread != null && heartbeatThread.isAlive()) {
                	heartbeatThread.interrupt();
                }
                
                if (receivingThread != null && receivingThread.isAlive()) {
                	receivingThread.interrupt();
                }
            }
            catch (Throwable t) {
            	// couldn't stop the threads for some reason
            }
            
            // close the underlying socket now that the threads have halted
            try {
            	Thread.sleep(5000);
                Connection.this.socket.disconnect();
            }
            catch (Throwable t) {
            	// couldn't stop the underlying socket for some reason
            }
        }
    } //class SendingRunnable
    
    
    
    /**
     * This is the body of a thread which runs periodically while the
     * connection is open and reads from the underlying MessageSocket
     * queue of incoming data to process messages that have been
     * received from the remote host.
     */
    public class ReceivingRunnable implements Runnable {
    	
    	public ReceivingRunnable() {
    		ConnectionManager.printDebug(id + "-CTRL",
    				                     "Created receiving thread.");
    	}
    	
    	@Override
    	public void run() {
    		// this is the milliseconds to wait between queue checks
    		int delay = 500;
    		
    		// wait for the initial socket connection to be established
    		while (socket == null) {
    			try {
    				Thread.sleep(delay);
    			}
    			catch (InterruptedException ie) {
    				// we have received the signal to shut down thread
    				return;
    			}
    		}
    		
    		// now that the connection is established, we need to loop
    		// as long as it is still open and while it still has more
    		// messages to process, reading each one and delivering to
    		// the appropriate section of code for handling the message
    		while (!socket.isDone()) {
    			try {
    				// read all the messages waiting in the queue
    				while (socket.hasNext()) {
    					String[] message = socket.nextMessage();
    					if (message == null) { break; }
    					
    					// process the message
    					onMessage(String.join("\n", message));
    				}
    				
    				// wait a bit before checking for more messages
    				Thread.sleep(delay);
    			}
    			catch (InterruptedException ie) {
    				// the message reading thread has been told to stop
    				return;
    			}
    			catch (Throwable t) {
    				// an error occurred processing incoming messages
    				ConnectionManager.printDebug("ERR", t.getMessage());
    			}
    		}
    	}
    }
    
    
    /**
     * This is the body of a thread which runs periodically while the
     * connection is open and sends "pings" in the form of overall status 
     * updates about the model to the remote program just to keep the socket
     * connection open and preventing it from closing due to any kind of an
     * idle timeout in the network library implementation at either end.  It
     * also allows for detection of the connection having closed prematurely,
     * which the Java libraries are not good at noticing natively.
     */
    private class HeartbeatRunnable implements Runnable {
        
        public HeartbeatRunnable() {
            ConnectionManager.printDebug(id + "-CTRL", 
                                         "Created heartbeat thread.");
        }
        
        @Override
        @SuppressWarnings("unused")
        public void run() {
            int count = 0;
            boolean hasStarted = false;
            
            // wait for session to exist if it hasn't been created yet
            while (socket == null) {
                try {
                    Thread.sleep(GlobalVariables.NETWORK_STATUS_REFRESH);
                }
                catch (InterruptedException ie) {
                    ConnectionManager.printDebug(id + "-CTRL", 
                                                 "Heartbeat stop.");
                    return;
                }
            }
            
            // loop as long as session is open and periodically send a message
            while (socket.isOpen()) {
                // wait a bit before firing off another ping message
                try {
                    Thread.sleep(GlobalVariables.NETWORK_STATUS_REFRESH);
                }
                catch (InterruptedException ie) {
                    ConnectionManager.printDebug(id + "-CTRL", 
                                                 "Heartbeat stop.");
                    return;
                }
                
                // send an inconsequential message over the socket just to
                // prevent it from being dormant, though in the future this
                // could be some kind of a generic simulation status message
                try {
                    // get the "environment" for the sim so we can poll it
                    RunEnvironment runEnv = RunEnvironment.getInstance();
                    
                    // get the current state of the simulation
                    String state = "Not running";
                    if (runEnv != null) {
                        // the "schedule" holding the simulation state
                        ISchedule schedule = runEnv.getCurrentSchedule();
                        
                        if (schedule != null) {
                            double tick = schedule.getTickCount();
                            hasStarted = true;
                            
                            if (tick < 0.0) {
                                // the model is still initializing
                                state = "Loading simulation";
                            }
                            else {
                                state = "Running @ tick #" + tick;
                            }
                        }
                    }
                    
                    // send the status message over the socket
                    String message = "STATUS," +
                            (new java.util.Date()).toString() + "\n" + state;
                    socket.send(new String[] { message });

                    
                    // check if the simulation has finished.  we could have checked
                    // this first, but we will do it last to give a chance so that we 
                    // will have sent one final status message with the end tick #.
                    /*if (hasStarted &&
                        !Connection.this.paused &&
                        !Connection.this.consuming) {

                    	String finalMessage = "STATUS," +
                                (new java.util.Date()).toString() + "\n" + "FInished";
                    	socket.send(new String[] { finalMessage } );
                    	break;
                    }*/

                }
                catch (IOException ioe) {
                    ConnectionManager.printDebug(id + "-ERR", 
                                                 ioe.getMessage());
                }
                catch (Throwable t) {
                    ConnectionManager.printDebug(id + "-ERR", t.getMessage());
                }
            }
            
            ConnectionManager.printDebug(id + "-SEND", "HEARTBEAT THREAD EXITING");
        }
    } //class HeartbeatRunnable
    
    //This method parses the string and creates an event from it
    private NetworkEventObject ParseString(String message){
    	
    	String delims = ",";
        String[] nextLine = message.split(delims);

        //** For Road closure message, it has the following format: EVENT, startTime, endTime, eventID, roadID*/
        int startTime = Math.round(Integer.parseInt(nextLine[1])/GlobalVariables.SIMULATION_STEP_SIZE);
		int endTime = Math.round(Integer.parseInt(nextLine[2])/GlobalVariables.SIMULATION_STEP_SIZE);
		// Make StartTime and endTime divisible by EVENT_CHECK_FREQUENCY
		startTime = startTime - (startTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
		endTime = endTime - (endTime % GlobalVariables.EVENT_CHECK_FREQUENCY);
		int eventID = Integer.parseInt(nextLine[3]);
		int roadID = Integer.parseInt(nextLine[4]);
		double value1 = GlobalVariables.BLOCKAGE_SPEED_FOREVENTS; 
		double value2 = -999;//A default value being stored for some new future use
		
		NetworkEventObject EventObject = new NetworkEventObject(startTime, endTime, eventID, roadID, value1, value2); 
		return EventObject;
    }
    
    //HG: This method inserts an event received from network communication into newEventQueue based on event's starting time 
  	public void insertExternalEvent(NetworkEventObject event){
  		int startTimeExternalEvent = event.startTime;
  		if(GlobalVariables.newEventQueue.peek() == null){//If the list is empty then no need for comparison with other events
  			if((int) RepastEssentials.GetTickCount() < event.endTime){ //Only add if the event's end time has not already passed
  				GlobalVariables.newEventQueue.add(event);
  			}
  		}else{
  			boolean flag = true; // a tag to identify the first event in newEventQueue that starts later than the external event
  			int queueSize = GlobalVariables.newEventQueue.size();
  			for (int index = 0; index < queueSize; index++) { //start iterating from the top of the queue
  				if(flag == false){
  					break;
  				}
  				if(startTimeExternalEvent < GlobalVariables.newEventQueue.get(index).startTime){ //add the new event 
  					GlobalVariables.newEventQueue.add(index, event);
  					flag = false;
  				}
  			}
  			if (flag == true) {
  			    // the new event does not start any event already in the queue, so add it to the end
  				GlobalVariables.newEventQueue.add(event);
  			  }
  		}
  		
  	}
}
