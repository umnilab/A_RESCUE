package evacSim.network;


import java.util.Vector;

import evacSim.GlobalVariables;


/**
 * evacSim.network.ConnectionManager
 * 
 * The connection manager is created with the start of the simulation
 * program and monitors the network for any incoming connections to the
 * simulation.  It creates an object to handle the interface between the
 * simulation and a remote listener through the use of a WebSocket-based
 * connection and the data collection buffer of the simulation.
 * 
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 17 October 2017
 */
public class ConnectionManager {
    
    /** This instance is the only connection manager object in the system. */
    private static ConnectionManager instance;
    
    /** The list of connections currently open in the system. */
    private Vector<Connection> connections;
        
    // TODO: Pick a permanent port number for the socket to use for listening.
    // This temporary value (33131) is the zip-code of downtown Miami.  Jetty
    // default (8080) is a common web port which may be in use or blocked.
    
    
    /**
     * Constructs the connection manager system which performs any steps
     * needed to build and start the WebSocket listening server within
     * the simulation program.
     * 
     * This constructor is private to prevent multiple instances of the
     * manager being created within the program.  All references to the
     * manager should be made through ConnectionManager.getInstance() which
     * will create the object the first time it is requested and return the
     * same object each time after that to ensure only one socket server is
     * ever created in the program.
     */
    private ConnectionManager() {
        // create the list for holding all the open connections
        this.connections = new Vector<Connection>();
        
        // start the server listening for incoming connections
        if (GlobalVariables.ENABLE_NETWORK) {
            Runnable startRunnable = new Runnable() {
                @Override
                public void run() {
                    try { Thread.sleep(5000); } catch (Throwable t) { }
                    ConnectionManager.this.startGatewayConnection();
                }
            };
            Thread startThread = new Thread(startRunnable);
            startThread.start();
        }
    }
    
    
    /**
     * Returns the singleton instance of the connection manager.
     * 
     * @return the singleton instance of the connection manager.
     */
    public static ConnectionManager getInstance() {
        if (ConnectionManager.instance == null) {
            ConnectionManager.instance = new ConnectionManager();
        }
        
        return ConnectionManager.instance;
    }
    
    
    /**
     * This method starts the custom messaging protocol socket connection to the
     * EvacSim gateway control program so this instance of the model can report
     * its status to remote users and receive commands from them.
     */
    public void startGatewayConnection() {
    	// the connection details for the gateway as provided by config file
    	String gwAddr = GlobalVariables.GATEWAY_ADDRESS;
        int gwPort = GlobalVariables.GATEWAY_PORT;
        
        // create a connection using these details
    	try {
    		this.addConnection(gwAddr, gwPort);
    	}
    	catch (Throwable t) {
    		// Something went wrong creating the network connection
    		// TODO: handle the gateway connection error
    		ConnectionManager.printDebug("ERR", t.getMessage());
    		return;
    	}    	
    }
    
    
    /**
     * Creates a connection to the specified remote system and adds it to
     * the list of all open connections.
     * 
     * @param address the address to the remote host being contacted.
     * @param port the port of the remote host where connections are accepted.
     * 
     * @throws IllegalArgumentException if connection details are not valid.
     */
    public void addConnection(String address, int port) {
    	// verify the address and port given are valid
    	if (address == null || address.trim().length() < 1) {
    		throw new IllegalArgumentException("No remote host specified.");
    	}
        if (port < 1 || port > 65535) {
        	throw new IllegalArgumentException("The port given is invalid.");
        }
        
        // attempt to create the connection object
        Connection connection = new Connection(address, port);
        
        // add the connection to the list of open connections
        this.connections.add(connection);
        
        // attempt to open the connection
        Runnable r = new Runnable() {
        	@Override
        	public void run() {
        		try {
        		  connection.open();
        		}
        		catch (Throwable t) {
        			// TODO: handle gateway connection errors
        		}
        	}
        };
        Thread t = new Thread(r);
        t.start();
    }
    
    
    /**
     * This method starts the WebSocket server on the listening port to
     * create new connection objects for incoming requests from remote
     * listening programs.  If the server is already accepting incoming
     * connections, this method will do nothing to alter it.
     *
    public void startServer() {
        if (this.server != null) {
            // the server already exists so check if it is running
            String serverState = this.server.getState();
            if (serverState == null) {
                // the state is unknown or something weird is happening, so
                // we will just destroy the existing server and replace it
                this.stopServer();
            }
            else {
                switch (serverState) {
                    case AbstractLifeCycle.STARTING:
                    case AbstractLifeCycle.STARTED:
                    case AbstractLifeCycle.RUNNING:
                        // the server is already running or will be soon
                        return;
                    case AbstractLifeCycle.STOPPING:
                        // server is still shutting down, so wait for it
                        try {
                            this.server.join();
                        }
                        catch (InterruptedException ie) {
                            // if server stop was interrupted, do we care?
                            ConnectionManager.printDebug("ERR", 
                                                         ie.getMessage());
                        }
                        // now that it's stopped, fall through to...
                    case AbstractLifeCycle.STOPPED:
                    case AbstractLifeCycle.FAILED:
                        // server exists but is not running, so it throw away
                        this.server = null;
                        break;
                    default:
                        // what do we do if state string is unexpected value?
                        break;
                }
            }
        }
        
        // create the new server on our listening port
        this.server = new Server(GlobalVariables.NETWORK_LISTEN_PORT);
        
        // attach a handler for generating sockets to incoming connections
        int maxMsgSize = GlobalVariables.NETWORK_MAX_MESSAGE_SIZE;
        WebSocketHandler socketHandler = null;
        try {
            socketHandler = new WebSocketHandler() {
                @Override
                public void configure(WebSocketServletFactory factory) {
                    // the default maximum size for a single message sent over
                    // the socket is way too small (64K) so we bump it higher
                    factory.getPolicy().setMaxTextMessageSize(maxMsgSize);
                    
                    // tell the factory which class has our socket methods
                    factory.register(Connection.class);
                }
            };
        }
        catch (Exception e) {
            //  what do we do if socket handler couldn't be created?
            ConnectionManager.printDebug("ERR", e.getMessage());
        }
        this.server.setHandler(socketHandler);
        
        // start the server listening
        try {
            this.server.start();
            ConnectionManager.printDebug("CTRL", "Started.");
        }
        catch (Exception e) {
            // deal with exception thrown from server not starting...
            ConnectionManager.printDebug("ERR", e.getMessage());
        }
    }
    */
    
    /**
     * This method will stop the server from accepting new incoming socket
     * connections and dispose of it, if it is already running or will be
     * running soon.  If the server is already stopped or failed, this
     * method will do nothing but dispose of the server object.
     *
    public void stopServer() {
        // check the server object exists
        if (this.server == null) {
            return;
        }
        
        // branch and handle the server stoppage depending on current state
        String serverState = this.server.getState();
        if (serverState == null) {
            // the state of the server could not be determined, so
            // we will just forcibly try to stop and destroy it
            try {
                this.server.stop();
                this.server.join();
                this.server = null;
            }
            catch (Exception e) {
                // an error happened stopping the server
            }
        }
        else {
            switch (serverState) {
                case AbstractLifeCycle.STOPPING:
                case AbstractLifeCycle.STOPPED:
                case AbstractLifeCycle.FAILED:
                    // the server is already stopped or stopping
                    this.server = null;
                    return;
                case AbstractLifeCycle.STARTING:
                    // the server is still starting, so wait for it to finish
                    // TODO: how to wait for "starting" server state?
                case AbstractLifeCycle.STARTED:
                case AbstractLifeCycle.RUNNING:
                default:
                    // server is running (or unknown state) so stop it
                    try {
                        this.server.stop();
                        this.server.join();
                        this.server = null;
                        ConnectionManager.printDebug("CTRL", "Stopped.");
                    }
                    catch (Exception e) {
                        // an error happened stopping the server
                        ConnectionManager.printDebug("ERR", e.getMessage());
                    }
            }
        }
    }
    */
    
    
    /**
     * Logs the given message to the standard output if debugging of 
     * the network has been enabled (via a constant boolean flag in 
     * the ConnectionManager class).  If debugging is not enabled, the 
     * message is ignored.
     * 
     * @param msg the debugging message to print to the standard output. 
     */
    public static void printDebug(String msg) {
        ConnectionManager.printDebug(null, msg);
    }
    
    
    /**
     * Logs the given message to the standard output if debugging of
     * the network has been enabled (via a constant boolean flag in 
     * the ConnectionManager class) with the given custom string in 
     * the prefix.  If debugging is not enabled, the message is ignored.
     * 
     * @param prefix the custom prefix to include at the beginning.
     * @param msg the debugging message to print to the standard output.
     */
    public static void printDebug(String prefix, String msg) {
        if (GlobalVariables.DEBUG_NETWORK) {
            String debugMessage = "<<NET";
            if (prefix != null && prefix.trim().length() > 0) {
                debugMessage += ":" + prefix;
            }
            debugMessage += ">> " + msg;
            
            System.out.println(debugMessage);
        }
    }
    
}
