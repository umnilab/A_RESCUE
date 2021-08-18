package evacSim.network;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

import java.net.Socket;

import java.text.DateFormat;
import java.text.SimpleDateFormat;

import java.util.Date;
import java.util.Vector;

import org.apache.log4j.Logger;

import evacSim.ContextCreator;


/**
 * evacSim.network.MessageSocket
 *
 * This class is used to create a wrapper around the basic Socket class
 * provided by Java so that data can be sent and received as discrete chunks
 * as if this is a messaging protocol like WebSockets.  This allows the
 * the client and server to retain some of the benefit of a protocol like
 * WebSockets for basic data-framing without any of the other overhead or
 * (more importantly) third-party library dependencies required to actually
 * use those protocols.  This class is the "client" which will be initiating
 * the connection to a listening socket at the specified remote host and port.
 * This class provides no interpretation of the contents of the messages sent
 * in either direction via this class and functions as merely a data conduit.
 *
 * @author Christopher Thompson (thompscs@purdue.edu)
 * @version 1.0
 * @date 27 January 2020
 */
class MessageSocket {

  /** This string on a line by itself is used to delimit the messages. */
  public static final String DELIMITER = "~~~";

  /** Whether or not to keep a log of all received messages. */
  private static final boolean RECV_LOG = true;

  /** Whether or not to keep a log of all sent messages. */
  private static final boolean SEND_LOG = true;

  /** The timeout in milliseconds to wait between reading data from socket. */
  private static final int RECV_WAIT = 10;

  /** The timeout in milliseconds to wait between keep-alive heartbeats. */
  private static final int HEARTBEAT = 5000;


  /** The address of the host of the remote end of the socket. */
  private String host;

  /** The port number of the remote end of the socket. */
  private int port;


  /** The socket object which handles the connection to the remote host. */
  private Socket socket;

  /** The stream reader which receives socket data from the remote host. */
  private BufferedReader reader;

  /** The stream writer which sends data over the socket to the remote host. */
  private BufferedWriter writer;


  /** A log of messages that have been received from the remote host. */
  private Vector<String[]> receiveLog;

  /** A log of messages that have been sent to the remote host. */
  private Vector<String[]> sendLog;

  /** A counter for keeping track of the next message to return. */
  private int readCounter;

  /** A counter for keeping track of the number of messages received. */
  private int receivedCounter;


  /** A thread which will loop and continue to receive all incoming data. */
  private Thread receivingThread;

  /** Whether or not the socket has finished and cannot receive more data. */
  private boolean done;


  /** A thread to periodically send messages to keep socket open and test. */
  private Thread keepAliveThread;

  /** Whether or not to utilize a heartbeat thread for the connection. */
  private boolean keepAlive;

  
  /** The console logging system used by the simulation. */
  private Logger logger;



  /**
   * Constructs the socket and opens it to the port on the host specified
   * and starts a thread to send a keep-alive message periodically if so
   * desired.  This constructor will also start a thread to read incoming
   * data from the remote host, breaking it automatically into discrete
   * messages when encountering the message terminating token.
   *
   * @param host The remote host to which a connection is established.
   * @param port The port of the remote host to which a connection is made.
   * @param keepAlive whether or not to start a thread for sending keep-alives.
   *
   * @throws IllegalArgumentException if the connection details were not valid.
   */
  MessageSocket(String host, int port, boolean keepAlive) {
	// retrieve a reference to the system-wide logging system
	this.logger = ContextCreator.logger;
	
    // check the validity of the remote host and port, then save these values
    if (host.trim().length() < 1) {
      // if the host string is empty, change it to null so that
      // the underlying socket class will treat it as localhost
      host = null;
    }
    this.host = host;

    if (port < 0 || port > 65535) {
      // if the given port is out of the valid range of possible
      // ports, this is not a situation which can be salvaged
      throw new IllegalArgumentException("Invalid port number: " + port);
    }
    this.port = port;

    // create the data structure to hold the message histories
    this.receiveLog = new Vector<String[]>();
    this.sendLog = new Vector<String[]>();

    // set the counters to their initial states
    this.receivedCounter = 0;
    this.readCounter = 0;

    // attempt to initiate the socket connection
    try {
      this.connect();
    }
    catch (Throwable t) {
      String msg = "Could not connect to " + host + ":" + port;
      throw new IllegalArgumentException(msg, t);
    }

    // create a thread to loop and process incoming data into messages
    Runnable receiveRunnable = new Runnable() {
      @Override
      public void run() {
        receiveLoop();
      }
    };
    this.receivingThread = new Thread(receiveRunnable);
    this.receivingThread.start();

    // create the thread to maintain the heartbeat messages
    this.keepAlive = keepAlive;
    if (keepAlive) {
      Runnable keepAliveRunnable = new Runnable() {
        @Override
        public void run() {
          keepAliveLoop();
        }
      };
      this.keepAliveThread = new Thread(keepAliveRunnable);
      this.keepAliveThread.start();
    }
  }


  /**
   * Constructs the socket and opens it to the port on the host specified.
   * This constructor will also start a thread to read incoming data from
   * the remote host, breaking it automatically into discrete messages when
   * encountering the message terminating token.  This version of the
   * constructor does not create a thread for sending keep-alive messages.
   *
   * @param host The remote host to which a connection is established.
   * @param port The port of the remote host to which a connection is made.
   *
   * @throws IllegalArgumentException if the connection details were not valid.
   */
  MessageSocket(String host, int port) {
    this(host, port, false);
  }


  /**
   * Connects to the remote host and creates the stream handlers for
   * both receiving data from the remote host and sending data to it.
   *
   * @throws IOException if a problem occured creating the socket or streams.
   */
  private void connect() throws IOException {
    // attempt to create the socket
    this.socket = new Socket(this.host, this.port);

    // setup a reader for pulling data received from the remote host
    InputStream is = this.socket.getInputStream();
    InputStreamReader isr = new InputStreamReader(is);
    this.reader = new BufferedReader(isr);

    // setup a writer for sending data to the remote host
    OutputStream os = this.socket.getOutputStream();
    OutputStreamWriter osw = new OutputStreamWriter(os);
    this.writer = new BufferedWriter(osw);
  }


  /**
   * Closes the socket and the stream handlers for the socket.
   *
   * @throws IOException if the socket or streams could not be closed.
   */
  public void disconnect() throws IOException {
	// halt the threads
	if (this.keepAliveThread != null && this.keepAliveThread.isAlive()) {
		try {
			this.keepAliveThread.interrupt();
		}
		catch (Throwable t) { }
	}
	
	if (this.receivingThread != null && this.receivingThread.isAlive()) {
		try {
			this.receivingThread.interrupt();
		}
		catch (Throwable t) { }
	}
	  
    // close the input and output streams
    if (this.reader != null) {
      this.reader.close();
    }
    if (this.writer != null) {
      this.writer.close();
    }

    // close the socket
    if (this.socket != null) {
      this.socket.close();
    }
  }


  /**
   * Returns whether or not the socket has finished.
   *
   * @return whether or not the socket has finished.
   */
  public boolean isDone() {
    return this.done;
  }
  
  
  /**
   * Returns whether or not the connection is currently open.
   * 
   * @return whether or not the connection is currently open.
   */
  public boolean isOpen() {
	  if (this.socket != null) {
		  if (this.socket.isConnected()) {
			  // this just means the socket has been opened previously.
			  // it does not mean the socket is still open.  for that we
			  // have to check to see if it has also been closed yet.
			  if (this.socket.isClosed()) {
				  // the socket has been opened, but it also has been closed
				  return false;
			  }
			  else {
				  // the socket has not yet been closed
				  return true;
			  }
		  }
		  else {
			  // the socket has never been opened
			  return false;
		  }
	  }
	  else {
		  // the socket doesn't exist, so it can't be open
		  return false;
	  }
  }


  /**
   * Returns the next unread entry on the queue of messages received from
   * the remote host.  Returns null if there are no more unread messages.
   * If the message to return is itself null or has no contents, an empty
   * String array (new String[0]) is returned.
   *
   * @return a String array which is the contents of the message.
   */
  public String[] nextMessage() {
    if (this.receiveLog == null) { return null; }

    // grab the element in the history at the current counter,
    // increment the counter for the next read, and return element
    try {
      String[] message = this.receiveLog.get(this.readCounter);
      this.readCounter++;

      if (message == null) { message = new String[0]; }
      return message;
    }
    catch (ArrayIndexOutOfBoundsException aioobe) {
      // there are no more unread items in the message queue
      return null;
    }
  }


  /**
   * Returns whether or not there are unread entries waiting to be read
   * in the queue of received messages.
   *
   * @return whether or not there are unread entries in the message queue.
   */
  public boolean hasNext() {
    if (this.receiveLog == null) { return false; }

    // see if there are any unread messages in the received history
    int size = this.receiveLog.size();
    if (this.readCounter < size) { return true;}

    // the counter has caught up to the size of the history log
    return false;
  }


  /**
   * Returns the size of the received message log.  If the received
   * message log does not exist yet, -1 is returned.
   *
   * @return the size of the received message queue.
   */
  public int logSize() {
    if (this.receiveLog == null) { return -1; }
    return this.receiveLog.size();
  }


  /**
   * Returns the size of the sent messages log.  If the sent
   * message log does not exist yet, -1 is returned.
   *
   * @return the size of the sent messages log.
   */
  public int sentLogSize() {
    if (this.sendLog == null) { return -1; }
    return this.sendLog.size();
  }


  /**
   * Returns the entry in the received message log at the specified index.
   * Returns null if the specified index is not a valid position for the log.
   *
   * @return the entry in the received message log at the specified index.
   */
  public String[] getLog(int index) {
    if (this.receiveLog == null) { return null; }
 
    try {
      return this.receiveLog.get(index);
    }
    catch (ArrayIndexOutOfBoundsException aioobe) {
      return null;
    }
  }


  /**
   * Returns the entry in the sent message log at the specified index.
   * Returns null if the specified index is not a valid position for the log.
   *
   * @return the entry in the sent message log at the specified index.
   */
  public String[] getSentLog(int index) {
    if (this.sendLog == null) { return null; }

    try {
      return this.sendLog.get(index);
    }
    catch (ArrayIndexOutOfBoundsException aioobe) {
      return null;
    }
  }


  /**
   * Sends the message given to the remote host.
   *
   * @param the message to send to the remote host.
   * @throws IOException if there is a problem sending the message.
   */
  public void send(String[] message) throws IOException {
    // check both the write exists and there was a message given
    if (this.writer == null) { return; }
    if (message == null) { return; }

    // send each line of the message
    for (int i=0; i < message.length; i++) {
      this.writer.write(message[i] + "\n");
    }

    // send the token used to delimit messages
    this.writer.write(DELIMITER + "\n");
    this.writer.flush();

    // append this message to the log of sent messages
    if (SEND_LOG) {
      if (this.sendLog != null) {
        this.sendLog.add(message);
      }
    }
  }


  /**
   * This method is intended to function as the body of a thread which is
   * started at the same time the socket connection is established and remains
   * running the entire time the connection is open to receive and process
   * all incoming data from the remote host into the discrete messages which
   * are placed into a queue to be read by other objects in the program which
   * take ownership of this socket object.
   */
  public void receiveLoop() {
    // this data structure (created new each time the first line of a
    // message is received) will hold the buffer of current message
    // contents until the message delimiter token is received
    Vector<String> body = null;
    
    // loop forever, until this thread dies or connection resets, and
    // continue to receive data from socket, breaking it into messages
    this.done = false;
    while (!this.done) {
      try {
        // loop and take all the data from buffer that we have right now
        while (this.reader != null && this.reader.ready()) {
          // read the next line from the buffer
          String line = this.reader.readLine();
          if (line == null) {
            line = "";
          }

          // since we are storing the message already broken into separate
          // lines, we do not want the terminating newline character if the
          // buffered reader passed it along in the contents of the read
          if (line.endsWith("\n")) {
            line = line.substring(0, line.length() - 1);
          }

          // check to see if this is a message terminator line
          if (line.equals(DELIMITER)) {
            // we've received the end of a message, so we need to put
            // it in the history and reset the current message array
            if (this.receiveLog != null) {
              String[] message = body.toArray(new String[body.size()]);
              body = null;

              this.receiveLog.add(message);
              this.receivedCounter++;
            }
          }
          else {
            // this is a normal line of data for the message body
            if (body == null) {
              body = new Vector<String>();
            }
            body.add(line);
          }
        }
      }
      catch (IOException ioe) {
        // an exception was thrown while trying to read from the socket
        // TODO: handle the socket error states
        logger.error("SOCKET READ ERROR!", ioe);
      }

      // take a brief pass in reading so that this class doesn't block the
      // operation of the rest of the jvm by hogging cpu waiting on net io
      try {
        Thread.sleep(RECV_WAIT);
      }
      catch (InterruptedException ie) {
        // this thread has been told to stop running
        this.done = true;
      }
    }
  }


  /**
   * This method is intended to be the body of a thread which is created at
   * the socket creation to send periodic "heartbeat" messages to the remote
   * host to function as a keep-alive mechanism and also to test if the
   * socket connection is still open.
   */
  public void keepAliveLoop() {
    int h = 0;

    while (true) {
      // wait the prescribed timeout period between heartbeat messages
      try {
        Thread.sleep(HEARTBEAT);
      }
      catch (InterruptedException ie) {
        return;
      }

      // construct the heartbeat message
      String[] heartbeat = new String[3];
      heartbeat[0] = "HEARTBEAT";

      DateFormat df = new SimpleDateFormat("yyyy-MM-dd - HH:mm:ss");
      heartbeat[1] = df.format(new Date());

      heartbeat[2] = "# " + (h++);

      // send the heartbeat messages, handling any errors that occur
      try {
        this.send(heartbeat);
      }
      catch (IOException ioe) {
        // TODO: handle the send error!
        logger.error("HEARTBEAT SENDING ERROR", ioe);

        this.done = true;
      }

      // check if we are supposed to stop yet
      if (this.done) {
        return;
      }
    }
  }

}
