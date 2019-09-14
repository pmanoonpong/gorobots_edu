// a Java socket server

// Keith Vertanen 4/99

import java.io.*;
import java.net.*;
import java.awt.*;

public class Server {

   boolean VERBOSE = false;		 // turn on/off debugging output
   static int BUFFSIZE = 400000;         // how many bytes our incoming buffer can hold

   static int INT_SIZE = 4;
   static int DOUBLE_SIZE = 8;
   static int FLOAT_SIZE = 4;

   byte data[];
   byte buff[];

   int port;
   int dataport;
   boolean reverse = false;
   ServerSocket server;
   Socket sock;
   DatagramSocket recv_sock, send_sock;
   String address;

   BufferedInputStream input;
   BufferedOutputStream output;
   

   public Server(int p, int datap) throws IOException
   {
	port = p;
	dataport = datap;
  
        try {
		server = new ServerSocket(port, 100);
	}
      	catch ( IOException e ) {
         	e.printStackTrace();
      	}

	if (dataport != -1)
	{
		// allocate the datagram socket
		try {
			recv_sock = new DatagramSocket(dataport);
			send_sock = new DatagramSocket();
		} 
		catch (SocketException se) {
			se.printStackTrace();
		}
	}

	// amortize the buffer allocation by just doing it once

	buff = new byte[BUFFSIZE];
	data = new byte[BUFFSIZE];
    }

    // wait for somebody to connect on our socket

    public void Connect() throws IOException
    {
	byte rev[] = new byte[1];
	rev[0] = 0;
	
	sock = server.accept();

	address = sock.getInetAddress().getHostName();

	if (VERBOSE) System.out.println("Server: opening socket to " +
			address + " on port " + port + 
			", datagrams on port = " + dataport);

	input = new BufferedInputStream(sock.getInputStream(), BUFFSIZE);
	output = new BufferedOutputStream(sock.getOutputStream(),BUFFSIZE);

	// now find out if they want reversed bytes or not

	input.read(rev);
	
	if (VERBOSE) 
		if (rev[0]==1) 
		{
			reverse = true;
			System.out.println("Server:  requested reversed bytes");
		}
		else
			System.out.println("Server:  requested normal byte order");
   }

   // send a string down the socket
   public int SendString(String str) throws IOException
   {

	/* convert our string into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(str.length());

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<str.length(); i++)
		out.write((byte) str.charAt(i));

	output.write(bytestream.toByteArray(), 0, bytestream.size());
	output.flush();

 	if (VERBOSE) System.out.println("Server: sending '" + str +"'");

	if (!RecvAck())
	    return -1;
	SendAck();

	return 1;
   }

   public int SendBytes(byte vals[], int len) throws IOException
   {
	if (VERBOSE) 
	{
		System.out.print("Server: sending " + len +" bytes: ");
		for (int i=0; i<len; i++)
			System.out.print(vals[i] + " ");
	}

	output.write(vals, 0, len);
	output.flush();

 	if (VERBOSE) System.out.println("");

	if (!RecvAck())
	    return -1;
	SendAck();

	return 1;
   }

   public int SendInts(int vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Server: sending " + len +" ints: ");

	/* convert our array of ints into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*INT_SIZE);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeInt(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	

	byte byte_array[] = bytestream.toByteArray();

	if (reverse)
	{
		// flip around our bytes if necessary

		byte flip_array[] = new byte[bytestream.size()];
		int i, j;

		for (i=0; i<len; i++)
		{
			for (j=0; j<INT_SIZE; j++)
			{
				flip_array[(i+1) * INT_SIZE - j - 1] = 
					byte_array[i * INT_SIZE + j];
			}
		}

		output.write(flip_array, 0, bytestream.size());
	}
	else
		output.write(byte_array, 0, bytestream.size());

	output.flush();

 	if (VERBOSE) System.out.println("");

	if (!RecvAck())
	    return -1;
	SendAck();

	return 1;
   }

   public int SendFloats(float vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Server: sending " + len +" floats: ");

	/* convert our array of floats into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*FLOAT_SIZE);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeFloat(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	
	byte byte_array[] = bytestream.toByteArray();

	if (reverse)
	{
		// flip around our bytes if necessary

		byte flip_array[] = new byte[bytestream.size()];
		int i, j;

		for (i=0; i<len; i++)
		{
			for (j=0; j<FLOAT_SIZE; j++)
			{
				flip_array[(i+1) * FLOAT_SIZE - j - 1] = 
					byte_array[i * FLOAT_SIZE + j];
			}
		}

		output.write(flip_array, 0, bytestream.size());
	}
	else
		output.write(byte_array, 0, bytestream.size());

	output.flush();

 	if (VERBOSE) System.out.println("");

	if (!RecvAck())
	    return -1;
	SendAck();

	return 1;
   }

   public int SendDoubles(double vals[], int len) throws IOException
   {
	if (VERBOSE) System.out.print("Server: sending " + len +" doubles: ");

	/* convert our array of floats into an array of bytes */

	ByteArrayOutputStream bytestream;
	bytestream = new ByteArrayOutputStream(len*DOUBLE_SIZE);

	DataOutputStream out;
	out = new DataOutputStream(bytestream);

	for (int i=0; i<len; i++)
	{
		out.writeDouble(vals[i]);
	 	if (VERBOSE) System.out.print(vals[i]+" ");
	}	

	byte byte_array[] = bytestream.toByteArray();

	if (reverse)
	{
		// flip around our bytes if necessary

		byte flip_array[] = new byte[bytestream.size()];
		int i, j;

		for (i=0; i<len; i++)
		{
			for (j=0; j<DOUBLE_SIZE; j++)
			{
				flip_array[(i+1) * DOUBLE_SIZE - j - 1] = 
					byte_array[i * DOUBLE_SIZE + j];
			}
		}

		output.write(flip_array, 0, bytestream.size());
	}
	else
		output.write(byte_array, 0, bytestream.size());

	output.flush();

	if (VERBOSE) System.out.println("");
	
	if (!RecvAck())
	    return -1;
	SendAck();

	return 1;

   }

   public void SendDatagram(byte vals[], int len) throws IOException
   {
	DatagramPacket sendPacket;

	if (VERBOSE) 
	{
		System.out.print("Server: sending datagram of " + len +" bytes: ");
		for (int i=0; i<len; i++)
			System.out.print(vals[i] + " ");
	}

	sendPacket = new DatagramPacket(vals, len,
					InetAddress.getByName(address), dataport);
	send_sock.send(sendPacket);

 	if (VERBOSE) System.out.println("");

   }

   // recv a string from the socket (terminates on terminal char)
   public String RecvString(char terminal) throws IOException
   {
	char c;
	String out;

	// would have liked to use readUTF, but it didn't seem to work
	// when talking to the c++ server

	out = new String("");

	while ((c=(char) input.read())!=terminal)
		out = out + String.valueOf(c);

	if (VERBOSE) System.out.println("Server: recv'd '" + out +"'");

	SendAck();
	if (!RecvAck())
	    return "NULL";

	return out;
   }

  public int RecvBytes(byte val[], int maxlen) throws IOException
  {
       int i;
       int totalbytes = 0;
       int numbytes;

	if (maxlen>BUFFSIZE)
		System.out.println("Sending more bytes then will fit in buffer!");

	while (totalbytes < maxlen)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			val[i] = data[i-totalbytes];

		totalbytes += numbytes;
	}

	if (VERBOSE) 
	{
		System.out.print("Server: received " + maxlen + " bytes - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	// we now send an acknowledgement to the server to let them
	// know we've got it

	SendAck();
	if(!RecvAck())
	    return -1;

	return maxlen;
  }

  public int RecvInts(int val[], int maxlen) throws IOException
  {
       int i;
       int totalbytes = 0;
       int numbytes;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*INT_SIZE>BUFFSIZE)
		System.out.println("Sending more ints then will fit in buffer!");

	while (totalbytes < maxlen*INT_SIZE)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;
	}

	// now we must convert the array of bytes to an array of ints


	if (reverse)
	{
		// flip around our bytes if necessary

	        ByteArrayInputStream bytestream_rev;
		DataInputStream instream_rev;


		byte flip_array[] = new byte[totalbytes];
		int j;

		for (i = 0; i < totalbytes / INT_SIZE; i++)
		{
			for (j = 0; j < INT_SIZE; j++)
			{
				flip_array[(i+1)*INT_SIZE - j - 1] = 
					buff[i * INT_SIZE + j];
			}

		}

		bytestream_rev = new ByteArrayInputStream(flip_array);
		instream_rev = new DataInputStream(bytestream_rev);

		for (i=0; i<maxlen; i++)
			val[i] = instream_rev.readInt();

	}	// end reverse section
	else
	{
	        ByteArrayInputStream bytestream;
		DataInputStream instream;

		bytestream = new ByteArrayInputStream(buff);
		instream = new DataInputStream(bytestream);

		for (i=0; i<maxlen; i++)
			val[i] = instream.readInt();
	}

	if (VERBOSE) 
	{
		System.out.print("Server: received " + maxlen + " ints - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	// we now send an acknowledgement to the server to let them
	// know we've got it

	SendAck();
	if(!RecvAck())
	    return -1;
	return maxlen;
  }

  public int RecvDoubles(double val[], int maxlen) throws IOException
  {
       int i;
       int numbytes;
       int totalbytes = 0;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*8>BUFFSIZE)
		System.out.println("Sending more doubles then will fit in buffer!");

	while (totalbytes < maxlen*DOUBLE_SIZE)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;
	
	}

	// now we must convert the array of bytes to an array of doubles

	if (reverse)
	{
		// flip around our bytes if necessary

	        ByteArrayInputStream bytestream_rev;
		DataInputStream instream_rev;


		byte flip_array[] = new byte[totalbytes];
		int j;

		for (i = 0; i < totalbytes / DOUBLE_SIZE; i++)
		{
			for (j = 0; j < DOUBLE_SIZE; j++)
			{
				flip_array[(i+1)*DOUBLE_SIZE - j - 1] = 
					buff[i * DOUBLE_SIZE + j];
			}

		}

		bytestream_rev = new ByteArrayInputStream(flip_array);
		instream_rev = new DataInputStream(bytestream_rev);

		for (i=0; i<maxlen; i++)
			val[i] = instream_rev.readDouble();

	}	// end reverse section
	else
	{
	        ByteArrayInputStream bytestream;
		DataInputStream instream;

		bytestream = new ByteArrayInputStream(buff);
		instream = new DataInputStream(bytestream);

		for (i=0; i<maxlen; i++)
			val[i] = instream.readDouble();
	}

	if (VERBOSE) 
	{
		System.out.print("Server: received " + maxlen + " doubles - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	SendAck();
	if (!RecvAck())
	    return -1;

	return maxlen;
  }

  public int RecvFloats(float val[], int maxlen) throws IOException
  {
       int i;
       int numbytes;
       int totalbytes = 0;

	/* for performance, we need to receive data as an array of bytes
		and then convert to an array of ints, more fun than
		you can shake a stick at! */

	if (maxlen*FLOAT_SIZE>BUFFSIZE)
		System.out.println("Sending more doubles then will fit in buffer!");

	while (totalbytes < maxlen*FLOAT_SIZE)
	{
		numbytes = input.read(data);

		// copy the bytes into the result buffer
		for (i=totalbytes; i<totalbytes+numbytes; i++)
			buff[i] = data[i-totalbytes];

		totalbytes += numbytes;

	}

	// now we must convert the array of bytes to an array of ints

	if (reverse)
	{
		// flip around our bytes if necessary

	        ByteArrayInputStream bytestream_rev;
		DataInputStream instream_rev;


		byte flip_array[] = new byte[totalbytes];
		int j;

		for (i = 0; i < totalbytes / FLOAT_SIZE; i++)
		{
			for (j = 0; j < FLOAT_SIZE; j++)
			{
				flip_array[(i+1)*FLOAT_SIZE - j - 1] = 
					buff[i * FLOAT_SIZE + j];
			}

		}

		bytestream_rev = new ByteArrayInputStream(flip_array);
		instream_rev = new DataInputStream(bytestream_rev);

		for (i=0; i<maxlen; i++)
			val[i] = instream_rev.readFloat();

	}	// end reverse section
	else
	{
	        ByteArrayInputStream bytestream;
		DataInputStream instream;

		bytestream = new ByteArrayInputStream(buff);
		instream = new DataInputStream(bytestream);

		for (i=0; i<maxlen; i++)
			val[i] = instream.readFloat();
	}


	if (VERBOSE) 
	{
		System.out.print("Server: received " + maxlen + " floats - ");
		for (i=0; i<maxlen; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	SendAck();
	if(!RecvAck())
	    return -1;
	return maxlen;
  }

  public int RecvDatagram(byte val[], int maxlen) throws IOException
  {
        int i;
        int numbytes;
	DatagramPacket receivePacket;

	if (maxlen>BUFFSIZE)
		System.out.println("Sending more bytes then will fit in buffer!");


	receivePacket = new DatagramPacket(val, maxlen);
	recv_sock.receive(receivePacket);

	numbytes = receivePacket.getLength();

	if (VERBOSE) 
	{
		System.out.print("Server: received " + numbytes + " bytes - ");
		for (i=0; i<numbytes; i++)
			System.out.print(val[i]+" ");
		System.out.println("");
	}

	return numbytes;
  }



   // shutdown the socket
   public void Close() throws IOException
   {
	sock.close();

 	if (VERBOSE) System.out.println("Server: closing socket");
   }

   // send a short ack to the server so they know we are ready for more

   private void SendAck() throws IOException
   {
	int ack = 0;

	if (VERBOSE)
		System.out.println("Sending ack...");

	output.write(ack);
	output.flush();

   }

   // recv a short ack from the server so we know they are ready for more

   private boolean RecvAck() throws IOException
   {
	int ack;

	if (VERBOSE)
	    System.out.println("Waiting for ack...");

	ack = (int) input.read();

	if(ack==-1)
	    return false;

	if (VERBOSE)
	    System.out.println("Ack recieved.");

	return true;
   }
}

