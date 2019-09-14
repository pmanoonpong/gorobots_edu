

import java.awt.image.*;
import java.util.*;
import java.io.*;
import java.net.*;

import april.jmat.*;
import april.jmat.geom.*;

import april.util.*;
import april.tag.*;

public class j_server_test 
{


    public static void main( String args[] ) throws IOException
    {
	int port = 5010;
	int dataport = -1;
	int rev = 1;

	System.out.println("Java Tag Server, listening on port " + port + ", datagram port " + dataport);
	Server mylink = new Server(port, dataport);

	System.out.println("Java Tag Server, waiting for connection...");
	mylink.Connect();

	//read image info

	String cImgH;
	String cImgW;

	cImgH = mylink.RecvString('\n');
	cImgW = mylink.RecvString('\n');

	if( cImgH == "NULL" || cImgW == "NULL")
	{
	    System.out.println("Server, could not receive data " );
	}

	int imgH = Integer.parseInt(cImgH);
	int imgW = Integer.parseInt(cImgW);
	double opticalCenter[] = new double[] { imgW/2.0, imgH/2.0 };
	int imgLen = imgH*imgW;

	// read grayscale image, one byte per pixel
	BufferedImage im = new BufferedImage(imgW, imgH, BufferedImage.TYPE_BYTE_GRAY);
	byte b[] = ((DataBufferByte) (im.getRaster().getDataBuffer())).getData();


	// Prerequired info for April Tags
	String tagFamilyClass = "april.tag.Tag36h11";
	double tagsize_m = 0.16256;
	double f_x = 555.72630;
	double f_y = 557.23982;

	double M[][] = new double[4][4];
	double loc[] = new double[6];


	TagFamily tf = (TagFamily) ReflectUtil.createObject(tagFamilyClass);
	TagDetector td = new TagDetector(tf);


	while(true)
	{
	    if( mylink.RecvBytes(b, imgLen) == -1 )
	    {
		System.out.println("Server, could not receive data " );
		break;
	    }
	    ArrayList<TagDetection> detections = td.process(im, opticalCenter);
	    String tagsNum = String.format("%d\n",detections.size());
	    mylink.SendString(tagsNum);

	    System.out.println("Number of tags:" + tagsNum);

	    for (TagDetection d : detections) {

		M = CameraUtil.homographyToPose(f_x, f_y, tagsize_m, d.homography);
		loc = LinAlg.matrixToXyzrpy(M);
		String tagsInfo = String.format("%d %d %f %f %f %f %f %f\n", System.currentTimeMillis(), d.id , loc[0], loc[1], loc[2], loc[3], loc[4], loc[5]);
		mylink.SendString(tagsInfo);
		System.out.println(tagsInfo);
	    }
	}


	System.out.println("Server, closing connection...");
	mylink.Close();	

	System.out.println("Server, done...");

    }
}
