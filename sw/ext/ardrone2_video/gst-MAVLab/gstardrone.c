
/**
 * SECTION:element-mavlab
 *
 * The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=5/1' ! videoscale ! video/x-raw-yuv, width=640, height=368 ! mavlab ! fakesink

 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdlib.h>     /* calloc, exit, free */
#include <unistd.h>		//usleep
#include <stdio.h> /* printf */


#include "socket.h"
#include "video_message_structs_sky.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gst/gst.h>
#include "gstardrone.h"
#include "guido.h"
#include "optic_flow.h"
#include "gps.h"
#include <math.h>


unsigned int imgWidth, imgHeight;
int mode;
gint adjust_factor;
unsigned char * img_uncertainty;

//optical flow
unsigned char * old_img;
int old_pitch,old_roll,old_alt;
int *accum;

unsigned int counter;

unsigned int socketIsReady;
unsigned int tcpport;
struct gst2ppz_message_struct_sky gst2ppz;
struct ppz2gst_message_struct_sky ppz2gst;

int x_buf[24];
int y_buf[24];
int diff_roll_buf[24];
int diff_pitch_buf[24];

int opt_trans_x_buf[32];
int opt_trans_y_buf[32];


unsigned int buf_point;
unsigned int buf_imu_point;
unsigned int buf_opt_trans_point;

float opt_angle_y_prev;
float opt_angle_x_prev;
void *TCP_threat( void *ptr);

void *line_threat( void *ptr);
unsigned char * img_line;
unsigned int bluecount_line;
unsigned int orangecount_line;
unsigned int alt_line;


int new_line_im;

//threshold variables
unsigned char minU_blue;
unsigned char maxU_blue;
unsigned char minV_blue;
unsigned char maxV_blue;

unsigned char minU_orange;
unsigned char maxU_orange;
unsigned char minV_orange;
unsigned char maxV_orange;

unsigned char min_gradient;


void houghtrans_line(unsigned char * img, float a_res, unsigned int width, unsigned int height, int b_steps, int nLines, int drawlines);
void getmax(int ** accumulator, int acount, int b_steps, int * x, int * y, int * max);
void icvHoughLinesStandard( unsigned char * image, unsigned int width, unsigned int height, float rho, float theta,int threshold, int linesMax , int * rhos_out, float * thetas_out);
int cmpfunc (const void * a, const void * b);
int image_index4(int xx, int yy);
int image_index(int xx, int yy);

GST_DEBUG_CATEGORY_STATIC (gst_mavlab_debug);
#define GST_CAT_DEFAULT gst_mavlab_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

//adjustable parameters
enum
{
  PROP_0,
  PROP_SILENT,
  PROP_TCP,
  ADJUST,
  MODE
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
    );

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
    );

GST_BOILERPLATE (Gstmavlab, gst_mavlab, GstElement,
    GST_TYPE_ELEMENT);

static void gst_mavlab_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_mavlab_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_mavlab_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_mavlab_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_mavlab_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
    "mavlab",
    "Passthrough element",
    "Calculates stuff on the video, to be fed to an autopilot",
    "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the mavlab's class */
static void
gst_mavlab_class_init (GstmavlabClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;
 // gstelement_class = (GstElementClass *) klass;

  gobject_class->set_property = gst_mavlab_set_property;
  gobject_class->get_property = gst_mavlab_get_property;

	g_object_class_install_property (gobject_class, PROP_SILENT,
    g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
    FALSE, G_PARAM_READWRITE));
		  
	g_object_class_install_property (gobject_class, PROP_TCP,
    g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
    0, G_PARAM_READWRITE));		  
		  
	g_object_class_install_property (gobject_class, ADJUST,
      g_param_spec_int ("adjust", "Adjust factor", "Change adjust factor for sky segmentation.",-1000,1000,
          0, G_PARAM_READWRITE));
	
	g_object_class_install_property (gobject_class, MODE,
      g_param_spec_int ("mode", "Change the mode", "Change the mode to do either only segmentation [0] or the whole bunch [1].",0,10,
          0, G_PARAM_READWRITE));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_mavlab_init (Gstmavlab * filter,
    GstmavlabClass * gclass)
{
	
  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_mavlab_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
                              GST_DEBUG_FUNCPTR(gst_mavlab_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;
    
}

static void
gst_mavlab_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstmavlab *filter = GST_MAVLAB (object);

  switch (prop_id) {
    case PROP_SILENT:
      filter->silent = g_value_get_boolean (value);
      break;
    case ADJUST:
      adjust_factor = g_value_get_int (value);
      break;	
    case PROP_TCP:
      tcpport = g_value_get_uint (value);
      break;		  
	case MODE:
      mode = g_value_get_int (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_mavlab_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstmavlab *filter = GST_MAVLAB (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, filter->silent);
      break;
	case ADJUST:
      g_value_set_int (value, adjust_factor);
      break;
	case PROP_TCP:
      g_value_set_uint (value, tcpport);
      break;
	case MODE:
      g_value_set_int (value, mode);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_mavlab_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstmavlab *filter;
  GstPad *otherpad;

  filter = GST_MAVLAB (gst_pad_get_parent (pad));
  otherpad = (pad == filter->srcpad) ? filter->sinkpad : filter->srcpad;
  gst_object_unref (filter);


  
  
  
  //make the image size known
   const GstStructure *str;
	str = gst_caps_get_structure (caps, 0);
	gint tmp;
	gst_structure_get_int (str, "width", &tmp);
	imgWidth = (unsigned int)tmp;  
	gst_structure_get_int (str, "height", &tmp);
	imgHeight = (unsigned int)tmp;
	g_print ("The video size is %dx%d\n", imgWidth, imgHeight);

	counter = 0;
	img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);
	old_img = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	old_pitch = 0;
	old_roll = 0;
	old_alt=0;
	ppz2gst.pitch = 0;
	ppz2gst.roll = 0;
	opt_angle_y_prev = 0;
	opt_angle_x_prev=0;
  
  
  	if (tcpport>0) {
		//start seperate threat to connect
		//seperate threat is needed because otherwise big delays can exist in the init or chain function
		//causing the gst to crash
	
		pthread_t th1;
		int th1_r;
		pthread_create(&th1,NULL,TCP_threat,&th1_r);	
	}
	
	
	minU_blue = 107;
	maxU_blue = 151;
	minV_blue = 142;
	maxV_blue = 195;

	minU_orange = 129;
	maxU_orange = 134;
	minV_orange = 84;
	maxV_orange = 126;

	min_gradient = 53;
	
	
	
	img_line = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	if (!img_line)
		printf( "Mem errorrrrrr 1\n");
	new_line_im = 1;
	pthread_t th2;
	int th2_r;
	pthread_create(&th2,NULL,line_threat,&th2_r);	
  
  return gst_pad_set_caps (otherpad, caps);
}

void *TCP_threat( void *ptr) {
	g_print("Waiting for connection on port %d\n",tcpport);
	socketIsReady = initSocket(tcpport);
   	if (!socketIsReady) { 
		g_print("Error initialising connection\n");	
	} else {
		g_print("Connected!\n");
	}


	while(1) {
		int res = Read_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));
		if	(res>1) {
			int tmp;
			tmp = (int)counter - (int)ppz2gst.counter;			
			if (tmp>6) {
				g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.counter, tmp); //delay of 3 is caused by the fact not every frame is used (15fps mod 3)
			}	
			
			
		minU_blue = ppz2gst.minU_blue;
		maxU_blue = ppz2gst.maxU_blue;
		minV_blue = ppz2gst.minV_blue;
		maxV_blue = ppz2gst.maxV_blue;

		minU_orange = ppz2gst.minU_orange;
		maxU_orange = ppz2gst.maxU_orange;
		minV_orange = ppz2gst.minV_orange;
		maxV_orange = ppz2gst.maxV_orange;

		min_gradient = ppz2gst.min_gradient;
			
			
			
			
						
		} else {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_mavlab_chain (GstPad * pad, GstBuffer * buf)
{
	Gstmavlab *filter;

	filter = GST_MAVLAB (GST_OBJECT_PARENT (pad));	
	unsigned char * img = GST_BUFFER_DATA(buf);   
	
	if (GST_BUFFER_SIZE(buf) != imgHeight*imgWidth*2) // -> wrong color space!!!
		g_print("GGGGGGGGGGGGGGGGRRRRRRRRRRRRRRRRRRRR");


	
	if (mode==0) 
	{
		segment_no_yco_AdjustTree(img,img_uncertainty,adjust_factor); 
	}
	else if (mode==1)
	{
		skyseg_interface_n(img, img_uncertainty, adjust_factor, counter, ppz2gst.pitch/36, ppz2gst.roll/36); 

		if (tcpport>0) { 	//if network was enabled by user
			if (socketIsReady) { 
				//gst2ppz.blob_x1 = blobP[0];
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
			}
		}
	}
	else if (mode==2)
	{	
		int MAX_POINTS, error;
		int n_found_points,mark_points;
		int *x, *y, *new_x, *new_y, *status;
		mark_points = 0; //settings this to one destroys result...
				
		//save most recent values of attitude for the currently available frame
		int current_pitch = ppz2gst.pitch;
		int current_roll = ppz2gst.roll;
		int current_alt = ppz2gst.alt;
	
		x = (int *) calloc(40,sizeof(int));
		new_x = (int *) calloc(40,sizeof(int));
		y = (int *) calloc(40,sizeof(int));
		new_y = (int *) calloc(40,sizeof(int));		
		status = (int *) calloc(40,sizeof(int));

		
		
		//test copy image to tmp variables:
		//unsigned char * img_copy = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
		//memcpy(img_copy,img,imgHeight*imgWidth*2);
		
		if (tcpport==0) {
			//test code if no network ppz communication is available
			current_alt = 100;
			current_pitch=0;
			current_roll=0;				
		}
		
		
		MAX_POINTS = 40;

		
		//active corner:
		
		int *active;
		active =(int *) calloc(40,sizeof(int));		
		int GRID_ROWS = 5;
		int ONLY_STOPPED = 0;		
		error = findActiveCorners(img, GRID_ROWS, ONLY_STOPPED, x, y, active, &n_found_points, mark_points,imgWidth,imgHeight);
		
		
		//normal corner:
		/*
		int suppression_distance_squared;
		suppression_distance_squared = 3 * 3;
		error = findCorners(img, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
		*/
		
		//middle of the picture
		/*
		x[0] = 20;
		y[0] = 12;	 
		x[1] = imgWidth-20;
		y[1] = 12;			
		x[2] = 20;
		y[2] = imgHeight - 12;			
		x[3] = imgWidth-20;
		y[3] = imgHeight - 12;			
		
		n_found_points=4;
		error=0;
		*/
		
		
		if(error == 0)
		{
			error = opticFlowLK(img, old_img, x, y, n_found_points, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_POINTS);	
			
			
			//calculate roll and pitch diff:
			int diff_roll = (current_roll- old_roll)*1024; 
			int diff_pitch = (current_pitch- old_pitch)*1024;
			
			//remember the last 6 values of the imu values, because of the better subtraction, due to the delay from the moving average on the opitcal flow
			diff_roll_buf[buf_imu_point] = diff_roll;
			diff_pitch_buf[buf_imu_point] = diff_pitch;
			buf_imu_point = (buf_imu_point+1) %24;
			
			//use delayed values for IMU, to compensate for moving average. Phase delay of moving average is about 50% of window size.
			diff_roll = diff_roll_buf[(buf_imu_point+12)%24];
			diff_pitch = diff_pitch_buf[(buf_imu_point+12)%24];
			
			//calculate mean altitude between the to samples:
			int mean_alt;
			if (current_alt>old_alt)
				mean_alt = (current_alt-old_alt)/2 + old_alt;
			else
				mean_alt = (old_alt-current_alt)/2 + current_alt;			
			
			
			//remember the frame and meta info
			memcpy(old_img,img,imgHeight*imgWidth*2);
			old_pitch = current_pitch;
			old_roll = current_roll;
			old_alt = current_alt;
			
			
			
			if(error == 0)
			{
				showFlow(img, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);
				
				int tot_x=0;
				int tot_y=0;
				int total_weight = 0;
				//int n_used_points = 0;
				int weight_stopped = 2;
				int weight_active = 1;
				for (int i=0; i<n_found_points;i++) {
					if(status[i] == 1)
					{
						//n_used_points++;
						if (active[i]) // not really seems to have much effect
						{
							tot_x = tot_x+weight_active*(new_x[i]-x[i]);
							tot_y = tot_y+weight_active*(new_y[i]-y[i]);		
							total_weight += weight_active;
						}
						else
						{
							tot_x = tot_x+weight_stopped*(new_x[i]-x[i]);
							tot_y = tot_y+weight_stopped*(new_y[i]-y[i]);		
							total_weight += weight_stopped;
						}
					}
				}
				
				//apply a moving average of 24
				if (total_weight) {
				
					//magical scaling needed in order to calibrate opt flow angles to imu angles
					int scalex = 1400; //1024*(1/0.75) 
					int scaley = 1400; //1024*(1/0.76)				
				
					x_buf[buf_point] = (tot_x*scalex)/total_weight;
					y_buf[buf_point] = (tot_y*scaley)/total_weight;
					buf_point = (buf_point+1) %24;
				}
				int x_avg = 0;
				int y_avg = 0;
				for (int i=0;i<24;i++) {
					x_avg+=x_buf[i];
					y_avg+=y_buf[i];
				}
						
						
				//compensate optic flow for attitude (roll,pitch) change:
				x_avg -=  diff_roll; 
				y_avg -= diff_pitch; 
								
				//calculate translation in cm/frame from optical flow in degrees/frame
				int opt_trans_x = 0;
				int opt_trans_y = 0;				
				opt_trans_x_buf[buf_opt_trans_point] = tan_zelf(x_avg/1024)*mean_alt/1000;
				opt_trans_y_buf[buf_opt_trans_point] = tan_zelf(y_avg/1024)*mean_alt/1000;
				buf_opt_trans_point = (buf_opt_trans_point + 1) % 32;
				for (int i=0;i<32;i++) {
					opt_trans_x+=opt_trans_x_buf[i];
					opt_trans_y+=opt_trans_y_buf[i];
				}
				opt_trans_x = opt_trans_x /32;
				opt_trans_y = opt_trans_y /32;
				
				
				
				g_print("006;%d;%d;%d;%d;%d;%d;%d;%d;%d\n",x_avg+diff_roll,diff_roll,x_avg,y_avg+diff_pitch,diff_pitch,y_avg,mean_alt,opt_trans_x,opt_trans_y);
				
				if (tcpport>0) { 	//if network was enabled by user
					if (socketIsReady) { 
						gst2ppz.counter = counter;
						gst2ppz.optic_flow_x = opt_trans_x;
						gst2ppz.optic_flow_y = opt_trans_y;
						Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
					}
				}
				
				
			} else g_print("error1\n");			
		} else g_print("error2\n");	
		
		free(x);
		free(new_x);
		free(y);
		free(new_y);
		free(status);
		free(active);
			//GST_BUFFER_DATA(buf) = old_img;
		//free(img_copy);

		
//		if (filter->silent == FALSE) {
//			g_print("Errorh: %d, n_found_points: %d\n",error,n_found_points);
//		}
	
	} else if (mode==3) {	
		//segment image:
		int current_alt = ppz2gst.alt;
		//calculate gradient over picture
		unsigned int darkcount = 0;
		unsigned int whitecount = 0;
		
		unsigned int bluecount;
		unsigned int orangecount;
		unsigned char * img_copy = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
		if (!img_copy)
			printf( "Mem errorrrrrr\n");
		memcpy(img_copy,img,imgHeight*imgWidth*2);
				
		unsigned int count = 0;
		unsigned int imgWidth2 = imgWidth*2;
		unsigned int maxcount = imgHeight * imgWidth *2 -4;				
		while (count < maxcount) {
			int id = count;
			int idx= id+4;
			int idy= id+imgWidth2;;
			int pxy = 0;
			
			int res1 = ((img[id] > minU_blue) && (img[id] < maxU_blue) && (img[id+2] > minV_blue) && (img[id+2] < maxV_blue));
			bluecount+=res1;
			int res2 = ((img[id] > minU_orange) && (img[id] < maxU_orange) && (img[id+2] > minV_orange) && (img[id+2] < maxV_orange));
			orangecount+=res2;
			
			if  (  res1 || res2 ) { // perform blue|orange segmentation				
				//calculate gradients
				int py1 = abs((int)img[id] - (int)img[idy]);						
				int px1 = abs((int)img[id] - (int)img[idx]);
				int py2 = abs((int)img[id+2] - (int)img[idy+2]);						
				int px2 = abs((int)img[id+2] - (int)img[idx+2]);
				pxy = (px1 + py1 + px2 + py2) ;
				
				whitecount++;
			} 
			
			//Y channels							
			if (pxy > min_gradient && img[id] ) { // segment on edges
				img[count+1] =255;
				img[count+3] =255;
				img[count] = 127; 
				img[count+2] =127 ;
				whitecount++;
			}
			else {
				darkcount++;
				img[count+1] =0;
				img[count+3] =0;
			}
			
			//color channels
			//img[count] = 127; 
			//img[count+2] =127 ;
		
			count +=4;				
			
		}	
		
		
		
		//clear the line around the screen:
		//horizontal
		int count_end = imgWidth*(imgHeight)*2-4;
		for (int i = 0 ; i < (int)imgWidth*4; i+=4) {
			
			//img[i] = 127;
			img[i+1] = 0;
			//img[i+2] = 127;
			img[i+3] = 0;
			
			//img[count_end] = 127;
			img[count_end+1] = 0;
			//img[count_end+2] = 127;
			img[count_end+3] = 0;
			count_end-=4;
		}
		//vertical	
		for (int i = 2*(int)imgWidth ; i < 2*(int)imgWidth*(int)imgHeight; i+=2*(int)imgWidth) {			
			for (int j = -4; j<4;j+=4) {
				if (i+j>0) {
					//img[i+j] = 127;
					img[i+1+j] = 0;
					//img[i+2+j] = 127;
					img[i+3+j] = 0;
				}
			}
		}
		
		
		//printf("Mean U: %lu, V: %lu\n",color_U/(imgWidth*imgHeight),color_V/(imgWidth*imgHeight) );
		

		
		
		
			//dilation
		count=0;
		while (count < maxcount) {
			int id = count+1;
			int idx= id+4;
			int idy= id+imgWidth2;;

			if (img[idy]==255) {
				
				if (img[idx]==0) {
					img[idx] = 255;
					img[idx+2] = 255;
					whitecount++;
					darkcount--;
				}
				if (img[id] == 0) {
					img[id] = 255;
					img[id+2] = 255;
					whitecount++;
					darkcount--;
				}
			}
			
			count +=4;				
		}	
		/*
				//erosion:	
		int erocount = 0;
		count=0;
		while (count < maxcount) {
			int id = count+1;
			int idx= id+4;
			int idy= id+imgWidth2;;

			if (!(img[id] && img[idx] && img[idy]) ){
				if (img[id]) {
					img[id] = 0;
					img[id+2] = 0;
					whitecount--;
					darkcount++;
					erocount++;
				}
			}
					
			count +=4;				
		}	
	

		//erosion:	
		count=0;
		while (count < maxcount) {
			int id = count+1;
			int idx= id+4;
			int idy= id+imgWidth2;;

			if (!(img[id] && img[idx] && img[idy]) ){
				if (img[id]) {
					img[id] = 0;
					img[id+2] = 0;
					whitecount--;
					darkcount++;
					erocount++;
				}
			}
					
			count +=4;				
		}		
		*/
		
		//send new segmented image to line thread if needed
		if (new_line_im == 1) {
			bluecount_line = bluecount;
			orangecount_line = orangecount;
			alt_line = current_alt;
			memcpy(img_line,img,imgHeight*imgWidth*2);
			new_line_im = 0; // todo, replace by mutex
		}
		
		//g_print("Harrow6 %d, %d, %d\n", counter,darkcount,whitecount);
		//fix strange bug that crashes dsp if picture is to uniform
		if (whitecount<170) {
		//	memcpy(img,img_copy,imgHeight*imgWidth*2);				
		}
		
		free(img_copy);		
		
		//send result
		if (tcpport>0) { 	//if network was enabled by user
			if (socketIsReady) { 
				//gst2ppz.blob_x1 = blobP[0];
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
			}
		}
		
	}
		
	counter++; // to keep track of data through ppz communication
	
	  
	  
	  /*
	  		//clear last line:
		int count_end = imgWidth*(imgHeight)*2-4;
		for (int i = 0 ; i < imgWidth*4; i+=4) {
			
			img[i] = i%255;
			img[i+1] = i%255;
			img[i+2] = i%255;
			img[i+3] = i%255;
			
			img[count_end] = i%255;
			img[count_end+1] = i%255;
			img[count_end+2] = i%255;
			img[count_end+3] = i%255;
			count_end-=4;
		}

		
				
		for (int i = 2*imgWidth ; i < 2*imgWidth*imgHeight; i+=2*imgWidth) {
			
			for (int j = -8; j<8;j+=4) {
				if (i+j>0) {
					img[i+j] = i%255;
					img[i+1+j] = i%255;
					img[i+2+j] = i%255;
					img[i+3+j] = i%255;
				}
			}
		}
	  
	  */
	  
	  
  return gst_pad_push (filter->srcpad, buf);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features

 */
static gboolean
mavlab_init (GstPlugin * mavlab)
{
  /* debug category for fltering log messages
   *
   * exchange the string 'Template mavlab' with your description
   */   
	 
  GST_DEBUG_CATEGORY_INIT (gst_mavlab_debug, "mavlab",
      0, "The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot");

  return gst_element_register (mavlab, "mavlab", GST_RANK_NONE,
      GST_TYPE_MAVLAB);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "MAVLab"
#endif

/* gstreamer looks for this structure to register mavlabs
 *
 * exchange the string 'Template mavlab' with your mavlab description
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "mavlab",
    "The MAVLab plugin for gstreamer will contain algorithms to perform analysis on the realtime video on board, to feed to the autopilot",
    mavlab_init,
    VERSION,
    "LGPL",
    "MAVLab",
    "http://gstreamer.net/"
)




void icvHoughLinesStandard( unsigned char * image, unsigned int width, unsigned int height, float rho, float theta,int threshold, int linesMax , int * rhos_out, float * thetas_out)
{
        
    int numangle, numrho;
    int total = 0;
    int i, j;
    float irho = 1 / rho;
    double scale;
    
    numangle = 3.1416 / theta;
    numrho = ((width + height) * 2 + 1) / rho;

	//g_print("\n%d; %d\n", numrho,numangle);
	
    float *tabSin = (float *) calloc(numangle,sizeof(float));
	float *tabCos = (float *) calloc(numangle,sizeof(float));
	int *sort_buf = (int *) calloc(numangle* numrho,sizeof(int));
	accum = (int *) calloc((numangle+2) * (numrho+2),sizeof(int));

    memset( accum, 0, sizeof(accum[0]) * (numangle+2) * (numrho+2) ); // unnecessary?

    float ang = 0;
    for(int n = 0; n < numangle; ang += theta, n++ )
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }
	
    // stage 1. fill accumulator
    for( i = 0; i < (int)height; i++ )
        for( j = 0; j < (int)width; j+=2 )
        {
			int id = image_index(j,i);
			
			if( image[id+1] != 0 )
			{
                for(int n = 0; n < numangle; n++ )
                {
                    int r = (j>>1) * tabCos[n] + i * tabSin[n];		//j/2 because uyvy image
                    r += (numrho - 1) / 2;
                    accum[(n+1) * (numrho+2) + r+1]++;					
                }
			}				
        }		

    // stage 2. find local maximums
    for(int r = 0; r < numrho; r++ )
        for(int n = 0; n < numangle; n++ )
        {
            int base = (n+1) * (numrho+2) + r+1;
            if( accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2] )
                sort_buf[total++] = base;
				
        }
		
    // stage 4. store the first min(total,linesMax) lines to the output buffer   
   qsort(sort_buf, total, sizeof(int), cmpfunc);

	
    scale = 1./(numrho+2);
    for( i = 0; i < linesMax && i < total; i++ )
    {        
        int idx = sort_buf[i];
        int n = idx*scale - 1;
        int r = idx - (n+1)*(numrho+2) - 1;
        rhos_out[i] = (r - (numrho - 1)*0.5f) * rho;	//length
        thetas_out[i] = n * theta;    //angle

		
		g_print("%d;%.2f;%d \n",rhos_out[i] ,thetas_out[i],accum[idx]);
    }
	g_print(" nLines: %d\n\n",total);
	//g_print("1/%d line: %d;%.2f;%d  \n ",total,rhos_out[0] ,thetas_out[0],accum[sort_buf[0]]);
	
	
	//if (accum[0] > 150)
	
	if (total > 1) {
		int foundcorner = 0;
		int falsepositives = 0;
		for (i = 1; i<linesMax;i++) {
		
			if (i < total ) {
					float diff = fabs(thetas_out[0] -thetas_out[i]);
					if (!(diff < 0.2 || diff > 2.94)) {
						if ((diff > 0.5 && diff < 2.5)) {
							g_print("Corner detected? %d; %d;%.2f;%d \n",i,rhos_out[i],thetas_out[i],accum[sort_buf[i]]);
							foundcorner ++;
						} else {
							falsepositives++;
						}
					}	 					
			}	
		}
		if (!foundcorner) {
			g_print("\n");
			/*use theta to compensate Yaw
			* theta can flip between 0 / pi,
			* use length to compensate x/y
			* 
			*/
			
			
			//
			
			
			
			
			
			}
			
			
	} else {
		g_print("\n");
	}
	
	
	
	free(tabSin);
	free(tabCos);
	free(sort_buf);
	free(accum);
	
	
}

void *line_threat( void *ptr) {

	int linesMax = 50;
	int * rhos_out = (int *) calloc(linesMax,sizeof(int));
	float * thetas_out = (float *) calloc(linesMax,sizeof(float));
		

	while (1) {
			
		if (!new_line_im) {
			icvHoughLinesStandard( img_line, imgWidth, imgHeight, 4, 3.14/180/4,100, linesMax , rhos_out, thetas_out);
		
			new_line_im =1;
		} else {		
			usleep(1000); // TODO: replace by mutex
		}

	}
	
	free(rhos_out);
	free(thetas_out);			
		



}

int cmpfunc (const void * a, const void * b)
{
int id1 = *(int*)a;
int id2 = *(int*)b;



   return ( accum[id2]-accum[id1] );
}

int image_index(int xx, int yy) {
	return ((yy * imgWidth + xx) * 2) & 0xFFFFFFFE; // always a multiple of 2
}

int image_index4(int xx, int yy) {
	return ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC; // always a multiple of 4
}
