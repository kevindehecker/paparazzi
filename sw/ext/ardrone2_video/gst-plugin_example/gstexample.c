
/**
 * SECTION:element-example
 *
 * The Example plugin for gstreamer will show how to make a plugin. It will output the maximum brightness of the image.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=5/1' ! videoscale ! video/x-raw-yuv, width=640, height=368 ! example ! fakesink

 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gst/gst.h>
#include <stdlib.h>     /* calloc, exit, free */
#include <pthread.h>
#include <unistd.h>	//usleep

#include "gstexample.h"
#include "socket.h"
#include "video_message_structs.h"
#include <stdio.h>
#include <stdlib.h>

#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2)
unsigned int imgWidth, imgHeight;
unsigned int tcpport;
unsigned char threshtune;
unsigned int counter;
unsigned int socketIsReady;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;




void brightspotDetector(unsigned char *frame_buf, unsigned int * max_idx,unsigned int * max_idy) ;
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist);
unsigned char getThreshold(unsigned int * OneDHist);
void createBinaryImage(unsigned char threshold, unsigned char * frame_buf);
void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y);
unsigned int cmpfunc (const void * a, const void * b);
unsigned int getMedian(unsigned int * hist,  unsigned int size);


void *TCP_threat( void *ptr);

GST_DEBUG_CATEGORY_STATIC (gst_example_debug);
#define GST_CAT_DEFAULT gst_example_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

enum
{
  PROP_0,
  PROP_SILENT,
  PROP_TCP,
  PROP_THRESHTUNE
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("ANY")
    );

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("ANY")
    );

GST_BOILERPLATE (Gstexample, gst_example, GstElement,
    GST_TYPE_ELEMENT);

static void gst_example_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_example_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_example_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_example_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_example_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
    "example",
    "Passthrough element",
    "Calculates stuff on the video, to be fed to an autopilot",
    "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the example's class */
static void
gst_example_class_init (GstexampleClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;
 // gstelement_class = (GstElementClass *) klass;

  gobject_class->set_property = gst_example_set_property;
  gobject_class->get_property = gst_example_get_property;

  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
          FALSE, G_PARAM_READWRITE));
		  
  g_object_class_install_property (gobject_class, PROP_TCP,
      g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
          0, G_PARAM_READWRITE));	

  g_object_class_install_property (gobject_class, PROP_THRESHTUNE,
      g_param_spec_uint ("threshtune", "threshtune tune", "Changes output of binary image function",0,65535,
          0, G_PARAM_READWRITE));		  
		  
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_example_init (Gstexample * filter,
    GstexampleClass * gclass)
{
	
  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_example_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
                              GST_DEBUG_FUNCPTR(gst_example_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
                                GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;
    
}

static void
gst_example_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstexample *filter = GST_EXAMPLE (object);

  switch (prop_id) {
    case PROP_SILENT:
      filter->silent = g_value_get_boolean (value);
      break;	  
    case PROP_TCP:
      tcpport = g_value_get_uint (value);
      break;	
    case PROP_THRESHTUNE:
      threshtune = g_value_get_uint (value);
      break;	  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_example_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstexample *filter = GST_EXAMPLE (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, filter->silent);
      break;
	case PROP_TCP:
      g_value_set_uint (value, tcpport);
      break;
	case PROP_THRESHTUNE:
      g_value_set_uint (value, threshtune);
      break;	  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_example_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstexample *filter;
  GstPad *otherpad;

  filter = GST_EXAMPLE (gst_pad_get_parent (pad));
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
	counter =0;
	
	//ppz2gst = (ppz2gst_message_struct *) malloc(sizeof(ppz2gst));

  //initialise socket:

	if (tcpport>0) {
		//start seperate threat to connect
		//seperate threat is needed because otherwise big delays can exist in the init or chain function
		//causing the gst to crash
	
		pthread_t th1;
		int th1_r;
		pthread_create(&th1,NULL,TCP_threat,&th1_r);
		//pthread_join(th1,NULL);

	}
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
		if	(res>0) {
			g_print("Wow, stress!!! Data back: %d\n",ppz2gst.heading);
		} else {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}



}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_example_chain (GstPad * pad, GstBuffer * buf)
{
	Gstexample *filter;

	filter = GST_EXAMPLE (GST_OBJECT_PARENT (pad));

	unsigned char * img = GST_BUFFER_DATA(buf);   
	unsigned int max_idx, max_idy;
	unsigned char maxY;
	brightspotDetector(img,&max_idx,&max_idy);
	
		
		//char * tmp = calloc(64,sizeof(char));
		//sprintf(tmp, "MaxY;%d;id_x;%d;id_y;%d;count;%d\n",maxY,max_idx,max_idy,counter);
		if (filter->silent == FALSE) {	
		//	g_print("%s", tmp);
		}
		if (tcpport>0) { 	//if network was enabled by user
			if (socketIsReady) { 
				if (filter->silent == FALSE) {	
					g_print("Sending data to ppz@port %d\n", tcpport);
				}
				gst2ppz.maxY = maxY;
				gst2ppz.max_idx = max_idx;
				gst2ppz.max_idy = max_idy;
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
				
			}

		}
		//free(tmp);
	counter++;
	
	  
  return gst_pad_push (filter->srcpad, buf);
}




void brightspotDetector(unsigned char *frame_buf, unsigned int * max_idx,unsigned int * max_idy) 
{
	unsigned char thresh;
	unsigned int * OneDHist =(unsigned int *) calloc(256,sizeof(unsigned int));
	unsigned int * hist_x =(unsigned int *) calloc(imgWidth,sizeof(unsigned int));
	unsigned int * hist_y = (unsigned int *)calloc(imgHeight,sizeof(unsigned int));
	get1DHist(frame_buf,OneDHist);

	
	thresh = getThreshold(OneDHist);
	//printf("Thresh: %d\n",thresh);
	createBinaryImage(thresh,frame_buf);
	
	
	get2DHist(frame_buf,hist_x,hist_y);
	*max_idx = getMedian(hist_x,imgWidth);
	*max_idy = getMedian(hist_y,imgHeight);
	printf("max x: %d, max y: %d\n",*max_idx,*max_idy);
	
	
	free(OneDHist);
	free(hist_x);
	free(hist_y);
}
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist) {
	unsigned int ix;
	unsigned int color_channels = 4;
	unsigned int step = 1 * color_channels;
	
	
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 		
        OneDHist[frame_buf[ix+1]]++;
		OneDHist[frame_buf[ix+3]]++;		
    }
/*
	for (ix=0;ix<256;ix++) {
		printf(" %d, ",OneDHist[ix]); 
	}
	printf("\n");
	printf("\n");

*/
} 

unsigned char getThreshold(unsigned int * OneDHist) {

	unsigned int total= (unsigned int)((float)(imgWidth*imgHeight)*((float)threshtune/100));
	unsigned char i;
	unsigned int tmptotal = 0;
	
	//printf("Total: %d\n",total);
	
	for (i = 0; i<255; i++) {
		tmptotal+=OneDHist[i];
		if (tmptotal> total)
			return i;
	}
	return 0;
}

void createBinaryImage(unsigned char threshold, unsigned char * frame_buf) {
	unsigned int ix;
	unsigned int color_channels = 4;
	unsigned int step = 1 * color_channels;
		
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 		
		//frame_buf[ix] = 0;
		//frame_buf[ix+2] = 0;
        if (frame_buf[ix+1] < threshold)
			frame_buf[ix+1] = 0;
		else
			frame_buf[ix+1] = 255;
        if (frame_buf[ix+3] < threshold)
			frame_buf[ix+3] = 0;	
		else
			frame_buf[ix+3] = 255;			
    }

}

void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y) {
	unsigned int x,y,ix;
		
	for (x=0; x<(imgWidth); x++)
	{
		unsigned int tmpsum = 0;
		for (y=0; y<(imgHeight); y++)
		{ 
			ix = image_index(x,y);
			tmpsum+=frame_buf[ix+1] > 0;
			//printf(" %d, ",ix); 
		}
	//	printf(" %d, ",tmpsum); 
		hist_x[x] = tmpsum;
		
	}
	printf("\n\n"); 
	//TODO: optimize loop below to integrate with loop above...
	for (y=0; y<(imgHeight); y++)
	{
		unsigned int tmpsum = 0;
		for (x=0; x<(imgWidth); x++)
		{ 
			tmpsum+=frame_buf[image_index(x,y) +1] > 0;
		}
		hist_y[y] = tmpsum;
	}
	
	/*
	for (ix=0;ix<imgWidth;ix++) {
		printf(" %d, ",hist_x[ix]); 
	}
	printf("\n");
	printf("\n");
	
	*/
	
	
}
unsigned int cmpfunc (const void * a, const void * b)
{
   return ( *(unsigned int*)a - *(unsigned int*)b );
}
unsigned int getMedian(unsigned int * hist,  unsigned int size) {
	
	unsigned int total = 0;
	unsigned int tmptotal = 0;
	
	for (unsigned int i = 0 ; i< size; i++) {	
		total+=hist[i];	
	}
	
	for (unsigned int i = 0 ; i< size; i++) {
		tmptotal+=hist[i];
	if (tmptotal>total/2) 
		return i;
	}
	return 0;	
}

void getxy(unsigned int max_y_ix, unsigned int * max_idx, unsigned int * max_idy) {	
	max_y_ix/=2;
	*max_idy = (max_y_ix / imgWidth);
	*max_idx = (max_y_ix) - *max_idy*imgWidth;
}






/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
example_init (GstPlugin * example)
{
  /* debug category for fltering log messages
   *
   * exchange the string 'Template example' with your description
   */   
	 
  GST_DEBUG_CATEGORY_INIT (gst_example_debug, "example",
      0, "The Example plugin for gstreamer will output the maximum brightness of the frames flowing by");

  return gst_element_register (example, "example", GST_RANK_NONE,
      GST_TYPE_EXAMPLE);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "Example"
#endif

/* gstreamer looks for this structure to register examples
 *
 * exchange the string 'Template example' with your example description
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "example",
    "The Example plugin for gstreamer will output the maximum brightness of the frames flowing by",
    example_init,
    VERSION,
    "LGPL",
    "Example",
    "http://gstreamer.net/"
)
