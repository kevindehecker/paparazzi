#ifndef VMSTRT_H
#define VMSTRT_H

//an exact copy of this file to exist in ppz
//this files keeps the structs that are serialized and streamed over tcp/ip through localhost

struct gst2ppz_message_struct {
  unsigned int maxY;		//maximum brightness
  unsigned int max_idx;		//x coordinate of brightest pixel
  unsigned int max_idy;		//y .....
  unsigned int counter;		//counter to keep track of data  
};
extern struct gst2ppz_message_struct gst2ppz;

struct ppz2gst_message_struct {
  unsigned int heading; 
};
extern struct ppz2gst_message_struct ppz2gst;

#endif  /*  VMSTRT_H  */

