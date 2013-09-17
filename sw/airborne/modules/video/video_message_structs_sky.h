#ifndef VMSTRTS_H
#define VMSTRTS_H

//an exact copy of this file to exist in ppz
//this files keeps the structs that are serialized and streamed over tcp/ip through localhost

struct gst2ppz_message_struct_sky {
	unsigned int counter;		//counter to keep track of data
	int optic_flow_x;		//optical flow output, attitude+altitude compensated shift in x direction
	int optic_flow_y;		//optical flow output, attitude+altitude shift in y direction
	
	int optic_flow_rot_pitch;		//optical flow output, rotation in x direction
	int optic_flow_rot_roll;		//optical flow output, shift in y direction
	
	int optic_flow_imu_pitch;		//imu output coupled to optic flow frames
	int optic_flow_imu_roll;		//imu output coupled to optic flow frames
	

};
extern struct gst2ppz_message_struct_sky gst2ppz;

struct ppz2gst_message_struct_sky {
	unsigned int counter;		//counter to keep track of data
	int pitch;
	int roll;
	int alt; // sonar altitude
	unsigned char minU_orange;
	unsigned char maxU_orange;
	unsigned char minV_orange;
	unsigned char maxV_orange;

	unsigned char minU_blue;
	unsigned char maxU_blue;
	unsigned char minV_blue;
	unsigned char maxV_blue;
	
	unsigned char min_gradient;
	
};
extern struct ppz2gst_message_struct_sky ppz2gst;

#endif  /*  VMSTRTS_H  */

