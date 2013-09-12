void gps_show();
int gps_parse();
unsigned char read_ublox();
int cos_zelf(int ix);
int sin_zelf(int);
int tan_zelf(int);
int acos_zelf(int, int);
int asin_zelf(int, int);
int atan_zelf(int, int);
int gps_head_zelf(int, int, int, int);
int gps_dist_zelf(int, int, int, int);
unsigned int isqrt_zelf(unsigned int);

typedef struct gps_data {
    int lat;  // lat degrees x 10^6
    int lon;  // lon degrees x 10^6
    int alt;
    int fix;
    int sat;
    int utc;
} gps_data;

extern struct gps_data gps_gga;

