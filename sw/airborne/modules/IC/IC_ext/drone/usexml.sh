set -ex
id=$1

cd ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/
tar -xf data_up.tar


cd ~/paparazzi/sw/airborne/modules/IC/IC_ext/pc/
cp ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/data/video/drone/distribution_buffer${id}.xml distribution_buffer.xml
cp ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/data/video/drone/graph_buffer${id}.xml  graph_buffer.xml
cp ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/data/video/drone/groundtruth_buffer${id}.xml groundtruth_buffer.xml
cp ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/data/video/drone/distribution_buf_pointer${id}.xml distribution_buf_pointer.xml
cp ~/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/tmp/data/video/drone/distribution_buf_size${id}.xml distribution_buf_size.xml
