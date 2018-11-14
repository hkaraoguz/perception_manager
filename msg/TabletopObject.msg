int32 id # id of the object
int32 pixelposcenterx  # x axis pixel position of the object center w.r.t kinect frame
int32 pixelposcentery  # y axis pixel position of the object center w.r.t kinect frame
int32 pixelpostablecenterx  # x axis pixel position of the object center w.r.t table corner
int32 pixelpostablecentery  # y axis pixel position of the object center w.r.t table corner
float32 metricposcenterx  # x axis metric position of the object center w.r.t base coordinates
float32 metricposcentery  # y axis metric position of the object center w.r.t base coordinates
float32 metricposcenterz  # z axis metric position of the object center w.r.t base coordinates
float32 metricpostablecenterx  # x axis metric position of the object center w.r.t table corner
float32 metricpostablecentery  # y axis metric position of the object center w.r.t table corner
sensor_msgs/Image image # image of the segment
geometry_msgs/Point32[] pixelhull # the convex hull of the object in pixel coordinates
#geometry_msgs/Point32[] metrichull # the convex hull of the object in metric coordinates
float32 width
float32 length
int32 averagehue # the average hue value of the segment
float32 angle # the angle of the object
