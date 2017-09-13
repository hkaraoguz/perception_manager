uint32 id # id of the object
int32 pixelposcenterx  # x axis pixel position of the object center
int32 pixelposcentery  # y axis pixel position of the object center
int32 metricposcenterx  # x axis metric position of the object center w.r.t robot coordinates
int32 metricposcentery  # y axis metric position of the object center w.r.t robot coordinates
sensor_msgs/Image image # image of the segment
geometry_msgs/Point32[] pixelhull # the convex hull of the object in metric coordinates
int8 averagehue # the average hue value of the segment
float32 angle # the angle of the object
