std_msgs/Header header
sensor_msgs/PointCloud2 TargetPointcloud
bool UseBoundingVolume
geometry_msgs/Transform BoundingVolumeOrigin
geometry_msgs/Vector3 BoundingVolumeDimensions

uint8 POINTCLOUD=0
uint8 POINTCLOUDFILE=1
uint8 MESH=2
uint8 SHAPE=3
uint8 InputType

sensor_msgs/PointCloud2 InputPointCloud
string InputResource
# For InputType == MESH
float64 InputMeshScaling
# For InputType == SHAPE
geometry_msgs/Vector3 Discretization
shape_msgs/SolidPrimitive InputShape
# For all input types
geometry_msgs/Transform InputTransform

int32 MaxICPIterations
float64 MaxICPCoorespondenceDistance

