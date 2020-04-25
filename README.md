# yammar_ws工作空间
智能机器人实验室-洋马项目组

# 编译要求：
1. 编译涉及HMI的Qt节点时，需要已安装Qt5库。

# 文件说明：
1. qt_gui_test是目前使用的HMI文件，qtros为相关的模板
2. launch_file程序包主要用来放launch文件
3. 

# 协作信息
===
王海力：
1. 目前将节点的topic接口名称定下来了，但是topic的类型都仅使用了std_msg::String，请各位自行按需更改。
2. 我本周的主要工作放在action的编写。

===
姜文俊：

===
杨正涛：
1. 大陆ARS404毫米波雷达ROS驱动上传mmw_capture文件夹，修改node为：“mmw_capture”, topic为“millimeter_wave”格式：sensor_msgs::PointCloud2

===
孙晗：

