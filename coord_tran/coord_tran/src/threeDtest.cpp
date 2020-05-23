#include "ros/ros.h"
#include "CoordinateTran.h"


//pcl::visualization::CloudViewer viewer1("Cloud in box after filter");//创建一个可视化窗口
//pcl::visualization::CloudViewer viewer2("Cloud in box after removing plane");
int main (){
  while(1){
 
     std::cout << "fresh "  << std::endl; //*
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("/home/qiuyilin/catkin_ws/cloud_cluster_0.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*


               
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  
  

  if(cloud->size() > 0){
  
   //viewer1.showCloud(cloud);
    //清楚无效值
    cloud->is_dense = 0;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    //统计滤波
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Static;
    Static.setInputCloud(cloud);
    Static.setMeanK(100);//邻居点数目k
    Static.setStddevMulThresh(2);//比例系数α  判断是否为离群点的阈值
    Static.filter(*cloud_filtered);

  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  
  // 设置点云分割参数
  //创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //创建分割时所需要的模型系数对象，存储内点的点索引集合对象inliers和coefficients
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  //可选择配置,设置模型系数需要优化
  seg.setOptimizeCoefficients (true);
  //必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阈值，输入点云
  seg.setModelType (pcl::SACMODEL_PLANE);//设置模型类型
  seg.setMethodType (pcl::SAC_RANSAC);//设置随机采样一致性方法类型
  seg.setMaxIterations (100);//最大迭代次数
  seg.setDistanceThreshold (0.002);//距离阈值，点被认为是局内点必须满足的条件  多厚的面

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  
  
  //采用平面分割模型对点云进行分割处理 提取点云中所有在平面上的点集
  std::cout << "cloud_filtered->points.size " << cloud_filtered->points.size ()<< " nr_points"<< nr_points <<std::endl;
  while (cloud_filtered->points.size () > 0.3 * nr_points)//只能去除七成以内的点
  {
    // Segment the largest planar component from the remaining cloud 分离出最大的那个平面组件
    //触发分割
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)//没有分割出新模型则跳出
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
   std::cout << "get in" << std::endl;
    // 从点云中抽取分割的处在平面上的点集
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  //viewer2.showCloud(cloud_filtered);
  
  // 为提取点云时使用的搜索对象利用输入点云cloud_filtered创建Kd树对象tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;//存储实际点云索引信息
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   ec.setClusterTolerance (0.02); // 单位是m
   ec.setMinClusterSize (100); //设置一个聚类需要的最少点数目为100
   ec.setMaxClusterSize (25000); //设置一个聚类需要的最大点数目为25000
   ec.setSearchMethod (tree);  //设置点云的搜索机制
   ec.setInputCloud (cloud_filtered); 
   ec.extract (cluster_indices); //从点云中提取聚类，将点云索引保存在cluster_indices中

  //迭代访问点云索引cluster_indices，直到分割出所有聚类
  int j = 0;
  std::vector<float> min_distance_2s;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //创建新的点云数据集cloud_cluster,将所有当前聚类写入到点云数据集中
    float min_distance_2 = 1000000;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
    //vector<float> min_distances;
    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
     //区分出哪个才是需要的那个聚类
     //取每个聚类离中心点最近的点  可以用kd树改写
    float distance_2 =pow( (cloud_filtered->points[*pit].x - 320),2 ) + pow( (cloud_filtered->points[*pit].y - 240),2 );
    if (distance_2< min_distance_2)
      min_distance_2 = distance_2;
    }
    min_distance_2s.push_back(min_distance_2);
   
    
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  // 在这些点中最小的聚类为需要的
   auto smallest = std::min_element(std::begin(min_distance_2s), std::end(min_distance_2s));
   int min_position = std::distance(std::begin(min_distance_2s), smallest) ;
   std::cout << "min element is " << *smallest<< " at position " << min_position<< std::endl;
   
  
  
  
  //最小包围盒
    std::stringstream ss2;
    ss2 << "/home/qiuyilin/catkin_ws/cloud_cluster_0.pcd";
    reader.read (ss2.str (), *cloud_final);
    
    //viewer.showCloud(cloud_final);
  


  
  
  //最小包围盒
//     1) compute the centroid (c0, c1, c2) and the normalized covariance
//     2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
//     3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
//     4) compute the max, the min and the center of the diagonal
//     5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)
  // start calculating time
    pcl::StopWatch time;
 
    //基于PCA主元分析法计算主轴方向
    //利用PCA主元分析法获得点云的三个主方向， 获取质心，计算协方差，获得协方差矩阵， 求取协方差矩阵的特征值和特长向量，特征向量即为主方向。
    
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_final, pcaCentroid);//计算重心
    Eigen::Matrix3f covariance;//定义协方差
    pcl::computeCovarianceMatrixNormalized(*cloud_final, pcaCentroid, covariance);//计算点云矩阵协方差
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
    
    float x1 = eigenVectorsPCA(0,2) ;
    float y1 = eigenVectorsPCA(1,2);
    //将二、三象限的向量换个方向
    float theta=asin(y1/sqrt(x1*x1+y1*y1))* 180 / 3.14;
    
     std::cout << "theta" << theta << std::endl;
    std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
    std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
    std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
    //利用1中获得的主方向和质心，将输入点云转换至原点，且主方向与坐标系方向重回，建立变换到原点的点云的包围盒。
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();//
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
    tm_inv = tm.inverse();
 
    std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
    std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_final, *transformedCloud, tm);//点云转换至原点，且主方向与坐标系方向重合
 
    pcl::PointXYZ min_p1, max_p1;
    Eigen::Vector3f c1, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);//给定点云的坐标范围
    c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());//型心 该点的内部作为Eigen :: Vector类型
 
    std::cout << "型心c1(3x1):\n" << c1 << std::endl;
 
    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);
 
    Eigen::Vector3f whd, whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    whd = whd1;
    float width,height;
    float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小（xyz）
    if(whd1(0)>whd1(1)){
      width= whd1(1);
      height=whd1(0);
    }
    else{
      width= whd1(0);
      height=whd1(1);     
    }
    std::cout << "宽=" << width << endl;//宽
    std::cout << "长=" <<  height << endl;//长
    std::cout << "depth1=" << whd1(2) << endl;//高
    std::cout << "scale1=" << sc1 << endl;
 

    //计算出单位四元数 确定点云姿态 旋转角度
    const Eigen::Quaternionf bboxQ(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxT= eigenVectorsPCA * c1 + pcaCentroid.head<3>();//或者 const Eigen::Vector3f    bboxT(c);
    //或者
    //     const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
    //     const Eigen::Vector3f    bboxT(c);
    


    
    //初始点云的主方向
    pcl::PointXYZ cp;
    cp.x = pcaCentroid(0);
    cp.y = pcaCentroid(1);
    cp.z = pcaCentroid(2);
    pcl::PointXYZ pcX;
    pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
    pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
    pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
    pcl::PointXYZ pcY;
    pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
    pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
    pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
    pcl::PointXYZ pcZ;
    pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
    pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
    pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;


   //Rectangular vertex 
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr transVertexCloud(new pcl::PointCloud<pcl::PointXYZ>);//存放变换后点云包围盒的6个顶点
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr VertexCloud(new pcl::PointCloud<pcl::PointXYZ>);//存放原来点云中包围盒的6个顶点
// 	transVertexCloud->width = 6;  
// 	transVertexCloud->height = 1;     
// 	transVertexCloud->is_dense = false;  
// 	transVertexCloud->points.resize(transVertexCloud->width * transVertexCloud->height);  
// 	transVertexCloud->points[0].x = max_p1.x;
// 	transVertexCloud->points[0].y = max_p1.y;
// 	transVertexCloud->points[0].z = max_p1.z;
// 	transVertexCloud->points[1].x = max_p1.x;
// 	transVertexCloud->points[1].y = max_p1.y;
// 	transVertexCloud->points[1].z = min_p1.z;
// 	transVertexCloud->points[2].x = max_p1.x;
// 	transVertexCloud->points[2].y = min_p1.y;
// 	transVertexCloud->points[2].z = min_p1.z;
// 	transVertexCloud->points[3].x = min_p1.x;
// 	transVertexCloud->points[3].y = max_p1.y;
// 	transVertexCloud->points[3].z = max_p1.z;
// 	transVertexCloud->points[4].x = min_p1.x;
// 	transVertexCloud->points[4].y = min_p1.y;
// 	transVertexCloud->points[4].z = max_p1.z;
// 	transVertexCloud->points[5].x = min_p1.x;
// 	transVertexCloud->points[5].y = min_p1.y;
// 	transVertexCloud->points[5].z = min_p1.z;
// 	pcl::transformPointCloud(*transVertexCloud, *VertexCloud, tm_inv);//6顶点点云
	
	// 逆变换回来的角度
	cout << whd1(0) << " "<< whd1(1) << " " << whd1(2) << endl;
	auto euler = bboxQ.toRotationMatrix().eulerAngles(0, 1, 2); //单位四元数转换为旋转矩阵
	std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler/3.14*180 << std::endl<<std::endl;
	
	//Output time consumption 
	std::cout << "运行时间" << time.getTime() << "ms" << std::endl;
 
    //visualization
    pcl::visualization::PCLVisualizer viewer2;
 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_final, 125, 125, 125);  //设置点云颜色
    viewer2.addPointCloud(cloud_final, color_handler, "cloud_final");//添加点云
    viewer2.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");//平移矩阵 单位四元数 长 宽 高
    viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
    viewer2.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
 
    viewer2.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
    viewer2.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
    viewer2.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
 
    viewer2.addCoordinateSystem(0.5f*sc1);
    viewer2.setBackgroundColor(0.0, 0.0, 0.0);
    

 
//     viewer2.addPointCloud(VertexCloud, "temp_cloud");
//     viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "temp_cloud");
    while (!viewer2.wasStopped())
    {
          viewer2.spinOnce();
    }
  }
    else 
  std::cout << "no data" << std::endl;
  }
  return 0;
  
    
}

