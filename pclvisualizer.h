#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtGui>
#include <QColorDialog>
#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_representation.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>


#include "ui_pclvisualizer.h"

//实现调用自带窗口的新包含
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "ROVT.h"

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointT_XYZ;
//typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT_XYZ> PointCloudT_XYZ;


class PCLVisualizer : public QMainWindow
{
	Q_OBJECT
public:
	PCLVisualizer(QWidget *parent = 0);
	~PCLVisualizer();

private:
	Ui::PCLVisualizerClass ui;
	//点云数据存储
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgs_source;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgs_target;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgs_icp;
	long total_point = 0;
	Eigen::Vector4f centroid;
	std::string subname;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//创建窗口的指针
	//QDialog*pDialog = new QDialog(this, Qt::Dialog);
	//初始化vtk部件
	void initialVtkWidget();

private slots:
	//创建打开槽
	void onOpen();
	void onExit();
	void onRemove();
	void onCovert();

	void onPassthroughx();
	void onPassthroughy();
	void onPassthroughz();

	void onButtonPassthrough();
	void onButtonVoxelGrid();
	void onButtonOutlier();
	void onButtonGreedyProjection();
	void onPlanarSegmentation();
	void onCyliderSegmentation();
	void onTranslation();

	void onOpenSource();
	void onOpenTarget();
	void onNormalDisTrans();
	void onRgsDispaly();

	void onIterativeClosestPoint();

	void passthrough();
	void setPropertyTable();
	void setConsoleTable();
	void Consolelog(QString cloud, QString operation, QString details);


	void actionConsole();
	void actionProperty();
	void actionTools();
	void onSave();
	void onOpencloud();

	void mainview();
	void leftview();
	void topview();

	void backview();
	void rightview();
	void bottomview();

	void backgroundcolor();

	void generatecube();
	void generatesphere();
	void generatecylinder();

};

#endif // PCLVISUALIZER_H