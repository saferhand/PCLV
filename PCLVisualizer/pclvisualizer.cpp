#include <QFileDialog>
#include <iostream>
#include <vtkRenderWindow.h>
#include "pclvisualizer.h"
#include "Extern.h"

PCLVisualizer::PCLVisualizer(QWidget *parent)
: QMainWindow(parent)
{
	ui.setupUi(this);
	//初始化
	initialVtkWidget();
	//连接信号和槽
	connect(ui.actionOpen,    SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionExit,    SIGNAL(triggered()), this, SLOT(onExit()));
	connect(ui.actionRemove,  SIGNAL(triggered()), this, SLOT(onRemove()));
	connect(ui.actionCovert,  SIGNAL(triggered()), this, SLOT(onCovert()));
	connect(ui.actionPx,      SIGNAL(triggered()), this, SLOT(onPassthroughx()));
	connect(ui.actionPy,      SIGNAL(triggered()), this, SLOT(onPassthroughy()));
	connect(ui.actionPz,      SIGNAL(triggered()), this, SLOT(onPassthroughz()));
	connect(ui.actionSave,    SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionOpencloud, SIGNAL(triggered()), this, SLOT(onOpencloud()));
	connect(ui.actionOpen_source_pcd, SIGNAL(triggered()), this, SLOT(onOpenSource()));
	connect(ui.actionOpen_target_pcd, SIGNAL(triggered()), this, SLOT(onOpenTarget()));
	connect(ui.consoleaction, SIGNAL(triggered()), this, SLOT(actionConsole()));
	connect(ui.propertyaction, SIGNAL(triggered()), this, SLOT(actionProperty()));
	connect(ui.toolaction, SIGNAL(triggered()), this, SLOT(actionTools()));
	connect(ui.actionFrontView, SIGNAL(triggered()), this, SLOT(mainview()));
	connect(ui.actionTopView, SIGNAL(triggered()), this, SLOT(topview()));
	connect(ui.actionLeftView, SIGNAL(triggered()), this, SLOT(leftview()));
	connect(ui.actionBackView, SIGNAL(triggered()), this, SLOT(backview()));
	connect(ui.actionRightView, SIGNAL(triggered()), this, SLOT(rightview()));
	connect(ui.actionBottomView, SIGNAL(triggered()), this, SLOT(bottomview()));
	connect(ui.actionBackgroundColor, SIGNAL(triggered()), this, SLOT(backgroundcolor()));
	connect(ui.actionCube, SIGNAL(triggered()), this, SLOT(generatecube()));
	connect(ui.actionCylider, SIGNAL(triggered()), this, SLOT(generatecylinder()));
	connect(ui.actionSphere, SIGNAL(triggered()), this, SLOT(generatesphere()));
}

PCLVisualizer::~PCLVisualizer()
{

}


/*
//新建函数内容
void PCLVisualizer::viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;
}
void PCLVisualizer::viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	PCLVisualizer::user_data++;
}*/

void PCLVisualizer::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_rgs_target.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_rgs_source.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_rgs_icp.reset(new pcl::PointCloud<pcl::PointXYZ>);


	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");

	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}




//读取文本型和二进制型点云数据
void PCLVisualizer::onOpen()
{
	//fileName返回文件名，窗口名，默认目录，打开文件类型
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Point Cloud"), ".", tr("Point Cloud Data (*.pcd)"));
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());

	std::string file_name = filename.toStdString();

	std::string subnametrue;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subnametrue.insert(subnametrue.begin(), *i);
	}
	subname = subnametrue;

	//std::string subname = getFileName(file_name);
	PointCloudT_XYZ::Ptr cloud_tmp(new PointCloudT_XYZ);
	total_point = 0;
	if (filename.isEmpty())
	return;
	int return_status;


	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);


	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}
	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);

	}

	total_point = cloud->points.size();
	pcl::compute3DCentroid(*cloud, centroid);

	viewer->addPointCloud(cloud, "cloud");
	viewer->updatePointCloud(cloud, "cloud");
	//viewer->setBackgroundColor(255,255,0);
	viewer->resetCamera();
	ui.qvtkWidget->update();
	setPropertyTable();
	setConsoleTable();
	Consolelog(QString::fromLocal8Bit(subname.c_str()), "Open", "Open point cloud for following processing!");
}
void PCLVisualizer::onCovert()
{
	QString fileName1 = QFileDialog::getOpenFileName(this, tr("Open PointCloud"), ".", tr("Open PCD files(*.txt)"));

	std::string file_name = fileName1.toStdString();

	std::string subnametrue;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subnametrue.insert(subnametrue.begin(), *i);
	}
	subname = subnametrue;

	QByteArray ba = fileName1.toLatin1();
	char *Dataname = ba.data();
	ifstream fileinput;
	ofstream fileoutput;

	fileinput.open(Dataname);
	fileoutput.open("output.pcd");

	double data[10000][9];
	long row = 0;
	long col = 1;
	double b, num;
	double x, y, z;
	while (!fileinput.eof())
	{
		for (col = 1; col <= 8; col++)
		{
			fileinput >> data[row][col];
		}
		row++;
	}
	fileinput.close();
	//write file header
	//Point cloud .pcd Version 7.0
	fileoutput << "# .PCD v0.7 - Point Cloud Data file format\n";
	fileoutput << "VERSION 0.7\n";
	//Point type x,y,z
	fileoutput << "FIELDS x y z\n";
	//Data bit size(char 1)(short 2)(int float 4)(double 8)
	fileoutput << "SIZE 4 4 4\n";
	//Data type (char short int I)(unsigned U) (float double F)
	fileoutput << "TYPE F F F\n";
	//Number of elements per dimension
	fileoutput << "COUNT 1 1 1\n";
	//Points number
	fileoutput << "WIDTH " << row << "\n";
	//Unordered cloud points
	fileoutput << "HEIGHT 1\n";
	//Viewpoint = trans + quaternion
	fileoutput << "VIEWPOINT 0 0 0 1 0 0 0\n";
	//Points number
	fileoutput << "POINTS " << row << "\n";
	//ascii or mmap
	fileoutput << "DATA ascii\n";

	//Calculate and output
	//	Xg Yg Zg alpha omega kappa distance theta
	//  1  2  3  4     5     6     7        8
	row = 0;
	fileinput.open(Dataname);
	while (!fileinput.eof())
	{
		//read data of one row the calculate the coorinate of each row
		for (col = 1; col <= 8; col++)
		{
			fileinput >> data[row][col];
		}
		//calculate the coordinate X Y Z (trigonometric radian)
		//b
		b = cos(data[row][5])*sin(data[row][4])*cos(data[row][6]) + sin(data[row][6])*sin(data[row][5]);
		//num
		num = data[row][7] * sin(data[row][8]) / sqrt(1 - b*b);
		//x
		x = data[row][1] + (data[row][7] * cos(data[row][8]) - num*b)*cos(data[row][5])*sin(data[row][4]) + num*cos(data[row][6]);
		//y
		y = data[row][2] + (data[row][7] * cos(data[row][8]) - num*b)*sin(data[row][5]) + num*sin(data[row][6]);
		//z
		z = data[row][3] + (data[row][7] * cos(data[row][8]) - num*b)*cos(data[row][5])*cos(data[row][4]);

		fileoutput << x << " " << y << " " << z << " ";
		fileoutput << "\n";
	}
	fileinput.close();
	fileoutput.close();
	Consolelog(QString::fromLocal8Bit(subname.c_str()), "Covert", "Have convert the file to pcd!");
}
void PCLVisualizer::onRemove()
{
	viewer->removeAllPointClouds();
	viewer->resetCamera();
	ui.qvtkWidget->update();
	Consolelog("NULL", "Remove", "Remove the point cloud in window!");
}
void PCLVisualizer::onExit()
{
	exit(0);
}
void PCLVisualizer::onPassthroughx()
{
	//QDialog *dialog = new QDialog;
	//dialog->show();

		QDialog*pDialog = new QDialog(this, Qt::Dialog);
		pDialog->setFixedSize(160, 130);

		QPushButton *buttonok = new QPushButton(pDialog);
		buttonok->setObjectName(QStringLiteral("OK"));
		buttonok->setGeometry(QRect(15, 100, 50, 20));
		buttonok->setText("OK");
		//connect(buttonok, SIGNAL(clicked()), pDialog, SLOT(passthrough()));
		connect(buttonok, SIGNAL(clicked(bool)), this, SLOT(passthrough()));

		QLabel *pFROM = new QLabel(pDialog);
		pFROM->setText("FROM:");
		pFROM->setGeometry(QRect(15, 20, 41, 18));
		QLineEdit *line_from = new QLineEdit(pDialog);
		//line_from->setObjectName(QStringLiteral("lineEdit_FilterLimits_to"));
		line_from->setGeometry(QRect(50, 20, 41, 18));

		QLabel *pTO = new QLabel(pDialog);
		pTO->setText("TO:");
		pTO->setGeometry(QRect(15, 50, 41, 18));
		QLineEdit *line_to = new QLineEdit(pDialog);
		line_to->setObjectName(QStringLiteral("line_to"));
		line_to->setGeometry(QRect(50, 50, 41, 18));
		
		QRadioButton *Negative = new QRadioButton(pDialog);
		Negative->setGeometry(QRect(15, 80, 100, 14));
		Negative->setText("Negative");

		pDialog->show();

}
void PCLVisualizer::onPassthroughy()
{
	QDialog*pDialog = new QDialog(this, Qt::Dialog);
	pDialog->setFixedSize(160, 130);

	QPushButton *buttonok = new QPushButton(pDialog);
	buttonok->setObjectName(QStringLiteral("OK"));
	buttonok->setGeometry(QRect(15, 100, 50, 20));
	buttonok->setText("OK");
	//connect(buttonok, SIGNAL(clicked()), pDialog, SLOT(passthrough()));
	connect(buttonok, SIGNAL(clicked(bool)), this, SLOT(passthrough()));

	QLabel *pFROM = new QLabel(pDialog);
	pFROM->setText("FROM:");
	pFROM->setGeometry(QRect(15, 20, 41, 18));
	QLineEdit *line_from = new QLineEdit(pDialog);
	//line_from->setObjectName(QStringLiteral("lineEdit_FilterLimits_to"));
	line_from->setGeometry(QRect(50, 20, 41, 18));

	QLabel *pTO = new QLabel(pDialog);
	pTO->setText("TO:");
	pTO->setGeometry(QRect(15, 50, 41, 18));
	QLineEdit *line_to = new QLineEdit(pDialog);
	line_to->setObjectName(QStringLiteral("line_to"));
	line_to->setGeometry(QRect(50, 50, 41, 18));

	QRadioButton *Negative = new QRadioButton(pDialog);
	Negative->setGeometry(QRect(15, 80, 100, 14));
	Negative->setText("Negative");

	pDialog->show();
}
void PCLVisualizer::onPassthroughz()
{
	QDialog*pDialog = new QDialog(this, Qt::Dialog);
	pDialog->setFixedSize(160, 130);

	QPushButton *buttonok = new QPushButton(pDialog);
	buttonok->setObjectName(QStringLiteral("OK"));
	buttonok->setGeometry(QRect(15, 100, 50, 20));
	buttonok->setText("OK");
	//connect(buttonok, SIGNAL(clicked()), pDialog, SLOT(passthrough()));
	connect(buttonok, SIGNAL(clicked(bool)), this, SLOT(passthrough()));

	QLabel *pFROM = new QLabel(pDialog);
	pFROM->setText("FROM:");
	pFROM->setGeometry(QRect(15, 20, 41, 18));
	QLineEdit *line_from = new QLineEdit(pDialog);
	//line_from->setObjectName(QStringLiteral("lineEdit_FilterLimits_to"));
	line_from->setGeometry(QRect(50, 20, 41, 18));

	QLabel *pTO = new QLabel(pDialog);
	pTO->setText("TO:");
	pTO->setGeometry(QRect(15, 50, 41, 18));
	QLineEdit *line_to = new QLineEdit(pDialog);
	line_to->setObjectName(QStringLiteral("line_to"));
	line_to->setGeometry(QRect(50, 50, 41, 18));

	QRadioButton *Negative = new QRadioButton(pDialog);
	Negative->setGeometry(QRect(15, 80, 100, 14));
	Negative->setText("Negative");

	pDialog->show();
}
void PCLVisualizer::onButtonPassthrough()
{
	//QDialog *dialog = new QDialog;
	//dialog->show();

	//Filters common set
	PointCloudT_XYZ::Ptr cloud_filtered(new PointCloudT_XYZ);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);

	//set FiltersName
	if (ui.radioButton_FilterFieldName_Z->isChecked())
		pass.setFilterFieldName("z");
	if (ui.radioButton_FilterFieldName_X->isChecked())
		pass.setFilterFieldName("x");
	if (ui.radioButton_FilterFieldName_Y->isChecked())
		pass.setFilterFieldName("y");
	pass.setFilterLimits(ui.lineEdit_FilterLimits_from->text().toFloat(),ui.lineEdit_FilterLimits_to->text().toFloat());
	if (ui.radioButton_FilterLimitsNegative->isChecked())
		pass.setFilterLimitsNegative(true);
	else
		pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_filtered);
	pcl::io::savePCDFileASCII("cloud_passthrough.pcd", *cloud_filtered);
	Consolelog("cloud_passthrough.pcd", "Passthrough", "Using Passthtough filters and save the point cloud!");
	viewer->addPointCloud(cloud_filtered, "cloud");
	viewer->updatePointCloud(cloud_filtered, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onSave()
{
	//QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "D:", tr("Point cloud data (*.pcd *.ply)"));
	//pcl::io::savePCDFileASCII();
	pcl::io::savePCDFileASCII("save.pcd", *cloud);
	//pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	Consolelog("", "Save", "Save the point cloud on the screen!");
	exit(0);
}
void PCLVisualizer::onOpencloud()
{
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::io::loadPCDFile("qq.pcd", *cloud);



	QString filename = QFileDialog::getOpenFileName(this, tr("Open Point Cloud"), ".", tr("Point Cloud Data (*.pcd)"));
	std::string file_name = filename.toStdString();

	std::string subnametrue;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subnametrue.insert(subnametrue.begin(), *i);
	}
	subname = subnametrue;
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	PointCloudT_XYZ::Ptr cloud_tmp(new PointCloudT_XYZ);
	if (filename.isEmpty())
		return;
	int return_status;
	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}
	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);
	}

	Consolelog(QString::fromLocal8Bit(subname.c_str()), "Open", "Open point cloud for following processing!");


	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	//pcl::visualization::PCLVisualizer viewer1;
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理
		user_data++;
	}
}
void PCLVisualizer::passthrough()
{
	;
}
void PCLVisualizer::onButtonVoxelGrid()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(ui.lineEdit_VoxelGrid_X->text().toFloat(), ui.lineEdit_VoxelGrid_Y->text().toFloat(), ui.lineEdit_VoxelGrid_Y->text().toFloat());
	sor.filter(*cloud_voxelgrid);
	pcl::io::savePCDFileASCII("cloud_voxelgrid.pcd", *cloud_voxelgrid);
	Consolelog("cloud_voxelgrid.pcd", "VoxelGrid", "Using VoxelGrid filters and save the point cloud!");
	viewer->addPointCloud(cloud_voxelgrid, "cloud");
	viewer->updatePointCloud(cloud_voxelgrid, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onButtonOutlier()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	//sor.setMeanK(50);
	sor.setMeanK(ui.lineEdit_Outlier_neibor->text().toFloat());
	//sor.setStddevMulThresh(1.0);
	sor.setStddevMulThresh(ui.lineEdit_Outlier_threshold->text().toFloat());
	sor.setNegative(true);
	sor.filter(*cloud_outlier);
	pcl::io::savePCDFileASCII("cloud_outlier_removal.pcd", *cloud_outlier);

	Consolelog("cloud_outlier_removal.pcd", "Outlierremoval", "Using Outlizer filters and save the point cloud!");
	viewer->addPointCloud(cloud_outlier, "cloud");
	viewer->updatePointCloud(cloud_outlier, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();

}
void PCLVisualizer::onButtonGreedyProjection()
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);


	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	//选取树的创建方式
	n.setSearchMethod(tree);
	//设置树的法向估计，默认20
	n.setKSearch(ui.lineEdit_Greedy_Ksearch->text().toInt());
	n.compute(*normals);
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	//最近搜索半径，默认0.025
	gp3.setSearchRadius(ui.lineEdit_Greedy_SearchRadius->text().toDouble());
	// Set typical values for the parameters
	gp3.setMu(2.5);
	//最大临近点个数，默认100
	gp3.setMaximumNearestNeighbors(ui.lineEdit_Greedy_MaxNear->text().toInt());
	//设置平面化角度,默认45度
	gp3.setMaximumSurfaceAngle(ui.lineEdit_Greedy_MaxSurfaceaAngle->text().toDouble()*M_PI/180); // 45 degrees
	//设置三角形内角角度限制10-120
	gp3.setMinimumAngle(ui.lineEdit_Greedy_MinAngle->text().toDouble()*M_PI/180); // 10 degrees
	gp3.setMaximumAngle(ui.lineEdit_Greedy_MaxAngle->text().toDouble()*M_PI / 180); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::io::savePLYFile("Model.ply", triangles);
	Consolelog("Model.ply", "GreedyProjection", "Greedy Projection for a point cloud!");
	viewer->addPolygonMesh(triangles, "my");
	viewer->resetCamera();
	ui.qvtkWidget->update();

}
void PCLVisualizer::onPlanarSegmentation()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);//存储直通滤波结果

	// filter
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);

	//pass.setFilterFieldName("y");
	if (ui.radioButton_SegmentationFilter_X->isChecked())
		pass.setFilterFieldName("x");
	if (ui.radioButton_SegmentationFilter_Y->isChecked())
		pass.setFilterFieldName("y");
	if (ui.radioButton_SegmentationFilter_Z->isChecked())
		pass.setFilterFieldName("z");

	//pass.setFilterLimits(-0.1, 0.6);
	pass.setFilterLimits(ui.lineEdit_SegmentationFilter_from->text().toFloat(), ui.lineEdit_SegmentationFilter_to->text().toFloat());
	pass.filter(*cloud_source_filtered);

	// segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> sac;
	sac.setInputCloud(cloud_source_filtered);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PLANE);

	//sac.setDistanceThreshold(0.01);
	//sac.setMaxIterations(100);
	sac.setDistanceThreshold(ui.lineEdit_Segmentation_Distancethreshold->text().toDouble());
	sac.setMaxIterations(ui.lineEdit_Segmentation_MaxIterations->text().toInt());
	sac.setProbability(0.95);
	sac.segment(*inliers, *coefficients);

	// extract the certain field
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setIndices(inliers);
	ei.setInputCloud(cloud_source_filtered);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	ei.filter(*cloud_target);

	//保存提取点云
	pcl::io::savePCDFileASCII("cloud_PlanarSegmentation.pcd", *cloud_target);
	Consolelog("cloud_PlanarSegmentation.pcd", "PlanarSegmentation", "Pick plane from a point cloud!");
	//显示经过提取后的点云
	//viewer->addPointCloud(cloud_target, "cloud");
	viewer->updatePointCloud(cloud_target, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onCyliderSegmentation()
{
	

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylider_segmentation(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	if (ui.radioButton_SegmentationCyliderFilter_X->isChecked())
		pass.setFilterFieldName("x");
	if (ui.radioButton_SegmentationCyliderFilter_Y->isChecked())
		pass.setFilterFieldName("y");
	if (ui.radioButton_SegmentationCyliderFilter_Z->isChecked())
		pass.setFilterFieldName("z");
	pass.setFilterLimits(ui.lineEdit_SegmentationCyliderFilter_from->text().toFloat(), ui.lineEdit_SegmentationCyliderFilter_to->text().toFloat());
	//pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	//seg.setMaxIterations(10000);
	seg.setMaxIterations(ui.lineEdit_SegmentationCylider_MaxIterations_2->text().toInt());
	seg.setDistanceThreshold(ui.lineEdit_SegmentationCylider_Distancethreshold->text().toDouble());
	//seg.setDistanceThreshold(0.05);
	//seg.setRadiusLimits(0, 0.1);
	seg.setRadiusLimits(0, ui.lineEdit_SegmentationCylider_Radiuslimits->text().toDouble());
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	 

	seg.segment(*inliers_cylinder, *coefficients_cylinder);

	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	extract.filter(*cloud_cylider_segmentation);
	pcl::io::savePCDFile("cloud_CyliderSegmentation.pcd", *cloud_cylider_segmentation);
	Consolelog("cloud_CyliderSegmentation.pcd", "CyliderSegmentation", "Pick cylider from a point cloud!");
	viewer->updatePointCloud(cloud_cylider_segmentation, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onNormalDisTrans()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	//approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
	approximate_voxel_filter.setLeafSize(ui.lineEdit_VoxelGridNDT_X->text().toFloat(),ui.lineEdit_VoxelGrid_Y->text().toFloat(),ui.lineEdit_VoxelGridNDT_Z->text().toFloat());
	approximate_voxel_filter.setInputCloud(cloud_rgs_source);
	approximate_voxel_filter.filter(*filtered_cloud);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	//ndt.setTransformationEpsilon(0.01);
	ndt.setTransformationEpsilon(ui.lineEdit_NormalDisTrans_Transepsilon->text().toDouble());

	// Setting maximum step size for More-Thuente line search.
	//ndt.setStepSize(0.1);
	ndt.setStepSize(ui.lineEdit_NormalDisTrans_Stepsize->text().toDouble());
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	//ndt.setResolution(1.0);
	ndt.setResolution(ui.lineEdit_NormalDisTrans_Gridresolution->text().toFloat());

	// Setting max number of registration iterations.
	//ndt.setMaximumIterations(35);
	ndt.setMaximumIterations(ui.lineEdit_NormalDisTrans_Maxiterations->text().toInt());

	// Setting point cloud to be aligned.
	ndt.setInputSource(filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget(cloud_rgs_target);
	Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);



	int a = clock();
	ndt.align(*output_cloud, init_guess);
	int b = clock();
	int c;
	c = b - a;

	//把C写到文件里
	ofstream fileoutput;
	fileoutput.open("time.txt");
	fileoutput << c;
	fileoutput.close();




	pcl::transformPointCloud(*cloud_rgs_source, *output_cloud, ndt.getFinalTransformation());
	pcl::io::savePCDFile("cloud_NDT.pcd", *output_cloud);







	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(cloud_rgs_target, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud_rgs_target, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "output cloud");

	// Starting visualizer
	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();
	Consolelog("cloud_NDT.pcd", "NDT", "Registrate point cloud by NDT tools!");
	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}




}
void PCLVisualizer::onOpenSource()
{
	//fileName返回文件名，窗口名，默认目录，打开文件类型
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Point Cloud"), ".", tr("Point Cloud Data (*.pcd)"));
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	PointCloudT_XYZ::Ptr cloud_tmp(new PointCloudT_XYZ);
	if (filename.isEmpty())
		return;
	int return_status;


	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);


	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}
	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud_rgs_source);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_rgs_source, vec);
	}



	viewer->addPointCloud(cloud_rgs_source, "cloud");
	viewer->updatePointCloud(cloud_rgs_source, "cloud");
	//viewer->setBackgroundColor(255,255,0);
	viewer->resetCamera();
	ui.qvtkWidget->update();


}
void PCLVisualizer::onOpenTarget()
{
	//fileName返回文件名，窗口名，默认目录，打开文件类型
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Point Cloud"), ".", tr("Point Cloud Data (*.pcd)"));
	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	PointCloudT_XYZ::Ptr cloud_tmp(new PointCloudT_XYZ);
	if (filename.isEmpty())
		return;
	int return_status;


	return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);


	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}
	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud_rgs_target);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_rgs_target, vec);
	}
	viewer->addPointCloud(cloud_rgs_target, "cloud");
	viewer->updatePointCloud(cloud_rgs_target, "cloud");
	//viewer->setBackgroundColor(255,255,0);
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onRgsDispaly()
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_rgs_source,255,0,0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_rgs_target,0,255,0);
	viewer->addPointCloud(cloud_rgs_source, source_color, "source_cloud");
	viewer->addPointCloud(cloud_rgs_target, target_color, "target_cloud");
	viewer->updatePointCloud(cloud_rgs_source, "source_cloud");
	viewer->updatePointCloud(cloud_rgs_target, "target_cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();
}
void PCLVisualizer::onTranslation()
{
	PointCloudT_XYZ::Ptr cloud_transformed(new PointCloudT_XYZ);
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	/*
	|-------> This column is the translation
	| 1 0 0 x |  \
	| 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	| 0 0 1 z |  /
	| 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)	*/
	transform_1(0, 3) = ui.lineEdit_Transform_X->text().toFloat();
	transform_1(1, 3) = ui.lineEdit_Transform_Y->text().toFloat();
	transform_1(2, 3) = ui.lineEdit_Transform_Z->text().toFloat();
	float theta = ui.lineEdit_Translation_Degree->text().toFloat()*M_PI/180;
	//float theta = M_PI / 4;
	if (ui.radioButton_Translation_X->isChecked())
	{
		transform_1(1, 1) = cos(theta);
		transform_1(2, 2) = cos(theta);
		transform_1(1, 2) = -sin(theta);
		transform_1(2, 1) = sin(theta);
	}
	if (ui.radioButton_Translation_Y->isChecked())
	{
		transform_1(0, 0) = cos(theta);
		transform_1(2, 2) = cos(theta);
		transform_1(2, 0) = -sin(theta);
		transform_1(0, 2) = sin(theta);
	}
	if (ui.radioButton_Translation_Z->isChecked())
	{
		transform_1(0, 0) = cos(theta);
		transform_1(0, 1) = -sin(theta);
		transform_1(1, 0) = sin(theta);
		transform_1(1, 1) = cos(theta);
	}



	pcl::transformPointCloud(*cloud, *cloud_transformed, transform_1);



	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		cloud_color(cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "target cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		cloud_transformed_color(cloud_transformed, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(cloud_transformed, cloud_transformed_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "output cloud");

	viewer_final->addCoordinateSystem(1.0, "global");
	viewer_final->initCameraParameters();

	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	/*pcl::visualization::PCLVisualizer viewerx("Matrix transformation example");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 255, 255);
	viewerx.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(cloud_transformed, 230, 20, 20); // Red
	viewerx.addPointCloud(cloud_transformed, transformed_cloud_color_handler, "transformed_cloud");
	viewerx.addCoordinateSystem(1.0, "globle");
	//viewer.addCoordinateSystem(1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0. 
	viewerx.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewerx.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewerx.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewerx.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewerx.spinOnce();
	}*/

}
void PCLVisualizer::onIterativeClosestPoint()
{
	int iterations = 1;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(ui.lineEdit_ICP_Maxiterations->text().toInt());
	*cloud_rgs_icp = *cloud_rgs_source;
	icp.setTransformationEpsilon(ui.lineEdit_ICP_Epsilon->text().toDouble());
	icp.setInputSource(cloud_rgs_icp);
	icp.setInputTarget(cloud_rgs_target);
	icp.align(*cloud_rgs_icp);
	pcl::io::savePCDFile("cloud_ICP.pcd", *cloud_rgs_icp);
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	
	// 原始的点云设置为白色的
	pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZ> cloud_in_color_h(cloud_rgs_source, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_rgs_target, cloud_in_color_h, "cloud_in_v1", v1);//设置原始的点云都是显示为白色
	viewer.addPointCloud(cloud_rgs_target, cloud_in_color_h, "cloud_in_v2", v2);

	// 进行icp配准的点云初始状态显示为绿色
	pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZ> cloud_tr_color_h(cloud_rgs_target, 20, 180, 20);
	viewer.addPointCloud(cloud_rgs_source, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP配准后的点云为红色
	pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZ> cloud_icp_color_h(cloud_rgs_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_rgs_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);


	// 设置背景颜色
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// 设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // 可视化窗口的大小

	// 注册按键回调函数
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
	Consolelog("cloud_ICP.pcd", "ICP", "Registrate point cloud by icp tools!");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		if (next_iteration)
		{
			// 最近点迭代算法
			icp.align(*cloud_rgs_icp);

			viewer.updatePointCloud(cloud_rgs_icp, cloud_icp_color_h, "cloud_icp_v2");
		}
		next_iteration = false;
	}
}
void PCLVisualizer::setPropertyTable()
{
	QStringList header;
	header << "Property" << "Value";
	ui.tableWidget->setHorizontalHeaderLabels(header);
	ui.tableWidget->setItem(0, 0, new QTableWidgetItem("Clouds"));
	ui.tableWidget->setItem(1, 0, new QTableWidgetItem("Points"));
	ui.tableWidget->setItem(2, 0, new QTableWidgetItem("RGB"));
	ui.tableWidget->setItem(3, 0, new QTableWidgetItem("Centroid"));
	ui.tableWidget->setColumnWidth(0, 80);
	ui.tableWidget->setColumnWidth(1, 160);
	ui.tableWidget->setItem(0, 1, new QTableWidgetItem(QString::fromLocal8Bit(subname.c_str())));
	ui.tableWidget->setItem(1, 1, new QTableWidgetItem(QString::number(cloud->points.size())));
	ui.tableWidget->setItem(2, 1, new QTableWidgetItem("NULL"));
	ui.tableWidget->setItem(3, 1, new QTableWidgetItem(QString::number(centroid[0]) + "," + QString::number(centroid[1]) + ","+ QString::number(centroid[2])));
}
void PCLVisualizer::Consolelog(QString cloud, QString operation, QString details)
{
	int rows = ui.consoleTable->rowCount();
	ui.consoleTable->setRowCount(++rows);
	ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(cloud));
	ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(details));

	ui.consoleTable->scrollToBottom();

}
void PCLVisualizer::setConsoleTable()
{
	QStringList header2;
	header2 << "Cloud" << "Operation" << "Details" ;
	ui.consoleTable->setHorizontalHeaderLabels(header2);
	ui.consoleTable->setColumnWidth(0, 70);
	ui.consoleTable->setColumnWidth(1, 80);
	ui.consoleTable->setColumnWidth(2, 160);
	ui.consoleTable->verticalHeader()->setDefaultSectionSize(22); //设置行距
	ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
}

void PCLVisualizer::actionConsole()
{
	if (ui.consoleaction->isChecked())
	{
		ui.ConsoleDock->setVisible(true);
	}
	else
	{
		ui.ConsoleDock->setVisible(false);
	}
}
void PCLVisualizer::actionProperty()
{
	if (ui.propertyaction->isChecked())
	{
		ui.PropertyDock->setVisible(true);
	}
	else
	{
		ui.PropertyDock->setVisible(false);
	}
}
void PCLVisualizer::actionTools()
{
	if (ui.toolaction->isChecked())
	{
		ui.ToolDock->setVisible(true);
	}
	else
	{
		ui.ToolDock->setVisible(false);
	}
}

void PCLVisualizer::mainview()
{
	viewer->setCameraPosition(0, -1, 0, 0, 0, 0, 0, 0, 1);
	ui.qvtkWidget->update();
}
void PCLVisualizer::leftview()
{
	viewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
	ui.qvtkWidget->update();
}
void PCLVisualizer::topview()
{
	viewer->setCameraPosition(0, 0, 1, 0, 0, 0, 0, 1, 0);
	ui.qvtkWidget->update();
}
void PCLVisualizer::backview()
{
	viewer->setCameraPosition(0, 1, 0, 0, 0, 0, 0, 0, 1);
	ui.qvtkWidget->update();
}
void PCLVisualizer::rightview()
{
	viewer->setCameraPosition(1, 0, 0, 0, 0, 0, 0, 0, 1);
	ui.qvtkWidget->update();
}
void PCLVisualizer::bottomview()
{
	viewer->setCameraPosition(0, 0, -1, 0, 0, 0, 0, 1, 0);
	ui.qvtkWidget->update();
}
void PCLVisualizer::backgroundcolor()
{
	QColor color = QColorDialog::getColor(Qt::white, this,
		"Select color for point cloud");
	if (color.isValid())
	{
		viewer->setBackgroundColor(color.red() / 255.0,
			color.green() / 255.0, color.blue() / 255.0);
		// 输出窗口
		Consolelog("NULL", "Background","Set Background color to "+ QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()));
		ui.qvtkWidget->update();
	}
}
void PCLVisualizer::generatecube()
{
	PointCloudT_XYZ::Ptr cloud_cube(new PointCloudT_XYZ);
	viewer->removeAllPointClouds();
	cloud_cube->width = 50000;
	cloud_cube->height = 1;            // 设置点云高，高为1，说明为无组织点云
	cloud_cube->is_dense = false;
	cloud_cube->resize(cloud_cube->width * cloud_cube->height);     // 重置点云大小
	for (size_t i = 0; i != cloud_cube->size(); ++i)
	{
		cloud_cube->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_cube->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_cube->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	Consolelog("Cloud_Cube","Generate cube","Genarate a cube point cloud");
	viewer->addPointCloud(cloud_cube, "cloud");
	viewer->updatePointCloud(cloud_cube, "cloud");
	viewer->resetCamera();
	ui.qvtkWidget->update();


}
void PCLVisualizer::generatecylinder()
{
	viewer->removeAllPointClouds();

	viewer->addCylinder(*(new pcl::ModelCoefficients()), "cylinder");
	viewer->resetCamera();
	ui.qvtkWidget->update();
	Consolelog("NULL", "Generate cylinder", "Genarate a 3D shape cylinder!");

}
void PCLVisualizer::generatesphere()
{
	viewer->removeAllPointClouds();
	PointT_XYZ p;
	p.x = 0; p.y = 0; p.z = 0;
	viewer->addSphere(p, 100, "sphere1");
	viewer->resetCamera();
	ui.qvtkWidget->update();
	Consolelog("NULL", "Generate sphere", "Genarate a 3D shape sphere!");
}