
#include <pcl\io\pcd_io.h>
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\ModelCoefficients.h>
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\sample_consensus\ransac.h>
#include <pcl\sample_consensus\method_types.h>
#include <pcl\sample_consensus\model_types.h>
#include <pcl\sample_consensus\sac_model_plane.h>
#include <pcl\sample_consensus\sac_model_sphere.h>
#include <pcl\sample_consensus\sac_model_perpendicular_plane.h>
#include <pcl\common\angles.h>
#include <pcl\ros\conversions.h>
#include <iostream>
#include <string> 

using namespace std;

// PCD dosyası okur ve okuma durumunu döndürür.
int main (int argc, char** argv)
{
	//İl yüklenecek nokta bulutu
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//Gürültü noktaları ayıklanmış nokta bulutu
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	//Voxel grid uygulanarak azaltılan nokta bulutu
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
	//Random Sample Consensus ile Yüzey Tanılama
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
	//RANSAC ile küresel objeleri Tanılama
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere (new pcl::PointCloud<pcl::PointXYZ>);
	//Random Sample Consensus ile Yüzey Tanılama (Dik Yüzeyler)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_perplane (new pcl::PointCloud<pcl::PointXYZ>);
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), 
		cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	
	//Dosyayı yükle
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd_01.pcd", *cloud) == -1)
	{
		PCL_ERROR("Nokta bulutu dosyası (test_pcd_01.pcd) okunamadı. \n");
		return(-1);
	}
	else
	{
		PCL_INFO("Nokta bulutu dosyası (test_pcd_01.pcd) okundu. \n");
	}
	//İstatiksel Gürültü Noktası Ayıklama kodu
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
	statFilter.setInputCloud(cloud);
	statFilter.setMeanK(50);
	statFilter.setStddevMulThresh(1);
	statFilter.filter(*cloud_filtered);

	pcl::io::savePCDFileASCII("test_pcd_02_filtered.pcd" , *cloud_filtered);
	cout << "test_pcd_02_filtered.pcd dosyasi yazildi." << endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_filtered);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_downsampled);

	pcl::io::savePCDFileASCII("test_pcd_03_downsampled.pcd" , *cloud_downsampled);
	cout << "test_pcd_03_downsampled.pcd dosyasi yazildi." << endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	//seg.setEpsAngle( pcl::deg2rad(90.0f) );
	seg.setDistanceThreshold (0.1);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int i = 0, nr_points = (int) cloud_downsampled->points.size ();

	while (cloud_downsampled->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_downsampled);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_downsampled);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
	int j=i;
	//std::string filename = "test_pcd_05_ransacsphere_" + std::to_string(j) + ".pcd";
    ss << "new_pcl_" << i << ".pcd";
	cout << ss.str () << " " << i << " " << endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_downsampled, *inliers, *cloud_p);
	pcl::io::savePCDFileASCII(ss.str () , *cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_downsampled.swap (cloud_f);
    i++;
  }


	/*
	//Yüzey tanılama
	std::vector<int> inliers;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_downsampled));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.1);
    ransac.computeModel();
   ransac.getInliers(inliers);

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_downsampled, inliers, *cloud_plane);
	pcl::io::savePCDFileASCII("test_pcd_04_ransacplane.pcd" , *cloud_plane);
	cout << "test_pcd_04_ransacplane.pcd dosyasi yazildi." << endl;
	
	//Küresel objeleri tanılama
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr 
		model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud_downsampled));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2 (model_s);
    ransac2.setDistanceThreshold (0.3);
    ransac2.computeModel();
    ransac2.getInliers(inliers);

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_downsampled, inliers, *cloud_sphere);
	pcl::io::savePCDFileASCII("test_pcd_05_ransacsphere.pcd" , *cloud_sphere);
	cout << "test_pcd_05_ransacsphere.pcd dosyasi yazildi." << endl;
	*/
	/*
	//Segmentation Tutorial
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_downsampled));

	pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model_plane, 0.03);
	bool result = sac.computeModel ();

	// İçerideki inlier'ları hesapla
	boost::shared_ptr<vector<int> > inliers2 (new vector<int>);
	sac.getInliers (*inliers2);
	cout << "Found model with " << inliers2->size () << " inliers2" << endl;
	// Model katsayılarını getir
	Eigen::VectorXf coeff;
	sac.getModelCoefficients (coeff);
	cout << "and plane normal is: " << coeff[0] << " , " << coeff[1] << " , " << coeff[2] << endl;

	//Tekrar yerleştirme (Tercihe bağlı uygulanabilir)
	Eigen::VectorXf coeff_refined;
	model_plane->optimizeModelCoefficients(*inliers2, coeff, coeff_refined);
	model_plane->selectWithinDistance(coeff_refined, 0.03,*inliers2);
	cout << "After refitting, model contains" << inliers2->size () << "inliers" << endl;
	cout << "and plane normal is:" << coeff_refined[0] << " , " << coeff_refined[1] << " , " << coeff_refined[2] << "." << endl;

	// Projeksiyon
	pcl::PointCloud<pcl::PointXYZ> proj_points;
	model_plane->projectPoints(*inliers2, coeff_refined, proj_points);

	//Normal vektör tanılama
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud (cloud_downsampled);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	normalEstimation.setRadiusSearch (0.03);
	normalEstimation.compute (*cloud_normals);
	pcl::io::savePCDFileASCII("test_pcd_04_cloud_normals.pcd" , *cloud_normals);
	cout << "test_pcd_04_cloud_normals.pcd dosyasi yazildi." << endl;

	return(0);
}


/*
//PCD dosyası oluşturup yazar.
#include <pcl\io\pcd_io.h>
#include <pcl\point_types.h>
#include <iostream>
using namespace std;

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//Cloud verisini doldur
	cloud.width		=	1000;
	cloud.height	=	1;
	cloud.is_dense	=	false;
	cloud.points.resize	(cloud.width * cloud.height);

	for (size_t i=0 ; i < cloud.points.size() ; ++i)
	{
		cloud.points[i].x	=	1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y	=	1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z	=	1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("test_pcd_01.pcd" , cloud);
	*/


	return(0);
}
