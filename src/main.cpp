#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <dirent.h>
#include <iostream>
#include <chrono>
#include <map>
#include <omp.h>

// #include "matplotlibcpp.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
 
#define EXPECTED_ARG_NUM 2 // source-path
#define ARG_STR_UPPER_LEN 50
#define FNAME_UPPER_LEN 20
#define POINT_NUM 1000000
#define GND_HEIGHT_THRES 0.1
#define VIS_FREQ 10 // Visualize every 10 times

// namespace plt = matplotlibcpp; //　Fundemental for matplotlib 

void saveStatAsPng(std::vector<int> &keys, std::vector<double> &vals)
{
	// plt::figure();
	// plt::plot(keys,vals);
	// plt::save("./FpsVsPointnum.png");

	return;
}

/**
 * @brief Save statistics as csv file
 * 
 * @param keys 
 * @param vals 
 */
bool saveStatAsCsv(std::vector<int> &keys, std::vector<double> &vals,std::vector<int> &indiceList){
	FILE *stream = fopen("./FpsVsPointnum.csv", "w+");
	if(stream == NULL){
		return false;
	}

	assert(keys.size() == vals.size());
	
	for(int i = 0; i < keys.size(); i++){
		fprintf(stream,"%d,%f,%d\n",keys[i],vals[i],indiceList[i]);
	}

	fclose(stream);
	return true;
}

int main(int argc, char **argv)
{
	unsigned int procCnt; // Processed count
	unsigned int visualizeFreq = VIS_FREQ; // visualize every 10 times
	
	std::vector<int> pointNumList;
	std::vector<double> fpsList;
	std::vector<int> indiceList;

	char *src_dpath = argv[1];

	if( argc == EXPECTED_ARG_NUM ){
		// fscanf(argv[1],src_dpath,sizeof(src_dpath));
		// fscanf(argv[2],dst_dpath,sizeof(dst_dpath));	

		printf("src:%s\n",src_dpath);
	}else{
		/* Error handling in case of invalid arguments */
		fprintf(stderr,"PARAM ERR. Given arg num=%d, Expected%d\n",argc,EXPECTED_ARG_NUM);
		fprintf(stderr,"Usage: <path/to/dir_contains_pcd(s)> \n");
		return -1;
	}

	/* Open src directory */
	DIR *dir = opendir(src_dpath);
	if (!dir) {
		perror(argv[0]);
		return -1;
	}

	/* Operate for files in directory */
	std::vector<std::string> filenames;
	struct dirent *ent;
	while((ent = readdir(dir)) != nullptr){
		char *fname = ent->d_name;
		filenames.emplace_back(std::string(fname));
	}

	#pragma omp parallel for
	for (int i = 0; i < filenames.size(); i++) {
		std::string fname = filenames[i];

		// Define src/dest file full-path
		char src_fpath[FNAME_UPPER_LEN+ARG_STR_UPPER_LEN] = {0};

		snprintf(src_fpath,sizeof(src_fpath),"%s/%s",src_dpath,fname.c_str());
		
		// allocate pointcloud data
		int32_t num = POINT_NUM;
		float *data = (float*)malloc(num * sizeof(float));
		
		// point (initial position)
		float *px = data + 0;
		float *py = data + 1;
		float *pz = data + 2;
		float *pr = data + 3;//reflection intensity
		
		// Read point cloud data
		FILE *stream;
		stream = fopen(src_fpath, "rb");
		if(stream == NULL){
			continue;
		}

		// Read sizeof(float) * num as Byte stream
		num = fread(data, sizeof(float), num, stream) / 4; //Read point cloud data, about 100,000+ points
		fclose(stream);
		std::cout << std::string(src_fpath) << ":" << num << " points  ";

		if(!(num > 0)){
			std::cout << "ignored" << std::endl;
			continue; // skip no point file
		}

		// Write point cloud data
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// Fill in the cloud data
		cloud->width  = num;
		cloud->height = 1;
		cloud->points.resize (cloud->width * cloud->height);

		/* Put read pointcloud to pcl::PointXYZ */
		for (int32_t i = 0; i < num; i++)
		{
			cloud->points[i].x = *px;
			cloud->points[i].y = *py;
			cloud->points[i].z = *pz;
			px += 4; py += 4; pz += 4; pr += 4; // goto next point
		}

		// Variable for measuring time
		std::chrono::system_clock::time_point  start, end;
		start = std::chrono::system_clock::now(); // Start measuring time

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (GND_HEIGHT_THRES);

		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		end = std::chrono::system_clock::now(); // Stop measuring time
		double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); // Convert elapsed time to double
		std::cout << elapsed << "ms" << std::endl;

		int pointAccuracy = 1;
		pointNumList.emplace_back((int)(cloud->points.size()/pointAccuracy) * pointAccuracy);
		fpsList.emplace_back(elapsed);
		indiceList.emplace_back(inliers->indices.size());

		// Visualize statistics every 'visualizeFreq'
		// if( procCnt % visualizeFreq){
		// 	saveStatAsPng(pointNumList,fpsList);
		// 	saveStatAsCsv(pointNumList,fpsList,indiceList);
		// }
		
		free(data);
		
		procCnt++;
	}

	saveStatAsPng(pointNumList,fpsList);
	saveStatAsCsv(pointNumList,fpsList,indiceList);

	closedir(dir);

    return 0;
}