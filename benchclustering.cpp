/* BSD 3-Clause License
 *  
 *  Copyright (c) 2020, tmkhoyan (Tigran Mkhoyan)
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Description:     main file for DBSCAN and Disjoint-set data structure classes comparison performance test
 *                  example compile: make
 *                  then run with:   ./runBenchcluster
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 *
 */

#include "header/benchclustering.h"
/*-------------------- settings benchmark          ------------------*/

int NITER = 10; 			// benchmark iterations

std::vector<int> clusterSizes 	= {5,   10,  50, 100, 500};
std::vector<int> clusterPupulation = {50,  50,  50,  50,  50};

int CVWAIT_MS = 1;
  /* -----------------   parameters dbscan  -------------------------------*/

float dminnorm = DMINNORM;  	// minnorm for first balanced distribution
int dsigma = xsigmap*1.8;  	// distance general norm

// dbscan new 
int dbscan_mnpts =20;
int dbscan_epsilon = dsigma;
int dbscan_maxpts = 100;

// disjoint
  /* -------------------   parameters disjoint  -----------------------------*/
int ds_dsigma = dsigma ; 


int main( int argc, char** argv){

std::vector<std::vector<cv::Point2f>> DBscanCentroids;      DBscanCentroids.reserve(clusterSizes.size());
std::vector<std::vector<cv::Point2f>> DisjointsetCentroids; DisjointsetCentroids.reserve(clusterSizes.size());
std::vector <double>  T_dbscan;   T_dbscan.reserve(clusterSizes.size());
std::vector <double>  T_disjoint; T_disjoint.reserve(clusterSizes.size());

cv::namedWindow("data",cv::WINDOW_NORMAL);
cv::namedWindow("clustering",cv::WINDOW_NORMAL);


std::string im_out_path = (argc>1)? argv[1] : "data_out/";

// check directories are valid
makeValidDir(im_out_path);

#ifdef COMPRESS_JPG
	std::vector<int> params_jpg;
	params_jpg.push_back(1);
	params_jpg.push_back(95);   
#else
	params_jpg.push_back(1);
	params_jpg.push_back(90);   // that's compression level, 95 == default , 1-- - max
#endif

#ifdef ADD_NOISE
	pxy_n.reserve(NOISE_PERC*clusterSizes.back());
#endif
	/*-------------------- open output storage ------------------*/

	fs_o.open(im_out_path+flname    +".yml", cv::FileStorage::WRITE);
	fs_o_db.open(im_out_path+flname +"_db.yml", cv::FileStorage::WRITE);
	fs_o_ds.open(im_out_path+flname +"_ds.yml", cv::FileStorage::WRITE);

	/*--------------------  start loop  ------------------*/

		int k = 0; // cluster size id
		// float dminnorm_;
	for(auto clustersize: clusterSizes){
		// std::cout << "iteration " << k << std::endl;
		/*--------------------initiate containers ------------------*/
		cv::Mat img;    // clean
		cv::Mat img_sh; // show
		cv::Mat img_bw;
		img = cv::Mat::zeros(imgw,imgh,CV_8UC3); // new matrix 

		// centers
		cv::Mat Xc = cv::Mat_<float>(clustersize,1);
		cv::Mat Yc = cv::Mat_<float>(clustersize,1);

		std::vector <cv::Point2f> 			pxy_c; pxy_c.reserve(clustersize);
		std::vector <std::vector<cv::Point2f>> 	Pxy_p; Pxy_p.reserve(clustersize);

		// float dminnorm_ = dminnorm*(1-(k+1)/clusterSizes.size());
		do{
		rng    = cv::RNG(cv::getCPUTickCount());
		/*-------------------- clusters ------------------*/

		rng.fill(Xc,cv::RNG::UNIFORM,xmin,xmax);
		rng.fill(Yc,cv::RNG::UNIFORM,ymin,ymax);

		pxy_c = mat2point<cv::Point2f>(Xc, Yc);
		std::cout << "balancing distribution ..." << std::endl; // do this only for the first one 

		} while(checkPairwiseNorm(pxy_c,dminnorm) and clustersize <=10);
		/*-------------------- cluster population normal distribution for the population ------------------*/
		rng_p    = cv::RNG(cv::getCPUTickCount());

		auto populationsize = clusterPupulation[k];

		for(int n=0; n<clustersize; n++){ // loop over clusters
			std::vector <cv::Point2f> pxy; pxy.reserve(populationsize);

			auto pc = pxy_c[n]; // pull out the current cluster center
			// generate offset distances with normal distribution
			cv::Mat xp  = cv::Mat_<float>(populationsize,1);	rng_p.fill(xp,cv::RNG::NORMAL, xmeanp,xsigmap);
			cv::Mat yp  = cv::Mat_<float>(populationsize,1);	rng_p.fill(yp,cv::RNG::NORMAL, ymeanp,ysigmap);
			// add cluster center position
			xp += pc.x;
			yp += pc.y;

			pxy = mat2point<cv::Point2f>(xp, yp);
			Pxy_p.emplace_back(pxy);
			// std::stringstream ss; ss << "cluster " << n <<  "pxy " << pc << ": "; 
			// printVec(pxy,ss.str());
		}
		// auto dpxy = pairwiseNorm(pxy_c); print2dvec(dpxy,"dpxy");
		// std::cout << "dpxy: dminnorm( " << dminnorm << " )" << checkPairwiseNorm(pxy_c,dminnorm) << std::endl;
		// std::cout << "dpxy: " << checkPairwiseNorm(pxy_c,dminnorm) << std::endl;

#ifdef ADD_NOISE 
		auto noisesize = clustersize*NOISE_PERC;
				// centers
		cv::Mat Xn = cv::Mat_<float>(noisesize,1);
		cv::Mat Yn = cv::Mat_<float>(noisesize,1);

		rng.fill(Xn,cv::RNG::UNIFORM,xmin,xmax);
		rng.fill(Yn,cv::RNG::UNIFORM,ymin,ymax);

		pxy_n = mat2point<cv::Point2f>(Xn, Yn);
		std::cout << "noise added..." << std::endl;
		std::cout << "noise added..." << pxy_n.size() << std::endl;
		std::cout << "noisesize" << pxy_n.size() << std::endl;



#endif
 /* ----------------------------------------------  draw points to gerenerate conturs on gray --------------------------------------------- */
		// default PSIZE is important for bounding rect in dbscan input!
		drawClusterPopulation(img,Pxy_p); // defaults to empty color vector an white point cloud needed for gray
		img.copyTo(img_sh); // working image
		// show result while running contours
		cv::imshow("data",img);
		cv::waitKey(1);
/* ------------------------------------------- imgae to gray and get contours -------------------------------------------*/
#ifdef USE_CONTOURS
			cv::cvtColor(img_sh, img_bw, cv::COLOR_BGR2GRAY);

			std::cout << "generating contours..." << std::endl;
			std::vector<std::vector<cv::Point>> contours =  getContours(img_bw); //get contours of the image;

			std::vector <cv::Rect> boxes  	= getRectPoints_(contours);      // gets the rectangle bounded by contour points
			std::vector <cv::Point2f> points  	= getGroupPoints_(contours); //unrolls all points
#else
		     // default PSIZE is important for bounding rect in dbscan input!
			std::vector <cv::Rect> boxes  	= getRectPoints_(Pxy_p,PSIZERECT);      // gets the rectangle bounded by contour points
			std::vector <cv::Point2f> points  	= getGroupPoints_(Pxy_p); //unrolls all points

#endif
			std::cout << "generated vector size " << boxes.size() << ", dsigma "<< dsigma << std::endl;

/* ------------------------------------------- dbscan -------------------------------------------*/
			std::cout << "initializing data classes dbscan and disjoint " << std::endl;

#ifdef DB_SCAN_MODE_OLD
	 DbScan dbscan(boxes,dbscan_epsilon,dbscan_mnpts); //provde boxes around contours ans distamce plus sigma
#else 
      DbScan dbscan(boxes,dbscan_epsilon,dbscan_mnpts,dbscan_maxpts); //provde boxes around contours ans distamce plus sigma
#endif
      /* ----------------------dbscan timing -----------------------------------*/
     double tdiff_sum = 0; 
     for(int iter = 0; iter < NITER; iter ++){
      auto td1 = std::chrono::high_resolution_clock::now(); // high_resolution_clock::time_point
      dbscan.run();

      auto td2 = std::chrono::high_resolution_clock::now();
      tdiff_sum += (std::chrono::duration_cast<std::chrono::microseconds>(td2 - td1)).count();
      }
      T_dbscan.emplace_back(static_cast<double>(tdiff_sum/NITER)/1000); // ms

      std::vector <cv::Point2f> mc_dbscan = dbscan.centroids;

      DBscanCentroids.push_back(mc_dbscan);

/* -------------------------------------------disjointset auto -------------------------------------------*/
      Disjointset disjointset(points,ds_dsigma); //provide contours ans distance parameter
      /* ----------------------dishointset timing -----------------------------------*/
      double tdiff_sumds = 0; 
     for(int iter = 0; iter < NITER; iter ++){
      auto td1_ds = std::chrono::high_resolution_clock::now(); // high_resolution_clock::time_point
      disjointset.run();
      auto td2_ds = std::chrono::high_resolution_clock::now();
      tdiff_sumds += (std::chrono::duration_cast<std::chrono::microseconds>(td2_ds - td1_ds)).count();
    }
      T_disjoint.emplace_back(static_cast<double>(tdiff_sumds/NITER)/1000); // ms

      std::vector <cv::Point2f> mc_kauto = disjointset.centroids;
      DisjointsetCentroids.push_back(mc_kauto);

 /* ----------------------------------------------  draw clustering --------------------------------------------- */
cv::Mat img_shw = img_sh.clone();

      dbscan.drawCentroids(img_shw,CV_RGB(255,0,255),sizec);   		   // centroid 
      dbscan.drawCentroids(img_shw,CV_RGB(255,0,255),dbscan_epsilon,5);   //epsilon

	writeimg(img_shw,im_out_path + "clustered_db",k, clustersize);

      disjointset.drawCentroids(img_sh,CV_RGB(255,255,0),sizec);   		// centroid
      disjointset.drawCentroids(img_sh,CV_RGB(255,255,0),ds_dsigma,5);  // sigma

      dbscan.drawCentroids(img_sh,CV_RGB(255,0,255),sizec);   		   // centroid 
      dbscan.drawCentroids(img_sh,CV_RGB(255,0,255),dbscan_epsilon,5);   //epsilon

      dbscan.drawNoise(img_sh,CV_RGB(255,0,255),sizec/2,-1,NOISE_COL);

	writeimg(img_sh,im_out_path + "clustered_dbds",k, clustersize);

     if(clustersize<100){
      disjointset.drawCentroidLabels(img_sh,CV_RGB(255,255,0),10.0,5,cv::Point2f(ds_dsigma,ds_dsigma));    
      dbscan.drawCentroidLabels(img_sh,CV_RGB(255,0,255),10.0,5,cv::Point2f(ds_dsigma,ds_dsigma));
	}

	writeimg(img_sh,im_out_path + "clustered_label_dbds",k, clustersize);

	// with the centers drawn
	drawClusters(img_sh,pxy_c,"w");
	writeimg(img_sh,im_out_path + "clustered_label_dbds_complete",k, clustersize);
		
 /* ----------------------------------------------  original image draw  --------------------------------------------- */
		auto colorMap = generateColorMap(clustersize);
		// cluster locations
		drawClusters(img,pxy_c,"w",colorMap);

		writeimg(img,im_out_path + "data",k, clustersize);

		drawClusterPopulation(img,Pxy_p,"s",colorMap);

		//write images 
		writeimg(img,im_out_path + "view",k, clustersize);
 /* ----------------------------------------------  show images --------------------------------------------- */
		cv::imshow("data",img);
		cv::imshow("clustering",img_sh);
 /* ----------------------------------------------  write data --------------------------------------------- */

		fs_o << "pc" +std::to_string(k) << cv::Mat(pxy_c.size(),2,CV_32FC1,pxy_c.data());
		int nthcluster = 0;
		fs_o << "size" + std::to_string(k) << clustersize;

		for(auto p : Pxy_p)
		fs_o << "pc" +std::to_string(k) + "P" + std::to_string(nthcluster++) << cv::Mat(p.size(),2,CV_32FC1,p.data()); // clustersizexpopulationsize
 /* ----------------------------------------------   info  --------------------------------------------- */
		
		if(clustersize<=10){
			std::cout << "xmin " << ymin << std::endl;
			std::cout << "xmax " << ymax << std::endl;
			std::cout << "ymin " << ymin << std::endl;
			std::cout << "ymax " << ymax << std::endl;

			// return 0;		
			std::cout << "Xc " << Xc << std::endl;
			std::cout << "Yc " << Yc << std::endl;

			printVec(pxy_c,"pxy_c: ");
		}

		std::cout << "case: " << k
			<< ", cluster: " << clustersize
			<< ", drawn"
			<< std::endl;

		cv::waitKey(CVWAIT_MS);
		k++;

	}
 /* ----------------------------------------------  write data --------------------------------------------- */
k = 0; std::cout << "writing points data: "<< std::endl;
for(auto p : DBscanCentroids){
    fs_o_db << "p" + std::to_string(k) << cv::Mat(p.size(),2,CV_32FC1,p.data());
k++;
}
//timing
fs_o_db << "T" <<T_dbscan;
std::cout << "written points data dbscan "<< std::endl;

k = 0; std::cout << "writing points data: "<< std::endl;
for(auto p : DisjointsetCentroids){
    fs_o_ds << "p" + std::to_string(k) << cv::Mat(p.size(),2,CV_32FC1,p.data());
k++;
}
//timing
fs_o_ds << "T" <<T_disjoint;
std::cout << "written points data disjoint "<< std::endl;

	return 0;
}

