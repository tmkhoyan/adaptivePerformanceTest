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
 * Description:     header file for DBSCAN and Disjoint-set data structure classes comparison performance test
 *                  example compile: make
 *                  then run with:   ./runBenchcluster
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// #include <fstream>
#include <iomanip>   

#include <string>
#include <vector>
#include <map>
#include <math.h>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort
#include <sys/time.h>
#include <sys/stat.h>
#include <iostream>

#include "disjointset.h"
#include "dbscan.h"

#define COMPRESS_JPG

#define PSIZE 10
#define PSIZERECT PSIZE*2.1 // 2x radius of circle as would be the output of contours
#define PSIZEC 20

// #define DB_SCAN_MODE_OLD true // otherwise cant use reorder
#define DMINNORM 1500.0 // minimum distance norm between points!
#define USE_CONTOURS 
#define ADD_NOISE  

#define NOISE_PERC 5 // percentage of cluster centers so twice the cluster centers is 2/50 of population

#define NOISE_COL cv::Scalar(190,128,128)


/*--------------------settings           ------------------*/
int sizec = 20; 			// draw size  of clusters
int psize  = 10;
int psizec = 20;

cv::Scalar pcolor = cv::Scalar(255,255,255); // white
cv::Scalar pcolorc = cv::Scalar(0,0,255); // bgr

std::vector<int> params_jpg;
std::vector <cv::Point2f> pxy_n;

int sizeParticle = 2;


//clusters
float imgw = 10000;
float imgh = 10000;

float xmax = imgh*1.0/10, xmin = imgh*9.0/10; 
float ymax = imgw*1.0/10, ymin = imgw*9.0/10; 

float xmean= xmin + (xmax-xmin)/2, xsigma=(xmax-xmin)/2; 
float ymean= ymin + (ymax-ymin)/2, ysigma=(ymax-ymin)/2; 

// population
float xmaxp = imgh/100, xminp = -imgh/100;
float ymaxp = imgw/100, yminp = -imgw/100;

float xmeanp= xminp + (xmaxp-xminp)/2, xsigmap=(xmaxp-xminp)/2; 
float ymeanp= yminp + (ymaxp-yminp)/2, ysigmap=(ymaxp-yminp)/2; 

/*--------------------initiate containers ------------------*/
const std::string flname="clusterdata"; 
cv::FileStorage fs_o;
cv::FileStorage fs_o_db; //partitioning
cv::FileStorage fs_o_ds; //partitioning

cv::RNG rng;
cv::RNG rng_p;
/*--------------------helper  functions           ------------------*/

template< typename E>
inline cv::Rect centeredRect(E x, E y, E whrect=PSIZERECT){
	cv::Rect_<E> r( x - whrect/2, y - whrect/2,whrect,whrect);
	return r;
}

template<typename T>
std::vector <T> flattenVector(const std::vector<std::vector<T>> &pvv){
	std::vector<T> pv; pv.reserve(pvv.size()*pvv[0].size());
	for(const auto & v: pvv)
		pv.insert(pv.end(),v.begin(), v.end());
	return pv;
}
#ifdef USE_CONTOURS
// using the contours founction
std::vector<cv::Rect> getRectPoints_(const std::vector <std::vector<cv::Point>> &contours){
	std::vector<cv::Rect> boxes; boxes.reserve(contours.size());
	for(auto & pgroups: contours){
		auto r = boundingRect(pgroups);
		boxes.emplace_back(boundingRect(pgroups))
		;}
	return boxes;
}

std::vector<cv::Point2f> getGroupPoints_(const std::vector <std::vector<cv::Point>> &contours){
	std::vector<cv::Point2f> points; points.reserve(points.size());
	for(auto & pgroups: contours){
		auto r = boundingRect(pgroups);
		cv::Point2f p = (r.br() + r.tl())*0.5;
		points.emplace_back(p);
	}
	return points;
}
#else
// use directly the point cloud, is much faster for benchmark!
std::vector<cv::Rect> getRectPoints_(const std::vector <std::vector<cv::Point2f>> &contours,int size_){
	float size = static_cast<float>(size_);
	auto points = flattenVector(contours);
	std::vector<cv::Rect> boxes; boxes.reserve(points.size());
	for(auto & p: points){
		cv::Rect r = centeredRect(p.x,p.y,size);
		std::cout << "r.width " <<  r.width << std::endl;
		std::cout << "r.height" <<  r.height << std::endl;
		boxes.emplace_back(r);
	}
	return boxes;
}

std::vector<cv::Point2f> getGroupPoints_(const std::vector <std::vector<cv::Point2f>> &contours){
	std::vector<cv::Point2f> points; points.reserve(points.size());
	points = flattenVector(contours);
	return points;
}
#endif


template<typename T>
T euclideanDist(const cv::Point_<T>& p,const  cv::Point_<T>& q) {
	cv::Point_<T> diff = p - q;
	return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}
template<typename T>
T angle(const cv::Point_<T>& v1, const cv::Point_<T>& v2)
{
    T cosAngle = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
    if (cosAngle > 1.0)
        return 0.0;
    else if (cosAngle < -1.0)
        return CV_PI;
    return std::acos(cosAngle);
}

template<typename T>
std::vector<std::vector<T >> pairwiseNorm( const std::vector<cv::Point_<T>> &pxy){
	std::vector<std::vector<T>> dpxy; dpxy.reserve(pxy.size()); //nxn
	for(auto &p: pxy){
		std::vector<T> dpxy_row; dpxy.reserve(pxy.size()); //1xn
		for(auto &q: pxy){
			auto d = euclideanDist(p,q);
			dpxy_row.emplace_back(d);
		}
		dpxy.emplace_back(dpxy_row);
	}
	return dpxy;
}
template<typename T>
bool checkPairwiseNorm (const std::vector<cv::Point_<T>> &pxy, T dmin){
	int m=0;
	for(auto &p: pxy){
		int n=0;
		for(auto &q: pxy){
			// std::cout << n << ", " << m << std::endl;
			if(euclideanDist(p,q)<dmin and n!=m){ 
			std::cout << "norm (" << dmin <<") n,m,dist = " << n <<"," << m << "," <<  euclideanDist(p,q) << std::endl;

			return true;
			}; // filter diagonal elements
			n++;
		}
		m++;
	}
	return false;
}

template <typename T>
inline std::vector<T> mat2point(cv::Mat x,cv::Mat y){
	typedef typename T::value_type E;
	cv::MatIterator_<E> it_x; // use x to iterate
	cv::MatIterator_<E> it_y; // advance y manually 
	std::vector <cv::Point_<E>> v; v.reserve(x.rows);

	it_y = y.begin<E>();   // set y beforfe the loop
	for(it_x = x.begin<E>();it_x != x.end<E>();++it_x){
		auto x = (*it_x); 			// assign x and advance y
		auto y = (*it_y); it_y++; 	//advance y manualy
		v.push_back(cv::Point_<E>(x,y));
	}
	return v;
}

template<typename T>
inline void print2dvec(const std::vector <std::vector<T>> &vpq,std::string h="", int precision=4){
	std::cout << h << std::endl
		<< "[" << std::endl;
	for(auto vq: vpq){
		for(auto p: vq){
			std::cout <<  std::setprecision(precision) << p << " ";
		}
		std::cout << "\n";
	}
	std::cout << "]" << std::endl;

}

template <typename T>
inline void printVec(const std::vector <T> &v,std::string h=""){
	std::cout << h << std::endl
		<< "[" << std::endl;
	std::for_each(v.begin(),v.end(),
			[](const T &s){std::cout << s << std::endl;});
	std::cout << "]" << std::endl;
}
void writeimg(cv::Mat img_, std::string h, int k, int clustersize){
	//write images 
	std::stringstream ss; 
	ss <<  h << k << "_" << clustersize << ".jpg";
	cv::imwrite(ss.str(),img_,params_jpg);
	std::cout << "img written : " << ss.str() << std::endl; 

}
/*--------------------draw functions           ------------------*/
//. by default if empty default color red for cluster white for points
void drawClusters(cv::Mat &img_, const std::vector<cv::Point2f> pxy_, std::string opt="s", const std::vector<cv::Scalar> &colorMap_=std::vector<cv::Scalar>(), int psizec=PSIZEC,int thickness=-1){
	if(opt=="w" or colorMap_.empty()){
		for(auto &p: pxy_ )
			cv::circle(img_, p , psizec, pcolorc, thickness);//, 8, 0 );
	}
	else{
		int k = 0;
		for(auto &p: pxy_ )
			cv::drawMarker(img_, p, colorMap_[k++], cv::MARKER_CROSS, psizec*5, 2);
		// cv::circle(img_, p , psizec, pcolorc, -1);//, 8, 0 );
	}
}
void drawClusterPopulation(cv::Mat &img_, const std::vector<std::vector<cv::Point2f>> &Pxy_,std::string opt="s", const std::vector<cv::Scalar> &colorMap_=std::vector<cv::Scalar>(),int psize=PSIZE, int thickness=-1){
	if(opt=="w" or colorMap_.empty()){
		for(auto &pxy: Pxy_)
			for(auto &p: pxy)
				cv::circle(img_, p , psize, pcolor, thickness);//, 8, 0 );

	} else{
		int k = 0;
		for(auto &pxy: Pxy_){
			for(auto &p: pxy){
				cv::circle(img_, p , psize, colorMap_[k], thickness);//, 8, 0 );
			}
			k++;
		}
	}
	// draw noise
#ifdef ADD_NOISE
			for(auto &p: pxy_n)
				cv::circle(img_, p , psize, pcolor, thickness);//, 8, 0 );
#endif
}
inline std::vector<cv::Scalar> generateColorMap(size_t n){
	cv::RNG rng(12345);
	std::vector<cv::Scalar>colorMap; colorMap.reserve(n);
	for(int i=0;i<n;i++){
		colorMap.push_back(cv::Scalar(rng.uniform(50,255), rng.uniform(50, 255), rng.uniform(50, 255)));
	}
	return colorMap;
}
inline std::string makeValidDir(const std::string & str){

	if (mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
	{
		if( errno == EEXIST ) { // alredy exists
			std::cout << "folder: " << str << " --- exists. " << std::endl;
		} else {
			std::cout << "cannot create folder error:" << strerror(errno) << std::endl;
			exit(0);
		}
	}
	return (str.back() == '/' ? str : (str +"/"));
}
