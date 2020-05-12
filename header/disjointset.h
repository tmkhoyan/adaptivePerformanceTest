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
 * Description:     header files for DBSCAN and Disjoint-set data structure classes
 *                  example compile: make
 *                  then run with:   ./runBenchcluster
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 *
 */

#pragma once

#include <map>
#include <sstream>
#include <math.h>
/* -------------------------------------------auto kmeans contours -------------------------------------------*/

class Disjointset
{
	public:

		std::vector<cv::Point2f> pdata;
		int distance;
		int nclusters;
		std::vector<cv::Point2f> centroids;
		std::vector<int> labels;
		std::map<int, int> grouplabels;
		std::map<int, int> grouplabels_reordered; // is empty at first reorder then we fill it using last reordered index
		std::vector<unsigned int> lastReorderedIndex; // after reordering such that we keep track of last reorder
		bool isReordered=false;

		Disjointset(std::vector<cv::Point2f> _pdata, int _distance): pdata(_pdata), distance(_distance){
			// for(auto x: contours){for(auto y: x){pdata.push_back(y);}} //unroll cintours vector and push back in one vector of points
			nclusters = 0;
		}; // second initialize with contours (group of points)
		~Disjointset(){
			// std::cout << "Kmeans object deleted" << std::endl; //will run itself
		}; //use brackets otherswise produces an error

		void run(){

			cv::Mat cntlabels, centers; //labels of contours function and the centers returned. This will be used to fuill the centroids

			//partitions based on euclidian distance
			int sqrtDistance = distance * distance; // criteria for treshold
			nclusters = cv::partition(pdata, labels, [sqrtDistance](const cv::Point& lhs, const cv::Point& rhs) {
					return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < sqrtDistance;
					});

			//get sorted point groups
			std::vector<std::vector<cv::Point2f>> pgroups = getPointGroups();

			centroids = getCentroids(); //also sets grouplabels
			for(size_t k=0; k<centroids.size();k++){grouplabels[k] = k;} //grouplabels is a map

			//UPDATE: not needed anymore. Better to use ouput of partition and take the centroids
			//perform k-means clustering
			//cv::kmeans(pdata, nclusters, cntlabels,cv::TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),3, KMEANS_PP_CENTERS, centers);
			//finally get centroids of the points this is given by kmeans
			// for(int k=0; k< centers.rows; k++){centroids.push_back(centers.at<Point2f>(k));}
			std::vector<unsigned int> idx(nclusters); // kep int to deal with -1 
			std::iota(idx.begin(), idx.end(), 0); // fill index 
			lastReorderedIndex = idx; // initial index just 1:nclusters

		}

		std::vector<std::vector<cv::Point2f> > getPointGroups(){
			// You can save all points in the same class in a vector (one for each class), just like findContours
			std::vector<std::vector<cv::Point2f>> pgroups(nclusters);
			// std::vector<std::vector<cv::Point2f>> grouplabels(nclusters);
			for (size_t i = 0; i < pdata.size(); ++i)
			{
				pgroups[labels[i]].emplace_back(pdata[i]); //
			}
			return (grouplabels_reordered.empty()? pgroups : reorderVectorMap(pgroups,grouplabels_reordered));

		}
		std::vector<cv::Point2f> getCentroids(){
			std::vector<std::vector<cv::Point2f>> pgroups = getPointGroups();
			std::vector <cv::Point2f> mc(pgroups.size());
			int k = 0;
			for(auto pgroup: pgroups){
				float sumx = 0; float sumy =0;
				size_t size = pgroup.size();
				for(auto p: pgroup){
					sumx += p.x;
					sumy += p.y;
				}
				mc[k].x = sumx/size;
				mc[k].y = sumy/size;
				k++;
			}
			return mc;
		}  
				//reordercentrooids vector based on idx provided 
		inline void reorderCentroids(const std::vector<unsigned int> &indexVector){
			std::vector <cv::Point2f> centroids_reordered_;
			std::map<int, int> grouplabels_reordered_;

			int k = 0;
			for(auto idx: indexVector){
				centroids_reordered_.emplace_back(centroids[idx]);
				grouplabels_reordered_[k++] = lastReorderedIndex[idx]; // key(sorted index) --> value (desired index)
				// std::cout << "idx " << idx <<  ",lastReorderedIndex[idx] " << lastReorderedIndex[idx] <<  std::endl;

			}
			grouplabels_reordered = grouplabels_reordered_;

			// reoder index vector so that we cam keep rotating
			lastReorderedIndex = reorderVector(lastReorderedIndex,indexVector);

			// grouplabels_reordered = (!grouplabels_reordered.empty())? : (indexVector.size() == centroids.size())? std::move(grouplabels_reordered_) : grouplabels_reordered_;
			centroids = (indexVector.size() == centroids.size())? std::move(centroids_reordered_) : centroids_reordered_;
			// std::cout << centroids << std::endl;
			// std::cout << centroids_reordered_ << std::endl;
			isReordered = true;
		}
				template <typename T>
			inline  std::vector<T> reorderVectorMap(std::vector<T> &v,const std::map<int, int> &indexMap){
				std::vector<T> sorted;

				for(auto idx: indexMap){
					sorted.emplace_back(v[idx.second]);
					// std::cout << "idx first: " << idx.first  << ","  << "idx second: " << idx.second << std::endl;
				}

				return sorted;
			}
		template <typename T>
			inline  std::vector<T> reorderVector(const std::vector<T> &v,const std::vector<unsigned int> &indexVector){
				std::vector<T> sorted;
				std::for_each(indexVector.begin(),indexVector.end(),
						[&v,&sorted](const unsigned int & index){sorted.emplace_back(v[index]);});
				return sorted;
			}  	
  /* ----------------------------  drawing funcs   -----------------------------------------*/

		void drawDataLabels(cv::Mat &img_color, cv::Scalar _color, double size=0.5,int thickness=1){
			// int k=0;
			for(size_t i=0;i<pdata.size();i++)
			{
			     putText(img_color,std::to_string(labels[i]),pdata[i],  cv::FONT_HERSHEY_COMPLEX,size,_color,thickness);
			}
		} 
		// invariant to order
		void drawCentroids(cv::Mat &img, cv::Scalar color,int size,int thickness=-1){
			for(auto p: centroids){
        		cv::circle(img, cv::Point2d(floor(p.x),floor(p.y)), size, color, thickness);//, 8, 0 );
    		}
		}
		void drawCentroidLabels(cv::Mat &img_color, cv::Scalar _color, double size=0.5,int thickness=1,cv::Point2f offset=cv::Point2f(0,0)){
			// int k=0;
			for(size_t i=0;i<centroids.size();i++)
			{
			     cv::putText(img_color,std::to_string(grouplabels[i]),centroids[i]+offset,  cv::FONT_HERSHEY_COMPLEX,size,_color,thickness);
			}
		}    
};


