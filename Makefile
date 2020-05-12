#!/bin/bash

# /* BSD 3-Clause License
#  *  
#  *  Copyright (c) 2020, tmkhoyan (Tigran Mkhoyan)
#  *  All rights reserved.
#  *  
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions are met:
#  *  
#  *  1. Redistributions of source code must retain the above copyright notice, this
#  *     list of conditions and the following disclaimer.
#  *  
#  *  2. Redistributions in binary form must reproduce the above copyright notice,
#  *     this list of conditions and the following disclaimer in the documentation
#  *     and/or other materials provided with the distribution.
#  *  
#  *  3. Neither the name of the copyright holder nor the names of its
#  *     contributors may be used to endorse or promote products derived from
#  *     this software without specific prior written permission.
#  *  
#  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  */

# /*
#  * Description:     makefile for performance test DBSCAN and Disjoint-set data structure 
#  *                  example compile: make
#  *                  then run with:   ./runBenchcluster
#  *
#  * Author:          Tigran Mkhoyan
#  * Email :          t.mkhoyan@tudelft.nl
#  *
#  */
	# Makefile for real time triangulation programm
.PHONY: all clean

# The program to build
# NAME       := auto_dbscan_distpart_save
NAME       := benchclustering

# Installation directories for pylon
# PYLON_ROOT ?= /opt/pylon5


OS=$(shell uname)
CVVERSION=$(shell opencv_version | cut -c 1-1)
# BINDIR ="bin/"

# ----------------------------specify----------------------------------------------#
# ----------------------------Input  ----------------------------------------------#
# ----------------------------path ------------------------------------------------#



OPT_IMAGEPATH_DIR:="d2_c2_f5_g10_30/frames" 			     	# R1.  --> use this for jpeg
# OPT_IMAGEPATH_DIR="d2_c2_f5_g5_30/frames" 			     	# R2
# OPT_IMAGEPATH_DIR:="d3_c2_sweep_g10_30_corr/frames" 	     	# R3
OPT_IMG_TYPE:=tif


OPT_IMAGEPATH_DIR="d2_c2_f5_g10_30/frames_jpg" 			     # R1.  --> use this for jpeg
# # OPT_IMAGEPATH_DIR="d2_c2_f5_g5_30/frames_jpg" 			     # R2
OPT_IMAGEPATH_DIR="d3_c2_sweep_g10_30_corr/frames_jpg" 	     # R3
OPT_IMG_TYPE:=jpg


OPT_COND:=R3


# adds space 
# ifeq ($(OPT_IMG_TYPE),jpg)
# # 	OPT_IMAGEPATH_DIR:=$(strip $($(OPT_IMAGEPATH_DIR)_$(OPT_IMG_TYPE)))
# 	OPT_IMAGEPATH_DIR:=$(echo $($(OPT_IMAGEPATH_DIR)_$(OPT_IMG_TYPE)))
# else
# 	OPT_IMAGEPATH_DIR:=$(OPT_IMAGEPATH_DIR)
# endif


OPT_FILE_OUT_DIR:="out"

# ------------------------------ some variables ----------------------------------------

ifeq ($(OS),Darwin) # Mac OS X
    SUFFIX=osx
    CLOUD_DIR="/Users/tmkhoyan/surfdrive/"
    BASE_DIR="/Users/tmkhoyan/Desktop/data_wt/"
endif

ifeq ($(OS),Linux)  
    SUFFIX=linux
    CLOUD_DIR="/media/tmkhoyan/surfdrive/"
    BASE_DIR="/media/tmkhoyan/Desktop/data_wt/"
endif

#cam 1 or 2
#OPT_CAM="CAM TE"
OPT_CAM="CAM TE" # current
OPT_CAM_TE="CAM TE" # current
OPT_CAM_LE="CAM LE" # current

OPT_IMAGEPATH:=$(BASE_DIR)$(OPT_IMAGEPATH_DIR)

OPT_FILE_OUT_LE=$(OPT_FILE_OUT_DIR)/input_le_$(OPT_IMG_TYPE)_$(OPT_COND)_$(SUFFIX).txt
OPT_FILE_OUT_TE=$(OPT_FILE_OUT_DIR)/input_te_$(OPT_IMG_TYPE)_$(OPT_COND)_$(SUFFIX).txt

# ----------------------------macros  ----------------------------------------------#

# OS problem awk -v path="$(OPT_IMAGEPATH)" quotes
define generate_imagelist_le
	ls $(OPT_IMAGEPATH) | sort -n -t_ -k2 | grep .$(OPT_IMG_TYPE) | grep -i $(OPT_CAM_LE) | gawk '{print}' | awk -v path=$(OPT_IMAGEPATH) '{print path"/"$$0}' >  $(OPT_FILE_OUT_LE)
endef

define generate_imagelist_te
	ls $(OPT_IMAGEPATH) | sort -n -t_ -k2 | grep .$(OPT_IMG_TYPE)| grep -i $(OPT_CAM_TE) | awk -v path=$(OPT_IMAGEPATH) '{print path"/" $$0}' > $(OPT_FILE_OUT_TE)
endef

# Build tools and flags
CXX+=-std=c++11
LD         := $(CXX)
CPPFLAGS   := -I /usr/local/include/opencv4 -I /usr/local/Cellar/tbb/2019_U3_1/include
CXXFLAGS   := -o3#e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := -I /usr/local/include/opencv4 -I /usr/local/Cellar/tbb/2019_U3_1/include
LDLIBS     := -L /usr/local/Cellar/tbb/2019_U3_1/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -pthread 


# Rules for building
all: $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)
# 	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS) jetsonGPIO.c

$(NAME).o: $(NAME).cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME).o $(NAME)

debug: 
	@echo  "$(OPT_IMAGEPATH)$(OPT_IMAGEPATH)" ;

genimagelist:
	@echo "Imagelist generated see file [imagelist_appended.txt]..." 	;\
	echo "for images at $(OPT_IMAGEPATH) ..."			;\
	$(generate_imagelist_le)	;\
	$(generate_imagelist_te)

# 	-I/opt/pylon5/include
# -Wl,--enable-new-dtags -Wl,-rpath,/opt/pylon5/lib64
# -L/opt/pylon5/lib64 -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

# g++ -std=c++11 -I/opt/pylon5/include  -DUSE_GIGE -I /usr/local/include/opencv4  -c -o triangulate_rt.o triangulate_rt.cpp
# g++ -std=c++11 -Wl,--enable-new-dtags -Wl,-rpath,/opt/pylon5/lib64 -I /usr/local/include/opencv4 -o triangulate_rt triangulate_rt.o -L/opt/pylon5/lib64 -Wl,-E -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -pthread 