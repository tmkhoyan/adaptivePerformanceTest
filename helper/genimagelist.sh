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
#  * Description:     helper file for DBSCAN and Disjoint-set data structure classes comparison performance test
#  *                  example compile: make
#  *                  then run with:   ./runBenchcluster
#  *
#  * Author:          Tigran Mkhoyan
#  * Email :          t.mkhoyan@tudelft.nl
#  *
#  */

if [[ "$(uname)" == "Darwin" ]]; then # bash linux cannot deal with this no space ! check this 
    OS=Darwin;
    SUFFIX=osx
    CLOUD_DIR="/Users/tmkhoyan/surfdrive/"
else
	OS=linux
    SUFFIX=linux
    CLOUD_DIR="/media/tmkhoyan/surfdrive/"
fi
echo "OS=$OS"


OPT_IMG_TYPE=jpg
OPT_FILE_OUT_DIR="benchmark_input"
OPT_IMAGEPATH_DIR="data_out" 			     	# R1.  --> use this for jpeg


OPT_COND="data"

OPT_IMAGEPATH=$(pwd)/${OPT_IMAGEPATH_DIR}

OPT_FILE_OUT=${OPT_FILE_OUT_DIR}/${OPT_COND}.txt


[[ -d "$OPT_FILE_OUT_DIR" ]] || mkdir -p "$OPT_FILE_OUT_DIR" # create if doesnt exist

ls "$OPT_IMAGEPATH" | sort -n -t_ -k2 | grep ."$OPT_IMG_TYPE" | grep -i "$OPT_COND" | awk '{print}' | awk -v path="$OPT_IMAGEPATH" '{print path "/" $0}'  > "$OPT_FILE_OUT"

echo " generated image file: $OPT_FILE_OUT"