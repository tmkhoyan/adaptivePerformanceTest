# autoLabelTool
A simple but effective automatic annotation tool with a flexible customisable detector for pixel level annotation accuracy 

## Installation 

### 1: go to main folder

```bash
cd ./autoLabelTool
make 
```
## Keyboard mapping

| KeyStroke | Action       | ASCII Code |
|-----------|--------------|------------|
| A         | set AUTO     |            |
| AA        | set AUTO_SEQ |            |
| M         | set MANUAL   |            |

## 4 : References
Please use the following DOI to cite cvyamlParser: 
[![DOI](https://zenodo.org/badge/184505001.svg)](https://zenodo.org/badge/latestdoi/184505001)

## 5: Licence 
Please refer to the licence file for information about code distribution, usage and copy rights. The code is provided under BSD 3-Clause License. 
Refer to the links below for additional licence info regarding OpenCV:
https://opencv.org/license/

## Usage

```bash
cd ./autoLabelTool
./runMain 
```
<!---
### 2: Run benchmark on you own pc:
In folder benchmark a simple script is provided to run readcvYaml on your own data.
Simply choose the number of iterations with N parameter and run benchmarktest_cvYaml.m. The benchmark was performed for 5x[1000x3] double, 5x[2000x3] float, 5x[2000x3] int, and 5x[3000] double, 5x[6000] float and 5x[6000] int vectors. See test_data.yaml for the actual dataset.

Here the result of the benchmark test on linux Optiplex system. The sorting is slightly more expensive as expected but negligible for the current dataset.

Average t per iteration    |  Boxplot data
:-------------------------:|:-------------------------:
![](misc/time_data.png)    |  ![](misc/boxplot.png)

-->
