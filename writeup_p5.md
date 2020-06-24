# **Extended Kalman filter project** 

## Writeup of Project

### Here is a simple record how I implemented the project and how is the result.

---

**Extended kalman filter Project**

The goals / steps of this project are the following:
* 1. complement the TO-DO code
* 2. predict the car position with KF and EKF


[//]: # (Image References)

[image1]: ./output/dataset_1_result.PNG "results of dataset1"
[image2]: ./output/dataset_2_result.png "results of dataset2"
[image3]: ./output/fusion_flow.PNG "results of dataset2"



## Encountered problems


My project encontered below problems:
1. std::bad_alloc event happened when runing the 'ExtendedKF' file
    This bug is beause kalman_filter equation's Matrixes dimension are not equal 
2. The RMSE is not satisfied
    I searched in the 'knowledge forum', I found that I forgot to transfer xy coordinate to polor coordinate, and using "y = z - H_j * x_" instead. This leads to big RMSE. 

## My work flow summary
1. complete the tools.cpp with function `caculateRMSE()` and `CaculateJacobian()`
2. complete the kalman_filter.cpp with function `KalmanFilter::Predict() ` and `KalmanFilter::Update()` and `KalmanFilter::UpdateEkf()`, I have a better understand of eigen lib with matrix operation.
3. complete the fusionEKF.cpp to initialize vaiables.
After implementing this flow and seeing the compiled file working well, I have a better understand how the different are work together!
![alt text][image3]

## My results
1. result of dataset 1:
![alt text][image1]
2. result of dataset 2:
![alt text][image2]



### What to explore next
* read the bonus paper to figure how the kalman filler applied in tracking problems.