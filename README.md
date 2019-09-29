# Adaptive Control for UAVs  #

Author: Jesus Tordesillas

This repo has the code implemented for the project of Nonlinear Control (2.152) at MIT.

```
@techreport{jtordeadaptive2019,
     title = {{Parameter Estimation and Dynamic Extension for a Quadrotor}},
     author = {Jesus Tordesillas},
     year = {2019},
     url = {https://github.com/jtorde/uav_adaptive_control},
     institution = {Aerospace Controls Lab},
     month = {05}
}
```

### Introduction:
Implementation in MATLAB of the following adaptive controllers:

* Tracking error based (TEB)
* Constant gain (CG)
* Bounded-gainforgetting (BGF)
* Cushioned-floor (CF)

![](./imgs/adaptive_control.PNG) 

See the [pdf file](https://github.com/jtorde/uav_adaptive_control/blob/master/Project_Jesus_Tordesillas_Torres.pdf) for a detailed description.

### Instructions:
Clone this repository:
```
git clone https://github.com/jtorde/uav_adaptive_control
```
and execute the file `main.m` in MATLAB

### Results for constant parameters:

* **Estimated parameters:**
![](./imgs/estimated_parameters1.png) 

* **Position error:**
![](./imgs/position_errors1.png) 

* **Inputs:**
![](./imgs/inputs1.png) 

### Results for time-varying parameters:

* **Estimated parameters:**
![](./imgs/estimated_parameterstime_var.png) 

* **Position error:**
![](./imgs/position_errorstime_var.png) 

* **Inputs:**
![](./imgs/inputstime_var.png) 

### Feedback Linearization:


![](./imgs/feedback_lin.png) 


### License
Academic license - for non-commercial use only
