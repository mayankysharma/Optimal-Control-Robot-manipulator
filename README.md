# ENPM667 Project 1
# An Optimal Control Approach to Robust Control of Robot Manipulators
## Group of 2:
* Yashveer Jain : 119252864
* Mayank Sharma : 119203859

## Overview:
With Uncertainty in state dynamics and input matrix, for Robot Manipulator, the controlling become difficult and there different ways to design controller, either using Robust Control Approach or Optimal Control Approach. Here we're implementing the paper which uses Optimal Control Approach to solve the controlling problem of Robot Manipulator.

## Results for $\epsilon = 0$
![](results/epsilon0.png)


## Install dependencies
```
pip install -r requirements.txt
```

## Run
```
python3 code/scara.py <path/to/store/result image file> <epsilon value b/w (0,1)>
```
* example:
```
python3 code/scara.py results/e_0.png 0
```

## Interactive code
Interactive code can be found [here](code/interative_scara.ipynb)

## Dependencies
* numpy
* matplotlib
* tqdm
* sympy

## Reference:
* An optimal control paper
* https://automaticaddison.com/linear-quadratic-regulator-lqr-with-python-code-example/

