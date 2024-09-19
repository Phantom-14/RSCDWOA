# RSCDWOA

RSCDWOA is applied to prevent the whole path falling into local optimum based on the reverse search (RS), chaos theory, differential evolution (DE), and whale optimization algorithm (WOA) methods. Users can optimize various types of problems, including trajectory planning and optimal control, based on the provided RSCDWOA function files.


## About

If our repo helps your academic projects, please cite our paper. Thank you!

Author: Yutong Zhu and Ye Zhang

Paper: [A hybrid optimization algorithm for multi-agent dynamic planning with guaranteed convergence in probability](https://doi.org/10.1016/j.neucom.2024.127764), Ye Zhang, Yutong Zhu, Haoyu Li, and Jingyu Wang. In [*Neurocomputing*](https://www.sciencedirect.com/journal/neurocomputing).

```
@article{zhang2024hybrid,
  title={A hybrid optimization algorithm for multi-agent dynamic planning with guaranteed convergence in probability},
  author={Zhang, Ye and Zhu, Yutong and Li, Haoyu and Wang, Jingyu},
  journal={Neurocomputing},
  volume={592},
  pages={127764},
  year={2024},
  publisher={Elsevier}
}
```

## Applications

### Example 1: Global Trajectory Planning on APF

This is a typical example of using RSCDWOA to achieve global optimality in trajectory planning, which avoids local optimality. For installation, the following commands may be helpful. 

```
git clone https://github.com/Phantom-14/RSCDWOA.git
cd ..
```

After installing the function files, you can use the functions to write the main program. Here we apply the algorithm to the APF for trajectory planning. 
