# AFCS (ASV Formation Control Simulator)

**AFCS** is a modular MATLAB simulator for multi-ASV formation control, trajectory generation, and robust obstacle avoidance in various marine environments. It provides ASV dynamics models, environment simulation (wind, waves, currents, static/dynamic obstacles), and distributed/decentralized control algorithms (including MPC, ADMM, graph-based strategies, etc.) for academic research, teaching, and engineering applications.

## Main Features

- Multi-ASV cooperative motion and formation control
- Path and trajectory generation (including task switching and structure reconfiguration)
- Nonlinear MPC with collision avoidance (static and dynamic obstacles)
- Environmental disturbance modeling (wind, wave, current)
- Flexible and extensible code structure, easy for customization
- Rich visualization: animated ASV formation, obstacle interaction, tracking performance, etc.

## Citation

If you use this simulator for academic work, please cite:


For algorithms and hydrodynamic modeling, you may also refer to:

> T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion Control. 2nd. Edition, Wiley. ISBN-13: 978-1119575054
> H. Yasukawa, Y. Yoshimura, Introduction of mmg standard method for ASV maneuvering predictions, Journal of marine science and technology 20 (2015) 37–52.
> M. A. Lewis, K.-H. Tan, High precision formation control of mobile robots using virtual structures, Autonomous robots 4 (1997) 387–403.
> Z. Sun, G. Zhang, Y. Lu, W. Zhang, Leader-follower formation control of underactuated surface vehicles based on sliding mode control and parameter estimation, ISA transactions 72 (2018) 15–24.
> B. S. Park, S. J. Yoo, An error transformation approach for connectivity-preserving and collision-avoiding formation tracking of networked uncertain underactuated surface vessels, IEEE transactions on cybernetics 49 (2018) 2955–2966.
> R. Isherwood, Wind resistance of merchant ASVs, Trans. RINA 115 (1973) 327–338.
> W. Blendermann, Parameter identification of wind loads on ASVs, Journal of Wind Engineering and Industrial Aerodynamics 51 (1994) 339–351.
> Y. Yang, J. Du, H. Liu, C. Guo, A. Abraham, A trajectory tracking robust controller of surface vessels with disturbance uncertainties, IEEE Transactions on Control Systems Technology 22 (2013) 1511–1518.
> G. Wen, J. Lam, J. Fu, S. Wang, Distributed mpc-based robust collision avoidance formation navigation of constrained multiple usvs, IEEE Transactions on Intelligent Vehicles 9 (2023) 1804–1816.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.



## Authors & Acknowledgements

This simulator is developed and maintained by:

- Wenxiang Wu (Wuhan University of Technology)
- Chenguang Liu (Wuhan University of Technology)
- Zhibo He (Wuhan University of Technology)
- Huimin Chen (Wuhan University of Technology)
- Wenzhou Yan (Wuhan University of Technology)
- Chenguang Wang (Wuhan University of Technology)
- Bo Liu (Wuhan University of Technology)
- Haihong Wang (Wuhan University of Technology)
- Xiumin Chu (Wuhan University of Technology)

Special thanks to Prof. Liu for guidance and to all early users who provided feedback.

## Feedback & Contribution

- Bug reports, feature requests, or code improvements are welcome.
- Please submit issues via [GitHub Issues](https://github.com/whutwwx/SFCT-A-toolbox-for-ship-formation-control-toolbox-in-Matlab/issues).
- For major code contributions, please create a pull request and briefly describe your changes.

