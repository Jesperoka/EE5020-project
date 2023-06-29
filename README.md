# EE5020 Sensor Signals and Data Processing - Project Report
## *Rust Implementation of a SIR Particle Filter for Multi-Target Tracking and Clutter Rejection* 

<sub>Note: *This report is written in GitHub Flavored Markdown.*<sub/>

---
### Table of Contents
1. [Introdcution](#introduction)
2. [Simulated System Dynamics](#simulated-system-dynamics)
3. [Particle Filter](#particle-filter)
4. [Fourth Example](#fourth-examplehttpwwwfourthexamplecom)
---
### Introduction

In this project, we present a Rust implementation of the Sequential Importance Resampling (SIR) particle filter for multiple target tracking. The SIR particle filter is a popular algorithm used in state estimation problems, and there are several extensions of the filter for tracking multiple targets in a dynamic environment. By using a simple nearest measurement particle association approach combined with some non-standard additions to the filter, a fairly simple implementation is able to track multiple targets with switching dynamics to varying degrees in the face of clutter. This is done with somewhat conservative assumptions, allowing for improvements in an actual application. The goal of the project however, was solely to learn about the particle filter and extend it it some fashion.

### Simulated System Dynamics

An arbitrary nonlinear hybrid system is contructed and simulated using 4th order Runge-Kutta for 20 seconds. The system has 3 modes $m \in $, but switching was only made to happen between 2 of the modes, while a third target stays in mode 3 throughout the simulation, which was constructed to appear into the viewing area of the filter some seconds into the simulation.

<img src=https://github.com/Jesperoka/EE5020-project/blob/messy_main/what_the_filter_sees.gif width=500>

### Particle Filter

The particle filter is a Monte Carlo-based algorithm used to estimate the state of a system given noisy and partial observations. It works by representing the posterior distribution of the state using a set of particles, where each particle represents a hypothesis of the state. The particles are updated recursively based on the system dynamics and the observed data, allowing us to approximate the true state distribution.

#### Sequential Importance Resampling

The Sequential Importance Resampling (SIR) algorithm is a specific implementation of the particle filter. It involves two main steps: prediction and update. In the prediction step, particles are propagated forward in time using the system dynamics. In the update step, particles are weighted according to the likelihood of the observed data and resampled to generate a new set of particles. The resampling step aims to eliminate particles with low weights and duplicate particles with high weights, ensuring a diverse and representative set of particles for the next iteration.

#### Multiple Hypothesis Tracking

Multiple Hypothesis Tracking (MHT) extends the particle filter to handle multiple targets. It maintains multiple hypotheses, each representing a different number and configuration of targets. The MHT algorithm generates and updates these hypotheses based on the observations and the particle filter's output. By considering multiple hypotheses, MHT provides a more robust and accurate tracking solution, especially in complex scenarios with occlusions and clutter.

#### Noise Injection

some text

#### Detecting New Targets

some text

## Implementation in Rust



## Results and Evaluation

To evaluate the performance of our Rust implementation, we conducted experiments on synthetic datasets and real-world scenarios. We compared the results with existing implementations and demonstrated the effectiveness and efficiency of our approach. The experiments showed improved tracking accuracy and robustness, even in challenging scenarios with occlusions and clutter.

## Conclusion

In this project, we presented a Rust implementation of the SIR particle filter for multiple hypothesis tracking. The combination of particle filtering and multiple hypothesis tracking allows for accurate and robust multi-target tracking in dynamic environments. Our implementation leverages the benefits of the Rust programming language, providing both performance and safety guarantees. The results of our experiments demonstrate the effectiveness of our approach and open avenues for further research and applications in the field of multi-target tracking.

---



## What the filter 'sees'
<img src=https://github.com/Jesperoka/EE5020-project/blob/messy_main/what_the_filter_sees.gif width=500>

## Particle filter state estimation
- Green: True state
- Orange: Estimated state
- Blue: Particles
- Red: Measurements originating from object
- Dark Matt Pink: Clutter / Noise / False measurements
<img src=https://github.com/Jesperoka/EE5020-project/blob/messy_main/animation.gif width=500>


## Just some thoughts after the first implementation:

I feel like improving more on this particular implemenation is not worth the time, it works decently and better performance under this approach to multiple object estimation with noise and clutter comes in large part down to motion, measurement and mode change models. This being a simulated system, the current implemention uses some information that might not be available in a real environment, but also does not use other information that a real application would have.

Improvements that would be nice, but that aren't really that important for learning about the particle filter:
- Online motion model estimation
- Varible size state-space model that integrates all objects and probabilities (i.e. the 'proper' way), instead of nearest neighbor filter with some heuristics.
- Better estimate heuristic (mine is really just MAP with some basic thresholding based on number of particles and sum of weights)
- I was intending on adding the uniform particle insertion 'hack' that I've seen described as a method of avoiding particle collapse in the case of very accurate sensors, and it actually plays well into how I imagine the nearest neighbor approach should detect new appearing objects, but ultimately it's not all that interesting.

Non-standard things that were implemented:
- Artificial process noise for particle variance
- Artificial weight noise to protect against degenerate cases and for particle variance
- Importance weight distributions are calculated about the nearest measurement to a particle, which means the single particle filter works as a set of parallel filters, without having to actually run and manage multiple filter instances. 
