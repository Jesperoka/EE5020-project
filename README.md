<!-- Set up MathJax LaTeX math rendering -->
<!-- <script src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/latest.js?config=TeX-MML-AM_CHTML"></script>
<script type="text/x-mathjax-config">
MathJax.Hub.Config({
  tex2jax: {
    inlineMath: [['$', '$'], ['\\(', '\\)']],
    processEscapes: true
  }
});
</script> -->

<!-- Set style of document -->
<!-- <style>
body {
    max-width: 800px;
    font-family: Arial, sans-serif;
    font-size: 12pt;
    line-height: 1.5;
    margin: 2cm;
    background-color: #0d1117;
    color: white;
}

h1, h2, h3, h4, h5, h6 {
  font-family: Arial, sans-serif;
  font-weight: bold;
  margin-bottom: 1.5em;
  color: white;
}
h1 {font-size: 20pt;} /*don't think these actually do anything*/
h2 {font-size: 18pt;}
h3 {font-size: 16pt;}
h4 {font-size: 14pt;}
h5 {font-size: 12pt;}
h6 {font-size: 10pt;}

p {
  font-family: "Times New Roman", serif;
  margin-bottom: 1.5em;
  color: white;
}
</style> -->

# EE5020 Sensor Signals and Data Processing - Project Report
## *SIR Particle Filter for Multi-Target Tracking and Clutter Rejection* 

<sub>Note: *This report is written in GitHub Flavored Markdown.*<sub/>

---
TODO: add all sections when done
### Table of Contents
1. [Introdcution](#introduction)
2. [Simulated System Dynamics](#simulated-system-dynamics)
3. [Particle Filter](#particle-filter)
4. [Fourth Example](#fourth-examplehttpwwwfourthexamplecom)
---
### Introduction

In this project, a Rust implementation of the Sequential Importance Resampling (SIR) particle filter for multiple target tracking is presented. The SIR particle filter is a popular algorithm used in state estimation problems, and there are several extensions of the filter for tracking multiple targets in a dynamic environment. By using a simple nearest measurement particle association approach, combined with some thresholding heuristics, a fairly simple implementation is able to track multiple targets with switching dynamics to varying degrees in the face of clutter. This is done with somewhat conservative assumptions, allowing for improvements in an actual application. The main goal of the project to learn about the particle filter and extend it in some fashion. It's written in Rust because I wanted to. 

### Simulated System Dynamics

An arbitrary nonlinear hybrid system is constructed and simulated using 4th order Runge-Kutta for 20 seconds. The system has 3 modes $m \in \{1, 2, 3\}$, but switching was only made to happen between $m=1$ and $m=2$ for two targets, while a third target stays in $m=3$ throughout the simulation. The third mode $m=3$ was constructed to have a target appear into the viewing area of the filter later in the simulation, to test the detection of new targets.

<h4 align="center">Example of True State Dynamics</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/messy_main/results/true_dynamics_3_obj.gif?raw=true" width=350>
</p>

As shown in the GIF above, the modes $m=1,2,3$ correspond to nonlinear systems being controlled to follow circular, figure-eight and linear paths repectively, and we can see jumps between modes $1$ and $2$. The jumps between models, close proximity between objects and intersecting paths give a sufficiently interesting set of targets to track that have, at times, hard to predict behavior.

Further, some noise is added to the measurements. For $m \in \{2,3\}$, i.e. the figure-eight and linear trajectories, the noise is Gaussian, while for $m=1$, i.e. the circular trajectory, the noise is Poisson distributed, biasing the measurement in one direction.

<h4 align="center">Noisy Measurements Originating from Targets</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/messy_main/results/true_dynamics_3_obj_with_measurements.gif?raw=true" width=350>
</p>

Finally, uniform clutter is also added to the measurements, and there is no way to distinguish between clutter and a measurement originating from a target based off any single frame/simulation step.

<h4 align="center">What the Filter Sees with Moderately High Clutter</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/messy_main/results/what_the_filter_sees_15.gif?raw=true
" width=350>
</p>

In the GIF above there are always 15 false measurements. Try to see if you can keep track of the targets, with the prior knowlegde of where they are going to be. It's fairly easy when they are moving in a constant pattern together, but it gets a bit harder when they jump to separate trajectories, and trying to keep your eyes on all three quickly becomes a challenge.

### Particle Filter

The particle filter is a Monte Carlo-based algorithm used to estimate the state of a system given noisy and/or partial observations. It works by representing the posterior distribution of the state using a set of particles, where each particle represents a hypothesis of the state. The particles are updated recursively based on modeled system dynamics, observed data (measurements) and modeled measurement noise, allowing us to approximate the true state distribution.

#### Sequential Importance Resampling

The Sequential Importance Resampling (SIR) algorithm is a specific implementation of the particle filter. It involves 3 main steps: predict, update and resample. In the prediction step, particles are propagated forward in time using the system dynamics. In the update step, particles are weighted according to the likelihood of the observed data and resampled to generate a new set of particles. The resampling step aims to eliminate particles with low weights and duplicate particles with high weights in an attempt to concentrate particles in regions of high posterior probability. This is commonly presented as a solution to the problem of particle degeneracy seen in the Sequential Importance Sampling (SIS) filter, wherein the majority of particles are quickly weighted close to zero, making them useless of the estimation of the posterior distribution of the state.

It should be noted that there are many particle filters, and even more variants to each of those filters. The description layed out here is what you will generally find to be the standard one, and its the one this project is mostly based on. For more info and comparison of different types of filter I would recommend starting with [[1]](#r1).

#### Particle Filter Assumptions

Because this is a simulated system, where the behavior of the true state is known, there is a need to assert what information is available to the particle filter and what is not.

#### Multiple Target Tracking

Something something nearest neightbor local distributions

#### Noise Injection

Something something artificial process noise, artificial weight noise

#### Detecting New Targets

Something something comparison between with and without redistributing

### Implementation in Rust

Do I need this? Include more code snippets elsewhere?

### Results and Qualitative Analysis

Should really get data on average error of estimates, number of false positives/negative etc. but ugh.

### Improvements for Next Time

Something something multi-object state space with variable number of objects combined density for whole state space. Doing that makes estimates easy, no need for fine tuning heuristics.

### Conclusion

Performs alright, learned a lot, will do better next time.

### References

<a id="r1"></a>[1] M.S. Arulampalam; S. Maskell; N. Gordon; T. Clapp (2002). A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. IEEE Transactions on Signal Processing, 50(2), 174 - 188. [https://doi.org/10.1109/78.978374]


---

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

<!-- Tell MathJax to typeset equations present in the document -->
<!-- <script type="text/javascript">
MathJax.Hub.Queue(["Typeset", MathJax.Hub]);
</script> -->