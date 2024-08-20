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
h1 {font-size: 20pt;}
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

<sub>Note: *This report is written in GitHub Flavored Markdown. Some content is invisible if you are viewing this in GitHub with Light Theme*<sub/>

---
### Table of Contents

1. [Introduction](#introduction)<br>
2. [Simulated System Dynamics](#simulated-system-dynamics)<br>
3. [Particle Filter](#particle-filter)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.1 [Sequential Importance Resampling](#sequential-importance-resampling)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.2 [Particle Filter Assumptions](#particle-filter-assumptions)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.3 [Number of Particles](#number-of-particles)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.4 [Multiple Target Tracking](#number-of-particles)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.5 [Detecting New Targets](#number-of-particles)<br>
4. [Results and Qualitative Analysis](#results-and-qualitative-analysis)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;4.1 [Main Criticism](#main-criticism) <br>
5. [Improvements for Next Time](#improvements-for-next-time)<br>
6. [Final Words](#final-words)<br>
7. [References](#references)<br>
---
### Introduction

In this project, a Rust implementation of the Sequential Importance Resampling (SIR) particle filter for multiple target tracking is presented. The SIR particle filter is a popular algorithm used in state estimation problems, and there are several extensions of the filter for tracking multiple targets in a dynamic environment. By using a simple nearest measurement particle association approach, combined with some thresholding heuristics, a fairly simple implementation is able to track multiple targets with switching dynamics to varying degrees in the face of clutter. This is done with somewhat conservative assumptions, allowing for improvements in an actual application. The main goal of the project to learn about the particle filter and extend it in some fashion. It's written in Rust because I wanted to start learning the language. 

### Simulated System Dynamics

An arbitrary nonlinear hybrid system is constructed and simulated using 4th order Runge-Kutta for 20 seconds. The system has 3 modes $m \in \{1, 2, 3\}$, but switching was only made to happen between $m=1$ and $m=2$ with a probability of $p_{jump}=0.01$ for two targets, while a third target stays in $m=3$ throughout the simulation. The third mode $m=3$ was constructed to have a target appear into the viewing area of the filter later in the simulation, to test the detection of new targets.

<h4 align="center"><a id="tsd"></a>Example of True State Dynamics</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/true_dynamics_3_obj.gif?raw=true" width=350>
</p>
<p align="center"><strong>Legend:</strong> Green: true state positions</p>

As shown in the GIF above, the modes $m=1,2,3$ correspond to nonlinear systems being controlled to follow circular, figure-eight and linear paths repectively, and we can see jumps between modes $1$ and $2$. The jumps between models, close proximity between objects and intersecting paths give a sufficiently interesting set of targets to track that have, at times, hard to predict behavior.

Further, some noise is added to the measurements. For $m \in \{2,3\}$, i.e. the figure-eight and linear trajectories, the noise is Gaussian, while for $m=1$, i.e. the circular trajectory, the noise is Poisson distributed, biasing the measurement in one direction.

<h4 align="center">Noisy Measurements Originating from Targets</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/true_dynamics_3_obj_with_measurements.gif?raw=true" width=350>
</p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Red: measurements</p>

Finally, uniform clutter is also added to the measurements, and there is no way to distinguish between clutter and a measurement originating from a target based off any single frame/simulation step.

<h4 align="center">What the Filter Sees with Moderately High Clutter</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/what_the_filter_sees_15.gif?raw=true" width=350>
</p>
<p align="center"><strong>Legend:</strong> Red: measurements (clutter and target oriented)</p>

In the GIF above there are always 15 false measurements. Try to see if you can keep track of the targets, with the prior knowlegde of where they are going to be. It's fairly easy when they are moving in a constant pattern together, but it gets a bit harder when they jump to separate trajectories, and trying to keep your eyes on all three quickly becomes a challenge.

### Particle Filter

The particle filter is a Monte Carlo-based algorithm used to estimate the state of a system given noisy and/or partial observations. It works by representing the posterior distribution of the state using a set of particles, where each particle represents a hypothesis of the state. The particles are updated recursively based on modeled system dynamics, observed data (measurements) and modeled measurement noise, allowing us to approximate the true state posterior distribution.

#### Sequential Importance Resampling

The Sequential Importance Resampling (SIR) algorithm is a specific implementation of the particle filter. It involves 3 main steps: predict, update and resample. In the prediction step, particles are propagated forward in time using the system dynamics. In the update step, particles are weighted according to the likelihood of the observed data and resampled to generate a new set of particles. The resampling step aims to eliminate particles with low weights and duplicate particles with high weights in an attempt to concentrate particles in regions of high posterior probability. This is commonly presented as a solution to the problem of particle degeneracy seen in the Sequential Importance Sampling (SIS) filter, wherein the majority of particles are quickly weighted close to zero, making them useless of the estimation of the posterior distribution of the state.

It should be noted that there are many particle filters, and even more variants to each of those filters. The description layed out here is what you will generally find to be the standard one, and its the one this project is mostly based on. For more info and comparison of different types of filter I would recommend starting with [[1]](#r1).

#### Particle Filter Assumptions

Because this is a simulated system, where the behavior of the true state is known, there is a need to assert what information is available to the particle filter and what is not. 

The filter has, as mentioned access to all measurements including the clutter, and can not distinguish between them, i.e. no signal amplitudes or the like.

Motion modes are modeled as:

<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/filter_modes_hd.png?raw=true" width=300>
<p>

with a Markov chain transition matrix as:

<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/filter_transition_matrix.png?raw=true" width=450>
<p>

so it's clear that the filter does not have a completely accurate model of the true dynamics, but there is some notion of how likely it is to go from, for instance, going straight quickly to turning left or right sharply. Additionally, the measurements given to the filter only convey position, while the filters motion model state also contains a heading angle. In a real application, one would obviously try to get as good a model as possible, possibly by adaptive means.

The filter also employs the addition of artificial process noise in the prediction phase, for greater particle diversity, which in general helps with robustness against modeling errors.

#### Number of Particles

So far, no mention has been made to the number of particles to use in the particle filter. The results of the standard SIR filter come from approximating the analytical form of the posterior distribution of the state, that arises from applying Bayes' theorem, with a finite sum. The approximation error decreases, as you might imagine, with the number of particles, and approaches zero as the number of particles approach infinity. 

One can try to adaptively change the number of particles in order to best use computational resources and keep error below a certain value, but that was not attempted in this implementation. 

For practical reasons related to getting consistent results and making changes to certain parts of the implementation that will be discussed later, a particle amount of 5000 was chosen in the end, as a fairly high amount of particles, that still allowed relatively quick iterations to the codebase. At earlier stages of development, as low as 50 and 500 particles were used without much issue, but as the scope extended to tracking multiple targets in noisy conditions, it was deemed better to just set a high particle number to eliminate a variable from the development process.

#### Multiple Target Tracking

There are multiple ways to approach tracking multiple targets. What could be said to be the proper/direct way to do it, is to include the number of targets are part of the state. This was not done here, because the project developed from a simple single target particle filter, and there is certainly some complexity in estimating how many targets exist in the face of clutter. If the number of targets is known, then there is no issue. A possible future project is to do it the proper way, simultaneously estimating the number of targets and including that number into a single combined state for all targets.

Either way, the approach taken here is to associate particles to their nearest measurement, and perform the update and resample steps relative to that measurement. In this way we form multiple approximations of the posterior distribution of the state around measurements. If there is no clutter / false measurments, then just pick the highest weight particle in each group as you state estimate and you're done.

<h4 align="center">MAP Estimates Without and With Clutter<br>Left: clutter amount = 0,&nbsp Right: clutter amount = 3</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_0_clutter_no_uniform_MAP.gif?raw=true" width="47%">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_3_clutter_no_uniform_MAP.gif?raw=true" width="47%">
<p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Dark Matt Pink: clutter | Red: measurements | Blue: particles</p>

As we can see from the GIF on the left above, targets that come into view later will not be detected until they come close enough to a group of particles. The GIF on the right reveals another issue that will be discussed shortly.

#### Detecting New Targets

To be able to detect new targets, we need particles around the new target. One idea is to inject some uniformly distributed particles every iteration, which should allow for particles to gather around the new target over time. Random particle injection is also sometimes used to combat the loss of diversity that comes with the removal and duplication of particles in the resampling step. The approach here though, is to skip the middle-man, and take the $k$ lowest weight particles, and redistribute them uniformly around randomly chosen measurements. In practice all measurements get some particles distributed around them.

<h4 align="center">Redistributing Particles<br> clutter amount = 0</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_0_clutter_with_uniform_MAP.gif?raw=true" width=350>
<p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Dark Matt Pink: clutter | Red: measurements | Blue: particles</p>

Since the clutter does not move like the targets, we might expect the weight of particles that might get distributed around the false measurments to immediately become close to zero, and we should be able to simply threshold estimates on the sum of the weights in a given particle group.

<h4 align="center">Redistributing Particles<br> clutter amount = 15</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_15_clutter_with_uniform_sum_threshold.gif?raw=true" width=350>
<p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Dark Matt Pink: clutter | Red: measurements | Blue: particles</p>

Unfortunately, when there's a fair amount of clutter, life's not that easy, since clutter can appear close together on consecutive timesteps. This brings us to how we can get the filter to perform, and to the main criticism of the approach taken. 

### Results and Qualitative Analysis

The quick-and-dirty solution was to create and fine-tune heuristics to eliminate state estimates in the spurious particle groups, to best as possible detect only the targets. The properties chosen to filter out (pun intended) the wrong estimates, **for a given particle group**, are as follows:

> **number of particles**,<br>
> **sum of particle weights**,<br>
> **value of highest weight particle**

additionally **estimates that pass those thresholds** are subject to filtering by:

> **distance from any estimate 1 timestep back**,<br>
> **distance from any estimate 2 timesteps back**

which sets a limit on 'teleporting' between timesteps. 
#### Results for different amounts of clutter

<h4 align="center">&nbspLeft: clutter amount = 3,&nbsp Right: clutter amount = 5</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_3_clutter_with_uniform.gif?raw=true" width="47%">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_5_clutter_with_uniform.gif?raw=true" width="47%">
</p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Dark Matt Pink: clutter | Red: measurements | Blue: particles</p>

For the cases of 3 and 5 false measurements at all times, the filter is able to track the targets with a few dropouts and very few false positives. Detection of the third target happens after a small delay, but seems to be decent from that point onwards.

<h4 align="center">&nbsp&nbspLeft: clutter amount = 15,&nbsp Right: clutter amount = 35</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_15_clutter_with_uniform.gif?raw=true" width="47%">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_35_clutter_with_uniform.gif?raw=true" width="47%">
</p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Dark Matt Pink: clutter | Red: measurements | Blue: particles</p>

Facing 15 false measurements at all times, the filter does struggle with some false positives, and there are a few more dropouts which sometimes go on for longer as well. It could be argued that we're still able to successfully track the targets to some degree, and we see that the third, slow moving target, again is tracked better. 

For the case of 35 false measurements at all times, the tracking fails completely for the circular and figure-eight trajectory targets, giving **at least** as many false positives as true positives, and still quite a few false negatives / dropouts as well. Investigating the third target on the other hand, there are some signs of life. The results cautiously suggest that if our area of application has very slow moving targets that we can model accurately enough, a filter like this one might still be able to give useful results with quite high levels of clutter.

#### Main criticism

Another issue that may or may not have been noticed from the low clutter results, is what the consequences of the tuned heuristics are when compared to the earlier MAP estimates.

<h4 align="center">Both: clutter amount = 0<br>&nbsp&nbsp&nbsp&nbspLeft: MAP Estimates,&nbsp Right: Tuned Heuristics</h4>
<p align="center">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_0_clutter_with_uniform_MAP.gif?raw=true" width="47%">
<img src="https://github.com/Jesperoka/EE5020-project/blob/main/results/3_obj_0_clutter_with_uniform.gif?raw=true" width="47%">
</p>
<p align="center"><strong>Legend:</strong> Green: true state positions | Orange: state estimates<br>Red: measurements | Blue: particles</p>

The side-by-side comparison reveals that the heuristics used have a negative effect on the estimates when there is no clutter. This suggests that this is generally **not** the approach to take forward into the future, unless you know there will be very little clutter, and you can quickly fine-tune the heuristics to fit that situation. In that case, this approach is probably a bit simpler to implement, especially if extending a single target filter to multi-target, or if it's your first time implementing a particle filter (as in this case).

This problem comes from the one-size-fits-all nature of tuning simple threshold heuristics, and is a fundamental problem with the approach.

### Improvements for Next Time

- One combined state posterior distribution
- Include number of target explicitly in state description
- Estimate number of targets
- Adaptive number of particles
- Adaptive motion model fitting
- Kerneled/Regularized particle filter
- Constrain state estimate motion directly (not always a good idea)

### Final words

Performs alright, learned a lot, will do better next time.

### References

<a id="r1"></a>[1] M.S. Arulampalam; S. Maskell; N. Gordon; T. Clapp (2002). A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. IEEE Transactions on Signal Processing, 50(2), 174 - 188. [https://doi.org/10.1109/78.978374]


---

<!-- Tell MathJax to typeset equations present in the document -->
<!-- <script type="text/javascript">
MathJax.Hub.Queue(["Typeset", MathJax.Hub]);
</script> -->
