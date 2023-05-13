# EE5020-project

Just some thoughts after the first implementation:

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
