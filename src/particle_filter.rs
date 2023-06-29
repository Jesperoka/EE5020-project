/// Particle Filter Implementation
use std::f32::consts::PI;
use std::f32::INFINITY;

use lazy_static::lazy_static;
use nalgebra::{Vector2, Vector3};
use rand::distributions::WeightedIndex;
use rand::rngs::ThreadRng;
use rand::seq::SliceRandom;
use rand_distr::{Distribution, Normal, Uniform};
use special::Gamma;

use crate::consts;
use crate::sys;

pub struct ParticleFilter {
    pub estimates: Vec<Vector2<f32>>,
    all_estimates: Vec<Vector2<f32>>,
    all_previous_estimates: Vec<Vector2<f32>>,
    all_previous_previous_estimates: Vec<Vector2<f32>>,
    pub samples: [(Vector3<f32>, u8); consts::INITIAL_NUM_PARTICLES],
    weights: [f32; consts::INITIAL_NUM_PARTICLES],
    scores: [f32; consts::INITIAL_NUM_PARTICLES],
    associations: [usize; consts::INITIAL_NUM_PARTICLES],
}

pub enum InitialDistributionType {
    UNIFORM,
    GAUSSIAN,
}

impl<'a> ParticleFilter {
    /// Sets up the particle filter at the first iteration.
    pub fn initialize(measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) -> Self {
        let mut particle_filter: ParticleFilter = ParticleFilter {
            estimates: measurements.clone(),
            all_estimates: measurements.clone(),
            all_previous_estimates: measurements.clone(),
            all_previous_previous_estimates: measurements.clone(),
            samples: [(Vector3::new(0.0, 0.0, 0.0), 1); consts::INITIAL_NUM_PARTICLES],
            weights: [1.0 / (consts::INITIAL_NUM_PARTICLES as f32); consts::INITIAL_NUM_PARTICLES],
            scores: [1.0; consts::INITIAL_NUM_PARTICLES],
            associations: [0; consts::INITIAL_NUM_PARTICLES],
        };

        let samples_per_measurement = consts::INITIAL_NUM_PARTICLES / measurements.len();
        let extra_samples_needed =
            consts::INITIAL_NUM_PARTICLES - samples_per_measurement * measurements.len();

        let mut distribution =
            Uniform::new(-consts::INITIAL_ERROR_BOUND, consts::INITIAL_ERROR_BOUND);
        match consts::INITIAL_DISTRIBUTION_TYPE {
            InitialDistributionType::UNIFORM => {
                ();
            } // do nothing
            InitialDistributionType::GAUSSIAN => {
                let distribution =
                    Normal::new(0.0, (1.0 / 2.0) * consts::INITIAL_ERROR_BOUND).unwrap();
            }
            _ => {
                ();
                println!("Unsupported initial distribution, falling back on Uniform.");
            }
        }

        let mut i: usize = 0;
        for &y in measurements {
            for _ in 0..samples_per_measurement {
                particle_filter.samples[i].1 = *consts::FILTER_MOTION_MODELS.choose(rng).unwrap();
                particle_filter.samples[i].0 = Vector3::new(y[0], y[1], 0.0)
                    + Vector3::new(
                        distribution.sample(rng),
                        distribution.sample(rng),
                        distribution.sample(rng),
                    );
                i += 1;
            }
        }
        for _ in 0..extra_samples_needed {
            let y_rand = *measurements.choose(rng).unwrap();
            particle_filter.samples[i].1 = *consts::FILTER_MOTION_MODELS.choose(rng).unwrap();
            particle_filter.samples[i].0 = Vector3::new(y_rand[0], y_rand[1], 0.0)
                + Vector3::new(
                    distribution.sample(rng),
                    distribution.sample(rng),
                    distribution.sample(rng),
                );
            i += 1;
        }
        assert!(i == consts::INITIAL_NUM_PARTICLES);
        return particle_filter;
    }

    /// Sample from mode change and motion model posterior probability distributions
    pub fn predict(&mut self, dt: f32, rng: &mut ThreadRng) {
        for s in &mut self.samples {
            s.1 = model_change_posterior(s.1, rng);
            s.0 = motion_model_posterior(s.0, s.1, dt, rng);
        }
    }

    /// Update sample weights according to measurment model conditional probabilities.
    /// Associates samples to their nearest measurment.
    pub fn update(&mut self, measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) {
        for (i, s) in self.samples.iter().enumerate() {
            let position = Vector2::new(s.0[0], s.0[1]);
            let (idx, y) = closest(position, measurements);

            self.associations[i] = idx;
            self.weights[i] = measurement_model_posterior(&(position, s.1), y);
        }

        self.estimates = self.compute_state_estimates();
        normalize(&mut self.weights);
        add_noise(&mut self.weights, rng); // helps prevent degenerate cases
    }

    /// Resample locally around each measurment with associated samples, using their weights.
    pub fn resample(&mut self, measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) {
        // TODO: interpolate weights to get a smoother distribution which helps with sample variety

        let n: usize = *self.associations.iter().max().unwrap();

        //----------- TODO: make function
        let mut proposal_distributions: Vec<Option<WeightedIndex<f32>>> = Vec::with_capacity(n);
        let mut weight_sets: Vec<Vec<f32>> = Vec::with_capacity(n);
        let mut index_sets: Vec<Vec<usize>> = Vec::with_capacity(n);

        for i in 0..n + 1 {
            weight_sets.push(Vec::new());
            index_sets.push(Vec::new());
        }

        for (i, &association) in self.associations.iter().enumerate() {
            weight_sets[association].push(self.weights[i]);
            index_sets[association].push(i);
        }

        for weights in weight_sets {
            if weights.len() != 0 {
                proposal_distributions.push(Some(WeightedIndex::new(&weights).unwrap()));
            } else {
                proposal_distributions.push(None);
            }
        }
        //-------------

        let mut sampled_values: Vec<(Vector3<f32>, u8)> = Vec::with_capacity(self.samples.len());
        for i in 0..self.samples.len() {
            if let Some(local_distribution) = &proposal_distributions[self.associations[i]] {
                let sample_index = index_sets[self.associations[i]][local_distribution.sample(rng)];
                sampled_values.push(self.samples[sample_index]);
            }
        }
        assert!(sampled_values.len() == self.samples.len());
        for i in 0..self.samples.len() {
            self.samples[i] = sampled_values[i];
        }

        // Put a few of the worst particles on randomly chosen measurements
        let k_worst_indices = argmin_k(
            &self.weights.to_vec(),
            (0.35 * (consts::INITIAL_NUM_PARTICLES as f32)) as usize,
        );
        for idx in k_worst_indices {
            let position = *measurements.choose(rng).unwrap()
                + Vector2::new(
                    consts::ARTIFICIAL_PROCESS_NOISE.sample(rng),
                    consts::ARTIFICIAL_PROCESS_NOISE.sample(rng),
                );
            self.samples[idx] = (
                Vector3::new(position[0], position[1], consts::ARTIFICIAL_PROCESS_NOISE.sample(rng)),
                *consts::FILTER_MOTION_MODELS.choose(rng).unwrap(),
            );
            self.weights[idx] = 0.0;
        }

        // Also: do the entropy stuff? or low variance resampling?
    }

    /// Assigns the particle with the highest weight, in a given particle group associated to a
    /// measurment, to be the state estimate for an object, if there are enough particles
    /// and the sum of the weights in that particle group are above a threshold.
    fn compute_state_estimates(&mut self) -> Vec<Vector2<f32>> {
        let num_candidates: usize = *self.associations.iter().max().unwrap() + 1;

        self.all_previous_previous_estimates = self.all_previous_estimates.clone();
        self.all_previous_estimates = self.all_estimates.clone();
        self.all_estimates = Vec::with_capacity(num_candidates);

        let mut sample_groups: Vec<Vec<Vector2<f32>>> = Vec::with_capacity(num_candidates);
        let mut weight_groups: Vec<Vec<f32>> = Vec::with_capacity(num_candidates);

        for _ in 0..(num_candidates + 1) {
            sample_groups.push(Vec::new());
            weight_groups.push(Vec::new());
        }

        for (i, s) in self.samples.iter().enumerate() {
            sample_groups[self.associations[i]].push(Vector2::new(s.0[0], s.0[1]));
            weight_groups[self.associations[i]].push(self.weights[i]);
        }

        // All the random numbers are just tuned to get ok performance with a fair amount of clutter
        for (weights, samples) in weight_groups.iter().zip(sample_groups.iter()) {
            if samples.len() as f32
                >= consts::INITIAL_NUM_PARTICLES as f32 / usize::max(1, num_candidates + 6) as f32
                && weights.iter().sum::<f32>() > 1.0 / usize::max(1, num_candidates + 13) as f32
                && weights.iter().fold(0.0, |a, &b| f32::max(a, b)) > 0.075
            {
                self.all_estimates.push(samples[argmax(&weights).unwrap()]);
            }
        }

        // filter our teleports 2 steps back
        let previous_estimates: Vec<Vector2<f32>> = self
            .all_previous_estimates
            .clone()
            .into_iter()
            .filter(|b| {
                self.all_previous_previous_estimates
                    .iter()
                    .any(|a| (a - b).norm() <= 9.0) // Check if distance <= threshold
            })
            .collect();

        // filter our teleports 1 step back
        let estimates: Vec<Vector2<f32>> = self
            .all_estimates
            .clone()
            .into_iter()
            .filter(|b| {
                previous_estimates.iter().any(|a| (a - b).norm() <= 9.0) // Check if distance <= threshold
            })
            .collect();

        return estimates;
    }
} // END impl

/// TODO: document
fn model_change_posterior(m: u8, rng: &mut ThreadRng) -> u8 {
    let mut m = m;
    match m {
        1 => {
            m = consts::MODEL_CHANGE_M1.sample(rng) as u8;
        }
        2 => {
            m = consts::MODEL_CHANGE_M2.sample(rng) as u8;
        }
        3 => {
            m = consts::MODEL_CHANGE_M3.sample(rng) as u8;
        }
        4 => {
            m = consts::MODEL_CHANGE_M4.sample(rng) as u8;
        }
        5 => {
            m = consts::MODEL_CHANGE_M5.sample(rng) as u8;
        }
        6 => {
            m = consts::MODEL_CHANGE_M6.sample(rng) as u8;
        }
        _ => {
            print!("oh noes..")
        }
    }
    return m + 1; // modes are indexed at 1, but I'm using WeightedIndex model change distributions
}

/// Simulation step for a particle with added noise.
fn motion_model_posterior(x: Vector3<f32>, m: u8, dt: f32, rng: &mut ThreadRng) -> Vector3<f32> {
    let artificial_noise: Vector3<f32> = Vector3::new(
        1.5 * consts::ARTIFICIAL_PROCESS_NOISE.sample(rng),
        1.5 * consts::ARTIFICIAL_PROCESS_NOISE.sample(rng),
        1.0 * consts::ARTIFICIAL_PROCESS_NOISE.sample(rng),
    );
    return x + dt * sys::straight_line_or_constant_turn_model(x, m) + artificial_noise;
}

/// Measurement model used for importance weighing.
fn measurement_model_posterior(s: &(Vector2<f32>, u8), y: Vector2<f32>) -> f32 {
    let lambda: f32 = consts::POISSON_MEAN; // using correct noise model for now.
    let mu: Vector2<f32> = Vector2::new(consts::GAUSSIAN_MEAN, consts::GAUSSIAN_MEAN);
    let sigma: f32 = consts::GAUSSIAN_STANDARD_DEVIATION;
    let weight: f32;

    match s.1 {
        3 | 5 => {
            weight = if (y - s.0)[0] >= 0.0 && (y - s.0)[1] >= 0.0 {
                bivariate_continuous_iid_poisson(y - s.0, lambda)
            } else {
                0.0
            };
        }
        1 | 2 | 4 | 6 => {
            weight = bivariate_iid_gaussian(y, s.0 + mu, sigma);
        }
        _ => {
            weight = 0.0;
            print!("Undefined mode.");
        }
    }
    return if !weight.is_nan() { weight } else { 0.0 };
}

/// Density function for bivariate gaussian PDF for two i.i.d random variables x[0] and x[1]
fn bivariate_iid_gaussian(x: Vector2<f32>, mu: Vector2<f32>, sigma: f32) -> f32 {
    return f32::exp(-0.5 * (x - mu).dot(&(x - mu))) / (2.0 * PI * sigma);
}

/// Continous Interpolation of the Poisson PMF for two i.i.d random variables x[0] and x[1]
fn bivariate_continuous_iid_poisson(x: Vector2<f32>, lambda: f32) -> f32 {
    let gamma_product = Gamma::gamma(x[0] + 1.0) * Gamma::gamma(x[1] + 1.0);
    assert!(gamma_product != 0.0);
    return (lambda.powf(x[0]) * lambda.powf(x[1]) * f32::exp(-2.0 * lambda)) / gamma_product;
}

/// Find the measurement closest to x
fn closest(x: Vector2<f32>, measurements: &Vec<Vector2<f32>>) -> (usize, Vector2<f32>) {
    let mut norm2 = INFINITY;
    let mut closest: Vector2<f32> = (*measurements)[0];
    let mut idx: usize = 0;
    for (i, &y) in measurements.iter().enumerate() {
        let dist = x.metric_distance(&y);
        (norm2, closest, idx) = if dist < norm2 {
            (dist, y, i)
        } else {
            (norm2, closest, idx)
        };
    }
    return (idx, closest);
}

/// Scale weights to sum to one (approximately).
fn normalize(weights: &mut [f32]) {
    let sum: f32 = weights.iter().sum();
    if sum == 0.0 {
        return;
    }
    for weight in weights {
        *weight /= sum;
    }
}

lazy_static! {
    static ref SMALL_UNIFORM: Uniform<f32> = Uniform::new(0.000000001, 0.0000001);
}
/// Helps deal with particle weight degeneration.
fn add_noise(weights: &mut [f32], rng: &mut ThreadRng) {
    for weight in weights {
        *weight += SMALL_UNIFORM.sample(rng);
    }
}

/// Get the index of the largest value element.
fn argmax(arr: &Vec<f32>) -> Option<usize> {
    return arr
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i);
}

/// Get the index of the lowest value element.
fn argmin(arr: &Vec<f32>) -> Option<usize> {
    return arr
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i);
}

/// Get the indices of the k lowest value elements.
fn argmin_k(vector: &Vec<f32>, k: usize) -> Vec<usize> {
    let mut indices: Vec<usize> = Vec::with_capacity(k);
    let mut values: Vec<f32> = Vec::with_capacity(k);

    for (idx, &val) in vector.iter().enumerate() {
        if indices.len() < k {
            indices.push(idx);
            values.push(val);
        } else {
            let min_idx: usize = argmin(&values).unwrap();
            if val < values[min_idx] {
                values[min_idx] = val;
            }
        }
    }
    return indices;
}

/// Just an implementation to print for debugging.
impl InitialDistributionType {
    pub fn to_string(&self) -> &str {
        return match self {
            InitialDistributionType::UNIFORM => "Uniform",
            InitialDistributionType::GAUSSIAN => "Gaussian",
        };
    }
}
// IDEA: get avg distance to the MAP estimate and use it to inform how large the velocity should be
