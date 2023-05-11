use std::f32::consts::PI;

use lazy_static::lazy_static;
use nalgebra::Vector2;
use rand::rngs::ThreadRng;
use rand_distr::{Uniform, Normal, Distribution};
use rand::seq::SliceRandom;
use rand::distributions::WeightedIndex;
use special::Gamma;

use std::io;
use std::io::Write;


pub struct ParticleFilter {
    pub estimate: Vector2<f32>,
    pub samples: [Vector2<f32>; crate::consts::INITIAL_NUM_PARTICLES],
    weights: [f32; crate::consts::INITIAL_NUM_PARTICLES],
}

pub enum InitialDistributionType {
    UNIFORM,
    GAUSSIAN,
}

impl ParticleFilter {

    // TODO: description
    pub fn initialize(measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) -> Self {

        let mut particle_filter: ParticleFilter = ParticleFilter{ 
            estimate: *measurements.choose(rng).unwrap(),
            samples: [Vector2::new(0.0, 0.0); crate::consts::INITIAL_NUM_PARTICLES],
            weights: [1.0/crate::consts::INITIAL_NUM_PARTICLES as f32; crate::consts::INITIAL_NUM_PARTICLES],
        };

        let samples_per_measurement = crate::consts::INITIAL_NUM_PARTICLES / measurements.len();
        let extra_samples_needed = crate::consts::INITIAL_NUM_PARTICLES - samples_per_measurement * measurements.len();
        let mut i: usize = 0;

        match crate::consts::INITIAL_DISTRIBUTION_TYPE {

            InitialDistributionType::UNIFORM => {
                let distribution: Uniform<f32> = Uniform::new(-crate::consts::INITIAL_ERROR_BOUND, crate::consts::INITIAL_ERROR_BOUND);
                for y in measurements {
                    for _ in 0..samples_per_measurement {
                        // particle_filter.samples[i] = y + Vector2::new(distribution.sample(rng), distribution.sample(rng));
                        particle_filter.samples[i] = Vector2::new(distribution.sample(rng), distribution.sample(rng));
                        i += 1;
                    }
                }
                for _ in 0..extra_samples_needed {
                    // particle_filter.samples[i] = measurements.choose(rng).unwrap() + Vector2::new(distribution.sample(rng), distribution.sample(rng));
                    particle_filter.samples[i] = Vector2::new(distribution.sample(rng), distribution.sample(rng));
                    i += 1;
                }
            }

            InitialDistributionType::GAUSSIAN => {
                unimplemented!();
            }
            _ => { print!("\nUnknown initial distribution type.\n"); }
        }

        assert!(i == crate::consts::INITIAL_NUM_PARTICLES);
        return particle_filter;
    }

    // TODO: description
    pub fn predict(&mut self, t: f32, m: u8,  dt: f32, rng: &mut ThreadRng) {
        let mut v: f32; // bad mutable, just doing it for now
        let mut x_goal: Vector2<f32>;

        lazy_static!{ static ref dist: Uniform<f32> = Uniform::new(-PI, PI); }
        // TODO: change m to be a markov chain model transition function m_k+1 = P(m_k+1 | m_k)
        // TODO: expand models to multiple objects
        // TODO: estimate velocity

        // Assumption: we know where the object is going at any time
        // We don't know any details about how it moves, so we assume a constant linear velocity
        for s in &mut self.samples {
            match m {
                1 => { v = 10.5;    x_goal  = crate::sys::circle_path(t); }      // FIXME: ok? 
                2 => { v = 9.5;     x_goal  = crate::sys::figure_eight_path(t); } // FIXME: ok?
                _ => { v = 0.0;     x_goal  = Vector2::new(0.0, 0.0); print!("\nUndefined mode."); }
            }
            let n1: f32 = if SMALL_UNIFORM.sample(rng) > 0.5*0.0000001 { 2.1 } else { -2.1 }; // TODO:
            let n2: f32 = if SMALL_UNIFORM.sample(rng) > 0.5*0.0000001 { 2.1 } else { -2.1 }; // TODO:
            *s = *s + v*(x_goal - *s)*dt + Vector2::new(n1, n2);
        }
    }

    // TODO: description
    pub fn update(&mut self, measurements: &Vec<Vector2<f32>>, m: u8, rng: &mut ThreadRng) {
        let y = measurements[0]; // only one measurement for the time being

        let lambda: f32         = crate::consts::POISSON_MEAN; // using correct noise model for now.
        let mu: Vector2<f32>    = Vector2::new(crate::consts::GAUSSIAN_MEAN, crate::consts::GAUSSIAN_MEAN);
        let sigma: f32          = crate::consts::GAUSSIAN_STANDARD_DEVIATION;

        for i in 0..self.samples.len() {
            match m {
                1 => {
                    let diff = y - self.samples[i];
                    // self.weights[i] = if diff[0] >= 0.0 && diff[1] >= 0.0 { bivariate_continuous_iid_poisson(diff, lambda) } else { 0.00000000001 }; 
                    self.weights[i] = bivariate_iid_gaussian(y, self.samples[i] + mu, sigma); 
                }
                2 => { 
                    self.weights[i] = bivariate_iid_gaussian(y, self.samples[i] + mu, sigma); 
                }
                _ => { print!("Undefined mode."); }
            }
        }
        self.estimate = self.samples[argmax(&self.weights).unwrap()];
        add_noise(&mut self.weights, rng); // helps prevent degenerate cases
        normalize(&mut self.weights);
    }

    // TODO: description
    pub fn resample(&mut self, rng: &mut ThreadRng) {

        // TODO: interpolate weights to get a smoother distribution which helps with sample variety
        let proposal_distribution = WeightedIndex::new(&self.weights).unwrap();
        let mut sampled_values: Vec<Vector2<f32>> = Vec::with_capacity(self.samples.len());

        assert!(self.samples.len() == self.weights.len());
        for _ in 0..self.samples.len()  { sampled_values.push(self.samples[proposal_distribution.sample(rng)]); }
        for i in 0..sampled_values.len(){ self.samples[i] = sampled_values[i]; }
    }
}

// Density function for bivariate gaussian PDF for two i.i.d random variables x[0] and x[1]
fn bivariate_iid_gaussian(x: Vector2<f32>, mu: Vector2<f32>, sigma: f32) -> f32 {
    return f32::exp(-0.5*(x - mu).dot(&(x - mu))) / (2.0*PI*sigma);
}

// Continous Interpolation of the Poisson PMF for two i.i.d random variables x[0] and x[1]
fn bivariate_continuous_iid_poisson(x: Vector2<f32>, lambda: f32) -> f32 {
    let gamma_product = Gamma::gamma(x[0]+1.0) * Gamma::gamma(x[1]+1.0);
    assert!(gamma_product != 0.0);
    return (lambda.powf(x[0])*lambda.powf(x[1])*f32::exp(-2.0*lambda)) / gamma_product;
}

fn normalize(weights: &mut [f32]) {
    let sum: f32 = weights.iter().sum();
    assert!(sum != 0.0);
    for weight in weights { *weight /= sum; }
}

// Helps deal with particle degeneration
lazy_static!{ static ref SMALL_UNIFORM: Uniform<f32> = Uniform::new(0.000000001, 0.0000001); }
fn add_noise(weights: &mut [f32], rng: &mut ThreadRng) {
    for weight in weights { *weight += SMALL_UNIFORM.sample(rng); }
}

fn argmax(arr: &[f32]) -> Option<usize> {
    arr.iter().enumerate().max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap()).map(|(i, _)| i)
}

