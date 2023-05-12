use std::f32::INFINITY;
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

use crate::consts as consts;
use crate::sys as sys;

pub struct ParticleFilter {
    pub estimates: Vec<Vector2<f32>>,
    pub samples: [(Vector2<f32>, u8); consts::INITIAL_NUM_PARTICLES],
    weights: [f32; consts::INITIAL_NUM_PARTICLES],
    associations: [usize; consts::INITIAL_NUM_PARTICLES],
}

pub enum InitialDistributionType {
    UNIFORM,
    GAUSSIAN,
}

impl<'a> ParticleFilter {

    // TODO: description
    pub fn initialize(measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) -> Self {

        let mut particle_filter: ParticleFilter = ParticleFilter{ 
            estimates: measurements.clone(),
            samples: [(Vector2::new(0.0, 0.0), 1); consts::INITIAL_NUM_PARTICLES],
            weights: [1.0/(consts::INITIAL_NUM_PARTICLES as f32); consts::INITIAL_NUM_PARTICLES],
            associations: [0; consts::INITIAL_NUM_PARTICLES],
        };

        let samples_per_measurement = consts::INITIAL_NUM_PARTICLES / measurements.len();
        let extra_samples_needed = consts::INITIAL_NUM_PARTICLES - samples_per_measurement * measurements.len();

        let mut distribution =  Uniform::new(-consts::INITIAL_ERROR_BOUND, consts::INITIAL_ERROR_BOUND);
        match consts::INITIAL_DISTRIBUTION_TYPE {
            InitialDistributionType::UNIFORM    =>  { (); } // do nothing 
            InitialDistributionType::GAUSSIAN   =>  { let distribution = Normal::new(0.0, (1.0/2.0)*consts::INITIAL_ERROR_BOUND).unwrap(); }
            _                                   =>  { (); println!("Unsupported initial distribution, falling back on Uniform."); }
        }
        
        let mut i: usize = 0;
        for &y in measurements {
            for _ in 0..samples_per_measurement {
                particle_filter.samples[i].1 = *consts::VALID_MODELS.choose(rng).unwrap();
                particle_filter.samples[i].0 = y + Vector2::new(distribution.sample(rng), distribution.sample(rng));
                i += 1;
            }
        }
        for _ in 0..extra_samples_needed {
            particle_filter.samples[i].1 = *consts::VALID_MODELS.choose(rng).unwrap();
            particle_filter.samples[i].0 = *measurements.choose(rng).unwrap() + Vector2::new(distribution.sample(rng), distribution.sample(rng));
            i += 1;
        }
        assert!(i == consts::INITIAL_NUM_PARTICLES);
        return particle_filter;
    }

    // TODO: description
    pub fn predict(&mut self, t: f32, dt: f32, rng: &mut ThreadRng) {
        // TODO: estimate velocity
        let v_hat: f32 = 15.0;

        for s in &mut self.samples {
            s.1 = model_change_posterior(s.1, rng);
            s.0 = motion_model_posterior(t, s.0, s.1, v_hat, dt, rng);
        }
    }

    // TODO: description
    pub fn update(&mut self, measurements: &Vec<Vector2<f32>>, rng: &mut ThreadRng) {

        let lambda: f32         = consts::POISSON_MEAN; // using correct noise model for now.
        let mu: Vector2<f32>    = Vector2::new(consts::GAUSSIAN_MEAN, consts::GAUSSIAN_MEAN);
        let sigma: f32          = consts::GAUSSIAN_STANDARD_DEVIATION;

        

        for (i, s) in self.samples.iter().enumerate() {
            
            let (idx, y) = closest(s.0, measurements);
            self.associations[i] = idx; 

            match s.1 { // WARNING: this is where we determine which y should be used for each sample
                1 => {
                    let diff = y - s.0; // FIXME: this is ugly, move to a separate function
                    self.weights[i] = if diff[0] >= 0.0 && diff[1] >= 0.0 { bivariate_continuous_iid_poisson(diff, lambda) } else { 0.0 }; 
                    // self.weights[i] = bivariate_iid_gaussian(y, self.samples[i] + mu, sigma); 
                }
                2 => { self.weights[i] = bivariate_iid_gaussian(y, s.0 + mu, sigma); }
                _ => { print!("Undefined mode."); }
            }
        }
        // TODO: make estimates. IDEA convolve to sharpen global distribution before picking N highest weights, so that
        // we dont just get two estimates around the same point. 
        // Estimates need to have a certain weight, and associated samples need to sum up to a
        // certain weight as well?
        self.estimates = vec![self.samples[argmax(&self.weights).unwrap()].0]; // TODO: make estimate
        add_noise(&mut self.weights, rng); // helps prevent degenerate cases
        normalize(&mut self.weights);
    }

    // TODO: description
    pub fn resample(&mut self, rng: &mut ThreadRng) {
        // TODO: interpolate weights to get a smoother distribution which helps with sample variety
            
        // add noise that is inversely proportional to number of samples?
        let n: usize = *self.associations.iter().max().unwrap();
        /// ----------- TODO: make function
        let mut proposal_distributions: Vec<WeightedIndex<f32>> = Vec::with_capacity(n);
        let mut weight_sets: Vec<Vec<f32>>                  = Vec::with_capacity(n);
        let mut index_sets: Vec<Vec<usize>>                 = Vec::with_capacity(n);

        for i in 0..n { weight_sets.push(Vec::new()); index_sets.push(Vec::new()); }

        for (i, &association) in self.associations.iter().enumerate() { 
            weight_sets[association].push(self.weights[i]); 
            index_sets[association].push(i);
        }

        for weights in weight_sets { proposal_distributions.push(WeightedIndex::new(&weights).unwrap()); }
        /// -------------

        let mut sampled_values: Vec<(Vector2<f32>, u8)> = Vec::with_capacity(self.samples.len());
        for i in 0..self.samples.len() {
            let local_distribution  = &proposal_distributions[self.associations[i]];
            let sample_index        = index_sets[self.associations[i]][local_distribution.sample(rng)];
            sampled_values.push(self.samples[sample_index]);                                                                                                                                                                    
        }
        assert!(sampled_values.len() == self.samples.len());
        for i in 0..self.samples.len() { self.samples[i] = sampled_values[i]; }

        ////////////////////

        let proposal_distribution = WeightedIndex::new(&self.weights).unwrap();
        let mut sampled_values: Vec<(Vector2<f32>, u8)> = Vec::with_capacity(self.samples.len());


        assert!(self.samples.len() == self.weights.len());
        for _ in 0..self.samples.len()   { sampled_values.push(self.samples[proposal_distribution.sample(rng)]); }
        for i in 0..sampled_values.len() { self.samples[i] = sampled_values[i]; }
        
        // WARNING: need to rebalance number of samples between tracked objects
        // might be a good idea to spawn particles around measurments that identified as object
        // but how to identify an object? If an object appears, and there are some particles around
        // it, we can check if the particles remain around it? but that wont work the way it is
        // now, since I'm not able to track the other object...
        //
        // Also: do the entropy stuff? or low variance resampling?
        
        // IDEA:    1.  always add uniform samples
        //          2.  make N = measurements.len() separate proposal dsitributions
        //              around each measurement. 
        //          3.  since each weight is calculated only in relation to its closest
        //              measurement, uniform particles will be associated to new objects and
        //              over time generate more particles around it, if it persists (i.e, it's not
        //              noise).

    }
}

// TODO: description
fn model_change_posterior(m: u8, rng: &mut ThreadRng) -> u8 {
    let mut choice = m;
    if consts::MODEL_CHANGE.sample(rng) { 
        while choice == m { choice = *consts::VALID_MODELS.choose(rng).unwrap(); }
    }
    return choice;
}

// Simulation step for a particle
fn motion_model_posterior(t: f32, x: Vector2<f32>,  m: u8, v_hat: f32, dt: f32, rng: &mut ThreadRng) -> Vector2<f32> {
    
    // TODO: replace goals with velocity estimate for more realism/generalizability
    let model_1_goal: Vector2<f32> = sys::circle_path(t);
    let model_2_goal: Vector2<f32> = sys::figure_eight_path(t);
    
    let artificial_noise: Vector2<f32> = Vector2::new(consts::ARTIFICIAL_PROCESS_NOISE.sample(rng), consts::ARTIFICIAL_PROCESS_NOISE.sample(rng));
    match m {
        1 => { return x + v_hat*(model_1_goal - x)*dt + artificial_noise; } 
        2 => { return x + v_hat*(model_2_goal - x)*dt + artificial_noise; }
        _ => { println!("Undefined mode."); return Vector2::new(0.0, 0.0); }
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

// Find the measurement closest to x
fn closest(x: Vector2<f32>, measurements: &Vec<Vector2<f32>>) -> (usize, Vector2<f32>) {
    let mut norm2 = INFINITY;
    let mut closest: Vector2<f32> = (*measurements)[0]; 
    let mut idx: usize;
    for (i, &y) in measurements.iter().enumerate() { 
        let dist = x.metric_distance(&y);
        (norm2, closest, idx) = if dist < norm2 { (dist, y, i) } else { (norm2, closest, idx) };
    }
    return (idx, closest);
}

// Scale weights to sum to one (approximately)
fn normalize(weights: &mut [f32]) {
    let sum: f32 = weights.iter().sum();
    assert!(sum != 0.0);
    for weight in weights { *weight /= sum; }
}

// Helps deal with particle degeneration // FIXME: move to consts
lazy_static!{ static ref SMALL_UNIFORM: Uniform<f32> = Uniform::new(0.000000001, 0.0000001); }
fn add_noise(weights: &mut [f32], rng: &mut ThreadRng) {
    for weight in weights { *weight += SMALL_UNIFORM.sample(rng); }
}

// Get the index of the largest value element
fn argmax(arr: &[f32]) -> Option<usize> {
    arr.iter().enumerate().max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap()).map(|(i, _)| i)
}

// IDEA: get avg distance to the MAP estimate and use it to inform how large the velocity should be
