/// Defines a couple of modes in a hybrid dynamic system.
use lazy_static::lazy_static;

use nalgebra::{Vector2, Vector3};
use rand::rngs::ThreadRng;
use rand_distr::Distribution;
use rand::prelude::SliceRandom;

use crate::consts as consts;

pub struct Sys {
    pub x: Vector2<f32>,                                // state
    pub f: fn(f32, Vector2<f32>, u8) -> Vector2<f32>,   // x_dot = f(t, x, m) continuous time dynamic system function
    pub h: fn(f32, Vector2<f32>, u8, &mut ThreadRng) -> Vector2<f32>,
}

/// A couple of different measurment noise distributions based on which mode a hybrid system is in.
pub fn measurement_function(t: f32, x: Vector2<f32>, m: u8, rng: &mut ThreadRng) -> Vector2<f32> {
    let mut noise: Vector2<f32> = Vector2::new(0.0, 0.0);
    match m {
        1 => { 
            noise[0] = consts::POISSON_DISTRIBUTION.sample(rng);
            noise[1] = consts::POISSON_DISTRIBUTION.sample(rng);
        }
        2 | 3 => { 
            noise[0] = consts::GAUSSIAN_DISTRIBUTION.sample(rng); 
            noise[1] = consts::GAUSSIAN_DISTRIBUTION.sample(rng); 
        }
        _ => { print!("Undefined mode.\n"); }
    }
    return x + noise; // TODO: give back multiple measurements and add background noise
}

/// A circlular reference trajectory.
pub fn circle_path(t: f32) -> Vector2<f32> {
    const A: f32 = 20.0;
    const W: f32 = 8.5;
    return Vector2::new(A*f32::cos(W*t), A*f32::sin(W*t));
}

/// A figure-eight shaped reference trajectory.
pub fn figure_eight_path(t: f32) -> Vector2<f32> {
    const A: f32 = 15.0;
    const W: f32 = 11.3;
    return Vector2::new(A*f32::cos(W*t), A*f32::sin(W*t)*f32::cos(W*t));
}

/// A linear reference trajectory.
pub fn line_path(t: f32) -> Vector2<f32> {
    const A: f32 = 7.0;
    const B: f32 = -5.0;
    const X0: f32 = -80.0;
    const Y0: f32 = 80.0;
    return Vector2::new(X0 + A*t, Y0 + B*t)
}

/// An arbitrary nonlinear hybrid system with couple of possible modes, being
/// controller by a simple proportional controller to follow a couple of different
/// reference trajectories.
#[allow(non_upper_case_globals)]
pub fn example_hybrid_system(t: f32, x: Vector2<f32>, m: u8) -> Vector2<f32> {
    const u1_max: f32 = 5.00;
    const u2_max: f32 = 8.50; 
    const a1: f32 = 0.03;
    const a2: f32 = 0.08;

    // A basic proportional controller trying to follow some reference paths
    const zero_vector: Vector2<f32> = Vector2::new(0.0, 0.0);
    let kp_1: f32 = 1.0;
    let kp_2: f32 = 2.0;

    let mut x_dot: Vector2<f32> = zero_vector;
    match m {
        1 => { 
            let e: Vector2<f32> = x - circle_path(t);
            x_dot[0] = a1*x[0] + f32::sin(0.5*x[1]) - f32::max(-u1_max, f32::min(u1_max, kp_1*e[0]));
            x_dot[1] = a1*x[1] + f32::cos(0.4*x[1]) - f32::max(-u1_max, f32::min(u1_max, kp_1*e[1]));
            // print!("{}, ", f32::sqrt(x_dot[0]*x_dot[0] + x_dot[1]*x_dot[1]));
        }
        2 => {
            let e: Vector2<f32> = x - figure_eight_path(t);
            x_dot[0] = a2*x[0] + f32::sin(0.1*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[0]));
            x_dot[1] = a2*x[1] + f32::cos(0.2*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[1]));
            // print!("{}, ", f32::sqrt(x_dot[0]*x_dot[0] + x_dot[1]*x_dot[1]));
        }
        3 => {
            let e: Vector2<f32> = x - line_path(t);
            x_dot[0] = a2*x[0] + f32::sin(0.1*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[0]));
            x_dot[1] = a2*x[1] + f32::cos(0.2*x[1]) - f32::max(-u2_max, f32::min(u2_max, kp_2*e[1]));
            // print!("m=3 {}", f32::sqrt(x_dot[0]*x_dot[0] + x_dot[1]*x_dot[1]));
        }
        _ => { print!("Undefined mode.\n"); }
    }
    return x_dot;
}

#[allow(non_upper_case_globals)]
pub fn straight_line_or_constant_turn_model(x: Vector3<f32>, m: u8) -> Vector3<f32> {
    let mut x_dot: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0);
    const v1: f32 = 2.0*58.0; 
    const v2: f32 = 2.0*57.2;
    const phi: f32 = 0.3;
    match m {
        1 => { // going straight fast
            x_dot[0] = v1*f32::cos(x[2]);
            x_dot[1] = v1*f32::sin(x[2]);
            x_dot[2] = 0.0;
        }
        2 => { // going straight slow
            x_dot[0] = v2*f32::cos(x[2]);
            x_dot[1] = v2*f32::sin(x[2]);
            x_dot[2] = 0.0;
        }
        3 => { // turning left, large radius
            x_dot[0] = v1*f32::cos(x[2]);
            x_dot[1] = v1*f32::sin(x[2]);
            x_dot[2] = phi; 
        }
        4 => { // turning left, small radius
            x_dot[0] = v1*f32::cos(x[2]);
            x_dot[1] = v1*f32::sin(x[2]);
            x_dot[2] = 2.0*phi; 
        }
        5 => { // turning right, large radius
            x_dot[0] = v1*f32::cos(x[2]);
            x_dot[1] = v1*f32::sin(x[2]);
            x_dot[2] = -phi; 
        }
        6 => { // turning right, small radius
            x_dot[0] = v1*f32::cos(x[2]);
            x_dot[1] = v1*f32::sin(x[2]);
            x_dot[2] = -2.0*phi; 
        }
        _ => {
            print!("uh oh spaghettio..");
        }
    }

    return x_dot;
}

/// The true markov chain for the jumping between modes in the hybrid system.
pub fn true_model_change_posterior(m: u8, rng: &mut ThreadRng) -> u8 {
    let mut choice = m;
    if consts::MODEL_CHANGE.sample(rng) { 
        while choice == m { choice = *consts::HYBRID_SYSTEM_JUMPABLE_MODELS.choose(rng).unwrap(); }
    }
    return choice;
}

/// Generates an amount of false measurements based on a clutter distribution.
pub fn clutter(amount: usize, rng: &mut ThreadRng) -> Vec<Vector2<f32>> {
    const RANGE: f32 = (consts::GRID_SIZE.0 - consts::ORIGIN.0) as f32;
    lazy_static!{ static ref CLUTTER: rand_distr::Uniform<f32> = rand_distr::Uniform::new(-RANGE, RANGE); }
    let mut clutter: Vec<Vector2<f32>> = Vec::with_capacity(amount);
    for _ in 0..amount {
        clutter.push(Vector2::new(CLUTTER.sample(rng), CLUTTER.sample(rng)))
    }
    return clutter;
}

