/// Defines functions to simulate a dynamic system

use nalgebra::Vector2;

pub fn rk4_step(t: f32, f: fn(f32, Vector2<f32>, u8) -> Vector2<f32>, x: Vector2<f32>, m: u8,  dt: f32) -> Vector2<f32> {
    let k1 = f(t, x, m);
    let k2 = f(t + (1.0/2.0)*dt, x + (1.0/2.0)*dt*k1, m);
    let k3 = f(t + (1.0/2.0)*dt, x + (1.0/2.0)*dt*k2, m);
    let k4 = f(t + dt, x + dt*k3, m);
    return x + (1.0/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
}

pub fn step(t: f32, system: &mut crate::sys::Sys, m: u8, dt: f32) {
    system.x = rk4_step(t, system.f, system.x, m, dt);
}
