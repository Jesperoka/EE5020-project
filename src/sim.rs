/// Defines functions to simulate a dynamic system

pub fn print_something() {
    println!("heyo");
}

pub fn rk4_step(f: fn(f32, u8), x: Vector2<f32>, m: u8,  dt: f32) -> Vec<f32> {
    let k1 = f(x, m);
    let k2 = f(&x.iter().zip(&k1).map(|(xi, ki)| xi + ki * dt / 2.0).collect::<Vec<_>>());
    let k3 = f(t + dt / 2.0, &x.iter().zip(&k2).map(|(xi, ki)| xi + ki * dt / 2.0).collect::<Vec<_>>());
    let k4 = f(t + dt, &x.iter().zip(&k3).map(|(xi, ki)| xi + ki * dt).collect::<Vec<_>>());
    x.iter()
        .zip(&k1)
        .zip(&k2)
        .zip(&k3)
        .zip(&k4)
        .map(|((((xi, ki1), ki2), ki3), ki4)| xi + (ki1 + 2.0 * ki2 + 2.0 * ki3 + ki4) * dt / 6.0)
        .collect::<Vec<_>>()
}
