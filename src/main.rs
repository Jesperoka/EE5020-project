mod sim; 
mod sys;
mod consts;
mod anim;

// use nalgebra::Vector2;
use std::io;
use std::io::Write;

fn main() {
    let mut data_giffer = anim::DataGiffer::new();
    let mut system = sys::Sys {x: consts::x0, f: sys::example_hybrid_system};

    let mut m: u8 = 1;

    println!("\nx0 = {}", system.x);
    let mut t: f32 = 0.0;

    print!("\n\nSimulating...");
    io::stdout().flush().unwrap();
    
    for i in 0..1000 {
        t += consts::dt;
        if i >= 500 { m = 2; }
        if i % 1 == 0 { data_giffer.draw_points(vec![system.x]); }
        sim::simulation_step(t, &mut system, m, consts::dt);
    }
    
    print!("\n\nDone simulating.\n\nExporting animation...");
    io::stdout().flush().unwrap();
    
    data_giffer.export_gif();
    print!("\n\nDone exporting.\n"); }
