use crate::consts;
use gif::{Encoder, Frame, Repeat};
/// TODO
use nalgebra::Vector2;
use std::fs::File;

pub struct DataGiffer {
    default_frame: [u8; consts::FLAT_ARRAY_SIZE],
    frames: Vec<[u8; consts::FLAT_ARRAY_SIZE]>,
    size: (i32, i32),
    origin: (i32, i32),
}

fn within_frame(point: (i32, i32), size: (i32, i32)) -> bool {
    return 0 <= point.0 && point.0 < size.0 && 0 <= point.1 && point.1 < size.1;
}

impl DataGiffer {
    pub fn new() -> Self {
        let new_data_giffer = DataGiffer {
            default_frame: Self::create_default_frame(),
            frames: Vec::default(),
            size: consts::GRID_SIZE,
            origin: consts::ORIGIN,
        };
        assert!(new_data_giffer.size.0 >= 0 && new_data_giffer.size.1 >= 0);
        assert!(within_frame(new_data_giffer.origin, new_data_giffer.size));
        return new_data_giffer;
    }

    fn create_default_frame() -> [u8; consts::FLAT_ARRAY_SIZE] {
        let mut default_frame: [u8; consts::FLAT_ARRAY_SIZE] = [0; consts::FLAT_ARRAY_SIZE];
        assert!(consts::NUM_CHANNELS as usize == consts::BACKGROUND_COLOR.len());

        for i in 0..consts::FLAT_ARRAY_SIZE {
            default_frame[i] = consts::BACKGROUND_COLOR[i % consts::NUM_CHANNELS as usize];
        }
        return default_frame;
    }

    /// Draws (x,y) points as (col, -row) relative to origin in a grid
    pub fn draw_points(
        &mut self,
        state_vecs: &Vec<Vector2<f32>>,
        color_indices: &Vec<&str>,
    ) -> bool {
        let mut all_inside: bool = true;
        let frame = &mut self.default_frame.clone();

        for (vector, color) in state_vecs.iter().zip(color_indices.iter()) {
            if self.might_over_or_underflow(vector) {
                all_inside = false;
                continue;
            }

            let pixel_point: (i32, i32) = (
                f32::round(-vector[1]) as i32 + self.origin.0,
                f32::round(vector[0]) as i32 + self.origin.1,
            );

            if !within_frame(pixel_point, self.size) {
                all_inside = false;
            } else {
                for point in self.filled_circle_centered_at(pixel_point, *consts::COLOR_RADIUS_MAP.get(color).unwrap()) {
                    self.draw_point(point, frame, color);
                }
            }
        }
        self.frames.push(*frame);
        return all_inside;
    }

    fn draw_point(&self, pixel_point: (i32, i32), frame: &mut [u8], color: &str) {
        assert!(pixel_point.0 >= 0 && pixel_point.1 >= 0);
        let y = pixel_point.0;
        let x = pixel_point.1;
        let m = self.size.1;
        let c = consts::NUM_CHANNELS;
        let flat_rgb_pixel_index_range =
            (y * m * c + x * c) as usize..(y * m * c + x * c + c) as usize;
        frame[flat_rgb_pixel_index_range].copy_from_slice(consts::COLORS.get(color).unwrap());
    }

    fn filled_circle_centered_at(&self, pixel_point: (i32, i32), radius: usize) -> Vec<(i32, i32)> {
        let radius = radius as i32;
        let center_x = pixel_point.0;
        let center_y = pixel_point.1;
        let mut filled_circle_pixel_points = Vec::new();

        for x in (center_x - radius)..=(center_x + radius) {
            for y in (center_y - radius)..=(center_y + radius) {
                if x < 0 || y < 0 || x > consts::GRID_SIZE.1 - 1 || y > consts::GRID_SIZE.0 - 1 {
                    continue;
                }

                let dx = x - center_x;
                let dy = y - center_y;
                let distance_squared = (dx * dx + dy * dy) as f32;

                if distance_squared <= (radius * radius) as f32 {
                    filled_circle_pixel_points.push((x, y));
                }
            }
        }
        return filled_circle_pixel_points;
    }

    pub fn export_gif(&self) {
        let mut file = File::create(consts::ANIMATION_FILENAME).unwrap();
        let mut encoder =
            Encoder::new(&mut file, self.size.0 as u16, self.size.1 as u16, &[]).unwrap();
        encoder.set_repeat(Repeat::Infinite).unwrap();

        for frame in &self.frames {
            let gif_frame = &mut Frame::from_rgb(self.size.0 as u16, self.size.1 as u16, frame);
            gif_frame.delay = u16::max((consts::dt as u16) * 100, 1);
            encoder
                .write_frame(&Frame::from_rgb(
                    self.size.0 as u16,
                    self.size.1 as u16,
                    frame,
                ))
                .unwrap();
        }
    }

    fn might_over_or_underflow(&self, vector: &Vector2<f32>) -> bool {
        return vector[0] as i32 >= i32::MAX - (self.origin.0 + 5)
            || vector[1] as i32 >= i32::MAX - (self.origin.1 + 5)
            || vector[0] as i32 <= i32::MIN + (self.origin.0 - 5)
            || vector[1] as i32 <= i32::MIN + (self.origin.1 - 5);
    }
}
