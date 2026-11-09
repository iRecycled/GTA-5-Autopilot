//! The ego car as the surround features see it: speed and direction, the
//! turn signal, and its box.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ego {
    /// Speed (m/s, ≥ 0).
    pub speed: f32,
    /// Reverse is selected.
    pub reverse: bool,
    pub turn_left: bool,
    pub turn_right: bool,
    /// Box edges in the ego frame (ISO 8855, m): front and rear x, left and
    /// right y (`right` is negative).
    pub front: f32,
    pub rear: f32,
    pub left: f32,
    pub right: f32,
}

impl Default for Ego {
    fn default() -> Self {
        Self { speed: 0.0, reverse: false, turn_left: false, turn_right: false, front: 2.35, rear: -2.35, left: 0.95, right: -0.95 }
    }
}

impl Ego {
    /// Velocity along x (m/s): negative when reversing.
    pub fn vx(&self) -> f32 {
        if self.reverse { -self.speed } else { self.speed }
    }

    pub fn half_width(&self) -> f32 {
        self.left.max(-self.right)
    }
}
