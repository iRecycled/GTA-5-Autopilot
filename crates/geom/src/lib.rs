//! Frames and transforms (D16).
//!
//! Everything the core sees is ISO 8855: x forward, y left, z up, angles
//! positive counter-clockwise seen from above, SI units. Each host adapter
//! converts once, at the boundary, using the helpers here, so there is exactly
//! one place where a host's convention is translated.

/// A 2D vector in some frame (m or m/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f32,
    pub y: f32,
}

impl V2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    pub fn len(self) -> f32 {
        self.x.hypot(self.y)
    }
    pub fn dot(self, o: V2) -> f32 {
        self.x * o.x + self.y * o.y
    }
}

impl core::ops::Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }
}

impl core::ops::Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        V2::new(self.x + o.x, self.y + o.y)
    }
}

/// A host frame whose axes are x right, y forward, z up (GTA V's entity frame,
/// and many other games'), converted to ISO 8855 (x forward, y left, z up).
pub fn iso_from_right_forward_up(x_right: f32, y_forward: f32, z_up: f32) -> (f32, f32, f32) {
    (y_forward, -x_right, z_up)
}

/// The ego car's pose in a host's world frame, using the world's own x/y axes
/// and a unit forward direction.
#[derive(Clone, Copy, Debug)]
pub struct EgoPose {
    pub pos: V2,
    /// Unit forward vector in world coordinates.
    pub fwd: V2,
}

impl EgoPose {
    /// From a world position and a (not necessarily unit) forward vector.
    /// Pitch is ignored: the result is a level vehicle frame.
    pub fn new(pos: V2, fwd: V2) -> Self {
        let l = fwd.len();
        let fwd = if l > 1e-6 { V2::new(fwd.x / l, fwd.y / l) } else { V2::new(1.0, 0.0) };
        Self { pos, fwd }
    }

    /// Left of forward, in world coordinates (world must be right-handed with z up).
    fn left(&self) -> V2 {
        V2::new(-self.fwd.y, self.fwd.x)
    }

    /// A world point in the ISO vehicle frame (x forward, y left).
    pub fn to_vehicle(&self, world: V2) -> V2 {
        let d = world - self.pos;
        V2::new(d.dot(self.fwd), d.dot(self.left()))
    }

    /// A world-frame vector (e.g. a velocity) rotated into the vehicle frame.
    pub fn rotate_to_vehicle(&self, v: V2) -> V2 {
        V2::new(v.dot(self.fwd), v.dot(self.left()))
    }

    /// A vehicle-frame point back into world coordinates.
    pub fn to_world(&self, veh: V2) -> V2 {
        let l = self.left();
        V2::new(self.pos.x + self.fwd.x * veh.x + l.x * veh.y, self.pos.y + self.fwd.y * veh.x + l.y * veh.y)
    }

    /// Yaw of the forward vector in the world frame (rad, CCW from world +x).
    pub fn yaw(&self) -> f32 {
        self.fwd.y.atan2(self.fwd.x)
    }
}

/// Wraps an angle to (-π, π].
pub fn wrap_pi(a: f32) -> f32 {
    let two_pi = core::f32::consts::TAU;
    let mut a = a % two_pi;
    if a > core::f32::consts::PI {
        a -= two_pi;
    } else if a <= -core::f32::consts::PI {
        a += two_pi;
    }
    a
}

/// Bearing of a vehicle-frame point seen from `from`, CCW from +x (rad).
pub fn bearing(from: V2, to: V2) -> f32 {
    let d = to - from;
    d.y.atan2(d.x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn right_forward_up_to_iso() {
        // A point 2 m to the right and 5 m ahead is x = 5 (forward), y = -2 (right is -y).
        assert_eq!(iso_from_right_forward_up(2.0, 5.0, 1.0), (5.0, -2.0, 1.0));
    }

    #[test]
    fn world_to_vehicle_and_back() {
        // Ego at (10, 20) facing world +y (north): forward = +y, left = -x.
        let ego = EgoPose::new(V2::new(10.0, 20.0), V2::new(0.0, 3.0));
        let ahead = ego.to_vehicle(V2::new(10.0, 30.0));
        assert!(close(ahead.x, 10.0) && close(ahead.y, 0.0));
        let left = ego.to_vehicle(V2::new(7.0, 20.0));
        assert!(close(left.x, 0.0) && close(left.y, 3.0));
        let back = ego.to_world(V2::new(4.0, -1.5));
        let again = ego.to_vehicle(back);
        assert!(close(again.x, 4.0) && close(again.y, -1.5));
        assert!(close(ego.yaw(), core::f32::consts::FRAC_PI_2));
    }

    #[test]
    fn velocity_rotation() {
        let ego = EgoPose::new(V2::new(0.0, 0.0), V2::new(0.0, 1.0));
        // A car moving north at 10 m/s moves "forward" in our frame.
        let v = ego.rotate_to_vehicle(V2::new(0.0, 10.0));
        assert!(close(v.x, 10.0) && close(v.y, 0.0));
    }

    #[test]
    fn wrap_and_bearing() {
        assert!(close(wrap_pi(3.0 * core::f32::consts::PI), core::f32::consts::PI));
        assert!(close(wrap_pi(-3.5), -3.5 + core::f32::consts::TAU));
        assert!(close(bearing(V2::new(0.0, 0.0), V2::new(1.0, 1.0)), core::f32::consts::FRAC_PI_4));
    }
}
