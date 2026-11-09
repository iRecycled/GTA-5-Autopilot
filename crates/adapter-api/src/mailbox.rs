//! Latest-wins mailbox: a seqlock over one slot.
//!
//! One writer, any number of readers, possibly in different processes (the
//! struct lives in shared memory). The writer never waits; a reader retries
//! only if it raced a write, and always gets the newest complete value.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering, fence};

#[repr(C)]
pub struct Mailbox<T: Copy> {
    /// Even: stable. Odd: a write is in progress. Counts writes × 2.
    seq: AtomicU32,
    _pad: u32,
    value: UnsafeCell<T>,
}

// The seqlock protocol makes concurrent access sound for `Copy` data.
unsafe impl<T: Copy + Send> Sync for Mailbox<T> {}

impl<T: Copy> Mailbox<T> {
    /// Writes a new value. Only one thread or process may write a given mailbox.
    pub fn write(&self, v: &T) {
        let s = self.seq.load(Ordering::Relaxed);
        self.seq.store(s.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: readers detect a torn read through `seq` and retry.
        unsafe { core::ptr::write_volatile(self.value.get(), *v) };
        self.seq.store(s.wrapping_add(2), Ordering::Release);
    }

    /// Reads the newest complete value, with the number of writes so far
    /// (0 = never written).
    pub fn read(&self) -> (u32, T) {
        loop {
            let s1 = self.seq.load(Ordering::Acquire);
            if s1 & 1 == 1 {
                core::hint::spin_loop();
                continue;
            }
            // SAFETY: as above; a torn value is discarded below.
            let v = unsafe { core::ptr::read_volatile(self.value.get()) };
            fence(Ordering::Acquire);
            if self.seq.load(Ordering::Relaxed) == s1 {
                return (s1 / 2, v);
            }
        }
    }

    /// Writes so far, without reading the value.
    pub fn writes(&self) -> u32 {
        self.seq.load(Ordering::Acquire) / 2
    }
}

impl<T: Copy + Default> Default for Mailbox<T> {
    fn default() -> Self {
        Self { seq: AtomicU32::new(0), _pad: 0, value: UnsafeCell::new(T::default()) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[derive(Clone, Copy, Default)]
    struct Pair {
        a: u64,
        b: u64,
    }

    #[test]
    fn read_after_write() {
        let m = Mailbox::<Pair>::default();
        assert_eq!(m.read().0, 0);
        m.write(&Pair { a: 7, b: 7 });
        let (n, v) = m.read();
        assert_eq!((n, v.a, v.b), (1, 7, 7));
    }

    #[test]
    fn never_tears_under_contention() {
        let m = Arc::new(Mailbox::<Pair>::default());
        let w = m.clone();
        let writer = std::thread::spawn(move || {
            for i in 1..=200_000u64 {
                w.write(&Pair { a: i, b: i });
            }
        });
        let mut last = 0;
        while last < 200_000 {
            let (_, v) = m.read();
            assert_eq!(v.a, v.b, "torn read");
            assert!(v.a >= last, "went backwards");
            last = v.a;
        }
        writer.join().unwrap();
    }
}
