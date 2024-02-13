use super::{AdcCapture, AdcCaptureBuffer, CAPTURE_LEN};
use crate::timing::SampleIndex;
use core::ops::{Deref, DerefMut};
use core::slice;

pub struct DmaGrant {
    ptr: *mut u16,
}

impl DmaGrant {
    const LEN: usize = AdcCapture::CONVS_PER_CHUNK;
    pub fn as_mut_ptr(&mut self) -> *mut u16 {
        self.ptr
    }

    pub(super) unsafe fn from_ptr(ptr: *mut u16) -> Self {
        Self { ptr }
    }
}

unsafe impl Send for DmaGrant {}

impl Deref for DmaGrant {
    type Target = [u16; Self::LEN];

    fn deref(&self) -> &Self::Target {
        unsafe { slice::from_raw_parts(self.ptr, Self::LEN) }
            .try_into()
            .unwrap()
    }
}

impl DerefMut for DmaGrant {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { slice::from_raw_parts_mut(self.ptr, Self::LEN) }
            .try_into()
            .unwrap()
    }
}

struct Grant {
    start: usize,
    len: usize,
}

struct RingSlice {
    start: usize,
    len: usize,
}

impl RingSlice {
    const MOD: usize = CAPTURE_LEN;

    pub fn pop_front(&mut self, len: usize) -> Option<Grant> {
        if len > self.len {
            return None;
        }

        let grant = Grant {
            start: self.start,
            len,
        };

        // Update state
        self.start += len;
        self.start %= Self::MOD;
        self.len -= len;

        Some(grant)
    }

    pub fn push_back(&mut self, grant: Grant) {
        let end = (self.start + self.len) % Self::MOD;
        assert_eq!(grant.start, end, "Grant did not match end of this buffer");

        let new_len = self.len + grant.len;
        assert!(new_len <= Self::MOD);
        self.len = new_len;
    }

    pub fn start(&self) -> usize {
        assert!(self.start < Self::MOD);
        self.start
    }

    pub fn end(&self) -> usize {
        (self.start + self.len) % Self::MOD
    }
}

pub struct DoubleBufferedRingBuffer {
    app_owned: RingSlice,
    dma_owned: RingSlice,
    free: RingSlice,
    first_idx: SampleIndex,
    buffer: *mut u16,
}

unsafe impl Send for DoubleBufferedRingBuffer {}

impl DoubleBufferedRingBuffer {
    pub fn new(buffer: &'static mut AdcCaptureBuffer) -> Self {
        assert_eq!(buffer.len(), CAPTURE_LEN);
        Self {
            app_owned: RingSlice { start: 0, len: 0 },
            dma_owned: RingSlice { start: 0, len: 0 },
            free: RingSlice {
                start: 0,
                len: buffer.len(),
            },
            first_idx: SampleIndex(0),
            buffer: buffer.as_mut_ptr(),
        }
    }

    pub fn len(&self) -> usize {
        CAPTURE_LEN
    }

    fn is_consistent(&self) -> bool {
        self.app_owned.end() == self.dma_owned.start()
            && self.dma_owned.end() == self.free.start()
            && self.free.end() == self.app_owned.start()
            && (self.app_owned.len + self.dma_owned.len + self.free.len) == self.len()
    }

    pub fn next_dma_buffer(&mut self) -> Option<DmaGrant> {
        let grant = self.free.pop_front(AdcCapture::CONVS_PER_CHUNK)?;
        assert!((grant.start + grant.len) <= CAPTURE_LEN);

        let dma = DmaGrant {
            ptr: unsafe { self.buffer.add(grant.start) },
        };
        self.dma_owned.push_back(grant);

        assert!(self.is_consistent());
        Some(dma)
    }

    pub fn dma_done(&mut self, dma: DmaGrant) {
        let grant = self
            .dma_owned
            .pop_front(AdcCapture::CONVS_PER_CHUNK)
            .expect("Can only return a DMA slice if you had one");

        let expecting_ptr = unsafe { self.buffer.add(grant.start) };
        assert_eq!(dma.ptr, expecting_ptr);

        // Ownership is passed to app
        self.app_owned.push_back(grant);
        assert!(self.is_consistent());
    }

    pub fn first_idx(&self) -> SampleIndex {
        self.first_idx
    }

    pub fn app_data(&self) -> (&[u16], &[u16]) {
        assert!(self.is_consistent());

        if self.app_owned.len == 0 {
            (&[], &[])
        } else if self.app_owned.start() < self.app_owned.end() {
            (
                unsafe {
                    slice::from_raw_parts(
                        self.buffer.add(self.app_owned.start()),
                        self.app_owned.len,
                    )
                },
                &[],
            )
        } else {
            let to_end = self.len() - self.app_owned.start();
            let after_start = self.app_owned.len - to_end;

            unsafe {
                (
                    slice::from_raw_parts(self.buffer.add(self.app_owned.start()), to_end),
                    slice::from_raw_parts(self.buffer, after_start),
                )
            }
        }
    }

    pub fn app_done(&mut self, len: usize) {
        let len = usize::min(len, self.app_owned.len);
        let grant = self.app_owned.pop_front(len).unwrap();
        self.free.push_back(grant);
        self.first_idx.0 = self.first_idx.0.wrapping_add(len as u64);
        assert!(self.is_consistent());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::array;
    use static_cell::StaticCell;

    #[test]
    fn roundtrip() {
        fn do_dma(mut dma_grant: DmaGrant) -> std::thread::JoinHandle<DmaGrant> {
            std::thread::spawn(move || {
                for (i, e) in dma_grant.iter_mut().enumerate() {
                    *e = i as u16;
                }
                dma_grant
            })
        }

        const EMPTY: (&[u16], &[u16]) = (&[], &[]);

        static BUFFER: StaticCell<AdcCaptureBuffer> = StaticCell::new();
        let buffer = BUFFER.init(array::from_fn(|i| i as u16));

        let mut b = DoubleBufferedRingBuffer::new(buffer);
        assert_eq!(b.app_data(), EMPTY);

        let dma_slice0 = b.next_dma_buffer().unwrap();
        assert_eq!(unsafe { *dma_slice0.ptr }, 0);
        let dma_slice0 = do_dma(dma_slice0);

        let dma_slice1 = b.next_dma_buffer().unwrap();
        assert_eq!(
            unsafe { *dma_slice1.ptr },
            AdcCapture::CONVS_PER_CHUNK as u16
        );
        let dma_slice1 = do_dma(dma_slice1); // Write 1 while writing 0

        b.dma_done(dma_slice0.join().unwrap());
        assert!(matches!(b.app_data(), (&[0, 1, ..], &[]))); // Read while 1 is writing
        assert_eq!(b.first_idx(), SampleIndex(0));

        b.dma_done(dma_slice1.join().unwrap());
        let app = b.app_data();
        assert_eq!(app.0.len(), 2 * AdcCapture::CONVS_PER_CHUNK);
        assert_eq!(app.1.len(), 0);
        assert_eq!(b.first_idx(), SampleIndex(0));

        b.app_done(AdcCapture::CONVS_PER_CHUNK);
        let app = b.app_data();
        assert_eq!(app.0.len(), AdcCapture::CONVS_PER_CHUNK);
        assert_eq!(app.1.len(), 0);
        assert_eq!(b.first_idx(), SampleIndex(AdcCapture::CONVS_PER_CHUNK as _));

        b.app_done(AdcCapture::CONVS_PER_CHUNK);
        assert_eq!(b.app_data(), EMPTY);

        b.app_done(AdcCapture::CONVS_PER_CHUNK);
        assert_eq!(b.app_data(), EMPTY);

        assert_eq!(b.free.len, b.len());

        // Fill the whole buffer
        let mut grants = vec![];
        while let Some(dma_buffer) = b.next_dma_buffer() {
            grants.push(do_dma(dma_buffer));
        }

        for (i, grant) in grants.into_iter().enumerate() {
            b.dma_done(grant.join().unwrap());
            assert_eq!(
                b.app_data().0.len() + b.app_data().1.len(),
                (i + 1) * AdcCapture::CONVS_PER_CHUNK
            );
        }

        assert_eq!(b.app_owned.len, b.len());

        let app = b.app_data();
        assert_eq!(app.0.len(), b.len() - (2 * AdcCapture::CONVS_PER_CHUNK));
        assert_eq!(app.1.len(), 2 * AdcCapture::CONVS_PER_CHUNK);

        for elem in app.0.into_iter().chain(app.1) {
            let _ = *elem;
        }

        // Release everything one element at a time
        while b.app_data() != EMPTY {
            b.app_done(1);
        }
        assert_eq!(b.free.len, b.len());

        assert_eq!(
            b.first_idx(),
            SampleIndex(2 * AdcCapture::CONVS_PER_CHUNK as u64 + CAPTURE_LEN as u64)
        );
    }
}
