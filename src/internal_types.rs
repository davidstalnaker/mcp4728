// Index for which byte to write next in a sequential write.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum SequentialWriteByteIndex {
    Zero,
    One,
}

impl SequentialWriteByteIndex {
    pub fn next(self) -> SequentialWriteByteIndex {
        match self {
            Self::Zero => Self::One,
            Self::One => Self::Zero,
        }
    }
}

// Index for which byte to write next in a multi write.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum MultiWriteByteIndex {
    Zero,
    One,
    Two,
}

impl MultiWriteByteIndex {
    pub fn next(self) -> MultiWriteByteIndex {
        match self {
            Self::Zero => Self::One,
            Self::One => Self::Two,
            Self::Two => Self::Zero,
        }
    }
}
