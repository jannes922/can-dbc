use crate::{ByteOrder, Signal, ValueType};

impl Signal {
    /// Decode a signal from raw CAN message data and return a [`Value`] wrapper.
    ///
    /// Returns `None` if the signal has zero size or if the data buffer is too
    /// short.
    #[must_use]
    pub fn decode_value(&self, data: &[u8]) -> Option<crate::Value> {
        self.decode(data).map(|raw| crate::Value {
            name: self.name.clone(),
            raw,
            offset: self.offset,
            factor: self.factor,
            unit: self.unit.clone(),
        })
    }

    /// Decode a raw signal value from CAN message data.
    ///
    /// Returns `None` if the signal has zero size or if the data buffer is too
    /// short.
    #[must_use]
    #[allow(
        clippy::cast_possible_truncation, // CAN start_bit is always < 64
        clippy::explicit_counter_loop     // BE bit traversal is non-linear
    )]
    pub fn decode(&self, data: &[u8]) -> Option<u64> {
        if self.size == 0 {
            return None;
        }

        let len = data.len();
        let mut result = 0;
        match self.byte_order {
            ByteOrder::LittleEndian => {
                let mut src_bit = self.start_bit as usize;
                let mut dst_bit = 0;
                for _ in 0..self.size {
                    let index = src_bit / 8;
                    if index >= len {
                        return None;
                    }
                    if (data[index] & (1 << (src_bit % 8))) != 0 {
                        result |= 1 << dst_bit;
                    }
                    src_bit += 1;
                    dst_bit += 1;
                }
            }
            ByteOrder::BigEndian => {
                let mut src_bit = self.start_bit as usize;
                let mut dst_bit = self.size.saturating_sub(1);
                for _ in 0..self.size {
                    let index = src_bit / 8;
                    if index >= len {
                        return None;
                    }
                    if (data[index] & (1 << (src_bit % 8))) != 0 {
                        result |= 1 << dst_bit;
                    }
                    if (src_bit % 8) == 0 {
                        src_bit += 15;
                    } else {
                        src_bit -= 1;
                    }
                    dst_bit = dst_bit.saturating_sub(1);
                }
            }
        }

        if self.value_type == ValueType::Signed && (result & (1 << (self.size - 1))) != 0 {
            for i in self.size..64 {
                result |= 1 << i;
            }
        }

        Some(result)
    }

    /// Encode a raw signal value into CAN message data.
    ///
    /// The `data` vector is automatically resized if the signal extends beyond
    /// the current buffer length.
    #[allow(
        clippy::cast_possible_truncation, // CAN start_bit is always < 64
        clippy::explicit_counter_loop     // BE bit traversal is non-linear
    )]
    pub fn encode(&self, data: &mut Vec<u8>, value: u64) {
        if self.size == 0 {
            return;
        }

        match self.byte_order {
            ByteOrder::LittleEndian => {
                let mut src_bit = self.start_bit as usize;
                let mut dst_bit = 0;
                for _ in 0..self.size {
                    let index = src_bit / 8;
                    if index >= data.len() {
                        data.resize(index + 1, 0);
                    }
                    if (value & (1 << dst_bit)) != 0 {
                        data[index] |= 1 << (src_bit % 8);
                    } else {
                        data[index] &= !(1 << (src_bit % 8));
                    }
                    src_bit += 1;
                    dst_bit += 1;
                }
            }
            ByteOrder::BigEndian => {
                let mut src_bit = self.start_bit as usize;
                let mut dst_bit = self.size.saturating_sub(1);
                for _ in 0..self.size {
                    let index = src_bit / 8;
                    if index >= data.len() {
                        data.resize(index + 1, 0);
                    }
                    if (value & (1 << dst_bit)) != 0 {
                        data[index] |= 1 << (src_bit % 8);
                    } else {
                        data[index] &= !(1 << (src_bit % 8));
                    }
                    if (src_bit % 8) == 0 {
                        src_bit += 15;
                    } else {
                        src_bit -= 1;
                    }
                    dst_bit = dst_bit.saturating_sub(1);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{ByteOrder, MultiplexIndicator, NumericValue, Signal, ValueType};

    fn test_signal_le() -> Signal {
        Signal {
            name: "Test".to_string(),
            multiplexer_indicator: MultiplexIndicator::Plain,
            start_bit: 0,
            size: 8,
            byte_order: ByteOrder::LittleEndian,
            value_type: ValueType::Unsigned,
            factor: 1.0,
            offset: 0.0,
            min: NumericValue::Uint(0),
            max: NumericValue::Uint(255),
            unit: String::new(),
            receivers: vec![],
        }
    }

    #[test]
    fn decode_le_u8() {
        let signal = test_signal_le();
        assert_eq!(signal.decode(&[0xAB]), Some(0xAB));
    }

    #[test]
    fn decode_be_u8() {
        let signal = Signal {
            byte_order: ByteOrder::BigEndian,
            start_bit: 7,
            ..test_signal_le()
        };
        assert_eq!(signal.decode(&[0xAB]), Some(0xAB));
    }

    #[test]
    fn decode_le_signed_negative() {
        let signal = Signal {
            value_type: ValueType::Signed,
            ..test_signal_le()
        };
        assert_eq!(signal.decode(&[0xFF]), Some(u64::MAX)); // -1 as i64
    }

    #[test]
    fn decode_truncates_on_short_buffer() {
        let signal = Signal {
            start_bit: 16,
            size: 8,
            ..test_signal_le()
        };
        assert_eq!(signal.decode(&[0x00, 0x00]), None);
    }

    #[test]
    fn encode_le_u8() {
        let signal = test_signal_le();
        let mut data = vec![0];
        signal.encode(&mut data, 0xAB);
        assert_eq!(data, vec![0xAB]);
    }

    #[test]
    fn encode_be_u8() {
        let signal = Signal {
            byte_order: ByteOrder::BigEndian,
            start_bit: 7,
            ..test_signal_le()
        };
        let mut data = vec![0];
        signal.encode(&mut data, 0xAB);
        assert_eq!(data, vec![0xAB]);
    }

    #[test]
    fn encode_resizes_buffer() {
        let signal = Signal {
            start_bit: 16,
            size: 8,
            ..test_signal_le()
        };
        let mut data = vec![];
        signal.encode(&mut data, 0xAB);
        assert_eq!(data, vec![0, 0, 0xAB]);
    }

    #[test]
    fn decode_value_wrapper() {
        let signal = test_signal_le();
        let value = signal.decode_value(&[0x64]).unwrap();
        assert_eq!(value.name, "Test");
        assert_eq!(value.raw, 0x64);
        assert!((value.value::<f64>() - 100.0).abs() < f64::EPSILON);
    }
}
