//! Extensions providing signal encoding/decoding and value representation.

mod signal;

/// A decoded CAN signal value with metadata.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Value {
    /// Signal name.
    pub name: String,
    /// Raw decoded value.
    pub raw: u64,
    /// Offset for physical value calculation.
    pub offset: f64,
    /// Factor for physical value calculation.
    pub factor: f64,
    /// Physical unit.
    pub unit: String,
}

impl Value {
    /// Compute the physical value from the raw value.
    ///
    /// # Type Parameters
    ///
    /// * `T` - The target numeric type. Must implement `num::NumCast`.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // u64->f64 precision loss is acceptable for physical values
    pub fn value<T>(&self) -> T
    where
        T: num::NumCast + Default,
    {
        T::from(self.raw as f64 * self.factor + self.offset).unwrap_or_default()
    }

    /// Compute the physical value and format it with the unit.
    ///
    /// # Type Parameters
    ///
    /// * `T` - The target numeric type. Must implement `num::NumCast` and `Display`.
    #[must_use]
    pub fn value_string<T>(&self) -> String
    where
        T: num::NumCast + std::fmt::Display + Default,
    {
        format!("{} {}", self.value::<T>(), self.unit)
    }
}
