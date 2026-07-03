/// Minimal library crate created by r_proj_maker.
pub fn hello() -> &'static str {
    "hello"
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_returns_hello() {
        assert_eq!(hello(), "hello");
    }
}
