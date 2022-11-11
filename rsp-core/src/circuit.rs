use crate::isa::Word;

pub type PortID = usize;

pub const PORT_DEFAULT_VALUE: Word = 0;
pub const PORT_NULL_VALUE: Word = 0;
pub const PORT_NULL_ID: PortID = 0;

pub struct Port {
    pub name: String,
    pub current_data: Word
}

/// Represents a collection of (output) ports of components on a circuit. Each port holds a single data word.
/// 
/// Once a port is registerd in the collection, it is never deleted. A unique ID is given out whenever a new port
/// is regstered. This ID can be stored as a reference to a port, without requiring an explicit (mutable) reference to the port data.
/// This allows circuits to create a cyclical network of ports without running into issues with mutability.
pub struct PortCollection {
    ports: Vec<Port>
}

impl PortCollection {
    /// Creates and emty PortCollection.
    pub fn new() -> Self {
        Self {
            ports: vec![
                Port {
                    name: String::from("NULL"),
                    current_data: PORT_NULL_VALUE
                }
            ]
        }
    }

    /// Adds a port to the collection and returns an ID to identify it.
    pub fn register_port(&mut self, value: Word, name: String) -> PortID {
        self.ports.push(
            Port {
                name,
                current_data: value
            }
        );

        return self.ports.len() - 1;
    }

    pub fn get_port_data(&self, id: PortID) -> Word {
        return self.ports[id].current_data;
    }

    pub fn set_port_data(&mut self, id: PortID, val: Word) {
        self.ports[id].current_data = val;
    }
}
