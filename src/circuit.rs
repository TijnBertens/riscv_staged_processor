use crate::risc_v::Word;

pub type PortID = usize;

pub const PORT_DEFAULT_VALUE: Word = 0;
pub const PORT_NULL_VALUE: Word = 0;
pub const PORT_NULL_ID: PortID = 0;

pub struct Port {
    pub name: String,
    pub current_data: Word
}

pub struct PortCollection {
    ports: Vec<Port>
}

impl PortCollection {
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
