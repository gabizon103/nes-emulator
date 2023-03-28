pub struct CPU {
    pub reg_a:u8,
    pub reg_x:u8,
    pub reg_y:u8,
    pub status:u8,
    pub pc:u16,
    memory:[u8;0xffff]
}

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    NoneAddressing
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            reg_a: 0,
            reg_x: 0,
            reg_y: 0,
            status: 0,
            pc: 0,
            memory:[0;0xffff]
        }
    }

    fn get_operand_address(&self,mode:&AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.pc, 
            AddressingMode::ZeroPage => self.mem_read(self.pc) as u16,
            AddressingMode::Absolute => self.mem_read_u16(self.pc),
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.pc);
                let addr = pos.wrapping_add(self.reg_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.pc);
                let addr = pos.wrapping_add(self.reg_y) as u16;
                addr
            }
            AddressingMode::Absolute_X => {
                let pos = self.mem_read_u16(self.pc);
                let addr = pos.wrapping_add(self.reg_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let pos = self.mem_read_u16(self.pc);
                let addr = pos.wrapping_add(self.reg_y as u16);
                addr
            }
            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.pc);

                let ptr:u8 = (base as u8).wrapping_add(self.reg_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.pc);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u16).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.reg_y as u16);
                deref
            }
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported",mode);
            }
        }
    }

    fn mem_read(&mut self,addr:u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self,addr:u16,data:u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&mut self,pos:u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos+1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self,addr:u16,data:u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(addr,lo);
        self.mem_write(addr+1,hi);
    }

    pub fn reset(&mut self) {
        self.reg_a = 0;
        self.reg_x = 0;
        self.reg_y = 0;
        self.status = 0;
        self.pc = self.mem_read_u16(0xfffc);
    }

    pub fn load_and_run(&mut self,program:Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn load(&mut self, program:Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xfffc,0x8000);
    }

    fn update_z_and_n(&mut self,result:u8) {
        if result == 0 {
            self.status = self.status | 0b0000_0010; // set Z flag 1
        }
        else {
            self.status = self.status & 0b1111_1101; // set Z flag 0
        }
        if result & 0b1000_0000 != 0 {
            self.status = self.status | 0b1000_0000; // set N flag 1
        }
        else {
            self.status = self.status & 0b0111_1111;
        }
    }

    fn lda(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.reg_a = value;
        self.update_z_and_n(self.reg_a);
    }

    fn sta(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr,self.reg_a);
    }

    fn tax(&mut self) { // transfer A to X
        self.reg_x = self.reg_a;
        self.update_z_and_n(self.reg_x);
    }

    fn inx(&mut self) { 
        self.reg_x = self.reg_x.wrapping_add(1);
        self.update_z_and_n(self.reg_x);
    }


    pub fn run(&mut self) {
        let ref opcodes:HashMap<u8,&'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read(self.pc);
            self.pc += 1;
            let pc_state = self.pc;
            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized",code));
            match opscode {
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1
                => { // LDA
                    self.lda(&opcode.mode);
                }
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91
                => { // STA
                    self.sta(&opcode.mode);
                }
                0x00 => {
                    return;
                }
                0xaa => { // TAX (transfer A to X)
                    self.tax();
                }
                0xe8 => { // INX (increment X)
                    self.inx();
                }
                _ => { todo!(); }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_imm_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x05,0x00]); // LDA #$05, BRK
        assert_eq!(cpu.reg_a,5);
        assert!(cpu.status & 0b0000_0010 == 0); // check Z is 0
        assert!(cpu.status & 0b1000_0000 == 0);    // check N is 0
    }

    #[test]
    fn test_0xa9_lda_z_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x00,0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x02,0xaa,0x00]);
        assert_eq!(cpu.reg_x,2);
    }

    #[test]
    fn test_0xe8_inx() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0x0d,0xaa,0xe8,0x00]);
        assert_eq!(cpu.reg_x,14);
        assert!(cpu.status & 0b0000_0010 == 0);  // check Z is 0
        assert!(cpu.status & 0b1000_0000 == 0);     // check N is 0
    }

    #[test]
    fn test_0xe8_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xff,0xaa,0xe8,0xe8,0x00]);
        assert_eq!(cpu.reg_x,1);
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9,0xc0,0xaa,0xe8,0x00]);
        assert_eq!(cpu.reg_x,0xc1);
    }
}


