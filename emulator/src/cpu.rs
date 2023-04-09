use std::collections::HashMap;
use crate::opcodes;


bitflags! {
    pub struct CPUFlags:u8 {
        const CARRY             = 0b0000_0001;
        const ZERO              = 0b0000_0010;
        const INTERRUPT_DISABLE = 0b0000_0100;
        const DECIMAL_MODE      = 0b0000_1000;
        const BREAK             = 0b0001_0000;
        const BREAK2            = 0b0010_0000;
        const OVERFLOW          = 0b0100_0000;
        const NEGATIVE          = 0b1000_0000;
    }
}


pub struct CPU {
    pub reg_a:u8,
    pub reg_x:u8,
    pub reg_y:u8,
    pub status:CPUFlags,
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
            status: CPUFlags::from_bits_truncate(0b100100),
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

    fn mem_read(&self,addr:u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self,addr:u16,data:u8) {
        self.memory[addr as usize] = data;
    }

    fn mem_read_u16(&self,pos:u16) -> u16 {
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
        self.status = CPUFlags::from_bits_truncate(0b100100);
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
            self.status.insert(CPUFlags::ZERO); // set Z flag 1
        }
        else {
            self.status.remove(CPUFlags::ZERO); // set Z flag 0
        }
        if result & 0b1000_0000 != 0 {
            self.status.insert(CPUFlags::NEGATIVE); // set N flag 1
        }
        else {
            self.status.remove(CPUFlags::NEGATIVE);
        }
    }

    fn set_carry_flag(&mut self) {
        self.status.insert(CPUFlags::CARRY);
    }

    fn clear_carry_flag(&mut self) {
        self.status.remove(CPUFlags::CARRY);
    }

    fn set_decimal_flag(&mut self) {
        self.status.insert(CPUFlags::DECIMAL_MODE);
    }

    fn clear_decimal_flag(&mut self) {
        self.status.remove(CPUFlags::DECIMAL_MODE);
    }

    fn set_overflow_flag(&mut self) {
        self.status.insert(CPUFlags::OVERFLOW);
    }

    fn clear_overflow_flag(&mut self) {
        self.status.remove(CPUFlags::OVERFLOW);
    }

    fn set_interrupt_flag(&mut self) {
        self.status.insert(CPUFlags::INTERRUPT_DISABLE);
    }

    fn clear_interrupt_flag(&mut self) {
        self.status.remove(CPUFlags::INTERRUPT_DISABLE);
    }

    fn set_reg_a(&mut self, value:u8) {
        self.reg_a = value;
        self.update_z_and_n(self.reg_a);
    }

    fn set_reg_x(&mut self, value:u8) {
        self.reg_x = value;
        self.update_z_and_n(self.reg_x);
    }

    fn set_reg_y(&mut self, value:u8) {
        self.reg_y = value;
        self.update_z_and_n(self.reg_y);
    }

    fn lda(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.set_reg_a(value);
    }

    fn sta(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr,self.reg_a);
    }

    fn tax(&mut self) { // transfer A to X
        self.set_reg_x(self.reg_a);
    }

    fn inx(&mut self) { 
        self.set_reg_x(self.reg_x.wrapping_add(1));
    }

    fn and(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.set_reg_a(self.reg_a & value);
    }

    fn asl(&mut self,mode:&AddressingMode) -> u8 {
        let addr = self.get_operand_address(mode);
        let mut value = self.mem_read(addr);
        if value >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        value = value << 1;
        self.mem_write(addr,value);
        self.update_z_and_n(value);
        value
    }

    fn asl_acc(&mut self) {
        let mut value = self.reg_a;
        if value >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        value = value << 1;
        self.reg_a = value;
    }

    fn branch(&mut self,condition:bool) {
        if condition {
            let jump:i8 = self.mem_read(self.pc) as i8;
            let jump_addr = self.pc.wrapping_add(1).wrapping_add(jump as u16);
            self.pc = jump_addr;
        }
    }

    fn bit(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let and = self.reg_a & value;

        if and == 0 {
            self.status.insert(CPUFlags::ZERO);
        } else {
            self.status.remove(CPUFlags::ZERO);
        }
    }

    fn clc(&mut self) {
        self.clear_carry_flag();
    }

    fn cld(&mut self) {
        self.clear_decimal_flag();
    }

    fn cli(&mut self) {
        self.clear_interrupt_flag();
    }

    fn clv(&mut self) {
        self.clear_overflow_flag();
    }

    fn cmp(&mut self, mode:&AddressingMode, cmp_with:u8) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        if value <= cmp_with {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        self.update_z_and_n(cmp_with.wrapping_sub(value));
    }

    fn dec(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let new = value.wrapping_sub(1);

        self.mem_write(addr,new);
        self.update_z_and_n(new);
    }

    fn dex(&mut self) {
        let new = self.reg_x.wrapping_sub(1);
        self.set_reg_x(new);
    }

    fn dey(&mut self) {
        let new = self.reg_y.wrapping_sub(1);
        self.set_reg_y(new);
    }

    fn eor(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        self.set_reg_a(self.reg_a ^ data);
    }

    fn inc(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        data = data.wrapping_add(1);
        self.mem_write(addr,data);
        self.update_z_and_n(data);
    }

    fn inx(&mut self) {
        self.set_reg_x(self.reg_x.wrapping_add(1));
    }

    fn iny(&mut self) {
        self.set_reg_y(self.reg_y.wrapping_add(1));
    }

    fn ldx(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        self.set_reg_x(data);
    }

    fn ldy(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        self.set_reg_y(data);
    }

    fn lsr_acc(&mut self) {
        let mut data = self.reg_a;
        if data & 0b0000_0001 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data >> 1;
        self.set_reg_a(data);
    }

    fn lsr(&mut self, mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        if data & 0b0000_0001 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data >> 1;
        self.set_reg_a(data);
    }

    fn nop(&mut self) {
        return
    }

    fn ora(&mut self,mode:&AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        self.set_reg_a(self.reg_a | data);
        self.update_z_and_n(self.reg_a);
    }


    pub fn run(&mut self) {
        let ref opcodes:HashMap<u8,&'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read(self.pc);
            self.pc += 1;
            let pc_state = self.pc;
            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized",code));
            match code {
                // LDA
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {self.lda(&opcode.mode);}

                // AND
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {self.and(&opcode.mode);}
                
                // ASL
                0x0a => {self.asl_acc();} 
                
                // ASL
                0x06 | 0x16 | 0x0e | 0x1e => {self.asl(&opcode.mode);}
                
                // STA
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {self.sta(&opcode.mode);}
                
                // BRK
                0x00 => {return;}

                // TAX
                0xaa => {self.tax();}

                // INX
                0xe8 => {self.inx();}

                // BIT
                0x24 | 0x2c => {self.bit(&opcode.mode);}

                // CLC
                0x18 => { self.clc(); }

                // CLD
                0xd8 => { self.cld(); } 

                // CLI
                0x58 => { self.cli(); }

                // CLV
                0xb8 => { self.clv(); }

                // CMP
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 
                    => { self.compare(&opcode.mode,self.reg_a); }

                // CPX
                0xe0 | 0xe4 | 0xec => { self.compare(&opcode.mode, self.reg_x); }

                // CPY
                0xc0 | 0xc4 | 0xcc => { self.compare(&opcode.mode, self.reg_y); }

                // DEC
                0xc6 | 0xd6 | 0xce | 0xde => { self.dec(&opcode.mode); }

                // DEX
                0xca => { self.dex(); }

                // DEY
                0xca => { self.dey(); }

                // EOR
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 
                    => { self.eor(&opcode.mode); }

                // INC
                0xe6 | 0xf6 | 0xee | 0xfe => { self.inc(&opcode.mode); }

                // INX 
                0xe8 => { self.inx(); }

                // INY
                0xc8 => { self.iny(); }

                // JMP (indirect addressing)
                0x6c => { /* TODO */}

                // JMP (absolute addressing)
                0x4c => { /* TODO */}

                // JSR 
                0x20 => {/* TODO */}

                // LDX
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe
                    => { self.ldx(&opcode.mode); }

                // LDY
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc
                    => { self.ldy(&opcode.mode); }

                // LSR (no addressing)
                0x4a => { self.lsr_acc(); }

                // LSR
                0x46 | 0x56 | 0x4e | 0x5e
                    => { self.lsr(&opcode.mode); }

                // NOP
                0xea => { self.nop(); }
                
                // ORA
                0x09 | 0x05 | 0x15 | 0x0d  | 0x19 | 0x01 | 0x11
                    => {self.ora(&opcode.mode);}

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


