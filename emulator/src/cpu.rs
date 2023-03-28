pub struct CPU {
    pub reg_a:u8,
    pub reg_x:u8,
    pub status:u8,
    pub pc:u16,
    //memory:[u8;0xffff]
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            reg_a: 0,
            reg_x: 0,
            status: 0,
            pc: 0
        }
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

    fn lda(&mut self,value:u8) {
        self.reg_a = value;
        self.update_z_and_n(self.reg_a);
    }

    fn tax(&mut self) { // transfer A to X
        self.reg_x = self.reg_a;
        self.update_z_and_n(self.reg_x);
    }

    fn inx(&mut self) { 
        self.reg_x = self.reg_x.wrapping_add(1);
        self.update_z_and_n(self.reg_x);
    }


    pub fn interpret(&mut self, program:Vec<u8>) {
        self.pc = 0;

        loop {
            let opscode = program[self.pc as usize];
            self.pc += 1;

            match opscode {

                0xa9 => { // LDA
                    let param = program[self.pc as usize];
                    self.pc += 1;
                    self.lda(param);
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
        cpu.interpret(vec![0xa9,0x05,0x00]); // LDA #$05, BRK
        assert_eq!(cpu.reg_a,5);
        assert!(cpu.status & 0b0000_0010 == 0); // check Z is 0
        assert!(cpu.status & 0b1000_0000 == 0);    // check N is 0
    }

    #[test]
    fn test_0xa9_lda_z_flag() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9,0x00,0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax() {
        let mut cpu = CPU::new();
        cpu.reg_a = 10;
        cpu.interpret(vec![0xaa,0x00]);
        assert_eq!(cpu.reg_x,10);
    }

    #[test]
    fn test_0xe8_inx() {
        let mut cpu = CPU::new();
        cpu.reg_x = 13;
        cpu.interpret(vec![0xe8,0x00]);
        assert_eq!(cpu.reg_x,14);
        assert!(cpu.status & 0b0000_0010 == 0);  // check Z is 0
        assert!(cpu.status & 0b1000_0000 == 0);     // check N is 0
    }

    #[test]
    fn test_0xe8_overflow() {
        let mut cpu = CPU::new();
        cpu.reg_x = 0xff;
        cpu.interpret(vec![0xe8,0xe8,0x00]);
        assert_eq!(cpu.reg_x,1);
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.interpret(vec![0xa9,0xc0,0xaa,0xe8,0x00]);
        assert_eq!(cpu.reg_x,0xc1);
    }
}


