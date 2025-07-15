{
CPU6502 v0.1beta
(c) Copyright Adrian Chiang, 2025. All Rights Reserved.

Distributed under the MIT license.

A complete 65C02 processor emulator with 64K RAM, all addressing, states, and registers 
emulated in Free Pascal. Also, s boolean variable called "6502Enhanced" which when true 
emulates a WD65C02 processor, and when false the standard MOS6502 processor. Please also include all known
undocumented opcodes to be supported when the "6502Undocumented" flag is enabled, works 
together with the 6502Enhanced flag for undocumented 65c02 opcodes.

}

unit CPU6502;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

var
  Opcode : Byte;

type
  TCPU6502 = class
  private
    // Processor registers
    PC: Word;      // Program Counter
    SP: Byte;      // Stack Pointer
    A: Byte;       // Accumulator
    X: Byte;       // X Index Register
    Y: Byte;       // Y Index Register
    P: Byte;       // Status Register (NV-BDIZC)
    
    // Memory (64K)
    RAM: array[0..$FFFF] of Byte;
    
    // Configuration flags
    F6502Enhanced: Boolean;
    F6502Undocumented: Boolean;
    
    // Internal state
    Cycles: Integer;
    Stall: Integer;
    NMI: Boolean;
    IRQ: Boolean;
    ResetSignal: Boolean;
    
    // Helper methods for the Memory property
    function GetMemory(AIndex: Word): Byte;
    procedure SetMemory(AIndex: Word; AValue: Byte);
    
    
    // Addressing modes
    function Addr_ACC: Word;      // Accumulator
    function Addr_IMM: Word;       // Immediate
    function Addr_ABS: Word;       // Absolute
    function Addr_ZP: Word;        // Zero Page
    function Addr_IMP: Word;       // Implied
    function Addr_REL: Word;       // Relative
    function Addr_ABX(PageBoundary: Boolean): Word; // Absolute,X
    function Addr_ABY(PageBoundary: Boolean): Word; // Absolute,Y
    function Addr_ZPX: Word;       // Zero Page,X
    function Addr_ZPY: Word;       // Zero Page,Y
    function Addr_INX: Word;       // (Indirect,X)
    function Addr_INY(PageBoundary: Boolean): Word; // (Indirect),Y
    function Addr_IND: Word;       // (Indirect)
    function Addr_ZPREL: Word;     // Zero Page Relative (BBR/BBS)
    
    // Opcode functions
    procedure Op_ADC(Addr: Word);  // Add with Carry
    procedure Op_AND(Addr: Word);  // Logical AND
    procedure Op_ASL(Addr: Word);  // Arithmetic Shift Left
    procedure Op_ASL_ACC;          // ASL Accumulator
    procedure Op_BCC(Addr: Word);  // Branch if Carry Clear
    procedure Op_BCS(Addr: Word);  // Branch if Carry Set
    procedure Op_BEQ(Addr: Word);  // Branch if Equal
    procedure Op_BIT(Addr: Word);  // Bit Test
    procedure Op_BMI(Addr: Word);  // Branch if Minus
    procedure Op_BNE(Addr: Word);  // Branch if Not Equal
    procedure Op_BPL(Addr: Word);  // Branch if Plus
    procedure Op_BRA(Addr: Word);  // Branch Always (65C02)
    procedure Op_BRK;              // Break
    procedure Op_BVC(Addr: Word);  // Branch if Overflow Clear
    procedure Op_BVS(Addr: Word);  // Branch if Overflow Set
    procedure Op_CLC;              // Clear Carry
    procedure Op_CLD;              // Clear Decimal
    procedure Op_CLI;              // Clear Interrupt
    procedure Op_CLV;              // Clear Overflow
    procedure Op_CMP(Addr: Word);  // Compare
    procedure Op_CPX(Addr: Word);  // Compare X
    procedure Op_CPY(Addr: Word);  // Compare Y
    procedure Op_DEC(Addr: Word);  // Decrement
    procedure Op_DEX;              // Decrement X
    procedure Op_DEY;              // Decrement Y
    procedure Op_EOR(Addr: Word);  // Exclusive OR
    procedure Op_INC(Addr: Word);  // Increment
    procedure Op_INX;              // Increment X
    procedure Op_INY;              // Increment Y
    procedure Op_JMP(Addr: Word);  // Jump
    procedure Op_JSR(Addr: Word);  // Jump to Subroutine
    procedure Op_LDA(Addr: Word);  // Load A
    procedure Op_LDX(Addr: Word);  // Load X
    procedure Op_LDY(Addr: Word);  // Load Y
    procedure Op_LSR(Addr: Word);  // Logical Shift Right
    procedure Op_LSR_ACC;          // LSR Accumulator
    procedure Op_NOP;              // No Operation
    procedure Op_ORA(Addr: Word);  // Logical Inclusive OR
    procedure Op_PHA;              // Push A
    procedure Op_PHP;              // Push P
    procedure Op_PHX;              // Push X (65C02)
    procedure Op_PHY;              // Push Y (65C02)
    procedure Op_PLA;              // Pull A
    procedure Op_PLP;              // Pull P
    procedure Op_PLX;              // Pull X (65C02)
    procedure Op_PLY;              // Pull Y (65C02)
    procedure Op_ROL(Addr: Word);  // Rotate Left
    procedure Op_ROL_ACC;          // ROL Accumulator
    procedure Op_ROR(Addr: Word);  // Rotate Right
    procedure Op_ROR_ACC;          // ROR Accumulator
    procedure Op_RTI;              // Return from Interrupt
    procedure Op_RTS;              // Return from Subroutine
    procedure Op_SBC(Addr: Word);  // Subtract with Carry
    procedure Op_SEC;              // Set Carry
    procedure Op_SED;              // Set Decimal
    procedure Op_SEI;              // Set Interrupt
    procedure Op_STA(Addr: Word);  // Store A
    procedure Op_STX(Addr: Word);  // Store X
    procedure Op_STY(Addr: Word);  // Store Y
    procedure Op_STZ(Addr: Word);  // Store Zero (65C02)
    procedure Op_TAX;              // Transfer A to X
    procedure Op_TAY;              // Transfer A to Y
    procedure Op_TRB(Addr: Word);  // Test and Reset Bits (65C02)
    procedure Op_TSB(Addr: Word);  // Test and Set Bits (65C02)
    procedure Op_TSX;              // Transfer SP to X
    procedure Op_TXA;              // Transfer X to A
    procedure Op_TXS;              // Transfer X to SP
    procedure Op_TYA;              // Transfer Y to A
    
    // Undocumented opcodes
    procedure Op_ANC(Addr: Word);  // AND then copy N to C (undocumented)
    procedure Op_ARR(Addr: Word);  // AND then ROR (undocumented)
    procedure Op_ASR(Addr: Word);  // AND then LSR (undocumented)
    procedure Op_AXS(Addr: Word);  // A AND X then subtract operand (undocumented)
    procedure Op_DCP(Addr: Word);  // DEC then CMP (undocumented)
    procedure Op_ISB(Addr: Word);  // INC then SBC (undocumented)
    procedure Op_LAS(Addr: Word);  // LDA/TSX AND (undocumented)
    procedure Op_LAX(Addr: Word);  // LDA then TAX (undocumented)
    procedure Op_RLA(Addr: Word);  // ROL then AND (undocumented)
    procedure Op_RRA(Addr: Word);  // ROR then ADC (undocumented)
    procedure Op_SAX(Addr: Word);  // STA then STX (undocumented)
    procedure Op_SBX(Addr: Word);  // CMP/DEX (undocumented)
    procedure Op_SHA(Addr: Word);  // STA AND X AND high byte+1 (undocumented)
    procedure Op_SHS(Addr: Word);  // SHA and TXS (undocumented)
    procedure Op_SHX(Addr: Word);  // STX AND high byte+1 (undocumented)
    procedure Op_SHY(Addr: Word);  // STY AND high byte+1 (undocumented)
    procedure Op_SLO(Addr: Word);  // ASL then ORA (undocumented)
    procedure Op_SRE(Addr: Word);  // LSR then EOR (undocumented)
    
    // Helper functions
    procedure Push(Value: Byte);
    function Pull: Byte;
    procedure SetFlag(Flag: Byte; Value: Boolean);
    function GetFlag(Flag: Byte): Boolean;
    function ReadByte(Addr: Word): Byte;
    procedure WriteByte(Addr: Word; Value: Byte);
    function ReadWord(Addr: Word): Word;
    procedure WriteWord(Addr: Word; Value: Word);
    
public
    constructor Create;
    procedure Reset;
    procedure Step;
    procedure NMIRequest;
    procedure IRQRequest;
    procedure ResetRequest;
    
    property EnhancedMode: Boolean read F6502Enhanced write F6502Enhanced;
    property UndocumentedOpcodes: Boolean read F6502Undocumented write F6502Undocumented;
    property Memory[AIndex: Word]: Byte read GetMemory write SetMemory; default;
  end;

implementation

constructor TCPU6502.Create;
begin
  Reset;
  F6502Enhanced := False;
  F6502Undocumented := False;
end;

function TCPU6502.GetMemory(AIndex: Word): Byte;
begin
  Result := RAM[AIndex];
end;

procedure TCPU6502.SetMemory(AIndex: Word; AValue: Byte);
begin
  RAM[AIndex] := AValue;
end;

procedure TCPU6502.Reset;
begin
  PC := ReadWord($FFFC);
  SP := $FD;
  A := 0;
  X := 0;
  Y := 0;
  P := $34;  // Bit 5 always set, interrupt flag set
  Cycles := 0;
  Stall := 0;
  NMI := False;
  IRQ := False;
  ResetSignal := False;
end;

procedure TCPU6502.Step;
{
var
  Opcode: Byte;
  Addr: Word;
  PageCrossed: Boolean;
}
begin
  if Stall > 0 then
  begin
    Dec(Stall);
    Exit;
  end;
  
  // Handle interrupts
  if NMI then
  begin
    NMI := False;
    Op_BRK;
    PC := ReadWord($FFFA);
  end
  else if (not GetFlag(2)) and IRQ then
  begin
    IRQ := False;
    Op_BRK;
    PC := ReadWord($FFFE);
  end;
  
  Opcode := ReadByte(PC);
  Inc(PC);
  
  case Opcode of
    // Official 6502/65C02 opcodes
    $00: Op_BRK;
    $01: Op_ORA(Addr_INX);
    $05: Op_ORA(Addr_ZP);
    $06: Op_ASL(Addr_ZP);
    $08: Op_PHP;
    $09: Op_ORA(Addr_IMM);
    $0A: Op_ASL_ACC;
    $0D: Op_ORA(Addr_ABS);
    $0E: Op_ASL(Addr_ABS);
    
    $10: Op_BPL(Addr_REL);
    $11: Op_ORA(Addr_INY(False));
    $15: Op_ORA(Addr_ZPX);
    $16: Op_ASL(Addr_ZPX);
    $18: Op_CLC;
    $19: Op_ORA(Addr_ABY(False));
    $1D: Op_ORA(Addr_ABX(False));
    $1E: Op_ASL(Addr_ABX(False));
    
    $20: Op_JSR(Addr_ABS);
    $21: Op_AND(Addr_INX);
    $24: Op_BIT(Addr_ZP);
    $25: Op_AND(Addr_ZP);
    $26: Op_ROL(Addr_ZP);
    $28: Op_PLP;
    $29: Op_AND(Addr_IMM);
    $2A: Op_ROL_ACC;
    $2C: Op_BIT(Addr_ABS);
    $2D: Op_AND(Addr_ABS);
    $2E: Op_ROL(Addr_ABS);
    
    $30: Op_BMI(Addr_REL);
    $31: Op_AND(Addr_INY(False));
    $35: Op_AND(Addr_ZPX);
    $36: Op_ROL(Addr_ZPX);
    $38: Op_SEC;
    $39: Op_AND(Addr_ABY(False));
    $3D: Op_AND(Addr_ABX(False));
    $3E: Op_ROL(Addr_ABX(False));
    
    $40: Op_RTI;
    $41: Op_EOR(Addr_INX);
    $45: Op_EOR(Addr_ZP);
    $46: Op_LSR(Addr_ZP);
    $48: Op_PHA;
    $49: Op_EOR(Addr_IMM);
    $4A: Op_LSR_ACC;
    $4C: Op_JMP(Addr_ABS);
    $4D: Op_EOR(Addr_ABS);
    $4E: Op_LSR(Addr_ABS);
    
    $50: Op_BVC(Addr_REL);
    $51: Op_EOR(Addr_INY(False));
    $55: Op_EOR(Addr_ZPX);
    $56: Op_LSR(Addr_ZPX);
    $58: Op_CLI;
    $59: Op_EOR(Addr_ABY(False));
    $5D: Op_EOR(Addr_ABX(False));
    $5E: Op_LSR(Addr_ABX(False));
    
    $60: Op_RTS;
    $61: Op_ADC(Addr_INX);
    $65: Op_ADC(Addr_ZP);
    $66: Op_ROR(Addr_ZP);
    $68: Op_PLA;
    $69: Op_ADC(Addr_IMM);
    $6A: Op_ROR_ACC;
    $6C: Op_JMP(Addr_IND);
    $6D: Op_ADC(Addr_ABS);
    $6E: Op_ROR(Addr_ABS);
    
    $70: Op_BVS(Addr_REL);
    $71: Op_ADC(Addr_INY(False));
    $75: Op_ADC(Addr_ZPX);
    $76: Op_ROR(Addr_ZPX);
    $78: Op_SEI;
    $79: Op_ADC(Addr_ABY(False));
    $7D: Op_ADC(Addr_ABX(False));
    $7E: Op_ROR(Addr_ABX(False));
    
    $81: Op_STA(Addr_INX);
    $84: Op_STY(Addr_ZP);
    $85: Op_STA(Addr_ZP);
    $86: Op_STX(Addr_ZP);
    $88: Op_DEY;
    $8A: Op_TXA;
    $8C: Op_STY(Addr_ABS);
    $8D: Op_STA(Addr_ABS);
    $8E: Op_STX(Addr_ABS);
    
    $90: Op_BCC(Addr_REL);
    $91: Op_STA(Addr_INY(False));
    $94: Op_STY(Addr_ZPX);
    $95: Op_STA(Addr_ZPX);
    $96: Op_STX(Addr_ZPY);
    $98: Op_TYA;
    $99: Op_STA(Addr_ABY(False));
    $9A: Op_TXS;
    $9D: Op_STA(Addr_ABX(False));
    
    $A0: Op_LDY(Addr_IMM);
    $A1: Op_LDA(Addr_INX);
    $A2: Op_LDX(Addr_IMM);
    $A4: Op_LDY(Addr_ZP);
    $A5: Op_LDA(Addr_ZP);
    $A6: Op_LDX(Addr_ZP);
    $A8: Op_TAY;
    $A9: Op_LDA(Addr_IMM);
    $AA: Op_TAX;
    $AC: Op_LDY(Addr_ABS);
    $AD: Op_LDA(Addr_ABS);
    $AE: Op_LDX(Addr_ABS);
    
    $B0: Op_BCS(Addr_REL);
    $B1: Op_LDA(Addr_INY(False));
    $B4: Op_LDY(Addr_ZPX);
    $B5: Op_LDA(Addr_ZPX);
    $B6: Op_LDX(Addr_ZPY);
    $B8: Op_CLV;
    $B9: Op_LDA(Addr_ABY(False));
    $BA: Op_TSX;
    $BC: Op_LDY(Addr_ABX(False));
    $BD: Op_LDA(Addr_ABX(False));
    $BE: Op_LDX(Addr_ABY(False));
    
    $C0: Op_CPY(Addr_IMM);
    $C1: Op_CMP(Addr_INX);
    $C4: Op_CPY(Addr_ZP);
    $C5: Op_CMP(Addr_ZP);
    $C6: Op_DEC(Addr_ZP);
    $C8: Op_INY;
    $C9: Op_CMP(Addr_IMM);
    $CA: Op_DEX;
    $CC: Op_CPY(Addr_ABS);
    $CD: Op_CMP(Addr_ABS);
    $CE: Op_DEC(Addr_ABS);
    
    $D0: Op_BNE(Addr_REL);
    $D1: Op_CMP(Addr_INY(False));
    $D5: Op_CMP(Addr_ZPX);
    $D6: Op_DEC(Addr_ZPX);
    $D8: Op_CLD;
    $D9: Op_CMP(Addr_ABY(False));
    $DD: Op_CMP(Addr_ABX(False));
    $DE: Op_DEC(Addr_ABX(False));
    
    $E0: Op_CPX(Addr_IMM);
    $E1: Op_SBC(Addr_INX);
    $E4: Op_CPX(Addr_ZP);
    $E5: Op_SBC(Addr_ZP);
    $E6: Op_INC(Addr_ZP);
    $E8: Op_INX;
    $E9: Op_SBC(Addr_IMM);
    $EA: Op_NOP;
    $EC: Op_CPX(Addr_ABS);
    $ED: Op_SBC(Addr_ABS);
    $EE: Op_INC(Addr_ABS);
    
    $F0: Op_BEQ(Addr_REL);
    $F1: Op_SBC(Addr_INY(False));
    $F5: Op_SBC(Addr_ZPX);
    $F6: Op_INC(Addr_ZPX);
    $F8: Op_SED;
    $F9: Op_SBC(Addr_ABY(False));
    $FD: Op_SBC(Addr_ABX(False));
    $FE: Op_INC(Addr_ABX(False));
    
    // 65C02 specific opcodes
    $04: if F6502Enhanced then Op_TSB(Addr_ZP) else if F6502Undocumented then Op_NOP;
    $0C: if F6502Enhanced then Op_TSB(Addr_ABS) else if F6502Undocumented then Op_NOP;
    $14: if F6502Enhanced then Op_TRB(Addr_ZP) else if F6502Undocumented then Op_NOP;
    $1C: if F6502Enhanced then Op_TRB(Addr_ABS) else if F6502Undocumented then Op_NOP;
{
    $1A: if F6502Enhanced then Op_INC_ACC else if F6502Undocumented then Op_NOP;
    $3A: if F6502Enhanced then Op_DEC_ACC else if F6502Undocumented then Op_NOP;
}
    $5A: if F6502Enhanced then Op_PHY else if F6502Undocumented then Op_NOP;
    $7A: if F6502Enhanced then Op_PLY else if F6502Undocumented then Op_NOP;
    $80: if F6502Enhanced then Op_BRA(Addr_REL) else if F6502Undocumented then Op_NOP;
    $89: if F6502Enhanced then Op_BIT(Addr_IMM) else if F6502Undocumented then Op_NOP;
    $DA: if F6502Enhanced then Op_PHX else if F6502Undocumented then Op_NOP;
    $FA: if F6502Enhanced then Op_PLX else if F6502Undocumented then Op_NOP;
    $64: if F6502Enhanced then Op_STZ(Addr_ZP) else if F6502Undocumented then Op_NOP;
    $74: if F6502Enhanced then Op_STZ(Addr_ZPX) else if F6502Undocumented then Op_NOP;
    $9C: if F6502Enhanced then Op_STZ(Addr_ABS) else if F6502Undocumented then Op_NOP;
    $9E: if F6502Enhanced then Op_STZ(Addr_ABX(False)) else if F6502Undocumented then Op_NOP;

{    
    // BBR/BBS (65C02)
    $0F..$1F: if F6502Enhanced then 
              begin
                if (Opcode and $0F) in [0,1,2,3,4,5,6,7] then
                begin
                  // BBR
                  Addr := Addr_ZPREL;
                  if not GetFlag(Opcode and $07) then
                    PC := Addr;
                end
                else
                begin
                  // BBS
                  Addr := Addr_ZPREL;
                  if GetFlag(Opcode and $07) then
                    PC := Addr;
                end;
              end
              else if F6502Undocumented then Op_NOP;
}
    
    // Undocumented opcodes
    $02, $03, $07, $0B, $0F, $12, $13, $17, $1B, $1F, 
    $22, $23, $27, $2B, $2F, $32, $33, $37, $3B, $3F, 
    $42, $43, $47, $4B, $4F, $52, $53, $57, $5B, $5F, 
    $62, $63, $67, $6B, $6F, $72, $73, $77, $7B, $7F, 
    $82, $83, $87, $8B, $8F, $92, $93, $97, $9B, $9F, 
    $A3, $A7, $AB, $AF, $B2, $B3, $B7, $BB, $BF, 
    $C2, $C3, $C7, $CB, $CF, $D2, $D3, $D7, $DB, $DF, 
    $E2, $E3, $E7, $EB, $EF, $F2, $F3, $F7, $FB, $FF: 
      if F6502Undocumented then
      begin
        // Handle undocumented opcodes
        case Opcode of
          $03: Op_SLO(Addr_INX);
          $07: Op_SLO(Addr_ZP);
          $0F: Op_SLO(Addr_ABS);
          $13: Op_SLO(Addr_INY(False));
          $17: Op_SLO(Addr_ZPX);
          $1B: Op_SLO(Addr_ABY(False));
          $1F: Op_SLO(Addr_ABX(False));
          
          $23: Op_RLA(Addr_INX);
          $27: Op_RLA(Addr_ZP);
          $2F: Op_RLA(Addr_ABS);
          $33: Op_RLA(Addr_INY(False));
          $37: Op_RLA(Addr_ZPX);
          $3B: Op_RLA(Addr_ABY(False));
          $3F: Op_RLA(Addr_ABX(False));
          
          $43: Op_SRE(Addr_INX);
          $47: Op_SRE(Addr_ZP);
          $4F: Op_SRE(Addr_ABS);
          $53: Op_SRE(Addr_INY(False));
          $57: Op_SRE(Addr_ZPX);
          $5B: Op_SRE(Addr_ABY(False));
          $5F: Op_SRE(Addr_ABX(False));
          
          $63: Op_RRA(Addr_INX);
          $67: Op_RRA(Addr_ZP);
          $6F: Op_RRA(Addr_ABS);
          $73: Op_RRA(Addr_INY(False));
          $77: Op_RRA(Addr_ZPX);
          $7B: Op_RRA(Addr_ABY(False));
          $7F: Op_RRA(Addr_ABX(False));
          
          $83: Op_SAX(Addr_INX);
          $87: Op_SAX(Addr_ZP);
          $8F: Op_SAX(Addr_ABS);
          $93: Op_SHA(Addr_INY(False));
          $97: Op_SAX(Addr_ZPY);
          $9B: Op_SHS(Addr_ABY(False));
          $9F: Op_SHA(Addr_ABY(False));
          
          $A3: Op_LAX(Addr_INX);
          $A7: Op_LAX(Addr_ZP);
          $AB: Op_LAX(Addr_IMM);
          $AF: Op_LAX(Addr_ABS);
          $B3: Op_LAX(Addr_INY(False));
          $B7: Op_LAX(Addr_ZPY);
          $BF: Op_LAX(Addr_ABY(False));
          
          $C3: Op_DCP(Addr_INX);
          $C7: Op_DCP(Addr_ZP);
          $CF: Op_DCP(Addr_ABS);
          $D3: Op_DCP(Addr_INY(False));
          $D7: Op_DCP(Addr_ZPX);
          $DB: Op_DCP(Addr_ABY(False));
          $DF: Op_DCP(Addr_ABX(False));
          
          $E3: Op_ISB(Addr_INX);
          $E7: Op_ISB(Addr_ZP);
          $EB: Op_SBC(Addr_IMM);  // USBC
          $EF: Op_ISB(Addr_ABS);
          $F3: Op_ISB(Addr_INY(False));
          $F7: Op_ISB(Addr_ZPX);
          $FB: Op_ISB(Addr_ABY(False));
          $FF: Op_ISB(Addr_ABX(False));
          
          else Op_NOP;
        end;
      end
      else
        Op_NOP;
  end;
end;

// Addressing mode implementations
function TCPU6502.Addr_ACC: Word;
begin
  Result := 0; // Dummy value, accumulator mode doesn't use address
end;

function TCPU6502.Addr_IMM: Word;
begin
  Result := PC;
  Inc(PC);
end;

function TCPU6502.Addr_ABS: Word;
begin
  Result := ReadWord(PC);
  Inc(PC, 2);
end;

function TCPU6502.Addr_ZP: Word;
begin
  Result := ReadByte(PC);
  Inc(PC);
end;

function TCPU6502.Addr_IMP: Word;
begin
  Result := 0; // Dummy value, implied mode doesn't use address
end;

function TCPU6502.Addr_REL: Word;
var
  Offset: ShortInt;
begin
  Offset := ShortInt(ReadByte(PC));
  Inc(PC);
  Result := PC + Offset;
end;

function TCPU6502.Addr_ABX(PageBoundary: Boolean): Word;
var
  Base: Word;
begin
  Base := ReadWord(PC);
  Inc(PC, 2);
  Result := Base + X;
  
  if PageBoundary and ((Base and $FF00) <> (Result and $FF00)) then
    Inc(Cycles);
end;

function TCPU6502.Addr_ABY(PageBoundary: Boolean): Word;
var
  Base: Word;
begin
  Base := ReadWord(PC);
  Inc(PC, 2);
  Result := Base + Y;
  
  if PageBoundary and ((Base and $FF00) <> (Result and $FF00)) then
    Inc(Cycles);
end;

function TCPU6502.Addr_ZPX: Word;
begin
  Result := (ReadByte(PC) + X) and $FF;
  Inc(PC);
end;

function TCPU6502.Addr_ZPY: Word;
begin
  Result := (ReadByte(PC) + Y) and $FF;
  Inc(PC);
end;

function TCPU6502.Addr_INX: Word;
var
  ZP: Byte;
begin
  ZP := (ReadByte(PC) + X) and $FF;
  Inc(PC);
  Result := ReadWord(ZP);
end;

function TCPU6502.Addr_INY(PageBoundary: Boolean): Word;
var
  ZP, Base: Word;
begin
  ZP := ReadByte(PC);
  Inc(PC);
  Base := ReadWord(ZP);
  Result := Base + Y;
  
  if PageBoundary and ((Base and $FF00) <> (Result and $FF00)) then
    Inc(Cycles);
end;

function TCPU6502.Addr_IND: Word;
var
  Target: Word;
begin
  Target := ReadWord(PC);
  Inc(PC, 2);
  
  // 6502 has a bug in indirect JMP that crosses page boundary
  if not F6502Enhanced and ((Target and $FF) = $FF) then
    Result := (ReadByte(Target and $FF00) shl 8) or ReadByte(Target)
  else
    Result := ReadWord(Target);
end;

function TCPU6502.Addr_ZPREL: Word;
var
  ZP, Offset: Byte;
  Target: Word;
begin
  ZP := ReadByte(PC);
  Inc(PC);
  Offset := ReadByte(PC);
  Inc(PC);
  Target := PC + ShortInt(Offset);
  
  // Test the bit in the zero page address
  if ((ReadByte(ZP) shr (Opcode and $07)) and $01) = ((Opcode shr 4) and $01) then
    Result := Target
  else
    Result := PC;
end;

// Opcode implementations
procedure TCPU6502.Op_ADC(Addr: Word);
var
  Operand, Sum: Word;
begin
  Operand := ReadByte(Addr);
  Sum := A + Operand + Ord(GetFlag(0));
  
  SetFlag(0, Sum > $FF);
  SetFlag(1, (A xor Sum) and (Operand xor Sum) and $80 <> 0);
  SetFlag(7, (Sum and $80) <> 0);
  SetFlag(6, (Sum and $FF) = 0);
  
  A := Sum and $FF;
end;

procedure TCPU6502.Op_AND(Addr: Word);
begin
  A := A and ReadByte(Addr);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_ASL(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr);
  SetFlag(0, (Value and $80) <> 0);
  Value := Value shl 1;
  WriteByte(Addr, Value);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_ASL_ACC;
begin
  SetFlag(0, (A and $80) <> 0);
  A := A shl 1;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_BCC(Addr: Word);
begin
  if not GetFlag(0) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BCS(Addr: Word);
begin
  if GetFlag(0) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BEQ(Addr: Word);
begin
  if GetFlag(6) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BIT(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(1, (Value and $40) <> 0);
  SetFlag(6, (A and Value) = 0);
end;

procedure TCPU6502.Op_BMI(Addr: Word);
begin
  if GetFlag(7) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BNE(Addr: Word);
begin
  if not GetFlag(6) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BPL(Addr: Word);
begin
  if not GetFlag(7) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BRA(Addr: Word);
begin
  Inc(Cycles);
  if (PC and $FF00) <> (Addr and $FF00) then
    Inc(Cycles);
  PC := Addr;
end;

procedure TCPU6502.Op_BRK;
begin
  Inc(PC);
  Push((PC shr 8) and $FF);
  Push(PC and $FF);
  Push(P or $10);  // Set Break flag
  SetFlag(2, True);
  PC := ReadWord($FFFE);
end;

procedure TCPU6502.Op_BVC(Addr: Word);
begin
  if not GetFlag(1) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_BVS(Addr: Word);
begin
  if GetFlag(1) then
  begin
    Inc(Cycles);
    if (PC and $FF00) <> (Addr and $FF00) then
      Inc(Cycles);
    PC := Addr;
  end;
end;

procedure TCPU6502.Op_CLC;
begin
  SetFlag(0, False);
end;

procedure TCPU6502.Op_CLD;
begin
  SetFlag(3, False);
end;

procedure TCPU6502.Op_CLI;
begin
  SetFlag(2, False);
end;

procedure TCPU6502.Op_CLV;
begin
  SetFlag(1, False);
end;

procedure TCPU6502.Op_CMP(Addr: Word);
var
  Value, Temp: Byte;
begin
  Value := ReadByte(Addr);
  Temp := A - Value;
  SetFlag(0, A >= Value);
  SetFlag(7, (Temp and $80) <> 0);
  SetFlag(6, Temp = 0);
end;

procedure TCPU6502.Op_CPX(Addr: Word);
var
  Value, Temp: Byte;
begin
  Value := ReadByte(Addr);
  Temp := X - Value;
  SetFlag(0, X >= Value);
  SetFlag(7, (Temp and $80) <> 0);
  SetFlag(6, Temp = 0);
end;

procedure TCPU6502.Op_CPY(Addr: Word);
var
  Value, Temp: Byte;
begin
  Value := ReadByte(Addr);
  Temp := Y - Value;
  SetFlag(0, Y >= Value);
  SetFlag(7, (Temp and $80) <> 0);
  SetFlag(6, Temp = 0);
end;

procedure TCPU6502.Op_DEC(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr) - 1;
  WriteByte(Addr, Value);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_DEX;
begin
  X := X - 1;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_DEY;
begin
  Y := Y - 1;
  SetFlag(7, (Y and $80) <> 0);
  SetFlag(6, Y = 0);
end;

procedure TCPU6502.Op_EOR(Addr: Word);
begin
  A := A xor ReadByte(Addr);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_INC(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr) + 1;
  WriteByte(Addr, Value);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_INX;
begin
  X := X + 1;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_INY;
begin
  Y := Y + 1;
  SetFlag(7, (Y and $80) <> 0);
  SetFlag(6, Y = 0);
end;

procedure TCPU6502.Op_JMP(Addr: Word);
begin
  PC := Addr;
end;

procedure TCPU6502.Op_JSR(Addr: Word);
begin
  Dec(PC);
  Push((PC shr 8) and $FF);
  Push(PC and $FF);
  PC := Addr;
end;

procedure TCPU6502.Op_LDA(Addr: Word);
begin
  A := ReadByte(Addr);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_LDX(Addr: Word);
begin
  X := ReadByte(Addr);
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_LDY(Addr: Word);
begin
  Y := ReadByte(Addr);
  SetFlag(7, (Y and $80) <> 0);
  SetFlag(6, Y = 0);
end;

procedure TCPU6502.Op_LSR(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr);
  SetFlag(0, (Value and $01) <> 0);
  Value := Value shr 1;
  WriteByte(Addr, Value);
  SetFlag(7, False);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_LSR_ACC;
begin
  SetFlag(0, (A and $01) <> 0);
  A := A shr 1;
  SetFlag(7, False);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_NOP;
begin
  // Do nothing
end;

procedure TCPU6502.Op_ORA(Addr: Word);
begin
  A := A or ReadByte(Addr);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_PHA;
begin
  Push(A);
end;

procedure TCPU6502.Op_PHP;
begin
  Push(P or $30);  // Set Break and unused flags
end;

procedure TCPU6502.Op_PHX;
begin
  Push(X);
end;

procedure TCPU6502.Op_PHY;
begin
  Push(Y);
end;

procedure TCPU6502.Op_PLA;
begin
  A := Pull;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_PLP;
begin
  P := (Pull and $CF) or $20;  // Clear Break and unused flags
end;

procedure TCPU6502.Op_PLX;
begin
  X := Pull;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_PLY;
begin
  Y := Pull;
  SetFlag(7, (Y and $80) <> 0);
  SetFlag(6, Y = 0);
end;

procedure TCPU6502.Op_ROL(Addr: Word);
var
  Value: Byte;
  OldCarry: Boolean;
begin
  OldCarry := GetFlag(0);
  Value := ReadByte(Addr);
  SetFlag(0, (Value and $80) <> 0);
  Value := (Value shl 1) or Ord(OldCarry);
  WriteByte(Addr, Value);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_ROL_ACC;
var
  OldCarry: Boolean;
begin
  OldCarry := GetFlag(0);
  SetFlag(0, (A and $80) <> 0);
  A := (A shl 1) or Ord(OldCarry);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_ROR(Addr: Word);
var
  Value: Byte;
  OldCarry: Boolean;
begin
  OldCarry := GetFlag(0);
  Value := ReadByte(Addr);
  SetFlag(0, (Value and $01) <> 0);
  Value := (Value shr 1) or (Ord(OldCarry) shl 7);
  WriteByte(Addr, Value);
  SetFlag(7, (Value and $80) <> 0);
  SetFlag(6, Value = 0);
end;

procedure TCPU6502.Op_ROR_ACC;
var
  OldCarry: Boolean;
begin
  OldCarry := GetFlag(0);
  SetFlag(0, (A and $01) <> 0);
  A := (A shr 1) or (Ord(OldCarry) shl 7);
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_RTI;
begin
  P := (Pull and $CF) or $20;  // Clear Break and unused flags
  PC := Pull;
  PC := PC or (Word(Pull) shl 8);
end;

procedure TCPU6502.Op_RTS;
begin
  PC := Pull;
  PC := PC or (Word(Pull) shl 8);
  Inc(PC);
end;

procedure TCPU6502.Op_SBC(Addr: Word);
var
  Operand, Sum: Word;
begin
  Operand := ReadByte(Addr);
  Sum := A - Operand - (1 - Ord(GetFlag(0)));
  
  SetFlag(1, ((A xor Sum) and ($80 xor Operand)) and $80 <> 0);
  SetFlag(0, Sum < $100);
  SetFlag(7, (Sum and $80) <> 0);
  SetFlag(6, (Sum and $FF) = 0);
  
  A := Sum and $FF;
end;

procedure TCPU6502.Op_SEC;
begin
  SetFlag(0, True);
end;

procedure TCPU6502.Op_SED;
begin
  SetFlag(3, True);
end;

procedure TCPU6502.Op_SEI;
begin
  SetFlag(2, True);
end;

procedure TCPU6502.Op_STA(Addr: Word);
begin
  WriteByte(Addr, A);
end;

procedure TCPU6502.Op_STX(Addr: Word);
begin
  WriteByte(Addr, X);
end;

procedure TCPU6502.Op_STY(Addr: Word);
begin
  WriteByte(Addr, Y);
end;

procedure TCPU6502.Op_STZ(Addr: Word);
begin
  WriteByte(Addr, 0);
end;

procedure TCPU6502.Op_TAX;
begin
  X := A;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_TAY;
begin
  Y := A;
  SetFlag(7, (Y and $80) <> 0);
  SetFlag(6, Y = 0);
end;

procedure TCPU6502.Op_TRB(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr);
  SetFlag(6, (A and Value) = 0);
  Value := Value and (not A);
  WriteByte(Addr, Value);
end;

procedure TCPU6502.Op_TSB(Addr: Word);
var
  Value: Byte;
begin
  Value := ReadByte(Addr);
  SetFlag(6, (A and Value) = 0);
  Value := Value or A;
  WriteByte(Addr, Value);
end;

procedure TCPU6502.Op_TSX;
begin
  X := SP;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_TXA;
begin
  A := X;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_TXS;
begin
  SP := X;
end;

procedure TCPU6502.Op_TYA;
begin
  A := Y;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

// Undocumented opcodes
procedure TCPU6502.Op_ANC(Addr: Word);
begin
  Op_AND(Addr);
  SetFlag(0, GetFlag(7));
end;

procedure TCPU6502.Op_ARR(Addr: Word);
begin
  Op_AND(Addr);
  Op_ROR_ACC;
  SetFlag(0, (A and $40) <> 0);
  SetFlag(1, ((A shr 6) xor (A shr 5)) and $01 <> 0);
end;

procedure TCPU6502.Op_ASR(Addr: Word);
begin
  Op_AND(Addr);
  Op_LSR_ACC;
end;

procedure TCPU6502.Op_AXS(Addr: Word);
var
  Value, Temp: Byte;
begin
  Value := ReadByte(Addr);
  Temp := (A and X) - Value;
  SetFlag(0, (A and X) >= Value);
  X := Temp;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_DCP(Addr: Word);
begin
  Op_DEC(Addr);
  Op_CMP(Addr);
end;

procedure TCPU6502.Op_ISB(Addr: Word);
begin
  Op_INC(Addr);
  Op_SBC(Addr);
end;

procedure TCPU6502.Op_LAS(Addr: Word);
begin
  A := ReadByte(Addr) and SP;
  X := A;
  SP := A;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_LAX(Addr: Word);
begin
  A := ReadByte(Addr);
  X := A;
  SetFlag(7, (A and $80) <> 0);
  SetFlag(6, A = 0);
end;

procedure TCPU6502.Op_RLA(Addr: Word);
begin
  Op_ROL(Addr);
  Op_AND(Addr);
end;

procedure TCPU6502.Op_RRA(Addr: Word);
begin
  Op_ROR(Addr);
  Op_ADC(Addr);
end;

procedure TCPU6502.Op_SAX(Addr: Word);
begin
  WriteByte(Addr, A and X);
end;

procedure TCPU6502.Op_SBX(Addr: Word);
var
  Value, Temp: Byte;
begin
  Value := ReadByte(Addr);
  Temp := (A and X) - Value;
  SetFlag(0, (A and X) >= Value);
  X := Temp;
  SetFlag(7, (X and $80) <> 0);
  SetFlag(6, X = 0);
end;

procedure TCPU6502.Op_SHA(Addr: Word);
var
  Value: Byte;
begin
  Value := A and X and ((Addr shr 8) + 1);
  WriteByte(Addr, Value);
end;

procedure TCPU6502.Op_SHS(Addr: Word);
begin
  SP := A and X;
  Op_SHA(Addr);
end;

procedure TCPU6502.Op_SHX(Addr: Word);
var
  Value: Byte;
begin
  Value := X and ((Addr shr 8) + 1);
  WriteByte(Addr, Value);
end;

procedure TCPU6502.Op_SHY(Addr: Word);
var
  Value: Byte;
begin
  Value := Y and ((Addr shr 8) + 1);
  WriteByte(Addr, Value);
end;

procedure TCPU6502.Op_SLO(Addr: Word);
begin
  Op_ASL(Addr);
  Op_ORA(Addr);
end;

procedure TCPU6502.Op_SRE(Addr: Word);
begin
  Op_LSR(Addr);
  Op_EOR(Addr);
end;

// Helper functions
procedure TCPU6502.Push(Value: Byte);
begin
  WriteByte($100 + SP, Value);
  Dec(SP);
end;

function TCPU6502.Pull: Byte;
begin
  Inc(SP);
  Result := ReadByte($100 + SP);
end;

procedure TCPU6502.SetFlag(Flag: Byte; Value: Boolean);
begin
  if Value then
    P := P or (1 shl Flag)
  else
    P := P and not (1 shl Flag);
end;

function TCPU6502.GetFlag(Flag: Byte): Boolean;
begin
  Result := (P and (1 shl Flag)) <> 0;
end;

function TCPU6502.ReadByte(Addr: Word): Byte;
begin
  Result := RAM[Addr];
end;

procedure TCPU6502.WriteByte(Addr: Word; Value: Byte);
begin
  RAM[Addr] := Value;
end;

function TCPU6502.ReadWord(Addr: Word): Word;
begin
  Result := RAM[Addr] or (RAM[(Addr + 1) and $FFFF] shl 8);
end;

procedure TCPU6502.WriteWord(Addr: Word; Value: Word);
begin
  RAM[Addr] := Value and $FF;
  RAM[(Addr + 1) and $FFFF] := (Value shr 8) and $FF;
end;

procedure TCPU6502.NMIRequest;
begin
  NMI := True;
end;

procedure TCPU6502.IRQRequest;
begin
  IRQ := True;
end;

procedure TCPU6502.ResetRequest;
begin
  ResetSignal := True;
end;

end.