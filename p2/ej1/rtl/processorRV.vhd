--------------------------------------------------------------------------------
-- Procesador RISC V uniciclo curso Arquitectura Ordenadores 2022
-- Initial Release G.Sutter jun 2022
-- 
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.RISCV_pack.all;

entity processorRV is
   port(
      Clk      : in  std_logic;                     -- Reloj activo en flanco subida
      Reset    : in  std_logic;                     -- Reset asincrono activo nivel alto
      -- InstructionIF memory
      IAddr    : out std_logic_vector(31 downto 0); -- Direccion Instr
      IDataIn  : in  std_logic_vector(31 downto 0); -- Instruccion leida
      -- Data memory
      DAddr    : out std_logic_vector(31 downto 0); -- Direccion
      DRdEn    : out std_logic;                     -- Habilitacion lectura
      DWrEn    : out std_logic;                     -- Habilitacion escritura
      DDataOut : out std_logic_vector(31 downto 0); -- Dato escrito
      DDataIn  : in  std_logic_vector(31 downto 0)  -- Dato leido
   );
end processorRV;

architecture rtl of processorRV is

  component alu_RV
    port (
      OpA     : in  std_logic_vector (31 downto 0); -- Operando A
      OpB     : in  std_logic_vector (31 downto 0); -- Operando B
      Control : in  std_logic_vector ( 3 downto 0); -- Codigo de control=op. a ejecutar
      Result  : out std_logic_vector (31 downto 0); -- Resultado
      SignFlag: out std_logic;                      -- Sign Flag
      carryOut: out std_logic;                      -- Carry bit
      ZFlag   : out std_logic                       -- Flag Z
    );
  end component;

  component reg_bank
     port (
        Clk   : in  std_logic;                      -- Reloj activo en flanco de subida
        Reset : in  std_logic;                      -- Reset as�ncrono a nivel alto
        A1    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd1
        Rd1   : out std_logic_vector(31 downto 0);  -- Dato del puerto Rd1
        A2    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd2
        Rd2   : out std_logic_vector(31 downto 0);  -- Dato del puerto Rd2
        A3    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Wd3
        Wd3   : in  std_logic_vector(31 downto 0);  -- Dato de entrada Wd3
        We3   : in  std_logic                       -- Habilitaci�n de la escritura de Wd3
     ); 
  end component reg_bank;

  component control_unit
     port (
        -- Entrada = codigo de operacion en la instruccion:
        OpCode   : in  std_logic_vector (6 downto 0);
        -- Seniales para el PC
        Branch   : out  std_logic;                     -- 1 = Ejecutandose instruccion branch
        -- Seniales relativas a la memoria
        ResultSrc: out  std_logic_vector(1 downto 0);  -- 00 salida Alu; 01 = salida de la mem.; 10 PC_plus4
        MemWrite : out  std_logic;                     -- Escribir la memoria
        MemRead  : out  std_logic;                     -- Leer la memoria
        -- Seniales para la ALU
        ALUSrc   : out  std_logic;                     -- 0 = oper.B es registro, 1 = es valor inm.
        AuipcLui : out  std_logic_vector (1 downto 0); -- 0 = PC. 1 = zeros, 2 = reg1.
        ALUOp    : out  std_logic_vector (2 downto 0); -- Tipo operacion para control de la ALU
        -- señal generacion salto
        Ins_jalr  : out  std_logic;                    -- 0=any instrucion, 1=jalr
        -- Seniales para el GPR
        RegWrite : out  std_logic                      -- 1 = Escribir registro
     );
  end component;

  component alu_control is
    port (
      -- Entradas:
      ALUOp  : in std_logic_vector (2 downto 0);     -- Codigo de control desde la unidad de control
      Funct3 : in std_logic_vector (2 downto 0);     -- Campo "funct3" de la instruccion (I(14:12))
      Funct7 : in std_logic_vector (6 downto 0);     -- Campo "funct7" de la instruccion (I(31:25))     
      -- Salida de control para la ALU:
      ALUControl : out std_logic_vector (3 downto 0) -- Define operacion a ejecutar por la ALU
    );
  end component alu_control;

 component Imm_Gen is
    port (
        instr     : in std_logic_vector(31 downto 0);
        imm       : out std_logic_vector(31 downto 0)
    );
  end component Imm_Gen;


  --Señales IF

  signal branch_true        : std_logic;
  signal PC_nextIF          : std_logic_vector(31 downto 0);
  signal PC_regIF           : std_logic_vector(31 downto 0);
  signal PC_plus4IF         : std_logic_vector(31 downto 0);

  signal InstructionIF      : std_logic_vector(31 downto 0); -- La instrucción desde lamem de instr

  signal PCWrite_DisableIF  : std_logic;

  --Señales IF/ID

  signal PC_regIFID         : std_logic_vector(31 downto 0);
  signal InstructionIFID    : std_logic_vector(31 downto 0);

  signal Write_DisableIFID  : std_logic;

  --Señales ID

  signal Ctrl_JalrID      : std_logic;
  signal Ctrl_BranchID    : std_logic;
  signal Ctrl_MemWriteID  : std_logic;
  signal Ctrl_MemReadID   : std_logic;
  signal Ctrl_ALUSrcID    : std_logic;
  signal Ctrl_RegWriteID  : std_logic;

  signal Ctrl_ALUOPID     : std_logic_vector(2 downto 0);
  signal Ctrl_PcLuiID     : std_logic_vector(1 downto 0);
  signal Ctrl_ResSrcID    : std_logic_vector(1 downto 0);  --MemtoReg

  signal Ctrl_HazardID    : std_logic;

  signal Inm_extID        : std_logic_vector(31 downto 0); -- La parte baja de la instrucción extendida de signo

  signal reg_RSID         : std_logic_vector(31 downto 0); --Read data 1
  signal reg_RTID         : std_logic_vector(31 downto 0); --Read data 2

  -- Instruction fields (Generadas en ID)
  signal Funct3ID         : std_logic_vector(2 downto 0);
  signal Funct7ID         : std_logic_vector(6 downto 0);
  signal RD_ID            : std_logic_vector(4 downto 0);
  signal RS1_ID           : std_logic_vector(4 downto 0);
  signal RS2_ID           : std_logic_vector(4 downto 0);

  --Señales ID/EX

  signal PC_regIDEX         : std_logic_vector(31 downto 0);

  signal Ctrl_JalrIDEX      : std_logic;
  signal Ctrl_BranchIDEX    : std_logic;
  signal Ctrl_MemWriteIDEX  : std_logic;
  signal Ctrl_MemReadIDEX   : std_logic;
  signal Ctrl_ALUSrcIDEX    : std_logic;
  signal Ctrl_RegWriteIDEX  : std_logic;

  signal Ctrl_ALUOPIDEX     : std_logic_vector(2 downto 0);
  signal Ctrl_PcLuiIDEX     : std_logic_vector(1 downto 0);
  signal Ctrl_ResSrcIDEX    : std_logic_vector(1 downto 0);  --MemtoReg

  signal Inm_extIDEX        : std_logic_vector(31 downto 0);

  signal reg_RSIDEX         : std_logic_vector(31 downto 0); --Read data 1
  signal reg_RTIDEX         : std_logic_vector(31 downto 0); --Read data 2

  signal Funct3IDEX         : std_logic_vector(2 downto 0);
  signal Funct7IDEX         : std_logic_vector(6 downto 0);

  signal RD_IDEX            : std_logic_vector(4 downto 0);
  signal RS1_IDEX           : std_logic_vector(4 downto 0);
  signal RS2_IDEX           : std_logic_vector(4 downto 0);

  --Señales EX

  signal Addr_BranchEX    : std_logic_vector(31 downto 0);
  signal Addr_jalrEX      : std_logic_vector(31 downto 0);

  signal AluControlEX     : std_logic_vector(3 downto 0);
  signal Alu_Op1EX        : std_logic_vector(31 downto 0);
  signal Alu_Op2EX        : std_logic_vector(31 downto 0);
  signal Alu_ResEX        : std_logic_vector(31 downto 0);
  signal Alu_SIGNEX       : std_logic;
  signal Alu_ZEROEX       : std_logic;

  --Señales EX/MEM

  signal Ctrl_BranchEXMEM    : std_logic;
  signal Ctrl_JalrEXMEM      : std_logic;
  signal Ctrl_MemWriteEXMEM  : std_logic;
  signal Ctrl_MemReadEXMEM   : std_logic;
  signal Ctrl_RegWriteEXMEM  : std_logic;
  signal Ctrl_ResSrcEXMEM    : std_logic_vector(1 downto 0);  --MemtoReg

  signal Addr_BranchEXMEM    : std_logic_vector(31 downto 0);
  signal Addr_jalrEXMEM      : std_logic_vector(31 downto 0);

  signal reg_RTEXMEM         : std_logic_vector(31 downto 0); --Read data 2
  signal RD_EXMEM             : std_logic_vector(4 downto 0);

  signal Alu_ResEXMEM        : std_logic_vector(31 downto 0);
  signal Alu_SIGNEXMEM       : std_logic;
  signal Alu_ZEROEXMEM       : std_logic;

  signal Funct3EXMEM         : std_logic_vector(2 downto 0);

  --Señales MEM

  signal Addr_Jump_destMEM  : std_logic_vector(31 downto 0);
  signal dataIn_MemMEM      : std_logic_vector(31 downto 0); -- From Data Memory
  signal desition_JumpMEM   : std_logic;

  --Señales MEM/WB

  signal Ctrl_RegWriteMEMWB  : std_logic;
  signal Ctrl_ResSrcMEMWB    : std_logic_vector(1 downto 0);  --MemtoReg

  signal Alu_ResMEMWB        : std_logic_vector(31 downto 0);
  signal RD_MEMWB             : std_logic_vector(4 downto 0);

  signal dataIn_MemMEMWB     : std_logic_vector(31 downto 0); -- From Data Memory

  --Señales WB
  
  signal reg_RD_dataWB  : std_logic_vector(31 downto 0);

begin

  -- Multiplexor IF --------------------------------------------------------
  PC_nextIF <= Addr_Jump_destMEM when desition_JumpMEM = '1' else PC_plus4IF;
  --------------------------------------------------------------------------

  PCWrite_DisableIF <= '1' when Ctrl_HazardID = '1' else '0';

  ---------------------------------------------------
  -- PC REGISTER PROCESS
  ---------------------------------------------------
  PC_reg_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      PC_regIF <= (22 => '1', others => '0'); -- 0040_0000
    elsif rising_edge(Clk) and PCWrite_disableIF = '0' then
      PC_regIF <= PC_nextIF;
    end if;
  end process;

  -- ADD IF ---------------------
  PC_plus4IF    <= PC_regIF + 4;
  -------------------------------

  -- INSTRUCTION MEMORY -----
  IAddr         <= PC_regIF;
  InstructionIF <= IDataIn;
  ---------------------------

  ---------------------------------------------------
  -- IFID PROCESS
  ---------------------------------------------------
  Write_DisableIFID <= '1' when Ctrl_HazardID = '1' else '0';

  IF_ID_REG: process(Clk, Reset, Write_DisableIFID)
  begin
    if Reset = '1' then
      PC_regIFID      <= (others => '0');
      InstructionIFID <= (others => '0');
    elsif rising_edge(Clk) and Write_Disable = '0' then
      PC_regIFID      <= PC_regIF;
      InstructionIFID <= InstructionIF;
    end if;
  end process;


  UnidadControl : control_unit
  port map(
    OpCode   => InstructionIFID(6 downto 0),
    -- Señales para el PC
    --Jump   => CONTROL_JUMP,
    Branch   => Ctrl_BranchID,
    -- Señales para la memoria
    ResultSrc=> Ctrl_ResSrcID,
    MemWrite => Ctrl_MemWriteID,
    MemRead  => Ctrl_MemReadID,
    -- Señales para la ALU
    ALUSrc   => Ctrl_ALUSrcID,
    AuipcLui => Ctrl_PcLuiID,
    ALUOP    => Ctrl_ALUOPID,
    -- señal generacion salto
    Ins_jalr => Ctrl_JalrID, -- 0=any instrucion, 1=jalr
    -- Señales para el GPR
    RegWrite => Ctrl_RegWriteID
  );

  RegsRISCV : reg_bank
  port map (
    Clk   => Clk,
    Reset => Reset,
    A1    => RS1_ID, --rs1
    Rd1   => reg_RSID,
    A2    => RS2_ID, --rs2
    Rd2   => reg_RTID,
    A3    => RD_MEMWB, 
    Wd3   => reg_RD_dataWB,
    We3   => Ctrl_RegWriteMEMWB
  );

  inmed_op : Imm_Gen
  port map (
        instr    => InstructionIFID,
        imm      => Inm_extID 
  );
  
  -- DIVISION DEL CAMPO INSTRUCTION  -------------------------------------------------
  Funct3ID      <= InstructionIFID(14 downto 12); -- Campo "funct3" de la instruccion
  Funct7ID      <= InstructionIFID(31 downto 25); -- Campo "funct7" de la instruccion
  RS1_ID        <= InstructionIFID(19 downto 15);
  RS2_ID        InstructionIFID(24 downto 20);
  RD_ID         <= InstructionIFID(11 downto 7);
  ------------------------------------------------------------------------------------

  -- Deteccion de Hazard UNIT
  Ctrl_HazardID <= '1' when Ctrl_MemReadIDEX = '1' and
                    ((InstructionIFID(19 downto 15) = RD_IDEX) or
                    (InstructionIFID(24 downto 20) = RD_IDEX)) else '0';  

  ---------------------------------------------------
  -- IDEX PROCESS
  ---------------------------------------------------
  IDEX_process: process(Clk, Reset)
  begin
    if Reset = '1' or (Ctrl_HazardID <= '1' and rising_edge(Clk)) then
      Ctrl_ALUSrcIDEX     <= '0';
      Ctrl_BranchIDEX     <= '0';
      Ctrl_JalrIDEX       <= '0';
      Ctrl_MemReadIDEX    <= '0';
      Ctrl_MemWriteIDEX   <= '0';
      Ctrl_RegWriteIDEX   <= '0';
      Ctrl_ResSrcIDEX     <= (others => '0');
      Ctrl_ALUOPIDEX      <= (others => '0');
      Ctrl_PcLuiIDEX      <= (others => '0');
      Funct3IDEX          <= (others => '0');
      Funct7IDEX          <= (others => '0');
      Inm_extIDEX         <= (others => '0');
      PC_regIDEX          <= (others => '0');
      RD_IDEX             <= (others => '0');
      reg_RSIDEX          <= (others => '0');
      reg_RTIDEX          <= (others => '0');
      RS1_IDEX            <= (others => '0');
      RS2_IDEX            <= (others => '0');
    elsif rising_edge(Clk) then
      Ctrl_ALUSrcIDEX     <= Ctrl_ALUSrcID;
      Ctrl_BranchIDEX     <= Ctrl_BranchID;
      Ctrl_JalrIDEX       <= Ctrl_JalrID;
      Ctrl_MemReadIDEX    <= Ctrl_MemReadID;
      Ctrl_MemWriteIDEX   <= Ctrl_MemWriteID;
      Ctrl_RegWriteIDEX   <= Ctrl_RegWriteID;
      Ctrl_ResSrcIDEX     <= Ctrl_ResSrcID; --MemtoReg
      Ctrl_ALUOPIDEX      <= Ctrl_ALUOPID;
      Ctrl_PcLuiIDEX      <= Ctrl_PcLuiID;
      Funct3IDEX          <= Funct3ID;
      Funct7IDEX          <= Funct7ID;
      Inm_extIDEX         <= Inm_extID;
      PC_regIDEX          <= PC_regIFID;
      RD_IDEX             <= RD_ID;
      reg_RSIDEX          <= reg_RSID;
      reg_RTIDEX          <= reg_RTID;
      RS1_IDEX            <= RS1_ID;
      RS2_IDEX            <= RS2_ID;

    end if;
  end process;

  -- ADD EX ---------------------------------------------------------
  Addr_BranchEX    <= PC_regIDEX + (Inm_extIDEX(30 downto 0) & '0'); -- Se realiza el Shift left 1
  Addr_jalrEX      <= reg_RSIDEX + Inm_extIDEX;
  -------------------------------------------------------------------

  Alu_RISCV : alu_RV
  port map (
    OpA      => Alu_Op1EX,
    OpB      => Alu_Op2EX,
    Control  => AluControlEX,
    Result   => Alu_ResEX,
    Signflag => Alu_SIGNEX,
    carryOut => open,
    Zflag    => Alu_ZEROEX
  );

  -- Multiplexor EX1 --------------------------------------------
  Alu_Op1EX    <= PC_regIDEX     when Ctrl_PcLuiIDEX = "00" else
                (others => '0')  when Ctrl_PcLuiIDEX = "01" else
                reg_RSIDEX; -- any other
  ---------------------------------------------------------------

  -- Multiplexor EX2 -----------------------------------------------------
  Alu_Op2EX    <= reg_RTIDEX when Ctrl_ALUSrcIDEX = '0' else Inm_extIDEX;
  ------------------------------------------------------------------------

  Alu_control_i: alu_control
  port map(
    -- Entradas:
    ALUOp  => Ctrl_ALUOPIDEX, -- Codigo de control desde la unidad de control
    Funct3  => Funct3IDEX,    -- Campo "funct3" de la instruccion
    Funct7  => Funct7IDEX,    -- Campo "funct7" de la instruccion
    -- Salida de control para la ALU:
    ALUControl => AluControlEX -- Define operacion a ejecutar por la ALU
  );

  Alu_Op1EX <= RD_dataWB when Ctrl_RegWriteMEMWB = '1' and
                RD_MEMWB /= "00000" and not
                ( Ctrl_RegWriteEXMEM = '1' and
                RD_EXMEM /= "00000" and
                RD_EXMEM = RS2_IDEX) and
                RD_MEMWB = RS2_IDEX else
                Alu_ResEXMEM when Ctrl_RegWriteEXMEM = '1' and
                RD_EXMEM /= "00000" and
                RD_EXMEM = RS2_IDEX else
                reg_RSIDEX;


  Alu_Op2_EX <= reg_RD_dataWB when Ctrl_RegWriteMEMWB = '1' and
                  RD_MEMWB /= "00000" and not
                  ( Ctrl_RegWriteEXMEM = '1' and
                  RD_EXMEM /= "00000" and
                  RD_EXMEM = RS1_IDEX) and
                  RD_MEMWB = RS1_IDEX else
                  Alu_ResEXMEM when Ctrl_RegWriteEXMEM = '1' and
                  RD_EXMEM /= "00000" and
                  RD_EXMEM = RS1_IDEX else
                  reg_RTIDEX;


  ---------------------------------------------------
  -- EXMEM PROCESS
  ---------------------------------------------------
  EXMEM_process: process(Clk, Reset)
  begin
    if Reset = '1' then
      Alu_SIGNEXMEM       <= '0';
      Alu_ZEROEXMEM       <= '0';
      Ctrl_BranchEXMEM    <= '0';
      Ctrl_JalrEXMEM      <= '0';
      Ctrl_MemReadEXMEM   <= '0';
      Ctrl_MemWriteEXMEM  <= '0';
      Ctrl_RegWriteEXMEM  <= '0';
      Ctrl_ResSrcEXMEM    <= (others => '0');
      Addr_BranchEXMEM    <= (others => '0');
      Addr_JalrEXMEM      <= (others => '0');
      Alu_ResEXMEM        <= (others => '0');
      Funct3EXMEM         <= (others => '0');
      RD_EXMEM             <= (others => '0');
      reg_RTEXMEM         <= (others => '0');
    elsif rising_edge(Clk) then
      Alu_SIGNEXMEM       <= Alu_SIGNEX;
      Alu_ZEROEXMEM       <= Alu_ZEROEX;
      Ctrl_BranchEXMEM    <= Ctrl_BranchIDEX;
      Ctrl_JalrEXMEM      <= Ctrl_JalrIDEX;
      Ctrl_MemReadEXMEM   <= Ctrl_MemReadIDEX;
      Ctrl_MemWriteEXMEM  <= Ctrl_MemWriteIDEX;
      Ctrl_RegWriteEXMEM  <= Ctrl_RegWriteIDEX;
      Ctrl_ResSrcEXMEM    <= Ctrl_ResSrcIDEX; --MemtoReg
      Addr_BranchEXMEM    <= Addr_BranchEX;
      Addr_JalrEXMEM      <= Addr_JalrEX;
      Alu_ResEXMEM        <= Alu_ResEX;
      Funct3EXMEM         <= Funct3IDEX;
      RD_EXMEM             <= RD_IDEX;
      reg_RTEXMEM         <= reg_RTIDEX;
    end if;
  end process;

  -- Operaciones de Jump y Branch -----------------------------------------------------------
  desition_JumpMEM  <= Ctrl_JalrEXMEM or (Ctrl_BranchEXMEM and branch_true);
  branch_true    <= '1' when ( ((Funct3EXMEM = BR_F3_BEQ) and (Alu_ZEROEXMEM = '1')) or
                               ((Funct3EXMEM = BR_F3_BNE) and (Alu_ZEROEXMEM = '0')) or
                               ((Funct3EXMEM = BR_F3_BLT) and (Alu_SIGNEXMEM = '1')) or
                               ((Funct3EXMEM = BR_F3_BGT) and (Alu_SIGNEXMEM = '0')) ) else
                    '0';
 
  Addr_Jump_destMEM <= Addr_JalrEXMEM   when Ctrl_JalrEXMEM = '1' else
                    Addr_BranchEXMEM when Ctrl_BranchEXMEM='1' else
                    (others =>'0');
  -------------------------------------------------------------------------------------------


  ---------------------------------------------------
  -- MEMWB PROCESS
  ---------------------------------------------------
  MEMWB_process: process(Clk, Reset)
  begin
    if Reset = '1' then    
      Ctrl_RegWriteMEMWB  <= '0';
      Ctrl_ResSrcMEMWB    <= (others => '0');
      Alu_ResMEMWB        <= (others => '0');
      dataIn_MemMEMWB     <= (others => '0');
      RD_MEMWB             <= (others => '0');
    elsif rising_edge(Clk) then
      Ctrl_RegWriteMEMWB  <= Ctrl_RegWriteEXMEM;
      Ctrl_ResSrcMEMWB    <= Ctrl_ResSrcEXMEM; --MemtoReg
      Alu_ResMEMWB        <= Alu_ResEXMEM;
      dataIn_MemMEMWB     <= dataIn_MemMEM;
      RD_MEMWB             <= RD_EXMEM;
    end if;
  end process;

  -- DATA MEMORY ----------------------
  DAddr         <= Alu_ResEXMEM;
  DDataOut      <= reg_RTEXMEM;
  DWrEn         <= Ctrl_MemWriteEXMEM;
  dRdEn         <= Ctrl_MemReadEXMEM;
  dataIn_MemMEM <= DDataIn;
  -------------------------------------

  -- Multiplexor WB --------------------------------------------
  reg_RD_dataWB <= dataIn_MemMEMWB when Ctrl_ResSrcMEMWB = "01" else
                 PC_plus4IF   when Ctrl_ResSrcMEMWB = "10" else 
                 Alu_ResMEMWB; -- When 00
  --------------------------------------------------------------

end architecture;
