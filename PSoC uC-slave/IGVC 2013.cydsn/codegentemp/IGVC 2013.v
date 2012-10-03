// ======================================================================
// IGVC 2013.v generated from TopDesign.cysch
// 10/02/2012 at 19:05
// ======================================================================

/* -- WARNING: The following section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_DIE_LEOPARD 1
`define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3
`define CYDEV_CHIP_REV_LEOPARD_ES3 3
`define CYDEV_CHIP_REV_LEOPARD_ES2 1
`define CYDEV_CHIP_REV_LEOPARD_ES1 0
`define CYDEV_CHIP_DIE_PANTHER 2
`define CYDEV_CHIP_REV_PANTHER_PRODUCTION 1
`define CYDEV_CHIP_REV_PANTHER_ES1 1
`define CYDEV_CHIP_REV_PANTHER_ES0 0
`define CYDEV_CHIP_DIE_EXPECT 2
`define CYDEV_CHIP_REV_EXPECT 1
`define CYDEV_CHIP_DIE_ACTUAL 2
/* -- WARNING: The previous section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_FAMILY_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_UNKNOWN 0
`define CYDEV_CHIP_FAMILY_PSOC3 1
`define CYDEV_CHIP_MEMBER_3A 1
`define CYDEV_CHIP_REVISION_3A_PRODUCTION 3
`define CYDEV_CHIP_REVISION_3A_ES3 3
`define CYDEV_CHIP_REVISION_3A_ES2 1
`define CYDEV_CHIP_REVISION_3A_ES1 0
`define CYDEV_CHIP_FAMILY_PSOC5 2
`define CYDEV_CHIP_MEMBER_5A 2
`define CYDEV_CHIP_REVISION_5A_PRODUCTION 1
`define CYDEV_CHIP_REVISION_5A_ES1 1
`define CYDEV_CHIP_REVISION_5A_ES0 0
`define CYDEV_CHIP_FAMILY_USED 2
`define CYDEV_CHIP_MEMBER_USED 2
`define CYDEV_CHIP_REVISION_USED 1
// Component: ZeroTerminal
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\ZeroTerminal\ZeroTerminal.v"
`endif

// VDAC8_v1_70(Data_Source=0, Initial_Value=100, Strobe_Mode=0, UseWorkaround=false, VDAC_Range=4, VDAC_Speed=2, Voltage=1600, CY_COMPONENT_NAME=VDAC8_v1_70, CY_CONTROL_FILE=<:default:>, CY_FITTER_NAME=VDAC8_1, CY_INSTANCE_SHORT_NAME=VDAC8_1, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=70, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=cydsfit No Version Information Found, INSTANCE_NAME=VDAC8_1, )
module VDAC8_v1_70_0 (strobe, data, vOut);
    input   strobe;
    input  [7:0] data;
    inout   vOut;
    electrical   vOut;

    parameter Data_Source = 0;
    parameter Initial_Value = 100;
    parameter Strobe_Mode = 0;

    electrical  Net_77;
    wire  Net_83;
    wire  Net_82;
    wire  Net_81;

    cy_psoc3_vidac8_v1_0 viDAC8 (
        .reset(Net_83),
        .idir(Net_81),
        .data(data[7:0]),
        .strobe(strobe),
        .vout(vOut),
        .iout(Net_77),
        .ioff(Net_82),
        .strobe_udb(strobe));
    defparam viDAC8.is_all_if_any = 0;
    defparam viDAC8.reg_data = 0;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_81));

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_82));

    ZeroTerminal ZeroTerminal_3 (
        .z(Net_83));

    cy_analog_noconnect_v1_0 cy_analog_noconnect_1 (
        .noconnect(Net_77));



endmodule

// VDAC8_v1_70(Data_Source=0, Initial_Value=100, Strobe_Mode=0, UseWorkaround=false, VDAC_Range=4, VDAC_Speed=2, Voltage=1600, CY_COMPONENT_NAME=VDAC8_v1_70, CY_CONTROL_FILE=<:default:>, CY_FITTER_NAME=VDAC8_2, CY_INSTANCE_SHORT_NAME=VDAC8_2, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=70, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=cydsfit No Version Information Found, INSTANCE_NAME=VDAC8_2, )
module VDAC8_v1_70_1 (strobe, data, vOut);
    input   strobe;
    input  [7:0] data;
    inout   vOut;
    electrical   vOut;

    parameter Data_Source = 0;
    parameter Initial_Value = 100;
    parameter Strobe_Mode = 0;

    electrical  Net_77;
    wire  Net_83;
    wire  Net_82;
    wire  Net_81;

    cy_psoc3_vidac8_v1_0 viDAC8 (
        .reset(Net_83),
        .idir(Net_81),
        .data(data[7:0]),
        .strobe(strobe),
        .vout(vOut),
        .iout(Net_77),
        .ioff(Net_82),
        .strobe_udb(strobe));
    defparam viDAC8.is_all_if_any = 0;
    defparam viDAC8.reg_data = 0;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_81));

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_82));

    ZeroTerminal ZeroTerminal_3 (
        .z(Net_83));

    cy_analog_noconnect_v1_0 cy_analog_noconnect_1 (
        .noconnect(Net_77));



endmodule

// Component: OneTerminal
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\OneTerminal"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\OneTerminal\OneTerminal.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\OneTerminal"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\OneTerminal\OneTerminal.v"
`endif

// Component: cy_virtualmux_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\cy_virtualmux_v1_0\cy_virtualmux_v1_0.v"
`endif

// Component: bI2C_v3_1
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cycomponentlibraryupdates\CyComponentLibraryUpdates.cylib\bI2C_v3_1"
`include "$CYPRESS_DIR\..\psoc\content\cycomponentlibraryupdates\CyComponentLibraryUpdates.cylib\bI2C_v3_1\bI2C_v3_1.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cycomponentlibraryupdates\CyComponentLibraryUpdates.cylib\bI2C_v3_1"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cycomponentlibraryupdates\CyComponentLibraryUpdates.cylib\bI2C_v3_1\bI2C_v3_1.v"
`endif

// Component: or_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0\or_v1_0.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\or_v1_0\or_v1_0.v"
`endif

// I2C_v3_1(Address_Decode=1, BusSpeed_kHz=100, CtlModeReplacementString=AsyncCtl, EnableWakeup=false, FF=false, Hex=false, I2C_Mode=2, I2cBusPort=0, Implementation=0, NotSlaveClockMinusTolerance=25, NotSlaveClockPlusTolerance=5, Slave_Address=8, SlaveClockMinusTolerance=5, SlaveClockPlusTolerance=50, UDB_MSTR=true, UDB_MULTI_MASTER_SLAVE=false, UDB_SLV=false, UdbInternalClock=true, UdbRequiredClock=1600, UdbSlaveFixedPlacementEnable=false, CY_COMPONENT_NAME=I2C_v3_1, CY_CONTROL_FILE=I2C_Slave_DefaultPlacement.ctl, CY_FITTER_NAME=I2C, CY_INSTANCE_SHORT_NAME=I2C, CY_MAJOR_VERSION=3, CY_MINOR_VERSION=1, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=cydsfit No Version Information Found, INSTANCE_NAME=I2C, )
module I2C_v3_1_2 (sda, scl, clock, reset, bclk, iclk);
    inout   sda;
    inout   scl;
    input   clock;
    input   reset;
    output  bclk;
    output  iclk;


    wire  Net_747;
    wire  Net_748;
    wire  Net_864;
    wire  Net_851;
    wire  Net_847;
    wire  Net_875;
    wire  Net_872;
    wire  Net_873;
    wire  Net_767;
    wire  sda_x_wire;
    wire [5:0] Net_643;
    wire  Net_854;
    wire  Net_855;
    wire  Net_697;
    wire  scl_x_wire;

	wire [0:0] tmpOE__Bufoe_SDA_net;

	cy_bufoe
		Bufoe_SDA
		 (.x(sda_x_wire),
		  .y(sda),
		  .oe(tmpOE__Bufoe_SDA_net),
		  .yfb(Net_855));

	assign tmpOE__Bufoe_SDA_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{Net_747} : {Net_747};

	wire [0:0] tmpOE__Bufoe_SCL_net;

	cy_bufoe
		Bufoe_SCL
		 (.x(scl_x_wire),
		  .y(scl),
		  .oe(tmpOE__Bufoe_SCL_net),
		  .yfb(Net_854));

	assign tmpOE__Bufoe_SCL_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{Net_748} : {Net_748};


	cy_isr_v1_0
		#(.int_type(2'b00))
		I2C_IRQ
		 (.int_signal(Net_697));


    OneTerminal OneTerminal_2 (
        .o(Net_747));

    OneTerminal OneTerminal_1 (
        .o(Net_748));

	// VirtualMux_IRQ (cy_virtualmux_v1_0)
	assign Net_697 = Net_643[5];

	// VirtualMux_SDA (cy_virtualmux_v1_0)
	assign sda_x_wire = Net_643[4];

	// VirtualMux_SCL (cy_virtualmux_v1_0)
	assign scl_x_wire = Net_643[3];

    bI2C_v3_1 bI2C_UDB (
        .clock(Net_767),
        .scl_in(Net_854),
        .sda_in(Net_855),
        .sda_out(Net_643[4]),
        .scl_out(Net_643[3]),
        .interrupt(Net_643[5]),
        .reset(reset));
    defparam bI2C_UDB.Mode = 2;


	cy_clock_v1_0
		#(.id("76026f1a-7483-46e3-a655-85c386f5f13f/be0a0e37-ad17-42ca-b5a1-1a654d736358"),
		  .source_clock_id(""),
		  .divisor(0),
		  .period("625000000"),
		  .is_direct(0),
		  .is_digital(1))
		IntClock
		 (.clock_out(Net_872));


	// VirtualMux_CLOCK (cy_virtualmux_v1_0)
	assign Net_767 = Net_872;


    assign bclk = Net_873 | Net_847;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_847));


    assign iclk = Net_872 | Net_875;

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_875));



endmodule

// Component: B_PWM_v2_10
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_PWM_v2_10"
`include "$CYPRESS_DIR\..\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_PWM_v2_10\B_PWM_v2_10.v"
`else
`define CY_BLK_DIR "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_PWM_v2_10"
`include "C:\Program Files (x86)\Cypress\PSoC Creator\2.0\PSoC Creator\psoc\content\cycomponentlibrary\CyComponentLibrary.cylib\B_PWM_v2_10\B_PWM_v2_10.v"
`endif

// PWM_v2_10(CaptureMode=0, CompareStatusEdgeSense=true, CompareType1=1, CompareType1Software=0, CompareType2=1, CompareType2Software=0, CompareValue1=3000, CompareValue2=3000, CtlModeReplacementString=AsyncCtl, CyGetRegReplacementString=CY_GET_REG16, CySetRegReplacementString=CY_SET_REG16, DeadBand=0, DeadBand2_4=0, DeadBandUsed=0, DeadTime=1, DitherOffset=0, EnableMode=0, FixedFunction=false, FixedFunctionUsed=0, InterruptOnCMP1=false, InterruptOnCMP2=false, InterruptOnKill=false, InterruptOnTC=false, IntOnCMP1=0, IntOnCMP2=0, IntOnKill=0, IntOnTC=0, KillMode=0, KillModeMinTime=0, MinimumKillTime=1, OneCompare=false, Period=30000, PWMMode=1, PWMModeCenterAligned=0, RegDefReplacementString=reg16, RegSizeReplacementString=uint16, Resolution=16, RstStatusReplacementString=sSTSReg_nrstSts, RunMode=0, TriggerMode=0, UseControl=true, UseInterrupt=true, UseStatus=true, VerilogSectionReplacementString=sP16, CY_COMPONENT_NAME=PWM_v2_10, CY_CONTROL_FILE=<:default:>, CY_FITTER_NAME=Servo, CY_INSTANCE_SHORT_NAME=Servo, CY_MAJOR_VERSION=2, CY_MINOR_VERSION=10, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=cydsfit No Version Information Found, INSTANCE_NAME=Servo, )
module PWM_v2_10_3 (pwm2, tc, clock, reset, pwm1, interrupt, capture, kill, enable, trigger, cmp_sel, pwm, ph1, ph2);
    output  pwm2;
    output  tc;
    input   clock;
    input   reset;
    output  pwm1;
    output  interrupt;
    input   capture;
    input   kill;
    input   enable;
    input   trigger;
    input   cmp_sel;
    output  pwm;
    output  ph1;
    output  ph2;

    parameter Resolution = 16;

    wire  Net_113;
    wire  Net_114;
    wire  Net_107;
    wire  Net_96;
    wire  Net_55;
    wire  Net_57;
    wire  Net_101;
    wire  Net_54;
    wire  Net_63;

    B_PWM_v2_10 PWMUDB (
        .reset(reset),
        .clock(clock),
        .tc(Net_101),
        .pwm1(pwm1),
        .pwm2(pwm2),
        .interrupt(Net_55),
        .kill(kill),
        .capture(capture),
        .enable(enable),
        .cmp_sel(cmp_sel),
        .trigger(trigger),
        .pwm(Net_96),
        .ph1(ph1),
        .ph2(ph2));
    defparam PWMUDB.CaptureMode = 0;
    defparam PWMUDB.CompareStatusEdgeSense = 1;
    defparam PWMUDB.CompareType1 = 1;
    defparam PWMUDB.CompareType2 = 1;
    defparam PWMUDB.DeadBand = 0;
    defparam PWMUDB.DitherOffset = 0;
    defparam PWMUDB.EnableMode = 0;
    defparam PWMUDB.KillMode = 0;
    defparam PWMUDB.PWMMode = 1;
    defparam PWMUDB.Resolution = 16;
    defparam PWMUDB.RunMode = 0;
    defparam PWMUDB.TriggerMode = 0;
    defparam PWMUDB.UseStatus = 1;

	// vmCompare (cy_virtualmux_v1_0)
	assign pwm = Net_96;

	// vmIRQ (cy_virtualmux_v1_0)
	assign interrupt = Net_55;

	// vmTC (cy_virtualmux_v1_0)
	assign tc = Net_101;

    OneTerminal OneTerminal_1 (
        .o(Net_113));

	// FFKillMux (cy_virtualmux_v1_0)
	assign Net_107 = Net_114;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_114));



endmodule

// VDAC8_v1_70(Data_Source=0, Initial_Value=100, Strobe_Mode=0, UseWorkaround=false, VDAC_Range=4, VDAC_Speed=2, Voltage=1600, CY_COMPONENT_NAME=VDAC8_v1_70, CY_CONTROL_FILE=<:default:>, CY_FITTER_NAME=VDAC8_3, CY_INSTANCE_SHORT_NAME=VDAC8_3, CY_MAJOR_VERSION=1, CY_MINOR_VERSION=70, CY_REMOVE=false, CY_SUPPRESS_API_GEN=false, CY_VERSION=cydsfit No Version Information Found, INSTANCE_NAME=VDAC8_3, )
module VDAC8_v1_70_4 (strobe, data, vOut);
    input   strobe;
    input  [7:0] data;
    inout   vOut;
    electrical   vOut;

    parameter Data_Source = 0;
    parameter Initial_Value = 100;
    parameter Strobe_Mode = 0;

    electrical  Net_77;
    wire  Net_83;
    wire  Net_82;
    wire  Net_81;

    cy_psoc3_vidac8_v1_0 viDAC8 (
        .reset(Net_83),
        .idir(Net_81),
        .data(data[7:0]),
        .strobe(strobe),
        .vout(vOut),
        .iout(Net_77),
        .ioff(Net_82),
        .strobe_udb(strobe));
    defparam viDAC8.is_all_if_any = 0;
    defparam viDAC8.reg_data = 0;

    ZeroTerminal ZeroTerminal_1 (
        .z(Net_81));

    ZeroTerminal ZeroTerminal_2 (
        .z(Net_82));

    ZeroTerminal ZeroTerminal_3 (
        .z(Net_83));

    cy_analog_noconnect_v1_0 cy_analog_noconnect_1 (
        .noconnect(Net_77));



endmodule

// top
module top ;

    wire  Net_259;
    wire  Net_256;
    electrical  Net_157;
    wire [7:0] Net_161;
    wire  Net_160;
    wire  Net_133;
    wire  Net_132;
    wire  Net_131;
    wire  Net_130;
    wire  Net_129;
    wire  Net_128;
    wire  Net_127;
    wire  Net_126;
    wire  Net_125;
    wire  Net_113;
    wire  Net_112;
    wire  Net_124;
    wire  Net_123;
    wire  Net_109;
    wire  Net_67;
    wire  Net_48;
    wire  Net_47;
    wire  Net_46;
    wire  Net_45;
    wire  Net_13;
    wire  Net_12;
    wire [7:0] Net_163;
    wire  Net_162;
    wire [7:0] Net_159;
    wire  Net_158;
    electrical  Net_178;
    electrical  Net_175;

    VDAC8_v1_70_0 VDAC8_1 (
        .strobe(1'b0),
        .data(8'b00000000),
        .vOut(Net_175));
    defparam VDAC8_1.Data_Source = 0;
    defparam VDAC8_1.Initial_Value = 100;
    defparam VDAC8_1.Strobe_Mode = 0;

    VDAC8_v1_70_1 VDAC8_2 (
        .strobe(1'b0),
        .data(8'b00000000),
        .vOut(Net_178));
    defparam VDAC8_2.Data_Source = 0;
    defparam VDAC8_2.Initial_Value = 100;
    defparam VDAC8_2.Strobe_Mode = 0;

	wire [0:0] tmpOE__Err_LED_1_net;
	wire [0:0] tmpFB_0__Err_LED_1_net;
	wire [0:0] tmpIO_0__Err_LED_1_net;
	wire [0:0] tmpINTERRUPT_0__Err_LED_1_net;
	electrical [0:0] tmpSIOVREF__Err_LED_1_net;

	cy_psoc3_pins_v1_10
		#(.id("ed092b9b-d398-4703-be89-cebf998501f6"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b10),
		  .width(1))
		Err_LED_1
		 (.oe(tmpOE__Err_LED_1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Err_LED_1_net[0:0]}),
		  .io({tmpIO_0__Err_LED_1_net[0:0]}),
		  .siovref(tmpSIOVREF__Err_LED_1_net),
		  .interrupt({tmpINTERRUPT_0__Err_LED_1_net[0:0]}));

	assign tmpOE__Err_LED_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__SDA_1_net;
	wire [0:0] tmpFB_0__SDA_1_net;
	wire [0:0] tmpINTERRUPT_0__SDA_1_net;
	electrical [0:0] tmpSIOVREF__SDA_1_net;

	cy_psoc3_pins_v1_10
		#(.id("22863ebe-a37b-476f-b252-6e49a8c00b12"),
		  .drive_mode(3'b100),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("B"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b00),
		  .width(1))
		SDA_1
		 (.oe(tmpOE__SDA_1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__SDA_1_net[0:0]}),
		  .io({Net_12}),
		  .siovref(tmpSIOVREF__SDA_1_net),
		  .interrupt({tmpINTERRUPT_0__SDA_1_net[0:0]}));

	assign tmpOE__SDA_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__SCL_1_net;
	wire [0:0] tmpFB_0__SCL_1_net;
	wire [0:0] tmpINTERRUPT_0__SCL_1_net;
	electrical [0:0] tmpSIOVREF__SCL_1_net;

	cy_psoc3_pins_v1_10
		#(.id("02f2cf2c-2c7a-49df-9246-7a3435c21be3"),
		  .drive_mode(3'b100),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("B"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b00),
		  .width(1))
		SCL_1
		 (.oe(tmpOE__SCL_1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__SCL_1_net[0:0]}),
		  .io({Net_13}),
		  .siovref(tmpSIOVREF__SCL_1_net),
		  .interrupt({tmpINTERRUPT_0__SCL_1_net[0:0]}));

	assign tmpOE__SCL_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    I2C_v3_1_2 I2C (
        .sda(Net_12),
        .scl(Net_13),
        .clock(1'b0),
        .reset(1'b0),
        .bclk(Net_47),
        .iclk(Net_48));


	cy_clock_v1_0
		#(.id("edc673e8-e795-486b-b5d4-5d43f33c54e3"),
		  .source_clock_id("39D5E4C2-EBFC-44ab-AE3D-19F9BBFD674D"),
		  .divisor(16),
		  .period("0"),
		  .is_direct(0),
		  .is_digital(1))
		Clock_1
		 (.clock_out(Net_67));


    PWM_v2_10_3 Servo (
        .reset(1'b0),
        .clock(Net_67),
        .tc(Net_124),
        .pwm1(Net_112),
        .pwm2(Net_113),
        .interrupt(Net_125),
        .capture(1'b0),
        .kill(1'b1),
        .enable(1'b1),
        .trigger(1'b0),
        .cmp_sel(1'b0),
        .pwm(Net_131),
        .ph1(Net_132),
        .ph2(Net_133));
    defparam Servo.Resolution = 16;

	wire [0:0] tmpOE__Y0_net;
	wire [0:0] tmpFB_0__Y0_net;
	wire [0:0] tmpIO_0__Y0_net;
	wire [0:0] tmpINTERRUPT_0__Y0_net;
	electrical [0:0] tmpSIOVREF__Y0_net;

	cy_psoc3_pins_v1_10
		#(.id("cc3bcd7e-5dc0-48ea-9bf6-6aa082be1ada"),
		  .drive_mode(3'b000),
		  .ibuf_enabled(1'b0),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("NONCONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(1),
		  .vtrip(2'b10),
		  .width(1))
		Y0
		 (.oe(tmpOE__Y0_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Y0_net[0:0]}),
		  .analog({Net_178}),
		  .io({tmpIO_0__Y0_net[0:0]}),
		  .siovref(tmpSIOVREF__Y0_net),
		  .interrupt({tmpINTERRUPT_0__Y0_net[0:0]}));

	assign tmpOE__Y0_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Servo0_net;
	wire [0:0] tmpFB_0__Servo0_net;
	wire [0:0] tmpIO_0__Servo0_net;
	wire [0:0] tmpINTERRUPT_0__Servo0_net;
	electrical [0:0] tmpSIOVREF__Servo0_net;

	cy_psoc3_pins_v1_10
		#(.id("4397d3c3-b601-4d66-810c-13d78c0fdb23"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b1),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b10),
		  .width(1))
		Servo0
		 (.oe(tmpOE__Servo0_net),
		  .y({Net_112}),
		  .fb({tmpFB_0__Servo0_net[0:0]}),
		  .io({tmpIO_0__Servo0_net[0:0]}),
		  .siovref(tmpSIOVREF__Servo0_net),
		  .interrupt({tmpINTERRUPT_0__Servo0_net[0:0]}));

	assign tmpOE__Servo0_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Servo1_net;
	wire [0:0] tmpFB_0__Servo1_net;
	wire [0:0] tmpIO_0__Servo1_net;
	wire [0:0] tmpINTERRUPT_0__Servo1_net;
	electrical [0:0] tmpSIOVREF__Servo1_net;

	cy_psoc3_pins_v1_10
		#(.id("cb11663f-2366-48d2-b10c-a64d5480badc"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b1),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b10),
		  .width(1))
		Servo1
		 (.oe(tmpOE__Servo1_net),
		  .y({Net_113}),
		  .fb({tmpFB_0__Servo1_net[0:0]}),
		  .io({tmpIO_0__Servo1_net[0:0]}),
		  .siovref(tmpSIOVREF__Servo1_net),
		  .interrupt({tmpINTERRUPT_0__Servo1_net[0:0]}));

	assign tmpOE__Servo1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

    VDAC8_v1_70_4 VDAC8_3 (
        .strobe(1'b0),
        .data(8'b00000000),
        .vOut(Net_157));
    defparam VDAC8_3.Data_Source = 0;
    defparam VDAC8_3.Initial_Value = 100;
    defparam VDAC8_3.Strobe_Mode = 0;

	wire [0:0] tmpOE__Test_Button_1_net;
	wire [0:0] tmpFB_0__Test_Button_1_net;
	wire [0:0] tmpIO_0__Test_Button_1_net;
	electrical [0:0] tmpSIOVREF__Test_Button_1_net;

	cy_psoc3_pins_v1_10
		#(.id("1425177d-0d0e-4468-8bcc-e638e5509a9b"),
		  .drive_mode(3'b011),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b01),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b00),
		  .width(1))
		Test_Button_1
		 (.oe(tmpOE__Test_Button_1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Test_Button_1_net[0:0]}),
		  .io({tmpIO_0__Test_Button_1_net[0:0]}),
		  .siovref(tmpSIOVREF__Test_Button_1_net),
		  .interrupt({Net_256}));

	assign tmpOE__Test_Button_1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__X1_net;
	wire [0:0] tmpFB_0__X1_net;
	wire [0:0] tmpIO_0__X1_net;
	wire [0:0] tmpINTERRUPT_0__X1_net;
	electrical [0:0] tmpSIOVREF__X1_net;

	cy_psoc3_pins_v1_10
		#(.id("021b69ac-040e-4aad-9b7a-e7db39fb174e"),
		  .drive_mode(3'b000),
		  .ibuf_enabled(1'b0),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("NONCONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(1),
		  .vtrip(2'b10),
		  .width(1))
		X1
		 (.oe(tmpOE__X1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__X1_net[0:0]}),
		  .analog({Net_175}),
		  .io({tmpIO_0__X1_net[0:0]}),
		  .siovref(tmpSIOVREF__X1_net),
		  .interrupt({tmpINTERRUPT_0__X1_net[0:0]}));

	assign tmpOE__X1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Test_Button_2_net;
	wire [0:0] tmpFB_0__Test_Button_2_net;
	wire [0:0] tmpIO_0__Test_Button_2_net;
	electrical [0:0] tmpSIOVREF__Test_Button_2_net;

	cy_psoc3_pins_v1_10
		#(.id("fd40b9dd-a246-4b9b-9e1b-e686dcd67216"),
		  .drive_mode(3'b011),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b01),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("I"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b00),
		  .width(1))
		Test_Button_2
		 (.oe(tmpOE__Test_Button_2_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Test_Button_2_net[0:0]}),
		  .io({tmpIO_0__Test_Button_2_net[0:0]}),
		  .siovref(tmpSIOVREF__Test_Button_2_net),
		  .interrupt({Net_259}));

	assign tmpOE__Test_Button_2_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Y1_net;
	wire [0:0] tmpFB_0__Y1_net;
	wire [0:0] tmpIO_0__Y1_net;
	wire [0:0] tmpINTERRUPT_0__Y1_net;
	electrical [0:0] tmpSIOVREF__Y1_net;

	cy_psoc3_pins_v1_10
		#(.id("63aaa2ba-0ae7-4777-8853-6a058bcf6a6e"),
		  .drive_mode(3'b000),
		  .ibuf_enabled(1'b0),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("NONCONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(1),
		  .vtrip(2'b10),
		  .width(1))
		Y1
		 (.oe(tmpOE__Y1_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Y1_net[0:0]}),
		  .analog({Net_178}),
		  .io({tmpIO_0__Y1_net[0:0]}),
		  .siovref(tmpSIOVREF__Y1_net),
		  .interrupt({tmpINTERRUPT_0__Y1_net[0:0]}));

	assign tmpOE__Y1_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__X0_net;
	wire [0:0] tmpFB_0__X0_net;
	wire [0:0] tmpIO_0__X0_net;
	wire [0:0] tmpINTERRUPT_0__X0_net;
	electrical [0:0] tmpSIOVREF__X0_net;

	cy_psoc3_pins_v1_10
		#(.id("9d3948ab-03f4-4df7-9e96-e22041253e32"),
		  .drive_mode(3'b000),
		  .ibuf_enabled(1'b0),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("NONCONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(1),
		  .vtrip(2'b10),
		  .width(1))
		X0
		 (.oe(tmpOE__X0_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__X0_net[0:0]}),
		  .analog({Net_175}),
		  .io({tmpIO_0__X0_net[0:0]}),
		  .siovref(tmpSIOVREF__X0_net),
		  .interrupt({tmpINTERRUPT_0__X0_net[0:0]}));

	assign tmpOE__X0_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Center_net;
	wire [0:0] tmpFB_0__Center_net;
	wire [0:0] tmpIO_0__Center_net;
	wire [0:0] tmpINTERRUPT_0__Center_net;
	electrical [0:0] tmpSIOVREF__Center_net;

	cy_psoc3_pins_v1_10
		#(.id("e60a1f0b-a2c4-4f1a-acb2-14d90a356a3a"),
		  .drive_mode(3'b000),
		  .ibuf_enabled(1'b0),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("NONCONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("A"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(1),
		  .vtrip(2'b10),
		  .width(1))
		Center
		 (.oe(tmpOE__Center_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Center_net[0:0]}),
		  .analog({Net_157}),
		  .io({tmpIO_0__Center_net[0:0]}),
		  .siovref(tmpSIOVREF__Center_net),
		  .interrupt({tmpINTERRUPT_0__Center_net[0:0]}));

	assign tmpOE__Center_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};

	wire [0:0] tmpOE__Err_LED_2_net;
	wire [0:0] tmpFB_0__Err_LED_2_net;
	wire [0:0] tmpIO_0__Err_LED_2_net;
	wire [0:0] tmpINTERRUPT_0__Err_LED_2_net;
	electrical [0:0] tmpSIOVREF__Err_LED_2_net;

	cy_psoc3_pins_v1_10
		#(.id("fcc0cf96-15dd-4ea0-9e3d-afa5895e2817"),
		  .drive_mode(3'b110),
		  .ibuf_enabled(1'b1),
		  .init_dr_st(1'b0),
		  .input_sync(1'b1),
		  .intr_mode(2'b00),
		  .io_voltage(""),
		  .layout_mode("CONTIGUOUS"),
		  .oe_conn(1'b0),
		  .output_conn(1'b0),
		  .output_sync(1'b0),
		  .pin_aliases(""),
		  .pin_mode("O"),
		  .por_state(4),
		  .use_annotation(1'b0),
		  .sio_group_cnt(0),
		  .sio_hyst(1'b0),
		  .sio_ibuf(""),
		  .sio_info(2'b00),
		  .sio_obuf(""),
		  .sio_refsel(""),
		  .sio_vtrip(""),
		  .slew_rate(1'b0),
		  .spanning(0),
		  .vtrip(2'b10),
		  .width(1))
		Err_LED_2
		 (.oe(tmpOE__Err_LED_2_net),
		  .y({1'b0}),
		  .fb({tmpFB_0__Err_LED_2_net[0:0]}),
		  .io({tmpIO_0__Err_LED_2_net[0:0]}),
		  .siovref(tmpSIOVREF__Err_LED_2_net),
		  .interrupt({tmpINTERRUPT_0__Err_LED_2_net[0:0]}));

	assign tmpOE__Err_LED_2_net = (`CYDEV_CHIP_MEMBER_USED == `CYDEV_CHIP_MEMBER_3A && `CYDEV_CHIP_REVISION_USED < `CYDEV_CHIP_REVISION_3A_ES3) ? ~{1'b1} : {1'b1};


	cy_isr_v1_0
		#(.int_type(2'b10))
		Button_1
		 (.int_signal(Net_256));



	cy_isr_v1_0
		#(.int_type(2'b10))
		Button_2
		 (.int_signal(Net_259));




endmodule

