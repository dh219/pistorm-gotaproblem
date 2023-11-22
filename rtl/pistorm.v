/*
 * November 2023
 *
 * Test firmware
 * Tested on: Pi3
 *
 *
 */

/* Build Definitions */
//`define L374
//`define 
//`define NEWARB

 
module pistorm(
    output reg     	PI_TXN_IN_PROGRESS, 	// GPIO0
    output reg     	PI_IPL_ZERO,        	// GPIO1
    input   [1:0]   PI_A,       			// GPIO[3..2]
//    input           PI_CLK,     			// GPIO4
    output      	PI_RESET,   			// GPIO5
    input           PI_RD,      			// GPIO6
    input           PI_WR,      			// GPIO7
    inout   [15:0]  PI_D,       			// GPIO[23..8]

    output reg      LTCH_A_0,
    output reg      LTCH_A_8,
    output reg    	LTCH_A_16,
    output reg     	LTCH_A_24,
    output          LTCH_A_OE_n,
    output          LTCH_D_RD_U,
    output          LTCH_D_RD_L,
    output      	LTCH_D_RD_OE_n,
    output reg     	LTCH_D_WR_U,
    output reg     	LTCH_D_WR_L,
    output          LTCH_D_WR_OE_n,

    input           M68K_CLK,
    output   [2:0]  M68K_FC,

    output       	M68K_AS_n,
    output       	M68K_UDS_n,
    output       	M68K_LDS_n,
    output       	M68K_RW,

    input           M68K_DTACK_n,
    input           M68K_BERR_n,

    input           M68K_VPA_n,
    output          M68K_E,
    output       	M68K_VMA_n,

    input   [2:0]   M68K_IPL_n,

    inout           M68K_RESET_n,
    inout           M68K_HALT_n,

    input           M68K_BR_n,
    output       	M68K_BG_n,
    input           M68K_BGACK_n,
    //input           M68K_C1,
    //input           M68K_C3,
    //input           CLK_SEL
	 output DEBUG
  );

	//wire c200m 								= PI_CLK;
	//reg [2:0] c8m_sync;
	wire c8m = M68K_CLK;
	//wire c8m 								= c8m_sync[2];
	//wire c1c3_clk 							= !(M68K_C1 ^ M68K_C3);

	localparam REG_DATA 					= 2'd0;
	localparam REG_ADDR_LO 					= 2'd1;
	localparam REG_ADDR_HI 					= 2'd2;
	localparam REG_STATUS 					= 2'd3;
	
	localparam LO							= 1'b0;
	localparam HI							= 1'b1;
	
	localparam S0							= 3'd0;
	localparam S1							= 3'd1;
	localparam S2							= 3'd2;
	localparam S3							= 3'd3;
	localparam S4							= 3'd4;
	localparam S5							= 3'd5;
	localparam S6							= 3'd6;
	localparam S7							= 3'd7;
	
	localparam E0							= 4'd0;
	localparam E2							= 4'd2;
	localparam E5							= 4'd5;
	localparam E6							= 4'd6;
	localparam E8							= 4'd8;
	localparam E9							= 4'd9;
	localparam E10							= 4'd10;

	//initial begin
		//PI_TXN_IN_PROGRESS 					<= 1'b0;
		//PI_IPL_ZERO 						<= 1'b1;
		//PI_RESET 							<= LO;
		//FC_INT 								<= 3'b111;
		//RW_INT 								<= HI;
		//M68K_E 								<= LO;
		//VMA_INT 							<= HI;
		//BG_INT 								<= 1'b1;
		//AS_INT 								<= HI;	 
	//end

	/* ******************************************** */
	/* 
	 */
	//reg [1:0] rd_sync;
	//reg [1:0] wr_sync;  

	//always @(posedge c200m) begin
	//	rd_sync 							<= {rd_sync[0], PI_RD};
	//	wr_sync 							<= {wr_sync[0], PI_WR};
	//end

	//wire rd_rising = !rd_sync[1] && rd_sync[0];
	//wire wr_rising = !wr_sync[1] && wr_sync[0];
	/* ******************************************** */

	reg [15:0] data_out;

	wire trigger;
	assign trigger = (PI_A == REG_STATUS && CMDRD); //rd_rising);
	assign PI_D = trigger ? {ipl, 11'b0, !( M68K_RESET_n || reset_out ), 1'b0} : 16'bz;
	

	reg [15:0] status;
	wire reset_out 							= !status[1]; /* ps_protocol.c -> ps_write_status_reg (STATUS_BIT_INIT) */

	assign M68K_RESET_n 					= reset_out ? LO : 1'bz;
	assign M68K_HALT_n 						= reset_out ? LO : 1'bz;

	reg op_rw 								= HI;
	reg op_uds_n 							= HI;
	reg op_lds_n 							= HI;

	wire [2:0] FC_INT;
	wire AS_INT;
	wire UDS_INT;
	wire LDS_INT;
	wire RW_INT;
	wire VMA_INT;

	reg M68K_VMA_nr = 1'd1;
	
	reg s0=1'd1; //M68K bus states
	reg s1=1'd0;
	reg s2=1'd0;
	reg s3=1'd0;
	reg s4=1'd0;
	reg s5=1'd0;
	reg s6=1'd0;
	reg s7=1'd0;
	
	reg CMDRD;
	always @(c8m, PI_RD) begin
		if ( PI_RD )
			CMDRD <= HI;
			
		else begin
			if ( CMDRD )
				CMDRD <= LO;
		end
	end
	
	//assign LTCH_D_WR_U 						= PI_A == REG_DATA && CMDWR;
	//assign LTCH_D_WR_L 						= PI_A == REG_DATA && CMDWR;
	assign LTCH_D_RD_OE_n					= !(PI_A == REG_DATA && CMDRD);
	
	
	reg a0;
	
	
	
	reg [2:0] ipl;
	reg [2:0] reset_d 						= 3'b000;
	//reg st_reset_out 						= 1'b1; //1=reset, 0=run
	//reg op_req 								= 1'b0; //1=bus operation pending
	//reg op_rw 							= 1'b1; //1=read, 0=write
	//reg op_a0 								= 1'b0; //1=lds, 0=uds, when sz=byte
	//reg op_sz 								= 1'b0; //1=byte, 0=word
  
  
	//always @(posedge c200m) begin
	//	c8m_sync 							<= {c8m_sync[1:0], M68K_CLK};
	//end

	//wire c8m_rising 						= !c8m_sync[1] && c8m_sync[0];
	//wire c8m_falling 						= c8m_sync[1] && !c8m_sync[0];
	
	/* INTERRUPT CONTROL */
	always @(negedge c8m) begin
		
		ipl 								<= ~M68K_IPL_n;
			
		reset_d[2:1] 						<= { reset_d[1:0], M68K_RESET_n };
		PI_IPL_ZERO 						<= ( ipl == 3'd0 && reset_d );
	end	
	
	/* RESET */
	reg [1:0] resetfilter=2'b11;
	wire oor = resetfilter==2'b01; //pulse when out of reset. delay by one clock pulse is required to prevent lock after reset
	always @(negedge c8m) begin
		resetfilter <= {resetfilter[0], M68K_RESET_n};
	end
	//assign PI_RESET = st_reset_out ? 1'b1 : M68K_RESET_n;
	//assign M68K_RESET_n = st_reset_out ? 1'b0 : 1'bz;
	//assign M68K_HALT_n = st_reset_out ? 1'b0 : 1'bz;
  
	/* E CLOCK */
	// A single period of clock E consists of 10 MC68000 clock periods (six clocks low, four clocks high)
	reg [3:0] e_counter = 4'd0;
	
	always @(negedge c8m) begin
		if (e_counter == E9)
			e_counter <= E0;
		else
			e_counter <= e_counter + 4'd1;
	end
	assign M68K_E = (e_counter > E5) ? HI : LO; //six clocks low (0-5), four clocks high (6-9)
	

	/* Bus Arbitration */
	reg [3:0] BG_DELAY 						= 4'b1111;
	reg [3:0] BR_DELAY 						= 4'b1111;
	reg [3:0] BR_DELAYr						= 4'b1111;
	reg [5:0] BGK_DELAY 					= 6'b000000;
	//reg [6:0] BGK_DELAY						= 7'd0;
	reg [4:0] BGK_DELAYr 					= 5'b00000;
	reg [3:0] BGK_DELAYf 					= 4'b0000;
	reg [3:0] AS_DELAY 						= 4'b1111;
	reg BG_INT								= HI;
	assign M68K_BG_n 						= BG_INT;
	
	
	reg [3:0] BRstart						= 4'd15;
	
	always @( posedge c8m ) begin
	
		BGK_DELAYr 							<= { BGK_DELAYr[4:0], M68K_BGACK_n };
		BR_DELAYr 							<= { BR_DELAYr[3:0], M68K_BR_n };
	end
	
	always @( negedge c8m ) begin
	
		BGK_DELAYf 							<= { BGK_DELAYf[3:0], M68K_BGACK_n };
	end
	
	
	
	// Define bus states
    parameter BUS_IDLE 						= 3'b000;
    parameter BUS_HOLDA 					= 3'b001;
    parameter BUS_HOLDB 					= 3'b010;
    parameter BUS_ADDR 						= 3'b011;
    parameter BUS_RW 						= 3'b100;
    parameter BUS_ACK 						= 3'b101;
    parameter BUS_STOP 						= 3'b110;
    parameter BUS_RQ 						= 3'b111;
	
	// Define FC bits
	parameter USER_DATA						= 3'b001;
	parameter USER_PGM						= 3'b010;
	parameter SVR_DATA						= 3'b101;
	parameter SVR_PGM						= 3'b110;
	parameter CPU_SPACE						= 3'b111;

    reg [2:0] nextBusState					= BUS_IDLE;  // Next bus state
	//reg [2:0] busState 						= BUS_IDLE;// Bus state (3 bits for 8 states)
	
	reg gotBR								= LO;
	reg rstBR								= LO;
	reg [1:0] checkBR						= 2'b00;
	
	always @(c8m) begin
	
		BR_DELAY 							<= { BR_DELAY[3:0],  M68K_BR_n };
		BGK_DELAY 							<= { BGK_DELAY[5:0], M68K_BGACK_n };
	end
	
	
	
	always @(c8m) begin
		/*
		checkBR 							<= { checkBR[1:0], M68K_BR_n };
		
		if ( rstBR )
			gotBR							= LO;
		
		else begin
		
			if ( checkBR[1:0] == 2'b10 && !gotBR ) 
				gotBR 						= HI;
		end
		*/
		if ( !M68K_BR_n && !gotBR )
			gotBR							<= HI;
			
		else begin
		
			if ( M68K_BR_n && gotBR )
				gotBR						<= LO;
				
		end
			
	end
	
	
	always @(c8m) begin
	
		nextBusState <= BUS_IDLE;
		
		/*
		
		// need a way to determine bus-cycle start 
		//if ( gotBR ) begin
		
			case (nextBusState)
			
				BUS_IDLE: begin
				
					if ( gotBR )
						nextBusState 		<= BUS_RW; //BUS_HOLDA;
				end
				
				BUS_HOLDA: begin
				
					nextBusState 			<= BUS_HOLDB;
				end
				
				BUS_HOLDB: begin
				
					nextBusState 			<= BUS_ADDR;
				end
				
				BUS_ADDR: begin
				
					nextBusState 			<= BUS_RW;
				end
				
				BUS_RW: begin
				
					if ( M68K_BG_n && !M68K_AS_n && s6 ) begin
						
						BG_INT 				<= LO;
						nextBusState 		<= BUS_ACK;
					end
				end
				
				BUS_ACK: begin
					if ( !gotBR )
						nextBusState 		<= BUS_STOP;
				end
				
				BUS_STOP: begin
				
					
					nextBusState 			<= BUS_RQ;
				end
				
				BUS_RQ: begin
					
					//if ( !M68K_BG_n ) begin
					
						//if ( c8m ) begin
						
							BG_INT 			<= HI;
							//rstBR			<= HI;
							nextBusState 	<= BUS_IDLE;
						//end
					//end
				end	
				
			endcase
		//end
		*/
	end	
	
	
	/*
	always @( c8m ) begin
		
		BG_DELAY 							<= { BG_DELAY[3:0],  M68K_BG_n };
		BR_DELAY 							<= { BR_DELAY[3:0],  M68K_BR_n };
		AS_DELAY							<= { AS_DELAY[3:0],  M68K_AS_n };
		BGK_DELAY 							<= { BGK_DELAY[5:0], M68K_BGACK_n };
		
		//if ( M68K_BG_n && !BR_DELAY[1] && !M68K_AS_n && (s4rst | s5 | s6 | s7) ) begin
		if ( M68K_BG_n && !M68K_BR_n && !M68K_AS_n && s6 ) begin
		
			//if (c8m_falling) begin
			if (c8m) begin
			
				BG_INT 						<= LO;
			end
		end
		
		else begin
			
			if ( !M68K_BG_n && !BGK_DELAY[1] && s0 ) begin			
			
				//if (c8m_rising)
				if (c8m)
					BG_INT 					<= HI;
			end
		end
	end	
	*/
	
	
	reg PI_BERR;
	assign PI_RESET 						= PI_BERR;
	
	/* Transaction flag */
	assign TXNstart 						= (PI_A == REG_ADDR_HI && PI_WR);
	
	always @(posedge TXNstart, posedge s7) begin
	
		if ( TXNstart ) begin
			PI_TXN_IN_PROGRESS 				<= HI;
			PI_BERR							<= HI;
		end
		
		else begin
			if ( s7 ) begin
				PI_TXN_IN_PROGRESS 			<= LO;
				PI_BERR						<= M68K_BERR_n;
			end
		end
	end
	
	
	/* WRITE commands */
	reg [2:0] op_fc 						= 3'b111;


	
	/*
	 * make PI_WR autonomous
	 * posedge PI_WR = write command sent
	 * one half-cycle later, complete command
	 */
	reg CMDWR;
	
	always @(c8m, PI_WR) begin
	
		if ( PI_WR )
			CMDWR <= HI;
			
		else if ( CMDWR ) //&& !PI_WR )
			CMDWR <= LO;
	end
	
	
	reg addrlo_done;
	reg addrhi_done;
	reg data_done;
	
	always @(CMDWR) begin
	
		if ( CMDWR ) begin
		
			case (PI_A)
				
				REG_DATA: begin
					LTCH_D_WR_U 			<= HI;
					LTCH_D_WR_L 			<= HI;
					data_done				<= HI;
				end
				
				REG_ADDR_LO: begin
					a0 						<= PI_D[0];
					LTCH_A_0				<= HI;
					LTCH_A_8				<= HI;
					addrlo_done				<= HI;
				end
				
				REG_ADDR_HI: begin
					op_rw 					<= PI_D[9];
					op_uds_n 				<= PI_D[8] ? a0 : LO;
					op_lds_n 				<= PI_D[8] ? !a0 : LO;
					op_fc 					<= PI_D[15:13];
					LTCH_A_16				<= HI;
					LTCH_A_24				<= HI;
					addrhi_done				<= HI;
				end
				
				REG_STATUS: begin
					status 					<= PI_D;
				end
			endcase
		end
		
		else begin
		
			if ( !CMDWR ) begin
			
				if ( data_done ) begin
					LTCH_D_WR_U 				<= LO;
					LTCH_D_WR_L 				<= LO;
					data_done					<= LO;
				end
				
				if ( addrlo_done ) begin
					LTCH_A_0					<= LO;
					LTCH_A_8					<= LO;
					addrlo_done					<= LO;
				end
				
				if ( addrhi_done ) begin
					LTCH_A_16					<= LO;
					LTCH_A_24					<= LO;
					addrhi_done					<= LO;
				end
				
			end
		end
	end 
		
	// Sync with 68K bus operations
	
	
	/* BUS TRANSFER STATE MACHINE */
	wire s1rst = s2 | oor;
	wire s2rst = s3 | oor;
	wire s3rst = s4 | oor;
	wire s4rst = s5 | oor;
	wire s5rst = s6 | oor;
	wire s6rst = s7 | oor;
	wire s7rst = s0 | oor;
	
	always @(negedge c8m, posedge s1rst) begin
		if(s1rst)
		  s1<=1'd0;
		//else if(s0 && PI_TXN_IN_PROGRESS && BGK_DELAY[3])
		else if(s0 && BGK_DELAY[3])
		  s1<=1'd1;
	end
	always @(posedge c8m, posedge s2rst) begin
		if(s2rst)
		  s2<=1'd0;
		else if(s1)
		  s2<=1'd1;
	end
	always @(negedge c8m, posedge s3rst) begin
		if(s3rst)
		  s3<=1'd0;
		else if(s2 && PI_TXN_IN_PROGRESS)
		  s3<=1'd1;
	end
	always @(posedge c8m, posedge s4rst) begin
		if(s4rst)
		  s4<=1'd0;
		else if(s3)
		  s4<=1'd1;
	end
	always @(negedge c8m, posedge s5rst) begin
		if(s5rst)
		  s5<=1'd0;
		else if(s4 && (!M68K_DTACK_n || !M68K_BERR_n || (!M68K_VMA_nr && e_counter == E8)))
		  s5<=1'd1;
	end
	always @(posedge c8m, posedge s6rst) begin
		if(s6rst)
		  s6<=1'd0;
		else if(s5)
		  s6<=1'd1;
	end
	always @(negedge c8m, posedge s7rst) begin
		if(s7rst)
		  s7<=1'd0;
		else if(s6)
		  s7<=1'd1;
	end
	always @(posedge c8m, posedge s1) begin
		if(s1)
		  s0<=1'd0;
		else if(s7 | oor)
		  s0<=1'd1;
	end
  
  
	// Entering S1, the processor drives a valid address on the address bus.
	// As the clock rises at the end of S7, the processor places the address and data buses in the high-impedance state
	assign FC_INT = (s0 | s1 | s2 | s3 | s4 | s5 | s6 | s7) ? op_fc : 3'b111;
	assign LTCH_D_WR_OE_n = PI_TXN_IN_PROGRESS ? op_rw : HI;
	
	assign LTCH_A_OE_n = PI_TXN_IN_PROGRESS ? LO : HI;
	
//	assign LTCH_D_RD_U = (s6rst | s7) ? HI : LO;
//	assign LTCH_D_RD_L = (s6rst | s7) ? HI : LO;

// On the falling edge of the clock entering state 7 (S7), the processor latches
// data from the addressed device and negates AS, UDS, and LDS.
	assign LTCH_D_RD_U = s7|s6rst ? HI : LO;
	assign LTCH_D_RD_L = s7|s6rst ? HI : LO;
	
// On the rising edge of S2, the processor asserts AS and drives R/W low.
// On the falling edge of the clock entering S7, the processor negates AS, UDS, or LDS
	assign AS_INT = PI_TXN_IN_PROGRESS ? LO : HI;
	
// READ : On the rising edge of state 2 (S2), the processor asserts AS and UDS, LDS, or DS
// WRITE : At the rising edge of S4, the processor asserts UDS, or LDS // wrong: DS should be set in s3
// On the falling edge of the clock entering S7, the processor negates AS, UDS, or LDS
	assign UDS_INT = PI_TXN_IN_PROGRESS ? op_uds_n : HI;
	assign LDS_INT = PI_TXN_IN_PROGRESS ? op_lds_n : HI;
	
// On the rising edge of S2, the processor asserts AS and drives R/W low.
// As the clock rises at the end of S7, the processor drives R/W high
	assign RW_INT = (PI_TXN_IN_PROGRESS && !op_rw) ? LO : HI;
   
//	output reg      M68K_VMA_n,
	wire vmarst= s7 | oor;
	always @(posedge c8m,posedge vmarst) begin
		if(vmarst)
			M68K_VMA_nr <= HI;
		else if(s4 && !M68K_VPA_n && e_counter == E2)
			M68K_VMA_nr <= LO;
	end
	assign VM_INT = M68K_VMA_nr ? HI : LO;
  
	

	assign M68K_FC 							= M68K_BGACK_n ? FC_INT		: 3'bzzz;
	assign M68K_AS_n 						= M68K_BGACK_n ? AS_INT 	: 1'bz;
	assign M68K_UDS_n 						= M68K_BGACK_n ? UDS_INT 	: 1'bz;
	assign M68K_LDS_n 						= M68K_BGACK_n ? LDS_INT 	: 1'bz;
	assign M68K_RW 							= M68K_BGACK_n ? RW_INT 	: 1'bz;
	assign M68K_VMA_n						= M68K_BGACK_n ? VMA_INT 	: 1'bz;
	
	assign DEBUG = PI_WR;
	
endmodule