// ============================================================================
// Copyright (c) 2013 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//  
//  
//                     web: http://www.terasic.com/  
//                     email: support@terasic.com
//
// ============================================================================
//Date:  Mon Jun 17 20:35:29 2013
// ============================================================================

`define ENABLE_HPS

module ghrd_top(

      
      ///////// ADC /////////
      inout              ADC_CS_N,
      output             ADC_DIN,
      input              ADC_DOUT,
      output             ADC_SCLK,

      ///////// AUD /////////
      input              AUD_ADCDAT,
      inout              AUD_ADCLRCK,
      inout              AUD_BCLK,
      output             AUD_DACDAT,
      inout              AUD_DACLRCK,
      output             AUD_XCK,

      ///////// CLOCK2 /////////
      input              CLOCK2_50,

      ///////// CLOCK3 /////////
      input              CLOCK3_50,

      ///////// CLOCK4 /////////
      input              CLOCK4_50,

      ///////// CLOCK /////////
      input              CLOCK_50,

      ///////// DRAM /////////
      output      [12:0] DRAM_ADDR,
      output      [1:0]  DRAM_BA,
      output             DRAM_CAS_N,
      output             DRAM_CKE,
      output             DRAM_CLK,
      output             DRAM_CS_N,
      inout       [15:0] DRAM_DQ,
      output             DRAM_LDQM,
      output             DRAM_RAS_N,
      output             DRAM_UDQM,
      output             DRAM_WE_N,

      ///////// FAN /////////
      output             FAN_CTRL,

      ///////// FPGA /////////
      output             FPGA_I2C_SCLK,
      inout              FPGA_I2C_SDAT,

      ///////// GPIO /////////
      inout     [35:0]         GPIO_0,
      inout     [35:0]         GPIO_1,
 

      ///////// HEX0 /////////
      output      [6:0]  HEX0,

      ///////// HEX1 /////////
      output      [6:0]  HEX1,

      ///////// HEX2 /////////
      output      [6:0]  HEX2,

      ///////// HEX3 /////////
      output      [6:0]  HEX3,

      ///////// HEX4 /////////
      output      [6:0]  HEX4,

      ///////// HEX5 /////////
      output      [6:0]  HEX5,

`ifdef ENABLE_HPS
      ///////// HPS /////////
      inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout       [3:0]  HPS_FLASH_DATA,
      output             HPS_FLASH_DCLK,
      output             HPS_FLASH_NCSO,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_I2C2_SCLK,
      inout              HPS_I2C2_SDAT,
      inout              HPS_I2C_CONTROL,
      inout              HPS_KEY,
      inout              HPS_LED,
      inout              HPS_LTC_GPIO,
      output             HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,
`endif /*ENABLE_HPS*/

      ///////// IRDA /////////
      input              IRDA_RXD,
      output             IRDA_TXD,

      ///////// KEY /////////
      input       [3:0]  KEY,

      ///////// LEDR /////////
      output      [9:0]  LEDR,

      ///////// PS2 /////////
      inout              PS2_CLK,
      inout              PS2_CLK2,
      inout              PS2_DAT,
      inout              PS2_DAT2,

      ///////// SW /////////
      input       [9:0]  SW,

      ///////// TD /////////
      input             TD_CLK27,
      input      [7:0]  TD_DATA,
      input             TD_HS,
      output            TD_RESET_N,
      input             TD_VS,


      ///////// VGA /////////
      output      [7:0] VGA_B,
      output            VGA_BLANK_N,
      output            VGA_CLK,
      output      [7:0] VGA_G,
      output            VGA_HS,
      output      [7:0] VGA_R,
      output            VGA_SYNC_N,
      output            VGA_VS
);



// internal wires and registers declaration
   wire        hps_fpga_reset_n;

// vga registers wires
	reg [18:0] vga_addr;
	reg [7:0]  vga_data; 
	reg		  vga_we;
	reg 		  vga_ctrl_clk;

// vga wires coming in/out of soc system
	//outputs
	wire [18:0] vga_hps_addr;
	wire 		   vga_hps_we;
	wire [7:0]	vga_hps_data;
	
	//inputs
	reg 		  vga_mem_rdy; 
	
// Memory bits
	reg  [7:0]	disp_bits;
	wire [7:0]	mem_bits;
	
// intermediate color signals
	wire [9:0]	mVGA_R;
	wire [9:0]	mVGA_G;
	wire [9:0]	mVGA_B;
	
// currently display coordinates
	wire [9:0]  cord_X; 
	wire [8:0]  cord_Y;	//display coods
	
// delayed Reset
	wire d_reset;
   wire reset; 

//state variable and parameters	
	reg [1:0]  state; 


  assign LEDR[1:0] = state; 
//VGA Module instantiation
    Reset_Delay			reset_delay_0(	
			 .iCLK									   (CLOCK_50),
			 .oRESET										(d_reset)	
			 );

    VGA_PLL          vga_pll_0 (
						   .refclk  			      (CLOCK_50),   //  refclk.clk
						   .rst      					(~d_reset),      //   reset.reset
						   .outclk_0 					(vga_ctrl_clk), // outclk0.clk
						   .outclk_1 					(VGA_CLK)  // outclk1.clk
    );
	 
	 VGA_Buffer vga_buffer_0(
		.address_a 										(vga_addr) , 
		.address_b 										({cord_X[9:0],cord_Y[8:0]}), // vga current address
		.clock 										   (vga_ctrl_clk),
		.data_a 											(vga_data), 						
		.data_b 											(1'b0), // never write on port b
		.wren_a 											(vga_we),
		.wren_b 											(1'b0), // never write on port b
		.q_a 												(),
		.q_b 												(mem_bits) // data used to update VGA
	); // data used to update VGA
	 


	/*assign LEDR = ({SW[1],SW[0]}==2'b00) ? vga_addr[9:0] : ({SW[1],SW[0]}==2'b01) 
													 ? {1'b0,vga_addr[18:10]} : ({SW[1],SW[0]}==2'b10) 
													 ? vga_we : ({SW[1],SW[0]}==2'b11) 
													 ? vga_data : 10'b1; 
	*/
	
	VGA_Controller	vga_controller_0	(	
							//	Control Signal
							.iCLK(vga_ctrl_clk),
							.iRST_N(d_reset),

							//	Host Side
							.iCursor_RGB_EN			(4'b0111),
							.oAddress					(),
							.oCoord_X					(cord_X),
							.oCoord_Y					(cord_Y),
							.iRed							(mVGA_R),
							.iGreen						(mVGA_G),
							.iBlue						(mVGA_B),
							
							 //	VGA Side
							.oVGA_R						(VGA_R),
							.oVGA_G						(VGA_G),
							.oVGA_B						(VGA_B),
							.oVGA_H_SYNC				(VGA_HS),
							.oVGA_V_SYNC				(VGA_VS),
							.oVGA_SYNC					(VGA_SYNC_N),
							.oVGA_BLANK					(VGA_BLANK_N)
	);
			
//Reset for the state machine
assign reset = ~KEY[0];
			
// make the color white
assign  mVGA_R = {disp_bits[7:5],5'b0};
assign  mVGA_G = {disp_bits[4:2],5'b0};
assign  mVGA_B = {disp_bits[1:0],6'b0};

always @ (negedge vga_ctrl_clk)
begin
	// register the m10k output for better timing on VGA
	// negedge seems to work better than posedge
	disp_bits <= mem_bits;
end

always @ (posedge vga_ctrl_clk)
begin
	
	if (reset)		
	begin
		//clear the screen
		vga_addr 	<= {cord_X[9:0],cord_Y[8:0]} ;	
		vga_we 		<= 1'b1;								
		vga_data 	<= 8'b0;	
	end
	else 
	begin
			vga_addr 	<= vga_hps_addr; 
			vga_we   	<= vga_hps_we; 
			vga_data 	<= vga_hps_data; 	
	end

		
end // always @ (posedge VGA_CTRL_CLK)
 


    soc_system soc_system_0 (
		  .clk_clk                               (CLOCK_50),                               //                            clk.clk
        .reset_reset_n                         (hps_fpga_reset_n),                         //                          reset.reset_n

	 
        //.pio_led_external_connection_export    (LEDR),     								// pio_led_external_connection.export
	 	  //.pio_sw_external_connection_export     (SW),      								// pio_sw_external_connection.export
		  
		  .pio_vga_addr_external_connection_export  (vga_hps_addr),  //  pio_vga_addr_external_connection.export
        .pio_vga_we_external_connection_export    (vga_hps_we),    //    pio_vga_we_external_connection.export
		  .pio_vga_data_external_connection_export  (vga_hps_data),    //    pio_vga_we_external_connection.export
		  .pio_mem_rdy_external_connection_export   (vga_mem_rdy),    //    pio_vga_we_external_connection.export

        .memory_mem_a                          ( HPS_DDR3_ADDR),                 // memory.mem_a
        .memory_mem_ba                         ( HPS_DDR3_BA),                   // .mem_ba
        .memory_mem_ck                         ( HPS_DDR3_CK_P),                 // .mem_ck
        .memory_mem_ck_n                       ( HPS_DDR3_CK_N),                 // .mem_ck_n
        .memory_mem_cke                        ( HPS_DDR3_CKE),                  // .mem_cke
        .memory_mem_cs_n                       ( HPS_DDR3_CS_N),                 // .mem_cs_n
        .memory_mem_ras_n                      ( HPS_DDR3_RAS_N),                // .mem_ras_n
        .memory_mem_cas_n                      ( HPS_DDR3_CAS_N),                // .mem_cas_n
        .memory_mem_we_n                       ( HPS_DDR3_WE_N),                 // .mem_we_n
        .memory_mem_reset_n                    ( HPS_DDR3_RESET_N),              // .mem_reset_n
        .memory_mem_dq                         ( HPS_DDR3_DQ),                   // .mem_dq
        .memory_mem_dqs                        ( HPS_DDR3_DQS_P),                // .mem_dqs
        .memory_mem_dqs_n                      ( HPS_DDR3_DQS_N),                // .mem_dqs_n
        .memory_mem_odt                        ( HPS_DDR3_ODT),                  // .mem_odt
        .memory_mem_dm                         ( HPS_DDR3_DM),                   // .mem_dm
        .memory_oct_rzqin                      ( HPS_DDR3_RZQ),                  // .oct_rzqin
       		
	     .hps_0_hps_io_hps_io_emac1_inst_TX_CLK ( HPS_ENET_GTX_CLK),              // hps_0_hps_io.hps_io_emac1_inst_TX_CLK
        .hps_0_hps_io_hps_io_emac1_inst_TXD0   ( HPS_ENET_TX_DATA[0] ),          // .hps_io_emac1_inst_TXD0
        .hps_0_hps_io_hps_io_emac1_inst_TXD1   ( HPS_ENET_TX_DATA[1] ),          // .hps_io_emac1_inst_TXD1
        .hps_0_hps_io_hps_io_emac1_inst_TXD2   ( HPS_ENET_TX_DATA[2] ),          // .hps_io_emac1_inst_TXD2
        .hps_0_hps_io_hps_io_emac1_inst_TXD3   ( HPS_ENET_TX_DATA[3] ),          // .hps_io_emac1_inst_TXD3
        .hps_0_hps_io_hps_io_emac1_inst_RXD0   ( HPS_ENET_RX_DATA[0] ),          // .hps_io_emac1_inst_RXD0
        .hps_0_hps_io_hps_io_emac1_inst_MDIO   ( HPS_ENET_MDIO ),                // .hps_io_emac1_inst_MDIO
        .hps_0_hps_io_hps_io_emac1_inst_MDC    ( HPS_ENET_MDC  ),                // .hps_io_emac1_inst_MDC
        .hps_0_hps_io_hps_io_emac1_inst_RX_CTL ( HPS_ENET_RX_DV),                // .hps_io_emac1_inst_RX_CTL
        .hps_0_hps_io_hps_io_emac1_inst_TX_CTL ( HPS_ENET_TX_EN),                // .hps_io_emac1_inst_TX_CTL
        .hps_0_hps_io_hps_io_emac1_inst_RX_CLK ( HPS_ENET_RX_CLK),               // .hps_io_emac1_inst_RX_CLK
        .hps_0_hps_io_hps_io_emac1_inst_RXD1   ( HPS_ENET_RX_DATA[1] ),          // .hps_io_emac1_inst_RXD1
        .hps_0_hps_io_hps_io_emac1_inst_RXD2   ( HPS_ENET_RX_DATA[2] ),          // .hps_io_emac1_inst_RXD2
        .hps_0_hps_io_hps_io_emac1_inst_RXD3   ( HPS_ENET_RX_DATA[3] ),          //                               .hps_io_emac1_inst_RXD3
        
		  
		  .hps_0_hps_io_hps_io_qspi_inst_IO0     ( HPS_FLASH_DATA[0]    ),     //                               .hps_io_qspi_inst_IO0
        .hps_0_hps_io_hps_io_qspi_inst_IO1     ( HPS_FLASH_DATA[1]    ),     //                               .hps_io_qspi_inst_IO1
        .hps_0_hps_io_hps_io_qspi_inst_IO2     ( HPS_FLASH_DATA[2]    ),     //                               .hps_io_qspi_inst_IO2
        .hps_0_hps_io_hps_io_qspi_inst_IO3     ( HPS_FLASH_DATA[3]    ),     //                               .hps_io_qspi_inst_IO3
        .hps_0_hps_io_hps_io_qspi_inst_SS0     ( HPS_FLASH_NCSO    ),     //                               .hps_io_qspi_inst_SS0
        .hps_0_hps_io_hps_io_qspi_inst_CLK     ( HPS_FLASH_DCLK    ),     //                               .hps_io_qspi_inst_CLK
        
		  .hps_0_hps_io_hps_io_sdio_inst_CMD     ( HPS_SD_CMD    ),     //                               .hps_io_sdio_inst_CMD
        .hps_0_hps_io_hps_io_sdio_inst_D0      ( HPS_SD_DATA[0]     ),      //                               .hps_io_sdio_inst_D0
        .hps_0_hps_io_hps_io_sdio_inst_D1      ( HPS_SD_DATA[1]     ),      //                               .hps_io_sdio_inst_D1
        .hps_0_hps_io_hps_io_sdio_inst_CLK     ( HPS_SD_CLK   ),     //                               .hps_io_sdio_inst_CLK
        .hps_0_hps_io_hps_io_sdio_inst_D2      ( HPS_SD_DATA[2]     ),      //                               .hps_io_sdio_inst_D2
        .hps_0_hps_io_hps_io_sdio_inst_D3      ( HPS_SD_DATA[3]     ),      //                               .hps_io_sdio_inst_D3
        		  
		  .hps_0_hps_io_hps_io_usb1_inst_D0      ( HPS_USB_DATA[0]    ),      //                               .hps_io_usb1_inst_D0
        .hps_0_hps_io_hps_io_usb1_inst_D1      ( HPS_USB_DATA[1]    ),      //                               .hps_io_usb1_inst_D1
        .hps_0_hps_io_hps_io_usb1_inst_D2      ( HPS_USB_DATA[2]    ),      //                               .hps_io_usb1_inst_D2
        .hps_0_hps_io_hps_io_usb1_inst_D3      ( HPS_USB_DATA[3]    ),      //                               .hps_io_usb1_inst_D3
        .hps_0_hps_io_hps_io_usb1_inst_D4      ( HPS_USB_DATA[4]    ),      //                               .hps_io_usb1_inst_D4
        .hps_0_hps_io_hps_io_usb1_inst_D5      ( HPS_USB_DATA[5]    ),      //                               .hps_io_usb1_inst_D5
        .hps_0_hps_io_hps_io_usb1_inst_D6      ( HPS_USB_DATA[6]    ),      //                               .hps_io_usb1_inst_D6
        .hps_0_hps_io_hps_io_usb1_inst_D7      ( HPS_USB_DATA[7]    ),      //                               .hps_io_usb1_inst_D7
        .hps_0_hps_io_hps_io_usb1_inst_CLK     ( HPS_USB_CLKOUT    ),     //                               .hps_io_usb1_inst_CLK
        .hps_0_hps_io_hps_io_usb1_inst_STP     ( HPS_USB_STP    ),     //                               .hps_io_usb1_inst_STP
        .hps_0_hps_io_hps_io_usb1_inst_DIR     ( HPS_USB_DIR    ),     //                               .hps_io_usb1_inst_DIR
        .hps_0_hps_io_hps_io_usb1_inst_NXT     ( HPS_USB_NXT    ),     //                               .hps_io_usb1_inst_NXT
        		  
		  .hps_0_hps_io_hps_io_spim1_inst_CLK    ( HPS_SPIM_CLK  ),    //                               .hps_io_spim1_inst_CLK
        .hps_0_hps_io_hps_io_spim1_inst_MOSI   ( HPS_SPIM_MOSI ),   //                               .hps_io_spim1_inst_MOSI
        .hps_0_hps_io_hps_io_spim1_inst_MISO   ( HPS_SPIM_MISO ),   //                               .hps_io_spim1_inst_MISO
        .hps_0_hps_io_hps_io_spim1_inst_SS0    ( HPS_SPIM_SS ),    //                               .hps_io_spim1_inst_SS0
      		
		  .hps_0_hps_io_hps_io_uart0_inst_RX     ( HPS_UART_RX    ),     //                               .hps_io_uart0_inst_RX
        .hps_0_hps_io_hps_io_uart0_inst_TX     ( HPS_UART_TX    ),     //                               .hps_io_uart0_inst_TX
		
		  .hps_0_hps_io_hps_io_i2c0_inst_SDA     ( HPS_I2C1_SDAT    ),     //                               .hps_io_i2c0_inst_SDA
        .hps_0_hps_io_hps_io_i2c0_inst_SCL     ( HPS_I2C1_SCLK    ),     //                               .hps_io_i2c0_inst_SCL
		
		  .hps_0_hps_io_hps_io_i2c1_inst_SDA     ( HPS_I2C2_SDAT    ),     //                               .hps_io_i2c1_inst_SDA
        .hps_0_hps_io_hps_io_i2c1_inst_SCL     ( HPS_I2C2_SCLK    ),     //                               .hps_io_i2c1_inst_SCL
        
		  .hps_0_hps_io_hps_io_gpio_inst_GPIO09  ( HPS_CONV_USB_N),  //                               .hps_io_gpio_inst_GPIO09
        .hps_0_hps_io_hps_io_gpio_inst_GPIO35  ( HPS_ENET_INT_N),  //                               .hps_io_gpio_inst_GPIO35
        .hps_0_hps_io_hps_io_gpio_inst_GPIO40  ( HPS_LTC_GPIO),  //                               .hps_io_gpio_inst_GPIO40
        
		  //.hps_0_hps_io_hps_io_gpio_inst_GPIO41  ( HPS_GPIO[1]),  //                               .hps_io_gpio_inst_GPIO41
        .hps_0_hps_io_hps_io_gpio_inst_GPIO48  ( HPS_I2C_CONTROL),  //                               .hps_io_gpio_inst_GPIO48
        .hps_0_hps_io_hps_io_gpio_inst_GPIO53  ( HPS_LED),  //                               .hps_io_gpio_inst_GPIO53
        .hps_0_hps_io_hps_io_gpio_inst_GPIO54  ( HPS_KEY),  //                               .hps_io_gpio_inst_GPIO54
        .hps_0_hps_io_hps_io_gpio_inst_GPIO61  ( HPS_GSENSOR_INT),  //                               .hps_io_gpio_inst_GPIO61
        .hps_0_h2f_reset_reset_n               (hps_fpga_reset_n),               //                hps_0_h2f_reset.reset_n

    );


endmodule

  