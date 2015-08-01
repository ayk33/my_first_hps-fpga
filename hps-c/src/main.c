#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "../inc/hps_0.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define red 0xe0
#define green 0x1c
#define blue 0x03
#define yellow 0xfc
#define cyan 0x1f
#define magenta 0xe3
#define white 0xff
#define black 0x00


//void *h2p_lw_led_addr;
//void *h2p_lw_sw_addr;
void *h2p_lw_vga_addr;
void *h2p_lw_vga_we_addr;
void *h2p_lw_vga_data_addr;
void *h2p_lw_mem_rdy_addr;
int count; 
/*********************************************************************
 * Draw a point on the VGA monitor 
 ********************************************************************/
void VGA_draw_point(int x1, int y1, short color)
{
	  *(uint32_t *)h2p_lw_vga_addr = (x1 << 9) + y1;	
    *(uint32_t *)h2p_lw_vga_data_addr = color; 
    *(uint32_t *)h2p_lw_vga_we_addr = 1; 
    
    *(uint32_t *)h2p_lw_vga_we_addr = 0; 

}


void VGA_clean_up()
{
	*(uint32_t *)h2p_lw_vga_we_addr = 0; 
  *(uint32_t *)h2p_lw_vga_data_addr = 0; 
}

/*
void VGA_draw_line(int x1, int y1, int x2, int y2)
{
  int i,j;
  
  m = (x2-x1)/(y2-y1); 
 
  
}
*/


int main() {
  //YOU MUST HIT RESET OF FPGA BEFORE YOU CAN ACTUALLY DRAW ANYTHING
  count = 0;
	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction;
	int led_mask;
	int sw_value;


	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
  //Pointers to the PIO ports
	//h2p_lw_led_addr    = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_LED_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	//h2p_lw_sw_addr     = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_SW_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_addr      = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_ADDR_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_we_addr   = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_WE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_data_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_DATA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_mem_rdy_addr  = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_MEM_RDY_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	

	// toggle the LEDs a bit

	loop_count = 0;
	led_mask = 0x01;
	sw_value = 0; 
	led_direction = 0; // 0: left to right direction
  
  
  //VGA_draw_point(320, 240, red);
  //VGA_draw_point(400, 100, green);
 
  
  int i,j; 
  for (i = 300; i < 400; i++)
   for (j = 200; j < 300; j++)
      VGA_draw_point(i,j,green);
    
    
  //VGA_point(32, 32);
  //VGA_point(33, 32);
  //VGA_point(40, 32);
  //VGA_draw_line(20,20,50,50);

 // printf("completed we = 0\n");
  

//	while(1){//loop_count < 60 ) {
		
		// control led
		//*(uint32_t *)h2p_lw_led_addr = *(uint32_t *)h2p_lw_sw_addr; 
  //  VGA_point(320, 240);
		// wait 100ms
		//usleep( 100000000);
   // VGA_point(400, 240);

	//} // while
	


	// clean up our memory mapping and exit
	VGA_clean_up();
  
  //Ensure that write enable for vga is disabled (this will hardware reset to work)

	
  if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
