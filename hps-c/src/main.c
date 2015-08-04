#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
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


void *h2p_lw_led_addr;
void *h2p_lw_sw_addr;
void *h2p_lw_vga_addr;
void *h2p_lw_vga_we_addr;
void *h2p_lw_vga_data_addr;
/*********************************************************************
 * Draw a point on the VGA monitor 
 ********************************************************************/
void VGA_draw_point(int x1, int y1, short color)
{
	  *(uint32_t *)h2p_lw_vga_addr = (x1 << 9) + y1;	
    *(uint32_t *)h2p_lw_vga_data_addr = color; 
    //Enable for memory to write 
    *(uint32_t *)h2p_lw_vga_we_addr = 1; 
    
    //disable to stop writing memory
    *(uint32_t *)h2p_lw_vga_we_addr = 0; 

}

void VGA_clean_up()
{
	*(uint32_t *)h2p_lw_vga_we_addr = 0; 
  *(uint32_t *)h2p_lw_vga_data_addr = 0; 
}

int main() {
  //YOU MUST HIT RESET OF FPGA BEFORE YOU CAN ACTUALLY DRAW ANYTHING
	void *virtual_base;
	int fd;



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
	h2p_lw_led_addr    = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_LED_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_sw_addr     = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_SW_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_addr      = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_ADDR_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_we_addr   = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_WE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_vga_data_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_VGA_DATA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
  
  //VGA_draw_point(320, 240, red);
  //VGA_draw_point(400, 100, green);
  
  //Timing variables
  clock_t begin, end;
  float time_spent;
  int num_iters = 50; 

  // Measure the elapsed time based on CPU time
  int i,j,k; 
  
  begin = clock();
  for (k = 0; k < num_iters; k++){
      
      for (i = 0; i < 640; i++)
       for (j = 0; j < 480; j++)      
          VGA_draw_point(i,j,k&255);
  }        
  end = clock();
  time_spent = (float)(end - begin);
  printf("Time taken for all iterations Cycles: %f (cycles) Time: %0.6f(s)\n", time_spent, (time_spent/(float)CLOCKS_PER_SEC));
  printf("Average time taken for each of the %d iterations was %f (cycles) and %0.6f(s)\n", num_iters, time_spent/num_iters, time_spent/(num_iters*(float)CLOCKS_PER_SEC));
    
   

	/*while(1){//loop_count < 60 ) {
		
		// control led with switchs
		//*(uint32_t *)h2p_lw_led_addr = *(uint32_t *)h2p_lw_sw_addr; 
 

	} // while
	*/


	// clean up our memory mapping and exit
	VGA_clean_up();

  if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
